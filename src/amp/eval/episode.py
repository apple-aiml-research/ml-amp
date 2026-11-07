#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One episode on one slot, end to end: claim, run the agent, score, finalize, record."""

from __future__ import annotations

import contextlib
import json
import os
import secrets
import signal
import subprocess
import threading
import time
import traceback
from collections.abc import Callable, Mapping
from dataclasses import dataclass, replace
from pathlib import Path

from amp import benchmarks
from amp.agents.credentials import runner_env_keep
from amp.agents.llm.base import BUDGET_MARK, TRANSIENT_MARK
from amp.contracts.eval import (
    CAPX_MAX_TRIAL_ATTEMPTS,
    CAPX_TRIAL_TIMEOUT_S,
    DELIVERED_STATUSES,
    EPISODE_FILE,
    OFFICIAL_KEY_ENV,
    TOKEN_KEYS,
    AgentRunConfig,
    EpisodeRequest,
    EpisodeResult,
    NodeRuntime,
)
from amp.contracts.robot import EpisodeRef
from amp.contracts.trace import NullTracer, parse_context, valid_src
from amp.fs.atomic import write_bytes_atomic
from amp.harnesses import registry
from amp.kernel.process import EPISODE_TAG_ENV, kill_tagged
from amp.practice.workspace import WORKSPACE_KEY, WorkspaceError, for_task
from amp.trace import EpisodeTrace, env_for_child, kernel_finalizer, load, register_secret

from . import artifacts, capgym_clock, envs, sandbox
from .gateway_client import ENDED_LOADING, LOAD_TIMEOUT, GatewayClient
from .layout import (
    AGENT_OUTPUT,
    META_FILE,
    RunLayout,
)
from .procs import TOKEN_ENV
from .progress import EpisodeProgress
from .slots import own_src, slot_paths
from .store import is_final_record, read_record, strip_tokens, write_json

AGENT_MODULE = "amp.agents.run"
PRACTICE_AGENT = ("-m", "amp.practice.agent")
TEXT_AGENT = ("-m", "amp.agents_text.run")
TEXT_VISIONS = ("vdm_strict",)
UNCLEAN_END_REASONS = frozenset({"api_error", "kernel_error", "crash", "killed", "unknown",
                                 "episode_over_before_start"})
AGENT_KILLS = frozenset({"timeout", "idle"})
AGENT_END_REASONS = frozenset({"crash", "no_report"})
INFRA_END_REASONS = UNCLEAN_END_REASONS - AGENT_END_REASONS
FATAL_CLAIM = (400, 404)
ABORT_CAUSES = ("canceled", "infra_error", "attempt_error", "error")
SIM_FAILED = "no score: the episode failed in the sim: "
LLM_INFRA = "model endpoint unavailable (infra)"
LIMIT_CAUSE_MARGIN_S = 600.0
OFFICIAL_END = "official_timeout"
RECOVERED_NOTE = "recovered from the slot's saved record of an earlier attempt"
RECOVERED_OUTPUTS = "its robot outputs remain on the slot, not uploaded"
ADOPTED_OUTPUTS = "adopted episode's robot outputs"


OFFICIAL_HOLD_WAIT_S = 3 * 3600.0


def agent_args(vision: str, extra: Mapping | None = None) -> tuple[str, ...]:
    """The agent process's arguments after its interpreter: the text-only agent for a vdm vision,
    the practice agent for a profile extra carrying a workspace."""
    if vision in TEXT_VISIONS:
        return TEXT_AGENT
    if WORKSPACE_KEY in (extra or {}):
        return PRACTICE_AGENT
    return ("-m", AGENT_MODULE)


@dataclass(frozen=True)
class RunnerOptions:
    poll_s: float = 2.0
    claim_backoff_s: float = 2.0
    claim_backoff_max_s: float = 30.0
    term_grace_s: float = 30.0
    progress_s: float = 10.0
    progress_heartbeat_s: float = 60.0
    row_wait_s: float = 600.0
    adopt_upload_s: float = 300.0


def instruction_mismatch(row: dict, want: str) -> str | None:
    """Why a scored row does not count: the robot ran another wording than the job asked for."""
    ran = row.get("instruction_type") or "default"
    if row.get("success") is None or ran == want:
        return None
    return f"instruction_type: asked for {want!r}, the robot ran {ran!r}"


def official_unit(req: EpisodeRequest) -> bool:
    """Whether CaP-X's trial protocol (its attempt, its clock) applies to this unit."""
    return bool(req.protocol.official) and benchmarks.is_capgym(task_set=req.task_set)


def _capx_trial_of(req: EpisodeRequest) -> bool:
    """CaP-X's trial protocol applies (official CaP-Gym), not RoboLab's official."""
    return getattr(req, "official_attempt", 0) >= 1 or official_unit(req)


def run_unit(req: EpisodeRequest, slot, *, runtime: NodeRuntime, run_dir: Path, tracer=None,
             abort: threading.Event | None = None,
             abort_status: Callable[[], str] | None = None,
             progress: Callable[[dict], None] | None = None,
             options: RunnerOptions | None = None,
             provenance: Mapping | None = None) -> EpisodeResult:
    """One unit on one slot: run_episode; an official CaP-Gym unit runs as its attempt 1."""
    if official_unit(req):
        req = replace(req, official_attempt=1)
    return run_episode(req, slot, runtime=runtime, run_dir=Path(run_dir), tracer=tracer,
                       abort=abort, abort_status=abort_status, progress=progress, options=options,
                       provenance=provenance)


def foreign_final(existing: dict | None, rec: dict) -> bool:
    """Whether an existing per-attempt record is another episode's and final for the attempt."""
    return (isinstance(existing, dict) and not existing.get("provisional")
            and existing.get("episode_id") not in (None, rec.get("episode_id"))
            and existing.get("generation", 0) == rec.get("generation", 0)
            and is_final_record(existing))


class _Early(Exception):
    """The unit ends before or without a claimed episode."""

    def __init__(self, status: str, error: str | None, record: dict | None = None):
        super().__init__(error or status)
        self.status, self.error, self.record = status, error, record


def run_episode(req: EpisodeRequest, slot, *, runtime: NodeRuntime, run_dir: Path, tracer=None,
                abort: threading.Event | None = None,
                abort_status: Callable[[], str] | None = None,
                progress: Callable[[dict], None] | None = None,
                options: RunnerOptions | None = None,
                provenance: Mapping | None = None) -> EpisodeResult:
    """Run (req.task, req.episode) of req.arm on slot; never raises."""
    ep = _Episode(req, slot, runtime, Path(run_dir), tracer or NullTracer(),
                  abort or threading.Event(), options or RunnerOptions(), progress, abort_status,
                  provenance)
    try:
        result = ep.run()
    except Exception as e:  # noqa: BLE001
        result = ep.crashed(e)
    finally:
        ep.cleanup()
    ep.attempted = ep.attempted or result.attempted
    ep.claim = ep.claim or ({"episode_id": result.episode_id} if result.episode_id else None)
    ep.report("done")
    return result


def _no_score(row: dict) -> str:
    """Why a row that ran to its end has no score: discarded, failed in the sim, or not scored."""
    if row.get("discarded"):
        return "no score: the episode was discarded"
    err = row.get("error")
    if row.get("outcome") == "failed" and isinstance(err, str) and err.strip():
        return f"{SIM_FAILED}{err.strip()[:300]}"
    return "no score: the episode was not scored"


class _Episode:
    def __init__(self, req: EpisodeRequest, slot, runtime: NodeRuntime, run_dir: Path, tracer,
                 abort: threading.Event, opts: RunnerOptions,
                 progress: Callable[[dict], None] | None = None,
                 abort_status: Callable[[], str] | None = None,
                 provenance: Mapping | None = None):
        try:
            req = replace(req, arm=replace(req.arm, agent=for_task(req.arm.agent, req.task)))
        except WorkspaceError as e:
            self.table_error = e
        self.req, self.slot, self.runtime, self.tracer = req, slot, runtime, tracer
        self.provenance = dict(provenance) if provenance else None
        self.abort, self.opts, self.progress = abort, opts, progress
        self.abort_status = abort_status
        self.self_test: dict | None = None
        self.harness_version: str | None = None
        self.phase: str | None = None
        self.load_timeout_s = benchmarks.load_timeout_s(req.task_set, req.protocol.load_timeout_s)
        self.layout = RunLayout(run_dir)
        self.paths = slot_paths(slot)
        self.node_id = self.paths.node_id
        self.client = GatewayClient(slot.gateway_url, slot.operator_key)
        self.t0 = time.monotonic()
        self.marks: dict[str, float] = {}
        self.claim: dict | None = None
        self.ep_dir: Path | None = None
        self.etr = None
        self.proc: subprocess.Popen | None = None
        self.agent_rc: int | None = None
        self.kill_reason: str | None = None
        self.end_reason: str | None = None
        self.context_overflow = False
        self.llm_transient: tuple[str, str] | None = None
        self.llm_wait_s = 0.0
        self.llm_budget: str | None = None
        self.live: dict = {}
        self.agent_status: dict = {}
        self.turns: EpisodeProgress | None = None
        self.sim_hung = False
        self.totals: dict = {}
        self.sandbox: dict | None = None
        self.row: dict | None = None
        self.recovered_id: str | None = None
        self.recovered_uri: str | None = None
        self.attempted = False
        self.attempt_written = False
        self.failures: list[str] = []
        self.notes: list[str] = []
        self.concluded = False
        self.kernel_tag = secrets.token_hex(16)

    table_error: WorkspaceError | None = None
    official_key: str | None = None
    official_ended = False
    clock_started_at: float | None = None
    infra_spans: tuple[tuple[float, float], ...] = ()
    agent_last_t: float | None = None
    discarded = False

    @property
    def official(self) -> bool:
        """An official capgym attempt: the gateway runs CaP-X's trial clock."""
        return getattr(self.req, "official_attempt", 0) >= 1

    @property
    def capx_trial(self) -> bool:
        """CaP-X's trial protocol applies (official CaP-Gym), not RoboLab's official."""
        return _capx_trial_of(self.req)

    @property
    def episode_id(self) -> str | None:
        return (self.claim or {}).get("episode_id") or self.recovered_id

    @property
    def trial_timeout_s(self) -> int:
        """CaP-X's nominal limit, recorded; the claim's clock is capgym_clock.UNBOUNDED_TRIAL_S."""
        return int(CAPX_TRIAL_TIMEOUT_S)

    @property
    def void(self) -> bool:
        row = self.row or {}
        return row.get("outcome") == "void" or (row.get("official") or {}).get("void") is True

    def report(self, phase: str | None = None) -> None:
        """Tell the caller where the unit is; a broken callback never reaches the runner."""
        self.phase = phase or self.phase
        if self.progress is None:
            return
        with contextlib.suppress(Exception):
            self.progress({"phase": self.phase,
                           "episode_id": self.episode_id, "attempted": self.attempted})

    def abort_cause(self) -> str:
        """abort_status()'s cause, "canceled" when it names none."""
        cause = "canceled"
        if self.abort_status is not None:
            with contextlib.suppress(Exception):
                cause = self.abort_status()
        return cause if cause in ABORT_CAUSES else "canceled"

    def abort_outcome(self) -> str:
        """The status an abort ends in: canceled when canceled, else attempt_error once moved."""
        cause = self.abort_cause()
        if cause == "canceled":
            return "canceled"
        return "attempt_error" if self.attempted else cause

    def mark(self, name: str) -> None:
        self.marks.setdefault(name, time.monotonic())

    def timings(self) -> dict[str, float]:
        m, out = self.marks, {}
        spans = {"claim_s": (None, "claimed"), "load_s": ("claimed", "ready"),
                 "agent_s": ("agent_start", "agent_end"), "end_s": ("agent_end", "row"),
                 "finalize_s": ("row", "finalized")}
        for k, (a, b) in spans.items():
            ta = self.t0 if a is None else m.get(a)
            if ta is not None and b in m:
                out[k] = round(m[b] - ta, 3)
        out["total_s"] = round(time.monotonic() - self.t0, 3)
        return out

    def result(self, status: str, error: str | None = None, **kw) -> EpisodeResult:
        row = self.row or {}
        t = self.totals
        tokens = {k: int(t[k]) for k in TOKEN_KEYS if isinstance(t.get(k), int | float)}
        base = {"job_id": self.req.job_id, "arm_id": self.req.arm.id, "task": self.req.task,
                "episode": self.req.episode, "status": status, "attempted": self.attempted,
                "episode_id": self.episode_id,
                "success": row.get("success"), "score": row.get("score"), "end": row.get("end"),
                "end_reason": self.end_reason, "cost_usd": t.get("cost_usd"),
                "tokens": tokens or None, "turns": t.get("turns"), "wall_s": t.get("wall_s"),
                "timings": self.timings(), "node": self.node_id,
                "slot": self.slot.slot_id, "sandbox": self.sandbox, "error": error,
                "void": self.void}
        if self.official:
            base["official"] = self.official_info()
        if self.void or self.llm_ended():
            base.update(success=None, score=None)
        return EpisodeResult(**{**base, **kw})

    def official_info(self) -> dict:
        """EpisodeResult.official: the attempt, how the gateway ended it, where CaP-X's limit fell."""
        o = (self.row or {}).get("official") or self.live.get("official") or {}
        timeout = o.get("timeout")
        return {"trial_timeout_s": self.trial_timeout_s,
                "max_attempts": CAPX_MAX_TRIAL_ATTEMPTS, "attempt": self.req.official_attempt,
                "timeout": timeout, "cause": o.get("cause") if timeout else None,
                "prior": [], "trial_clock": capgym_clock.TRIAL_CLOCK, **self.limit_info()}

    def limit_info(self) -> dict:
        """The attempt's start and where trial_timeout_s fell on the adjusted clock."""
        start, end = self.clock_started_at, self.agent_last_t
        if start is None:
            return {"started_at": None, "limit_at": None, "limit_infra_s": None}
        at = capgym_clock.adjusted_deadline(start, self.trial_timeout_s, self.infra_spans)
        reached = end is not None and end >= at
        return {"started_at": round(start, 3), "limit_at": round(at, 3) if reached else None,
                "limit_infra_s": round(capgym_clock.infra_s(start, at, self.infra_spans), 3)
                if reached else None}

    def run(self) -> EpisodeResult:
        try:
            self.preamble()
        except _Early as e:
            return self.early(e)
        return self.conclude()

    def preamble(self) -> None:
        req = self.req
        if self.abort.is_set():
            raise _Early("abort", "aborted before it started")
        prior = self.prior_record()
        if prior is not None:
            raise _Early("resumed", None, prior)
        if self.table_error is not None:
            raise _Early("error", f"workspaces: {self.table_error}")
        try:
            self.slot.prepare(req.code_ref, req.arm.harness)
        except (ValueError, LookupError) as e:
            raise _Early("error", f"prepare: {type(e).__name__}: {e}") from e
        except Exception as e:
            raise _Early("infra_error", f"prepare: {type(e).__name__}: {e}") from e
        self.check_sandbox()
        self.resolve_version()
        self.report("claim")
        self.do_claim()
        self.mark("claimed")
        self.lay_out()
        self.wait_ready()
        self.mark("ready")
        self.run_agent()

    def check_sandbox(self) -> None:
        """The sandbox self-test, before holding a robot; a requested sandbox must pass it."""
        requested = self.req.protocol.sandbox
        self.self_test = sandbox.self_test_report(requested, self.runtime.kernel_python)
        self.sandbox = sandbox.record(requested, self.self_test, {})
        st = self.self_test
        if self.official and (st is None or st.get("ok") is not True):
            raise _Early("infra_error",
                         f"official CaP-Gym needs a sandboxed kernel, not {requested!r}: the "
                         "agent's official key would be readable by model code")
        if st is not None and st.get("ok") is not True:
            bad = sorted(k for k, v in (st.get("checks") or {}).items() if not v)
            raise _Early("infra_error", f"sandbox {st.get('backend')} failed its self-test: "
                         f"{', '.join(bad) or st.get('error') or 'no checks ran'}")

    def prior_record(self) -> dict | None:
        """The unit's delivered record: this unit never runs."""
        rec = read_record(self.layout.root, self.req.task, self.req.episode)
        return rec if rec is not None and rec["status"] in DELIVERED_STATUSES else None

    def do_claim(self) -> None:
        req, opts = self.req, self.opts
        deadline = time.monotonic() + self.load_timeout_s
        wait = opts.claim_backoff_s
        claim_id = secrets.token_hex(16)
        while True:
            if self.abort.is_set():
                raise _Early("abort", "aborted while claiming")
            try:
                code, body = self.client.claim(agent=req.arm.agent.label, harness=req.arm.harness,
                                               task=req.task, episode=req.episode,
                                               slot=self.slot.slot_id,
                                               run_id=req.run_id, task_set=req.task_set,
                                               instruction_type=req.protocol.instruction_type,
                                               official=self.official_claim(),
                                               claim_id=claim_id)
            except OSError as e:
                code, body = 0, {"message": f"gateway unreachable: {e}"}
            msg = body.get("message") or ""
            if code == 200 and body.get("token") and body.get("episode_id"):
                key = body.pop("official_key", None)
                self.claim = {**body, "gateway": self.slot.gateway_url}
                register_secret(body["token"])
                if isinstance(key, str) and key:
                    register_secret(key)
                    self.official_key = key
                if self.official and self.official_key is None:
                    raise _Early("infra_error", "claim answered no official_key: the gateway "
                                 "cannot run CaP-X's trial clock")
                if body.get("task") not in (None, req.task):
                    raise _Early("infra_error", f"claim answered task {body.get('task')!r}")
                ran = body.get("instruction_type") or "default"
                if ran != req.protocol.instruction_type:
                    raise _Early("infra_error", f"claim answered instruction_type {ran!r}, not "
                                 f"{req.protocol.instruction_type!r}")
                self.tracer.event("eval.episode.begin", self.unit_info(task=True))
                return
            if code == 409 and "live" not in msg:
                prior = self.prior_record()
                if prior is not None:
                    raise _Early("resumed", None, prior)
                raise self.saved_early(body.get("saved"), f"claim 409: {msg or 'already saved'}")
            if code in FATAL_CLAIM:
                raise _Early("error", f"claim {code}: {msg}")
            pause = min(wait, opts.claim_backoff_max_s)
            if time.monotonic() + pause > deadline:
                raise _Early("infra_error", f"claim {code or 'unreachable'} until the load "
                             f"timeout ({self.load_timeout_s:g}s): {msg}")
            self.abort.wait(pause)
            wait *= 2

    def saved_early(self, saved, why: str) -> _Early:
        """A 409 for an episode the slot saved and no final record: adopt it, refuse it, or error."""
        row = (saved or {}).get("row") if isinstance(saved, dict) else None
        if not self.own_saved_row(row):
            return _Early("error", f"{why}; no delivered record, so that attempt's outputs are lost")
        if (gone := self.not_adoptable(row["episode_id"])) is not None:
            return _Early("infra_error", f"{why}; the earlier attempt's saved episode "
                          f"{row['episode_id']} {gone}: not adopted, so the unit runs fresh")
        moved = row.get("attempted") is True or (row.get("executes") or 0) > 0
        if not moved and row.get("success") is None and row.get("score") is None:
            return _Early("error", f"{why}; no delivered record, so that attempt's outputs are lost")
        self.row, self.recovered_id = dict(row), row["episode_id"]
        self.recovered_uri = self.earlier_artifact_uri(row["episode_id"])
        if moved:
            self.moved("saved record")
        status = self.status()
        self.failures.append(f"{why}; {RECOVERED_NOTE} ({row['episode_id']}); {RECOVERED_OUTPUTS}")
        return _Early(status, "; ".join(self.failures))

    def earlier_artifact_uri(self, eid: str) -> str:
        """The adopted attempt's artifacts: its own record's artifact_uri, else where it put them."""
        rec = None
        with contextlib.suppress(Exception):
            rec = read_record(self.layout.root, self.req.task, self.req.episode)
        if isinstance(rec, dict) and rec.get("episode_id") == eid and rec.get("artifact_uri"):
            return rec["artifact_uri"]
        return str(self.layout.episodes / self.layout.final_name(self.req.task, eid))

    def adopt_outputs(self, error: str) -> str:
        """The error once the adopted episode's robot outputs are listed, best effort and bounded."""
        box: list[tuple[bool, str]] = []
        prog = {"put": 0, "of": None}
        stop, limit = threading.Event(), self.opts.adopt_upload_s

        def halted() -> str | None:
            if self.abort.is_set():
                return "the episode was aborted"
            return f"the {limit:g}s limit" if stop.is_set() else None

        def work():
            try:
                box.append(self._adopt_outputs(halted, prog))
            except Exception as e:  # noqa: BLE001
                box.append((False, f"{type(e).__name__}: {e}"))

        t = threading.Thread(target=work, name="adopt-outputs", daemon=True)
        t.start()
        end = time.monotonic() + limit
        while t.is_alive() and not self.abort.is_set() and (left := end - time.monotonic()) > 0:
            t.join(min(left, 0.05))
        stop.set()
        if box:
            ok, note = box[0]
        else:
            why = halted()
            ok, note = False, (f"{prog['put']} of {prog['of']} copied before {why}"
                               if prog["of"] is not None else f"none staged before {why}")
        self.notes.append(f"{ADOPTED_OUTPUTS}: {note}")
        if not ok:
            return error
        self.failures = [f.replace(f"; {RECOVERED_OUTPUTS}", "") for f in self.failures]
        return error.replace(f"; {RECOVERED_OUTPUTS}", "")

    def _adopt_outputs(self, halted: Callable[[], str | None], prog: dict) -> tuple[bool, str]:
        """Stage the adopted episode's robot outputs where conclude() puts a normal one's; True
        only when every selected file is there."""
        req, lay, eid = self.req, self.layout, self.recovered_id
        if why := halted():
            return False, f"none staged before {why}"
        res, ep_dir = self.paths.results_dir, self.adopted_dir()
        if res is None:
            return False, "the slot has no results dir"
        if ep_dir is None:
            return False, f"{self.recovered_uri} is not this run's episode dir; not copied"
        ref = EpisodeRef(episode_id=eid, task=req.task, episode=req.episode, episode_dir=ep_dir,
                         results_dir=res / req.run_id / req.task, run_id=req.run_id)
        got = artifacts.collect_adopted(lay, ref, self.slot.backend(), req.run_id,
                                        halt=lambda: halted() is not None)
        if why := halted():
            return False, f"none staged before {why}"
        n, gaps = len(got.rels) + len(got.refused), got.errors + got.refused
        if not got.rels:
            return False, f"none of episode {req.episode} in {ref.results_dir}" + \
                "".join(f"; {g}" for g in gaps)
        prog["of"], put = n, list(got.rels)
        if len(put) != n or gaps:
            return False, f"{len(put)} of {n} copied" + "".join(f"; {g}" for g in gaps)
        return True, f"copied {n} of {n}: {', '.join(put)}"

    def adopted_dir(self) -> Path | None:
        """The local episode dir the adopted record's artifact_uri names, if it is this run's."""
        name = self.layout.final_name(self.req.task, self.recovered_id)
        uri = self.recovered_uri or ""
        base = uri.rstrip("/").rsplit("/", 1)[-1]
        rest = base[len(name):] if base.startswith(name) else None
        if not (rest == "" or (rest and rest[0] == "-" and rest[1:].isdigit())):
            return None
        d = self.layout.episodes / base
        return d if uri == str(d) else None

    def not_adoptable(self, eid: str) -> str | None:
        """Why the slot's saved attempt eid is not this unit's to adopt (its own record says infra
        cut it), or None."""
        rec = None
        with contextlib.suppress(OSError, ValueError):
            rec = read_record(self.layout.root, self.req.task, self.req.episode)
        if isinstance(rec, dict) and rec.get("episode_id") == eid and (
                rec.get("end_reason") in INFRA_END_REASONS or LLM_INFRA in (rec.get("error") or "")):
            return f"ended by infra ({rec.get('end_reason') or LLM_INFRA})"
        return None

    def own_saved_row(self, row) -> bool:
        """Whether a saved ledger row is this unit's: its run (job and harness configuration), task, episode, code."""
        req = self.req
        if not isinstance(row, dict) or row.get("outcome") != "saved" \
                or not isinstance(row.get("episode_id"), str) or not row["episode_id"] \
                or "code_ref" not in row:
            return False
        return (row.get("run_id") == req.run_id and row.get("task") == req.task
                and row.get("episode") == req.episode and row.get("harness") == req.arm.harness
                and row["code_ref"] is None)

    def resolve_version(self) -> None:
        """Set harness_version from the harness's manifest (an overlay harness is generated)."""
        try:
            self.harness_version = str(registry.load(self.req.arm.harness).version) or None
        except Exception as e:  # noqa: BLE001
            self.tracer.event("eval.harness_version.unresolved",
                              {"code_ref": self.req.code_ref, "harness": self.req.arm.harness,
                               "error": f"{type(e).__name__}: {e}"}, lvl="warn")

    def official_claim(self) -> dict | None:
        if not self.official:
            return None
        return {"trial_timeout_s": capgym_clock.UNBOUNDED_TRIAL_S,
                "attempt": self.req.official_attempt, "max_attempts": CAPX_MAX_TRIAL_ATTEMPTS}

    def start_clock(self) -> bool:
        """Start the gateway's trial clock as the agent starts; nothing has moved if it fails."""
        try:
            code, body = self.client.official_start(self.claim["token"])
        except OSError as e:
            code, body = 0, {"message": str(e)}
        if code == 200:
            self.clock_started_at = time.time()
            return True
        self.failures.append(f"official trial clock not started: {code or 'unreachable'}: "
                             f"{body.get('message')}")
        return False

    def unit_info(self, *, task: bool) -> dict:
        req = self.req
        d = {"job_id": req.job_id, "arm_id": req.arm.id, "harness": req.arm.harness,
             "agent": req.arm.agent.label, "episode": req.episode, "code_ref": req.code_ref,
             "slot": self.slot.slot_id, "node": self.node_id,
             "episode_id": self.episode_id}
        if self.harness_version:
            d["harness_version"] = self.harness_version
        if task:
            d["task"] = req.task
        return d

    def lay_out(self) -> None:
        c, req = self.claim, self.req
        eid = c["episode_id"]
        ep_dir = self.layout.live_dir(eid)
        ep_dir.mkdir(parents=True, exist_ok=False)
        self.ep_dir = ep_dir
        info = {"gateway": self.slot.gateway_url, "token": c["token"], "base": c["base"],
                "episode_id": eid, "instruction": c.get("instruction") or ""}
        write_bytes_atomic(ep_dir / EPISODE_FILE, json.dumps(info).encode(), mode=0o600)
        try:
            AgentRunConfig(agent=req.arm.agent, harness=req.arm.harness,
                           kernel_python=self.runtime.kernel_python,
                           protocol=req.protocol).write(ep_dir)
        except (TypeError, ValueError) as e:
            raise _Early("error", f"agent config: {type(e).__name__}: {e}") from e
        span = parse_context(self.tracer.context())[1]
        self.etr = EpisodeTrace.open(ep_dir, eid, base_span=span)
        self.etr.event("eval.episode.begin", {**self.unit_info(task=False),
                                              "protocol": req.to_dict()["protocol"]})

    def episode_status(self) -> dict | None:
        try:
            code, body = self.client.episode(self.claim["episode_id"])
        except OSError:
            return None
        return body if code == 200 else None

    def wait_ready(self) -> None:
        deadline = self.marks["claimed"] + self.load_timeout_s
        while True:
            st = self.episode_status()
            if st is not None and st.get("over"):
                self.row = st.get("row")
                self.failures.append(f"{ENDED_LOADING}"
                                     f"{(self.row or {}).get('error') or (self.row or {}).get('end')}")
                return
            if st is not None and (st.get("live") or {}).get("state") == "ready":
                return
            if self.abort.is_set():
                self.kill_reason = "abort"
                return
            if time.monotonic() >= deadline:
                self.failures.append(f"{LOAD_TIMEOUT} ({self.load_timeout_s:g}s)")
                return
            self.abort.wait(min(self.opts.poll_s, 1.0))

    def run_agent(self) -> None:
        if self.failures or self.kill_reason or self.row is not None:
            return
        ep_dir = self.ep_dir
        cmd = [self.runtime.agent_python,
               *agent_args(self.req.arm.agent.vision, self.req.arm.agent.extra)]
        cmd.append(str(ep_dir))
        src = own_src()
        try:
            with self.etr.span("eval.agent", {"cmd": cmd[:-1]}):
                env = {k: v for k, v in envs.node_env(keep=(*runner_env_keep(os.environ), TOKEN_ENV)).items()
                       if not k.startswith("PYTHON")}
                env.update(PYTHONPATH=str(src), PYTHONUNBUFFERED="1", **env_for_child(self.etr))
                env[EPISODE_TAG_ENV] = self.kernel_tag
                if self.official_key is not None:
                    env[OFFICIAL_KEY_ENV] = self.official_key
                self.mark("agent_start")
                if self.official and not self.start_clock():
                    return
                self.report("agent")
                with open(ep_dir / AGENT_OUTPUT, "ab") as out:
                    try:
                        self.proc = subprocess.Popen(cmd, cwd=ep_dir, env=env, stdout=out,
                                                     stderr=subprocess.STDOUT,
                                                     stdin=subprocess.DEVNULL,
                                                     start_new_session=True)
                    except OSError as e:
                        self.failures.append(f"agent did not start: {e}")
                        return
                self.start_progress()
                self.supervise()
                self.progress_tick(final=True)
        finally:
            self.mark("agent_end")

    def start_progress(self) -> None:
        o = self.opts
        with contextlib.suppress(Exception):
            self.turns = EpisodeProgress(self.ep_dir, self.unit_info(task=True),
                                         every_s=o.progress_s, heartbeat_s=o.progress_heartbeat_s)

    def progress_tick(self, *, final: bool = False) -> None:
        if self.turns is not None:
            self.turns.tick(self.agent_status, final=final)

    def supervise(self) -> None:
        p, proto = self.proc, self.req.protocol
        start = self.marks["agent_start"]
        while True:
            try:
                self.agent_rc = p.wait(timeout=self.opts.poll_s)
                self.poll_live()
                return
            except subprocess.TimeoutExpired:
                pass
            if self.abort.is_set():
                return self.kill("abort")
            if time.monotonic() - start >= proto.episode_timeout_s:
                self.poll_live()
                return self.kill("timeout")
            idle = self.poll_live()
            self.progress_tick()
            if self.official_ended:
                return self.kill(OFFICIAL_END)
            if idle is not None and idle >= proto.idle_timeout_s:
                return self.kill("idle")

    def poll_live(self) -> float | None:
        """Track attempted from the gateway; the agent's idle seconds, if it answered."""
        st = self.episode_status()
        if st is not None:
            live = self.live = st.get("live") or {}
            row = st.get("row") or {}
            if (live.get("executes") or 0) > 0 or (row.get("executes") or 0) > 0:
                self.moved("gateway executes")
            if self.official and OFFICIAL_END in (live.get("end"), row.get("end")):
                self.official_ended = True
        try:
            code, body = self.client.agent_episode(self.claim["token"])
        except OSError:
            return None
        self.agent_status = body if code == 200 else {}
        idle = body.get("idle_s") if code == 200 else None
        return float(idle) if isinstance(idle, int | float) else None

    def moved(self, source: str) -> None:
        first = not self.attempted
        self.attempted = True
        if first:
            self.report()
        if self.attempt_written:
            return
        req = self.req
        rec = {**self.unit_info(task=True), "task": req.task, "at": time.time(),
               "source": source}
        if self.official:
            rec["generation"] = 0
        try:
            write_json(self.layout.attempt_path(req.task, req.episode), rec)
            self.attempt_written = True
        except OSError as e:
            self.notes.append(f"attempt marker not written yet: {e}")

    def kill(self, reason: str) -> None:
        if self.kill_reason is None and reason == "timeout":
            self.sim_hung = bool(self.live.get("sim_hung"))
        self.kill_reason = self.kill_reason or reason
        p = self.proc
        if p is None or p.poll() is not None:
            self.agent_rc = None if p is None else p.returncode
            return
        for sig, wait in ((signal.SIGTERM, self.opts.term_grace_s), (signal.SIGKILL, 10.0)):
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.killpg(p.pid, sig)
            try:
                self.agent_rc = p.wait(timeout=wait)
                return
            except subprocess.TimeoutExpired:
                self.notes.append(f"agent pid {p.pid} survived {sig.name}")

    def read_agent_trace(self) -> None:
        try:
            tr = load(self.ep_dir)
        except Exception as e:  # noqa: BLE001
            self.notes.append(f"trace unreadable: {e}")
            return
        ends = [e for e in tr.by_kind("agent.run.end") if e.trusted and e.src == "agent"]
        if ends:
            d = ends[-1].data
            self.end_reason = d.get("end_reason")
            self.totals = d.get("totals") if isinstance(d.get("totals"), dict) else {}
        fatal = [e for e in tr.by_kind("agent.llm.error")
                 if e.trusted and e.src == "agent" and e.data.get("fatal")]
        self.context_overflow = bool(fatal) and fatal[-1].data.get("context_overflow") is True
        retrying = False
        spans = []
        for e in tr.events:
            if not e.trusted or e.src != "agent":
                continue
            self.agent_last_t = e.t
            if e.kind == "agent.episode.over":
                break
            if e.kind == "agent.llm.retry" and (span := capgym_clock.retry_span(e.t, e.data)):
                spans.append(span)
            if e.kind == "agent.llm.retry":
                retrying = e.data.get("transient") is True and e.data.get("retry_in_s") is not None
                if e.data.get("transient") is True:
                    self.llm_wait_s += sum(float(e.data.get(k) or 0.0)
                                           for k in ("elapsed_s", "retry_in_s"))
            elif e.kind in ("agent.llm.usage", "agent.llm.error"):
                retrying = False
            err = str(e.data.get("error") or "") if e.kind == "agent.llm.error" else ""
            if TRANSIENT_MARK in err:
                self.llm_transient = ("exhausted" if e.data.get("fatal") else "deadline", err)
            if BUDGET_MARK in err and e.data.get("fatal"):
                self.llm_budget = err
        self.infra_spans = tuple(spans)
        if retrying and self.llm_transient is None:
            self.llm_transient = ("killed", f"a model call was retrying {TRANSIENT_MARK}s")
        kernel: dict = {}
        for e in tr.events:
            if not e.trusted or e.src != "kernel":
                continue
            if e.kind == "kernel.start.begin":
                kernel["backend"] = e.data.get("backend")
            elif e.kind == "kernel.close":
                kernel["relay"] = e.data.get("relay")
        self.sandbox = sandbox.record(self.req.protocol.sandbox, self.self_test, kernel)

    def end_episode(self) -> None:
        """Release or finish through the gateway, then wait for the ledger row."""
        if self.row is not None:
            return
        st = self.episode_status()
        if st is not None and st.get("over"):
            self.row = st.get("row")
            return
        token = self.claim["token"]
        clean = (self.agent_rc == 0 and self.kill_reason is None and not self.failures
                 and self.end_reason not in UNCLEAN_END_REASONS)
        scored = clean or (not self.attempted and self.agent_fault() is not None)
        if self.canceled():
            self.discard(token)
        else:
            try:
                code, body = self.client.finish(token) if scored else self.client.release(token)
                if code != 200:
                    self.notes.append(f"{'finish' if scored else 'release'} {code}: "
                                      f"{body.get('message')}")
            except OSError as e:
                self.notes.append(f"ending the episode failed: {e}")
        wait = self.row_wait_s()
        now = time.monotonic()
        deadline, heard = now + wait, now
        while time.monotonic() < deadline:
            st = self.episode_status()
            if st is not None and st.get("over"):
                self.row = st.get("row")
                return
            if not self.discarded and self.canceled():
                self.discard(token)
                deadline = min(deadline, time.monotonic() + self.row_wait_s())
            if st is not None:
                heard = time.monotonic()
                if ((st.get("live") or {}).get("executes") or 0) > 0:
                    self.moved("gateway live row")
            elif wait > self.opts.row_wait_s and time.monotonic() - heard >= self.opts.row_wait_s:
                self.failures.append(f"no ledger row: the gateway has not answered for "
                                     f"{self.opts.row_wait_s:g}s")
                return
            time.sleep(min(self.opts.poll_s, 1.0))
        self.failures.append(f"no ledger row within {wait:g}s")

    def discard(self, token: str) -> None:
        """The unit was canceled: end its episode unscored now."""
        self.discarded = True
        try:
            code, body = self.client.release(token, discard=True)
            if code != 200:
                self.notes.append(f"discarding release {code}: {body.get('message')}")
        except OSError as e:
            self.notes.append(f"discarding the episode failed: {e}")

    def row_wait_s(self) -> float:
        """How long to wait for the row: past its backend's post-publish tail, or RoboLab's horizon."""
        if self.discarded:
            return self.opts.row_wait_s
        wait = benchmarks.row_wait_s(self.req.task_set, self.opts.row_wait_s)
        start = self.marks.get("agent_start")
        if not self.req.protocol.official or start is None or self.official or self.discarded:
            return wait
        left = self.req.protocol.episode_timeout_s - (time.monotonic() - start)
        return max(wait, left, OFFICIAL_HOLD_WAIT_S)

    def status(self) -> str:
        row = self.row or {}
        if (row.get("executes") or 0) > 0:
            self.moved("ledger row")
        if self.kill_reason == "abort" or self.discarded:
            out = self.abort_outcome()
            cause = self.abort_cause()
            self.failures.append(f"aborted ({out}{'' if cause == out else f'; cause {cause}'})")
            return out
        limit_ended = self.official and row.get("end") == OFFICIAL_END
        fails = list(self.failures)
        if self.void:
            why = row.get("error") or (row.get("official") or {}).get("void_reason")
            fails.append(f"void episode: {why or 'invalid'}")
        timed_out = (self.kill_reason == "timeout" or row.get("end") == "timeout") \
            and row.get("success") is not None and benchmarks.is_capgym(task_set=self.req.task_set) \
            and not self.hung_kill()
        if timed_out:
            self.notes.append("capgym: graded on the live state at the limit")
        llm = self.llm_infra()
        if not (timed_out or limit_ended or llm) and (self.agent_rc not in (0, None)
                                                       or (self.agent_rc is None
                                                           and self.proc is not None)):
            fails.append(f"agent exit {self.agent_rc}")
        if self.kill_reason and not (timed_out and self.kill_reason == "timeout") \
                and not (limit_ended and self.kill_reason == OFFICIAL_END) and not llm:
            fails.append(f"agent killed: {self.kill_reason}")
        if self.end_reason in UNCLEAN_END_REASONS and not (timed_out and self.end_reason == "killed") \
                and not (limit_ended and self.end_reason in ("killed", "api_error")) and not llm:
            fails.append(f"agent ended: {self.end_reason}")
        if llm:
            fails.append(llm)
        if row.get("end") == "error":
            fails.append(f"gateway: {row.get('error') or 'error'}")
        if self.hung_kill():
            fails.append(f"ended at its {self.limit()} limit while the sim was not answering")
        mismatch = instruction_mismatch(row, self.req.protocol.instruction_type)
        if mismatch:
            fails.append(mismatch)
        self.failures = list(dict.fromkeys(fails))
        if mismatch:
            return "error"
        if row.get("success") is not None and not self.failures:
            return "ok"
        if row.get("success") is None and not self.failures:
            self.failures.append(_no_score(row))
        if not self.attempted and not self.void:
            if (fault := self.scored_fault()) is not None:
                self.failures.append(f"the agent {fault} before the robot moved: scored as it "
                                     "stands, not retried")
            elif self.hung_kill():
                self.failures.append("infra, retried")
        return self.failure_status()

    def llm_infra(self) -> str | None:
        """The failure text when the model endpoint (a spent budget, transient failures) ended it."""
        if self.llm_budget is not None and self.end_reason == "api_error":
            return f"{LLM_INFRA}: budget: {self.llm_budget[:300]}"
        surfaced = self.capx_trial and self.model_timeout_ended()
        if self.llm_transient is None and surfaced:
            return f"{LLM_INFRA}: exhausted: a model call's surfaced timeout ({TRANSIENT_MARK}s)"
        if self.llm_transient is None:
            return None
        kind, err = self.llm_transient
        if kind in ("deadline", "killed") and self.llm_wait_s <= LIMIT_CAUSE_MARGIN_S:
            return None
        if kind == "killed" and (self.kill_reason != "timeout" or self.hung_kill()
                                 or benchmarks.is_capgym(task_set=self.req.task_set)):
            return None
        if self.kill_reason and kind != "killed" \
                and not (self.capx_trial and self.kill_reason == OFFICIAL_END):
            return None
        if kind == "exhausted" and self.end_reason != "api_error" and not surfaced:
            return None
        if kind != "exhausted":
            err = f"{err}; transient waits {self.llm_wait_s:.0f} s of the wall limit"
        return f"{LLM_INFRA}: {kind}: {err[:300]}"

    def model_timeout_ended(self) -> bool:
        """The gateway ended the attempt on the agent's surfaced model timeout (path 2)."""
        o = (self.row or {}).get("official") or self.live.get("official") or {}
        return o.get("cause") == "model_timeout" and o.get("timeout") == "outside_block"

    def llm_ended(self) -> bool:
        """The model endpoint, not the agent, ended the attempt: no score of it is the agent's."""
        return self.llm_infra() is not None

    def limit(self) -> str | None:
        """The idle or wall-clock limit that ended the episode, the runner's kill or the gateway's."""
        end = (self.row or {}).get("end")
        return self.kill_reason or (end if end in AGENT_KILLS else None)

    def hung_kill(self) -> bool:
        """A wall-clock end while the sim had stopped answering: the wait was the sim's."""
        return self.limit() == "timeout" and (self.sim_hung or bool((self.row or {}).get("sim_hung")))

    def agent_fault(self) -> str | None:
        """How the agent ended the episode itself ("killed: idle"), or None for an infra cause."""
        if "agent_start" not in self.marks or self.proc is None or self.void \
                or (self.row or {}).get("end") == "error" or self.llm_ended():
            return None
        if (kill := self.limit()) is not None:
            return f"killed: {kill}" if kill in AGENT_KILLS and not self.hung_kill() else None
        if self.end_reason in AGENT_END_REASONS:
            return f"ended: {self.end_reason}"
        if self.end_reason == "api_error" and self.context_overflow:
            return "ended: api_error (its context overflowed)"
        return None

    def scored_final(self) -> bool:
        """A score the gateway published that is the agent's: not void, not a hung-sim limit."""
        return (self.row or {}).get("success") is not None and not self.void \
            and not self.hung_kill() and not self.llm_ended()

    def scored_fault(self) -> str | None:
        """agent_fault() once the gateway scored the episode; unscored, the cause is infra."""
        return self.agent_fault() if (self.row or {}).get("success") is not None else None

    def failure_status(self) -> str:
        """attempt_error once moved or on a scored agent fault; void: infra_error."""
        if self.void:
            return "infra_error"
        return "attempt_error" if self.attempted or self.scored_fault() is not None \
            else "infra_error"

    def ingest_streams(self) -> list[tuple]:
        eid = self.claim["episode_id"]
        out = []
        for root in (self.paths.gateway_state_dir, self.paths.server_state_dir):
            d = root / "episodes" / eid / "trace" if root else None
            if d is None or not d.is_dir():
                continue
            for p in sorted(d.glob("*.jsonl")):
                src = p.name[:-len(".jsonl")].replace("__", "/")
                if valid_src(src) and not src.startswith(("eval", "agent")):
                    out.append((p, src, True))
        return out

    def backend_ref(self, ep_dir: Path) -> EpisodeRef:
        res, run_id = self.paths.results_dir, self.req.run_id
        return EpisodeRef(episode_id=self.claim["episode_id"], task=self.req.task,
                          episode=self.req.episode, episode_dir=ep_dir, run_id=run_id,
                          results_dir=(res / run_id / self.req.task) if res
                          else ep_dir / "_no_results")

    def finalize_trace(self, record: dict) -> None:
        backend = None
        try:
            backend = self.slot.backend()
            if backend is not None:
                backend.ingest(self.backend_ref(self.ep_dir), self.etr)
        except Exception as e:  # noqa: BLE001
            self.notes.append(f"backend ingest: {type(e).__name__}: {e}")
        self.etr.event("eval.episode.end", record)
        self.etr.close()
        index = EpisodeTrace([kernel_finalizer]).finalize(self.ep_dir,
                                                          ingest=self.ingest_streams())
        refused = [i["target"] for i in index.get("ingested", []) if i.get("refused")
                   not in (None, "missing")]
        if refused:
            self.notes.append(f"trace ingest refused: {refused}")

    def reveal(self) -> None:
        base = self.layout.episodes / self.layout.final_name(self.req.task,
                                                             self.claim["episode_id"])
        dst, k = base, 2
        while dst.exists():
            dst, k = base.with_name(f"{base.name}-{k}"), k + 1
        os.rename(self.ep_dir, dst)
        self.ep_dir = dst

    def write_meta(self) -> None:
        meta = {"claim": strip_tokens(self.claim), "request": self.req.to_dict(), "gateway_row": self.row,
                "agent": {"exit": self.agent_rc, "killed": self.kill_reason,
                          "end_reason": self.end_reason},
                "runtime": self.provenance,
                "notes": self.notes, "failures": self.failures}
        write_json(self.ep_dir / META_FILE, meta)

    def canceled(self) -> bool:
        """Aborted because the run was canceled."""
        return self.abort.is_set() and self.abort_cause() == "canceled"

    def conclude(self) -> EpisodeResult:
        """Everything after a claim, whatever happened before; each step is best effort."""
        self.concluded = True
        if self.proc is not None and self.proc.poll() is None:
            self.kill(self.kill_reason or "runner")
        self.reap_kernels()
        self.read_agent_trace()
        self.report("release")
        self.end_episode()
        self.mark("row")
        status = self.status()
        self.report("finalize")
        result = self.result(status, "; ".join(self.failures) or None, artifact_uri=None)
        try:
            self.finalize_trace(self.record(result))
        except Exception as e:  # noqa: BLE001
            self.notes.append(f"trace finalize: {type(e).__name__}: {e}")
        self.reveal()
        result.artifact_uri = str(self.ep_dir)
        self.write_record(result)
        self.write_meta()
        backend = None
        with contextlib.suppress(Exception):
            backend = self.slot.backend()
        collected = artifacts.collect(self.layout, self.ep_dir, self.backend_ref(self.ep_dir),
                                      backend, self.req.run_id)
        self.notes.extend(collected.errors)
        self.mark("finalized")
        result.attempted = self.attempted
        result.timings = self.timings()
        self.write_meta()
        self.write_record(result)
        self.tracer.event("eval.episode.end", result.to_record())
        return result

    def record(self, result: EpisodeResult) -> dict:
        """The unit record: the result's, plus runtime facts."""
        rec = result.to_record()
        if self.notes:
            rec["notes"] = list(self.notes)
        if self.provenance:
            rec["runtime"] = dict(self.provenance)
        return rec

    def write_record(self, result: EpisodeResult) -> None:
        req = self.req
        rec = self.record(result)
        write_json(self.layout.unit_path(req.task, req.episode), rec)
        if self.official:
            self.write_official(rec)

    def write_official(self, rec: dict) -> None:
        """The official attempt's own record, unless another episode's final one is there."""
        req, rec = self.req, dict(rec)
        rec.pop("notes", None)
        rec["generation"] = 0
        if rec.get("cost_usd") is not None:
            rec["cost_usd"] = round(max(rec["cost_usd"], 0.0), 9)
        if rec.get("turns") is not None:
            rec["turns"] = max(rec["turns"], 0)
        if rec.get("wall_s") is not None:
            rec["wall_s"] = round(max(rec["wall_s"], 0.0), 9)
        if rec.get("tokens"):
            rec["tokens"] = {k: max(v, 0) for k, v in rec["tokens"].items()}
        path = self.layout.official_path(req.task, req.episode)
        try:
            existing = json.loads(path.read_bytes())
        except (OSError, ValueError):
            existing = None
        if foreign_final(existing, rec):
            self.notes.append("another episode's final attempt record stands; not overwritten")
            return
        write_json(path, rec)

    def early(self, e: _Early) -> EpisodeResult:
        if e.record is not None:
            prior = EpisodeResult.from_record(e.record)
            prior.status = "resumed"
            prior.timings = self.timings()
            prior.error = None
            self.tracer.event("eval.episode.end", prior.to_record())
            return prior
        if e.status == "abort":
            e.status = self.abort_outcome()
        if self.recovered_id is not None and RECOVERED_OUTPUTS in (e.error or ""):
            e.error = self.adopt_outputs(e.error)
        if self.claim is not None:
            self.failures.append(e.error or e.status)
            with contextlib.suppress(OSError):
                self.client.release(self.claim["token"])
        result = self.result(e.status, e.error, **(
            {"artifact_uri": self.recovered_uri} if self.recovered_uri else {}))
        if e.status != "canceled" or self.claim is not None:
            with contextlib.suppress(Exception):
                self.write_record(result)
        self.tracer.event("eval.episode.end", result.to_record())
        return result

    def crashed(self, exc: Exception) -> EpisodeResult:
        err = f"{type(exc).__name__}: {exc}\n{traceback.format_exc(limit=5)}"[-4000:]
        self.failures.append(err)
        if self.claim is not None and self.ep_dir is not None and not self.concluded \
                and self.etr is not None:
            try:
                return self.conclude()
            except Exception as e2:  # noqa: BLE001
                self.failures.append(f"conclude: {type(e2).__name__}: {e2}")
        scored = self.scored_final()
        if scored:
            self.failures.append("the runner failed after the gateway scored the episode: "
                                 "the score stands, not retried")
        status = "attempt_error" if (self.attempted or scored) and not self.void \
            else "infra_error"
        if self.claim is not None and not self.attempted:
            with contextlib.suppress(Exception):
                self.client.release(self.claim["token"])
        try:
            result = self.result(status, "; ".join(self.failures)[-4000:])
        except Exception:  # noqa: BLE001
            result = EpisodeResult(job_id=self.req.job_id, arm_id=self.req.arm.id,
                                   task=self.req.task, episode=self.req.episode, status=status,
                                   attempted=self.attempted, error=err)
        with contextlib.suppress(Exception):
            self.write_record(result)
        return result

    def reap_kernels(self) -> None:
        """Kill what the agent's kernels left running: outside a PID namespace a daemon outlives
        the agent, and an agent killed hard never closed its kernel."""
        if self.proc is None:
            return
        with contextlib.suppress(Exception):
            left = kill_tagged(self.kernel_tag)
            if left:
                self.notes.append(f"killed {len(left)} process(es) the kernel left running")

    def cleanup(self) -> None:
        if self.proc is not None and self.proc.poll() is None:
            self.kill(self.kill_reason or "runner exit")
        self.reap_kernels()
        if self.etr is not None:
            with contextlib.suppress(Exception):
                self.etr.close()
