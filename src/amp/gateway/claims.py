#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Directed, keyed claims and each episode's lifecycle on its slot, from claim to ledger row."""

from __future__ import annotations

import hmac
import http.client
import itertools
import json
import logging
import os
import re
import secrets
import threading
import time
import traceback
from collections.abc import Callable, Iterable, Mapping
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field
from pathlib import Path
from urllib.parse import quote

from amp.contracts.eval import INSTRUCTION_TYPES, OFFICIAL_KEY_HEADER
from amp.contracts.trace import TRACE_HEADER, NullTracer, Tracer
from amp.fs.atomic import write_json_atomic
from amp.gateway.errors import HTTPError
from amp.gateway.ledger import Counters, Ledger, episode_row, iso, official_row
from amp.gateway.official import (
    GRADE,
    OUTSIDE_BLOCK,
    PATH2,
    STOP,
    Official,
    grade_failure_is_infra,
    key_digest,
    parse_claim,
)
from amp.gateway.policy import HarnessPolicy, PolicyStore
from amp.gateway.proxy import KEY_HEADER
from amp.gateway.tracing import close_tracer, emit, make_tracer
from amp.gateway.workers import DOWN_AFTER_FAILURES, Slot, SlotState

log = logging.getLogger("amp.gateway")

STATE_FORMAT = "amp-gateway-1"
STATE_FILE = "gateway_state.json"
CALL_TIMEOUT_S = 10.0
SAMPLE_TIMEOUT_S = 1.0
OFFICIAL_TICK_S = 0.05
CATALOG_MAX_AGE_S = 5.0
ACTIVITY_SAVE_S = 10.0
ENDED_KEEP_S = 3 * 3600.0
REFUSAL_LOG_EVERY = 64
RETRY_AFTER_S = 30
AGENT_MAX = 200
GRADE_BARRIER_S = 2 * CALL_TIMEOUT_S + 2
GRADE_FEATURE = "grade"
RUN_ID = re.compile(r"(?!\.\.?$)[A-Za-z0-9_.-]{1,200}")
CLAIM_ID = re.compile(r"[A-Za-z0-9_-]{8,64}")
ORPHAN_GRACE_S = 120.0


class MotionRefused(HTTPError):
    """A motion command outside a begun cell of an official episode (A2)."""

    def __init__(self):
        super().__init__(409, "official: the robot moves only inside a code cell, before the "
                              "trial's time limit", {"official": "outside_cell"})


def _instruction(t: dict, itype: str) -> str:
    """A catalog task's instruction in wording itype; "" if its catalog lacks that wording."""
    if itype == "default":
        return t.get("instruction") or ""
    variants = t.get("instructions")
    return str(variants.get(itype) or "") if isinstance(variants, dict) else ""


@dataclass
class Episode:
    token: str
    episode_id: str
    agent: str
    harness: str
    worker: str
    task: str
    instruction: str
    claimed_at: float
    info_mode: str = "standard"
    code_ref: str | None = None
    run_id: str = ""
    task_set: str = ""
    load_timeout_s: float | None = None
    instruction_type: str = "default"
    claim_id: str = ""
    seen: bool = False
    job_id: int | None = None
    episode: int | None = None
    ready_at: float | None = None
    last_active: float = 0.0
    end: str | None = None
    end_at: float | None = None
    finish_requested: bool = False
    released: bool = False
    publish_sent: bool = False
    abort_sent: bool = False
    over: bool = False
    outcome: str | None = None
    ended_at: float | None = None
    error: str | None = None
    discard: bool = False
    abandoned: bool = False
    sim_hung: bool = False
    counters: Counters = field(default_factory=Counters)
    official: Official | None = None

    @property
    def state(self) -> str:
        if self.end is not None or self.over:
            return "over"
        return "ready" if self.ready_at is not None else "loading"

    @classmethod
    def from_dict(cls, d: dict) -> Episode:
        known = set(cls.__dataclass_fields__)
        d = {k: v for k, v in d.items() if k in known}
        d["counters"] = Counters(**(d.get("counters") or {}))
        if isinstance(d.get("official"), dict):
            d["official"] = Official.from_dict(d["official"])
        return cls(**d)


TracerFactory = Callable[[Path, str], Tracer]


class Gateway:
    """The slots, the episodes on them and the polls that move episodes along; no HTTP."""

    def __init__(self, slots: list[tuple[str, str]], policies: PolicyStore, operator_key: str,
                 state_dir: str | Path, *, idle_timeout_s: float = 1800.0,
                 episode_timeout_s: float = 21600.0, load_timeout_s: float = 900.0,
                 request_timeout_s: float = 600.0, poll_s: float = 1.0,
                 tracer_factory: TracerFactory = make_tracer,
                 clock: Callable[[], float] = time.time,
                 concurrent_task_sets: Iterable[str] = (),
                 slot_task_sets: Callable[[], Mapping[str, str]] | None = None,
                 slot_task_sets_path: str | None = None,
                 load_timeouts: Mapping[str, float] | None = None,
                 official_tick_s: float = OFFICIAL_TICK_S,
                 orphan_grace_s: float = ORPHAN_GRACE_S,
                 monotonic: Callable[[], float] = time.monotonic):
        if not slots:
            raise ValueError("the gateway needs at least one slot")
        if not operator_key:
            raise ValueError("the operator key is empty")
        self.slots = [Slot(n, u) for n, u in slots]
        self.by_name = {s.name: s for s in self.slots}
        if len(self.by_name) != len(self.slots):
            raise ValueError("slot names must be unique")
        self.policies = policies
        self._key = operator_key.encode()
        self.state_dir = Path(state_dir)
        self.ledger = Ledger(self.state_dir)
        self.state_file = self.state_dir / STATE_FILE
        self.idle_timeout_s = float(idle_timeout_s)
        self.episode_timeout_s = float(episode_timeout_s)
        self.load_timeout_s = float(load_timeout_s)
        self.load_timeouts = {str(k): float(v) for k, v in (load_timeouts or {}).items()}
        self.request_timeout_s = float(request_timeout_s)
        self.poll_s = float(poll_s)
        self.official_tick_s = float(official_tick_s)
        self.orphan_grace_s = float(orphan_grace_s)
        self.concurrent = frozenset(concurrent_task_sets)
        self.slot_task_sets = slot_task_sets
        self.slot_task_sets_path = slot_task_sets_path
        self._hosts_error: str | None = None
        self._tracer_factory = tracer_factory
        self.clock = clock
        self.mono = monotonic
        self.episodes: dict[str, Episode] = {}
        self.tracers: dict[str, Tracer] = {}
        self.catalogs: dict[str, tuple[dict, float]] = {}
        self.stopping = threading.Event()
        self._lock = threading.RLock()
        self._claim_lock = threading.Lock()
        self._grade_cv = threading.Condition(self._lock)
        self._threads: list[threading.Thread] = []
        self._saved_at = 0.0
        self._inflight: dict[str, dict[int, float]] = {}
        self._owed_since: dict[str, float] = {}
        self._rid = itertools.count()
        self._exec_inflight: dict[str, int] = {}
        self._turns: dict[str, threading.Condition] = {}
        self._publishing: set[str] = set()
        self._applying: set[str] = set()
        self._official_wake = threading.Event()
        self._graders = {s.name: ThreadPoolExecutor(max_workers=1,
                                                    thread_name_prefix=f"amp-gateway-grade-{s.name}")
                         for s in self.slots}
        self._gen = 0
        self._born: dict[str, int] = {}
        self._claim_keys: dict[str, str] = {}
        self._up_at = clock()
        self._load()

    def start(self) -> None:
        for s in self.slots:
            self._poll(s)
        for s in self.slots:
            t = threading.Thread(target=self._poll_loop, args=(s,), daemon=True,
                                 name=f"amp-gateway-poll-{s.name}")
            self._threads.append(t)
            t.start()
        t = threading.Thread(target=self._official_loop, daemon=True, name="amp-gateway-official")
        self._threads.append(t)
        t.start()

    def stop(self) -> None:
        self.stopping.set()
        self._official_wake.set()
        for pool in self._graders.values():
            pool.shutdown(wait=False, cancel_futures=True)
        for t in self._threads:
            t.join(timeout=CALL_TIMEOUT_S + 1)
        with self._lock:
            self._save()
            for tr in self.tracers.values():
                close_tracer(tr)

    def check_key(self, headers) -> None:
        got = (headers.get(KEY_HEADER) or "").encode()
        if not hmac.compare_digest(got, self._key):
            raise HTTPError(401, f"operator endpoints need the {KEY_HEADER} header")

    def tracer(self, ep: Episode) -> Tracer:
        with self._lock:
            tr = self.tracers.get(ep.episode_id)
            if tr is None:
                tr = NullTracer("gateway") if ep.over else self._tracer_factory(
                    self.state_dir, ep.episode_id)
                self.tracers[ep.episode_id] = tr
            return tr

    def policy_of(self, ep: Episode) -> HarnessPolicy:
        return self.policies.get(ep.harness)

    def _live_on(self, s: Slot) -> Episode | None:
        return next((e for e in self.episodes.values() if e.worker == s.name and not e.over), None)

    def _free(self, s: Slot) -> bool:
        return (s.excluded is None and s.failures == 0 and self._live_on(s) is None
                and not s.releasing and s.state.settled and not s.state.publishing)

    def _observe(self, s: Slot, raw, ok) -> None:
        """Under the lock: record one api/state answer."""
        change = s.observe(raw, ok)
        if change is not None:
            log.info("%s (%s) %s", s.name, s.url,
                     change if change == "serves agents" else f"is excluded: {change}")

    def _refresh_free(self) -> None:
        """Under _claim_lock: re-read every slot without a live episode."""
        for s in self.slots:
            with self._lock:
                live = self._live_on(s)
            if live is None:
                ok, raw = s.fetch_state()
                with self._lock:
                    self._observe(s, raw, ok)

    def _catalog_of(self, s: Slot, force: bool = False) -> dict | None:
        """Slot s's own catalog (GET api/tasks), cached CATALOG_MAX_AGE_S; None if it has none."""
        with self._lock:
            hit = self.catalogs.get(s.name)
        if not force and hit is not None and time.monotonic() - hit[1] < CATALOG_MAX_AGE_S:
            return hit[0]
        cat = None
        if s.excluded is None:
            try:
                status, body = s.request("GET", "/api/tasks", timeout=CALL_TIMEOUT_S)
            except (OSError, http.client.HTTPException):
                status, body = 0, None
            if status == 200 and isinstance(body, dict) and isinstance(body.get("tasks"), list):
                cat = body
        with self._lock:
            if cat is None:
                self.catalogs.pop(s.name, None)
            else:
                self.catalogs[s.name] = (cat, time.monotonic())
        return cat

    def claim(self, body: dict) -> dict:
        """A new episode of exactly (harness, task, episode) on a free slot (or the named one)."""
        agent, harness = body.get("agent"), body.get("harness")
        task, k, want = body.get("task"), body.get("episode"), body.get("slot")
        run_id = body.get("run_id") or ""
        task_set = body.get("task_set") or ""
        itype = body.get("instruction_type") or "default"
        cid = body.get("claim_id") or ""
        off = parse_claim(body["official"]) if body.get("official") is not None else None
        if itype not in INSTRUCTION_TYPES:
            raise ValueError(f"instruction_type must be one of {list(INSTRUCTION_TYPES)}, "
                             f"got {itype!r}")
        if not isinstance(agent, str) or not 1 <= len(agent.strip()) <= AGENT_MAX:
            raise ValueError(f"agent must be a string of 1 to {AGENT_MAX} characters")
        if not isinstance(harness, str) or not harness:
            raise ValueError("harness must be a harness name")
        if not isinstance(task, str) or not task:
            raise ValueError("task must be a task name")
        if isinstance(k, bool) or not isinstance(k, int) or k < 0:
            raise ValueError(f"episode must be an integer >= 0, got {k!r}")
        if want is not None and not isinstance(want, str):
            raise ValueError("slot must be a slot name")
        if run_id and not (isinstance(run_id, str) and RUN_ID.fullmatch(run_id)):
            raise ValueError("run_id must be 1 to 200 of [A-Za-z0-9_.-], not '.' or '..'")
        if task_set and not (isinstance(task_set, str) and RUN_ID.fullmatch(task_set)):
            raise ValueError("task_set must be a task set name")
        if cid and not (isinstance(cid, str) and CLAIM_ID.fullmatch(cid)):
            raise ValueError("claim_id must be 8 to 64 of [A-Za-z0-9_-]")
        if want is not None and want not in self.by_name:
            raise HTTPError(404, f"unknown slot {want!r}")
        try:
            hp = self.policies.get(harness)
        except KeyError:
            raise HTTPError(404, f"unknown harness {harness!r}") from None
        except ValueError as e:
            raise HTTPError(500, f"harness {harness!r} has a bad route policy: {e}") from None
        agent = agent.strip()
        if (again := self._replay(cid, harness, task, k, want, run_id, off)) is not None:
            return again
        with self._claim_lock:
            if (again := self._replay(cid, harness, task, k, want, run_id, off)) is not None:
                return again
            self._refresh_free()
            with self._lock:
                open_ = {s.name for s in self.slots if self._free(s)}
            cats = {}
            for s in self.slots:
                if want in (None, s.name):
                    cat = self._catalog_of(s, force=s.name in open_)
                    if cat is not None:
                        cats[s.name] = cat
            with self._lock:
                if not cats:
                    self._busy(f"{want} has no task catalog yet" if want
                               else "no slot has a task catalog yet")
                rows = {n: next((t for t in cat["tasks"]
                                 if isinstance(t, dict) and t.get("name") == task), None)
                        for n, cat in cats.items()}
                rows = {n: t for n, t in rows.items() if t is not None}
                if not rows:
                    where = f" on {want} (its library has {len(cats[want]['tasks'])} tasks)" \
                        if want else ""
                    raise HTTPError(404, f"unknown task {task!r}{where}")
                if itype != "default":
                    rows = {n: t for n, t in rows.items() if _instruction(t, itype)}
                    if not rows:
                        raise HTTPError(400, f"no worker's catalog has the {itype!r} instruction "
                                             f"of {task!r}: the task defines no such wording, or "
                                             "the worker's runtime predates instruction types")
                saved = not run_id and any(isinstance(e, dict) and e.get("episode") == k
                                           for t in rows.values()
                                           for e in t.get("episodes") or [])
                live = [e for e in self.episodes.values()
                        if not e.over and e.task == task and e.run_id == run_id]
                if saved or any(e.episode == k for e in live):
                    self._conflict(task, k, "saved" if saved else "live")
                hosts = self._hosts() if task_set in self.concurrent else {}
                if live and not (task_set in self.concurrent and
                                 all(e.task_set == task_set for e in live)):
                    self._busy(f"another episode of {task} is live")
                free = [s for s in self.slots if self._free(s) and want in (None, s.name)]
                if not free:
                    self._busy(f"{want} is taken" if want else "every worker is taken")
                free = [s for s in free if s.name in rows]
                if not free:
                    self._busy(f"no free worker serves {task}")
                if off is not None:
                    free = [s for s in free if GRADE_FEATURE in s.state.features]
                    if not free:
                        self._busy(f"no free worker serving {task} grades official blocks")
                if live:
                    free = [s for s in free if hosts.get(s.name) == task_set]
                    if not free:
                        self._busy(f"another episode of {task} is live")
            eid = secrets.token_hex(8)
            dropped = []
            for s in free:
                r = self._select(s, task, k, hp, run_id, eid, itype,
                                 off["attempt"] if off is not None else 0)
                if r == "dropped":
                    dropped.append(s.name)
                    continue
                if r == "taken":
                    self._conflict(task, k, "saved",
                                   self._saved_of(s, run_id, task, k) if run_id else None)
                if r == "claimed":
                    self._busy(f"{task} is claimed by another worker")
                if isinstance(r, int):
                    return self._new_episode(s, rows[s.name], r, agent, hp, k, run_id,
                                             eid, task_set if hosts.get(s.name) == task_set else "",
                                             itype, self.load_timeouts.get(task_set), off, cid)
            if dropped and len(dropped) == len(free):
                raise HTTPError(400, f"no robot server queued the {itype!r} instruction of "
                                     f"{task!r} ({', '.join(dropped)}): the robot server "
                                     "predates instruction types")
            self._busy("no free worker accepted the job")

    def _hosts(self) -> Mapping[str, str]:
        """The supervisor's slot -> task set map; {} (nothing exempt) when it cannot be read."""
        got, error = {}, None
        if self.slot_task_sets is None:
            if self.concurrent:
                error = "concurrent task sets configured but no slot task-set map"
        else:
            try:
                got = self.slot_task_sets()
            except (OSError, ValueError) as e:
                error = f"{type(e).__name__}: {e}"
            else:
                if not isinstance(got, Mapping):
                    got, error = {}, f"not an object: {type(got).__name__}"
        self._note_hosts(error)
        return got

    def _note_hosts(self, error: str | None) -> None:
        """Log the slot task-set map turning unreadable or readable again, once per change."""
        if (error is None) == (self._hosts_error is None):
            return
        self._hosts_error = error
        where = f" {self.slot_task_sets_path}" if self.slot_task_sets_path else ""
        if error is not None:
            log.warning("slot task-set map%s unreadable (%s): no claim is exempt", where, error)
        else:
            log.info("slot task-set map%s readable again", where)

    def _conflict(self, task, k, why, saved: dict | None = None):
        extra = {"conflict": True, **({"saved": saved} if saved is not None else {})}
        raise HTTPError(409, f"{task} episode {k} is already {why}", extra)

    def _saved_of(self, s: Slot, run_id: str, task: str, k: int) -> dict:
        """Outside the lock: what slot s saved of a scoped run's episode, its row and its record."""
        rows = [r for r in self.ledger.rows() if r.get("worker") == s.name
                and r.get("run_id") == run_id and r.get("task") == task and r.get("episode") == k]
        row = next((r for r in reversed(rows) if r.get("outcome") == "saved"),
                   rows[-1] if rows else None)
        return {"row": row, "record": self._record_of(s, run_id, task, k)}

    def _busy(self, message):
        raise HTTPError(503, message, {"busy": True, "retry_after_s": RETRY_AFTER_S})

    def _select(self, s: Slot, task: str, k: int, hp: HarnessPolicy, run_id: str, eid: str,
                itype: str = "default", official_attempt: int = 0):
        """The slot's job_id for the new episode, or why not: "claimed" (another slot has the
        task), "taken" (that episode index is saved), "next" (try the next slot), "dropped"
        (its robot server queued the default wording, not itype)."""
        body = {"task": task, "mode": "new", "episode": k, "info_mode": hp.server.info_mode,
                "obstacles": hp.server.obstacles}
        if run_id:
            body["run_id"] = run_id
        if itype != "default":
            body["instruction_type"] = itype
        if official_attempt:
            body["official_attempt"] = official_attempt
        try:
            status, reply = s.request("POST", "/api/tasks/select", body, timeout=CALL_TIMEOUT_S,
                                      headers={TRACE_HEADER: f"{eid}/"})
        except (OSError, http.client.HTTPException) as e:
            log.warning("%s: select %s failed: %s; releasing it", s.name, task, e)
            if not self._discard(s)[0]:
                with self._lock:
                    s.release_owed = True
                    self._save()
            return "next"
        reply = reply if isinstance(reply, dict) else {}
        if status == 200 and isinstance(reply.get("job_id"), int):
            if itype != "default" and reply.get("instruction_type") != itype:
                log.warning("%s: select %s queued no %r instruction; releasing it", s.name, task,
                            itype)
                if not self._discard(s)[0]:
                    with self._lock:
                        s.orphan_job = reply["job_id"]
                        self._save()
                return "dropped"
            return reply["job_id"]
        log.warning("%s: select %s answered %s: %s", s.name, task, status, reply.get("message"))
        if status == 409 and reply.get("claimed"):
            return "claimed"
        if status == 409 and reply.get("episode_taken"):
            return "taken"
        return "next"

    def _discard(self, s: Slot, idle_ok: bool = False,
                 force: bool = False) -> tuple[bool, int | None, str | None]:
        """Release the slot's job: (done, status, message); a refusal logs with backoff."""
        body = {"discard": True, **({"force": True} if force else {})}
        status, reply = self._call(s, "/api/episode/release", body, quiet=True)
        msg = reply.get("message") if isinstance(reply, dict) else None
        done = status == 200 or (idle_ok and status == 409 and "no episode is running" in
                                 (msg or ""))
        with self._lock:
            if done:
                s.release_refusals = 0
                return True, status, msg
            s.release_refusals += 1
            n = s.release_refusals
            if n & (n - 1) == 0 or n % REFUSAL_LOG_EVERY == 0:
                log.warning("%s: releasing its job answered %s: %s (refusal %d; retrying)",
                            s.name, status, msg, n)
        return False, status, msg

    def _new_episode(self, s: Slot, t: dict, job_id: int, agent: str, hp: HarnessPolicy,
                     k: int, run_id: str, eid: str, task_set: str = "",
                     itype: str = "default", load_timeout_s: float | None = None,
                     off: dict | None = None, cid: str = "") -> dict:
        now = self.clock()
        key = secrets.token_urlsafe(24) if off is not None else None
        ep = Episode(token=secrets.token_urlsafe(18), episode_id=eid,
                     agent=agent, harness=hp.name, worker=s.name, task=t["name"],
                     instruction=_instruction(t, itype), claimed_at=now,
                     info_mode=hp.server.info_mode, run_id=run_id,
                     task_set=task_set, instruction_type=itype,
                     load_timeout_s=load_timeout_s if load_timeout_s is not None
                     and load_timeout_s > self.load_timeout_s else None,
                     job_id=job_id, episode=k, claim_id=cid,
                     last_active=now,
                     official=None if key is None else Official(**off, key_sha256=key_digest(key)))
        with self._lock:
            s.orphan_job, s.release_owed, s.release_refusals = None, False, 0
            self.episodes[ep.token] = ep
            if key is not None and cid:
                self._claim_keys[ep.token] = key
            self._gen += 1
            self._born[ep.token] = self._gen
            self._save()
        emit(self.tracer(ep), "gateway.claim", {
            "agent": agent, "harness": hp.name, "policy": hp.policy.name, "task": ep.task,
            "episode": k, "slot": s.name, "job_id": job_id, "info_mode": ep.info_mode,
            "code_ref": ep.code_ref, "run_id": run_id, "task_set": task_set,
            "routes": str(hp.source), **({"official": off} if off is not None else {})})
        log.info("%s: episode %s of %s/%s for %r (job %s)", s.name, ep.episode_id, hp.name,
                 ep.task, agent, job_id)
        return self._claim_reply(ep, key)

    def _claim_reply(self, ep: Episode, key: str | None) -> dict:
        return {"ok": True, "token": ep.token, "episode_id": ep.episode_id,
                "base": f"s/{ep.token}/", "state": ep.state, "instruction": ep.instruction,
                "instruction_type": ep.instruction_type, "task": ep.task, "episode": ep.episode,
                "harness": ep.harness, "slot": ep.worker,
                **({"official_key": key} if key is not None else {})}

    def _replay(self, cid: str, harness: str, task: str, k: int, want: str | None, run_id: str,
                off: dict | None) -> dict | None:
        """The live episode an earlier claim with this claim_id made, whose reply was lost."""
        if not cid:
            return None
        with self._lock:
            ep = next((e for e in self.episodes.values() if not e.over and e.claim_id == cid), None)
            if ep is None or ep.end is not None or ep.released or ep.counters.moved \
                    or (ep.harness, ep.task, ep.episode, ep.run_id) != (harness, task, k, run_id) \
                    or want not in (None, ep.worker) or (off is None) != (ep.official is None):
                return None
            key = self._claim_keys.get(ep.token)
            if ep.official is not None and (key is None or ep.official.started_at is not None
                                            or ep.official.attempt != (off or {}).get("attempt")):
                return None
            self._seen(ep)
            reply = self._claim_reply(ep, key)
        emit(self.tracer(ep), "gateway.claim.replay", {"slot": ep.worker, "state": ep.state},
             lvl="warn")
        log.warning("%s: claim of %s episode %s retried; replaying episode %s", ep.worker, task,
                    k, ep.episode_id)
        return reply

    def lookup(self, token: str) -> Episode | None:
        with self._lock:
            return self.episodes.get(token)

    def _seen(self, ep: Episode) -> None:
        """Under the lock: its runner is alive, so the orphan grace never ends it."""
        if not ep.seen:
            ep.seen = True
            self._save()

    def episode_json(self, ep: Episode) -> dict:
        now = self.clock()
        with self._lock:
            ready = ep.ready_at
            stop = ep.end_at if ep.end_at is not None else now
            waiting = ep.end_at is None and self._waiting(ep)
            idle = max(0.0, stop - ep.last_active) if ready is not None and not waiting else 0.0
            self._seen(ep)
            return {"ok": True, "episode_id": ep.episode_id, "state": ep.state,
                    "instruction": ep.instruction, "idle_s": round(idle, 1),
                    "idle_timeout_s": self.idle_timeout_s,
                    "wall_s": round(max(0.0, stop - ready), 1) if ready is not None else 0.0,
                    "episode_timeout_s": self.episode_timeout_s, "end": ep.end}

    def finish(self, ep: Episode) -> dict:
        """The agent is done: publish (or, while loading, publish as soon as it is ready)."""
        self._official_apply_deadline(ep)
        with self._lock:
            self._seen(ep)
            if ep.end is None and not ep.over:
                if ep.official is not None:
                    ep.official.finish_in_cell(self.mono())
                if ep.ready_at is None:
                    ep.finish_requested = True
                else:
                    self._decide(ep, "finished")
                self._save()
        emit(self.tracer(ep), "gateway.finish", {"state": ep.state})
        self._kick(ep)
        return {"ok": True}

    def release(self, body: dict) -> dict:
        """The operator ends the episode: discarded if the robot never moved (or on discard), else
        published."""
        token = body.get("token")
        abandon = body.get("discard", False)
        if not isinstance(abandon, bool):
            raise TypeError("discard must be true or false")
        ep = self.lookup(token) if isinstance(token, str) else None
        if ep is None:
            raise HTTPError(404, "no such episode")
        self._official_apply_deadline(ep)
        decided = False
        with self._lock:
            self._seen(ep)
            if ep.end is None and not ep.over:
                ep.released = True
                ep.discard = abandon or ep.ready_at is None or not ep.counters.moved
                ep.abandoned = abandon
                self._decide(ep, "ended")
                self._save()
                decided = True
            elif abandon and not ep.over and not ep.discard:
                ep.discard = ep.abandoned = True
                self._save()
                decided = True
        if decided:
            tr = self.tracer(ep)
            c = ep.counters
            emit(tr, "gateway.release", {"discarded": ep.discard, "executes": c.executes,
                                         "executes_inflight": c.executes_inflight,
                                         "ready": ep.ready_at is not None})
            if ep.discard:
                emit(tr, "gateway.discard", {"why": "abandoned" if ep.abandoned
                                             else "not ready" if ep.ready_at is None
                                             else "the robot never moved"})
        self._kick(ep)
        return {"ok": True, "episode_id": ep.episode_id, "state": ep.state, "end": ep.end,
                "discarded": ep.discard}

    def official_start(self, body: dict) -> dict:
        """The runner started the agent: the official trial clock starts now (once)."""
        token = body.get("token")
        ep = self.lookup(token) if isinstance(token, str) else None
        if ep is None:
            raise HTTPError(404, "no such episode")
        if body.get("op") != "start":
            raise ValueError("op must be \"start\"")
        with self._lock:
            o = ep.official
            if o is None:
                raise HTTPError(409, "not an official episode")
            if ep.end is not None or ep.over:
                raise HTTPError(410, "the episode is over", {"end": ep.end})
            if ep.ready_at is None:
                raise HTTPError(409, "the episode is still loading", {"state": "loading"})
            now = self.mono()
            o.start(now, self.clock())
            ep.seen = True
            self._claim_keys.pop(ep.token, None)
            self._save()
            left = o.deadline - now if o.deadline is not None else 0.0
        self._official_wake.set()
        emit(self.tracer(ep), "gateway.official.start", {"trial_timeout_s": o.trial_timeout_s,
                                                          "attempt": o.attempt,
                                                          "max_attempts": o.max_attempts})
        return {"ok": True, "deadline_in_s": round(left, 3)}

    def check_official_key(self, ep: Episode, headers) -> Official:
        o = ep.official
        if o is None:
            raise HTTPError(404, "not an official episode")
        if not o.key_ok(headers.get(OFFICIAL_KEY_HEADER) or ""):
            raise HTTPError(401, f"official routes need the {OFFICIAL_KEY_HEADER} header")
        return o

    def official_cell(self, ep: Episode, body: dict) -> dict:
        """The agent loop's cell boundary: {"op": "begin"} or {"op": "end", "cell", "interrupted"}."""
        op = body.get("op")
        if op not in ("begin", "end"):
            raise ValueError("op must be \"begin\" or \"end\"")
        n, interrupted = body.get("cell"), body.get("interrupted", False)
        if op == "end" and (isinstance(n, bool) or not isinstance(n, int)
                            or not isinstance(interrupted, bool)):
            raise ValueError("end needs an integer cell and a boolean interrupted")
        s = self.by_name[ep.worker]
        if op == "begin":
            return self._official_begin(ep, s)
        self._official_apply_deadline(ep)
        motion = self._sample_motion(s)
        with self._lock:
            o = ep.official
            self._official_live(ep)
            now = self.mono()
            motion = motion or self._exec_inflight.get(ep.token, 0) > 0
            acts = o.at_deadline(now, motion)
            ended = any(a[0] == PATH2 for a in acts)
            if not ended:
                more = o.end(now, n, interrupted, motion)
                acts += more
                ended = any(a[0] == PATH2 for a in more)
            if ended:
                self._official_outside(ep, "clock")
            self._active(ep)
            self._save()
        emit(self.tracer(ep), "gateway.official.cell", {
            "op": op, "cell": o.cell, "interrupted": interrupted, "motion": motion,
            "fired": o.fired, "pending": o.pending})
        self._official_do(s, ep, acts)
        if ended:
            self._kick(ep)
        return {"ok": True}

    def _official_live(self, ep: Episode) -> None:
        """Under the lock: 409 unless the episode is ready and not over (the loop stops on it)."""
        if ep.end is not None or ep.over:
            raise HTTPError(409, "the episode is over", {"end": ep.end, **self._fired(ep)})
        if ep.ready_at is None:
            raise HTTPError(409, "the episode is still loading", {"state": "loading"})

    def _official_begin(self, ep: Episode, s: Slot) -> dict:
        """A cell begin: end the previous block, wait out its grade (the barrier), admit the cell."""
        o = ep.official
        self._official_apply_deadline(ep)
        motion = self._sample_motion(s)
        with self._lock:
            self._official_live(ep)
            now = self.mono()
            motion = motion or self._exec_inflight.get(ep.token, 0) > 0
            acts = o.at_deadline(now, motion)
            ended = any(a[0] == PATH2 for a in acts)
            if not ended:
                more = o.begin_close(now, motion)
                acts += more
                ended = any(a[0] == PATH2 for a in more)
            if ended:
                self._official_outside(ep, "clock")
            self._active(ep)
            self._save()
        self._official_do(s, ep, acts, sync=False)
        if ended:
            self._kick(ep)
            raise HTTPError(409, "the trial reached its time limit", {"end": ep.end,
                                                                     **self._fired(ep)})
        waited = self._await_grades(ep)
        if waited:
            motion = self._sample_motion(s)
        with self._lock:
            self._official_live(ep)
            now = self.mono()
            motion = motion or self._exec_inflight.get(ep.token, 0) > 0
            acts = o.at_deadline(now, motion)
            ended = any(a[0] == PATH2 for a in acts)
            if ended:
                self._official_outside(ep, "clock")
            else:
                reply = o.begin_open()
            self._active(ep)
            self._save()
            if not ended:
                reply["interrupt_in_s"] = o.interrupt_in(self.mono())
        emit(self.tracer(ep), "gateway.official.cell", {
            "op": "begin", "cell": o.cell, "interrupted": None, "motion": motion,
            "fired": o.fired, "pending": o.pending, "waited_for_grade": waited})
        self._official_do(s, ep, acts, sync=False)
        if ended:
            self._kick(ep)
            raise HTTPError(409, "the trial reached its time limit", {"end": ep.end,
                                                                     **self._fired(ep)})
        return reply

    def _await_grades(self, ep: Episode) -> bool:
        """Block (outside the lock) until the episode's pending grades are done or it ends."""
        o = ep.official
        waited, until = False, time.monotonic() + GRADE_BARRIER_S
        with self._grade_cv:
            while o.grading and ep.end is None and not ep.over and not self.stopping.is_set():
                left = until - time.monotonic()
                if left <= 0:
                    for n in list(o.grading):
                        o.distrust(self.mono(), n)
                    self._wake(ep)
                    self._save()
                    break
                waited = True
                self._grade_cv.wait(min(left, 1.0))
        return waited

    def official_model_timeout(self, ep: Episode) -> dict:
        """A model call surfaced a timeout: path 2 whenever the episode is live (H)."""
        self._official_apply_deadline(ep)
        motion = self._sample_motion(self.by_name[ep.worker])
        acts = []
        with self._lock:
            if ep.end is None and not ep.over and ep.ready_at is not None:
                idle = not (motion or self._exec_inflight.get(ep.token, 0) > 0)
                self._official_outside(ep, "model_timeout", idle)
                acts = [(STOP,)]
                self._save()
        self._official_do(self.by_name[ep.worker], ep, acts)
        if acts:
            self._kick(ep)
        return {"ok": True, "end": ep.end}

    def _fired(self, ep: Episode) -> dict:
        o = ep.official
        return {"official_timeout": o.fired} if o is not None and o.fired == OUTSIDE_BLOCK \
            and ep.end == "official_timeout" else {}

    def _sample_motion(self, s: Slot) -> bool:
        """Motion in flight now, read from the slot; its last poll if it does not answer in time."""
        try:
            status, raw = s.request("GET", "/api/state", timeout=SAMPLE_TIMEOUT_S)
        except (OSError, http.client.HTTPException):
            return s.state.motion
        return SlotState(raw).motion if status == 200 and isinstance(raw, dict) \
            else s.state.motion

    def _official_loop(self) -> None:
        """Decide each official deadline when it passes, whatever the poll threads are doing."""
        while not self.stopping.is_set():
            now, due, soonest = self.mono(), [], None
            with self._lock:
                for e in self.episodes.values():
                    o = e.official
                    if o is None or e.over or e.end is not None or o.deadline is None \
                            or o.handled or o.fired is not None or e.token in self._applying:
                        continue
                    if o.due(now):
                        due.append(e)
                        self._applying.add(e.token)
                    else:
                        soonest = min(soonest or 1e9, o.deadline - now)
            for e in due:
                threading.Thread(target=self._official_apply_thread, args=(e,), daemon=True,
                                 name="amp-gateway-deadline").start()
            if soonest is None and not due and not self._applying:
                wait = 1.0
            else:
                wait = min(self.official_tick_s, max(soonest if soonest is not None
                                                     else self.official_tick_s, 0.001))
            self._official_wake.wait(wait)
            self._official_wake.clear()

    def _official_apply_thread(self, ep: Episode) -> None:
        try:
            self._official_apply_deadline(ep)
        except Exception:  # noqa: BLE001
            traceback.print_exc()
        finally:
            with self._lock:
                self._applying.discard(ep.token)

    def _official_apply_deadline(self, ep: Episode) -> None:
        """Decide the deadline now if it has passed and no one has: the slot is read first, outside
        the lock, so the decision under it never waits on HTTP."""
        o = ep.official
        if o is None:
            return
        with self._lock:
            if ep.end is not None or ep.over or not o.due(self.mono()):
                return
        s = self.by_name[ep.worker]
        motion = self._sample_motion(s)
        with self._lock:
            now = self.mono()
            if ep.end is not None or ep.over or not o.due(now):
                return
            motion = motion or self._exec_inflight.get(ep.token, 0) > 0
            acts = o.at_deadline(now, motion)
            ended = any(a[0] == PATH2 for a in acts)
            if ended:
                self._official_outside(ep, "clock")
            self._save()
        emit(self.tracer(ep), "gateway.official.deadline", {
            "motion": motion, "phase": o.phase, "path": o.fired or ("pending" if o.pending
                                                                     else None),
            "late_s": round(now - (o.deadline or now), 3)})
        self._official_do(s, ep, acts, sync=False)
        if ended:
            self._kick(ep)

    def _official_outside(self, ep: Episode, cause: str, idle: bool = True) -> None:
        """Under the lock: path 2. A moved episode is discarded before the last attempt, or void."""
        o = ep.official
        o.fire_outside(self.mono(), cause, idle)
        if o.force:
            ep.discard = True
        if o.void:
            ep.error = o.void_reason
        self._decide(ep, "official_timeout")

    def _official_do(self, s: Slot, ep: Episode, acts: list, sync: bool = True) -> None:
        """Run the clock's slot calls outside the lock: stops here (or on a thread), grades async."""
        for act in acts:
            if act[0] == STOP:
                if sync:
                    self._official_stop(s, ep)
                else:
                    threading.Thread(target=self._official_stop, args=(s, ep), daemon=True,
                                     name="amp-gateway-stop").start()
            elif act[0] == GRADE:
                try:
                    self._graders[s.name].submit(self._official_grade, s, ep, act[1])
                except RuntimeError:
                    return

    def _official_stop(self, s: Slot, ep: Episode) -> None:
        status, _ = self._call(s, "/api/stop", {})
        emit(self.tracer(ep), "gateway.official.stop", {"status": status})

    def _official_grade(self, s: Slot, ep: Episode, n: int) -> None:
        """Grade block n: one retry if it failed for an infra cause and the episode is live."""
        o = ep.official
        for first in (True, False):
            status, reply = self._call(s, "/api/episode/grade", {"cell": n}, quiet=True)
            done = self.mono()
            ok = status == 200 and isinstance(reply, dict) and reply.get("ok") is True
            retry = False
            with self._lock:
                if ep.over:
                    return
                if ok:
                    o.graded(done, n)
                    o.grade_done(n)
                else:
                    infra = grade_failure_is_infra(status, reply)
                    o.grade_failed(done, n, infra)
                    retry = first and infra and ep.end is None
                    if not retry:
                        o.grade_done(n)
                self._grade_cv.notify_all()
                self._wake(ep)
                completed = o.completed
                self._save()
            emit(self.tracer(ep), "gateway.official.grade",
                 {"cell": n, "status": status, "completed": completed, "try": 1 if first else 2},
                 lvl="info" if ok else "warn")
            if not retry:
                return

    def _official_regrade(self, s: Slot, ep: Episode, n: int) -> bool:
        """Attempt 3, path 2: grade the current state once more as the last cell's end state."""
        status, reply = self._call(s, "/api/episode/grade", {"cell": n}, quiet=True)
        ok = status == 200 and isinstance(reply, dict) and reply.get("ok") is True
        emit(self.tracer(ep), "gateway.official.regrade", {"cell": n, "status": status, "ok": ok},
             lvl="info" if ok else "warn")
        return ok

    def blocked(self, ep: Episode, label: str) -> None:
        with self._lock:
            ep.counters.block(label[:200])

    def begin(self, ep: Episode, key: str) -> bool:
        """Count a forwarded request; True if it is an execute under the episode's policy."""
        executes = key in self.policy_of(ep).policy.executes
        with self._lock:
            o = ep.official
            if executes and o is not None:
                if o.refuses_motion():
                    ep.counters.block(f"POST {key} (official, outside a cell)"[:200])
                    raise MotionRefused
                self._exec_inflight[ep.token] = self._exec_inflight.get(ep.token, 0) + 1
            ep.counters.begin(key, executes)
            if executes:
                self._save()
            self._active(ep)
        return executes

    def done(self, ep: Episode, key: str, status: int, filtered: bool, executes: bool) -> None:
        with self._lock:
            ep.counters.done(key, status, filtered, executes)
            self._active(ep)

    def started(self, ep: Episode, key: str | None = None, executes: bool = False) -> int:
        """Register a request to forward; an official one waits its turn first, or is refused."""
        with self._lock:
            o = ep.official
            if o is not None:
                turn = self._turns.get(ep.token)
                if turn is None:
                    turn = self._turns[ep.token] = threading.Condition(self._lock)
                while (o.grading or self._inflight.get(ep.token)) and ep.end is None \
                        and not ep.over and not self.stopping.is_set():
                    turn.wait(1.0)
                refusal = None
                if ep.end is not None or ep.over:
                    refusal = HTTPError(410, "the episode is over", {"end": ep.end})
                    if ep.over:
                        self._turns.pop(ep.token, None)
                elif self.stopping.is_set():
                    refusal = HTTPError(503, "the gateway is stopping")
                elif executes and o.refuses_motion():
                    ep.counters.block(f"POST {key} (official, outside a cell)"[:200])
                    refusal = MotionRefused()
                if refusal is not None:
                    left = ep.counters.requests.pop(key, 1) - 1
                    if left:
                        ep.counters.requests[key] = left
                    ep.counters.done(key, refusal.status, False, executes)
                    raise refusal
            rid, now = next(self._rid), self.clock()
            self._inflight.setdefault(ep.token, {})[rid] = now
            self._owed_since.setdefault(ep.token, now)
            return rid

    def stopped(self, ep: Episode, rid: int | None, replied: bool, executes: bool = False) -> None:
        """The request is over; a reply, whatever its status, shows the sim still answers."""
        with self._lock:
            if executes and ep.official is not None:
                left = self._exec_inflight.get(ep.token, 0) - 1
                if left > 0:
                    self._exec_inflight[ep.token] = left
                else:
                    self._exec_inflight.pop(ep.token, None)
            self._inflight.get(ep.token, {}).pop(rid, None)
            self._wake(ep)
            if replied:
                self._owed_since.pop(ep.token, None)
            self._active(ep)

    def _wake(self, ep: Episode, every: bool = False) -> None:
        """Under the lock: the episode's next queued agent request may go (every: all re-check)."""
        turn = self._turns.get(ep.token)
        if turn is not None and every:
            turn.notify_all()
        elif turn is not None:
            turn.notify()

    def _waiting(self, ep: Episode) -> bool:
        return bool(self._inflight.get(ep.token))

    def _sim_hung(self, ep: Episode, now: float) -> bool:
        """A request waits on a sim that has answered none for a whole request timeout."""
        since = self._owed_since.get(ep.token)
        return since is not None and self._waiting(ep) and now - since >= self.request_timeout_s

    def _active(self, ep: Episode) -> None:
        if ep.end is None:
            ep.last_active = self.clock()
        if time.monotonic() - self._saved_at >= ACTIVITY_SAVE_S:
            self._save()

    def status(self) -> dict:
        with self._lock:
            out = {"ok": True, "directed": True, "harnesses": self.policies.loaded(),
                   "workers": [self._slot_json(s) for s in self.slots],
                   "live": [self._live_json(e) for e in self.episodes.values() if not e.over],
                   "episodes": self.ledger.rows()}
            if self.concurrent:
                self._hosts()
                out["slot_task_sets"] = {"path": self.slot_task_sets_path,
                                         "error": self._hosts_error}
            return out

    def episode_status(self, episode_id: str) -> dict:
        """One episode by id: its live row, or its ledger row once it is over."""
        with self._lock:
            ep = next((e for e in self.episodes.values() if e.episode_id == episode_id), None)
            if ep is not None and not ep.over:
                self._seen(ep)
                return {"ok": True, "over": False, "live": self._live_json(ep)}
        row = self.ledger.find(episode_id)
        if row is None:
            raise HTTPError(404, "no such episode")
        return {"ok": True, "over": True, "row": row}

    def _slot_json(self, s: Slot) -> dict:
        ep = self._live_on(s)
        if s.unreachable or s.excluded is not None:
            state = "down"
        elif ep is not None:
            state = "loading" if ep.state == "loading" else "busy"
        else:
            state = "free" if self._free(s) else "busy"
        raw = s.state.raw
        out = {"name": s.name, "url": s.url, "state": state,
               "task": ep.task if ep else raw.get("task"),
               "episode": ep.episode if ep else raw.get("episode"),
               "episode_id": ep.episode_id if ep else None}
        if s.excluded is not None:
            out["excluded"] = s.excluded
        return out

    def _live_json(self, ep: Episode) -> dict:
        c = ep.counters
        return {"episode_id": ep.episode_id, "harness": ep.harness, "code_ref": ep.code_ref,
                "run_id": ep.run_id,
                "task": ep.task,
                "episode": ep.episode, "agent": ep.agent, "worker": ep.worker,
                "state": ep.state, "end": ep.end, "claimed_at": iso(ep.claimed_at),
                "ready_at": iso(ep.ready_at), "executes": c.executes,
                "requests": dict(c.requests), "filtered": dict(c.filtered),
                "inflight": len(self._inflight.get(ep.token) or ()),
                "sim_hung": self._sim_hung(ep, self.clock()),
                **({"official": official_row(ep.official)} if ep.official is not None else {})}

    def _decide(self, ep: Episode, end: str) -> None:
        """Under the lock: the episode ends now (publishing it, or aborting it)."""
        ep.end, ep.end_at = end, self.clock()
        with self._grade_cv:
            self._grade_cv.notify_all()
            self._wake(ep, every=True)
        if end == "timeout":
            ep.sim_hung = self._sim_hung(ep, ep.end_at)
        log.info("%s: episode %s of %s ends: %s", ep.worker, ep.episode_id, ep.task, end)
        emit(self.tracer(ep), "gateway.end", {"end": end, "error": ep.error})

    def _kick(self, ep: Episode) -> None:
        """Act on a decided end at once rather than at the next poll."""
        s = self.by_name.get(ep.worker)
        if s is not None:
            self._act(s, ep)

    def _poll_loop(self, s: Slot) -> None:
        while not self.stopping.wait(self.poll_s):
            try:
                self._poll(s)
            except Exception:  # noqa: BLE001
                traceback.print_exc()

    def _poll(self, s: Slot) -> None:
        gen = self._gen
        ok, raw = s.fetch_state()
        now = self.clock()
        orphan = False
        with self._lock:
            ep = self._live_on(s)
            if ep is not None and self._born.get(ep.token, 0) > gen:
                return
            self._observe(s, raw, ok)
            if ep is None:
                held = {s.state.job_id, s.state.pending_id} - {None}
                orphan = ok and (s.orphan_job in held or (s.release_owed and bool(held)))
                if ok and not orphan:
                    s.release_refusals = 0
            elif not ok:
                if s.failures >= DOWN_AFTER_FAILURES:
                    ep.error = "the worker stopped answering"
                    s.orphan_job = ep.job_id
                    self._over(ep, "error", None)
                return
        if orphan:
            with self._claim_lock, self._lock:
                releasing = s.releasing = self._live_on(s) is None
            if releasing:
                released = False
                try:
                    if not s.release_refusals:
                        log.warning("%s: releasing the episode a failure left behind", s.name)
                    released = self._discard(s)[0]
                finally:
                    with self._lock:
                        s.releasing = False
                        if released:
                            s.orphan_job, s.release_owed = None, False
                            self._save()
        if ep is not None:
            self._advance(s, ep, s.state, now)

    def _advance(self, s: Slot, ep: Episode, st: SlotState, now: float) -> None:
        last = st.last_end
        acts: list = []
        self._official_apply_deadline(ep)
        with self._lock:
            if ep.over or (ep.official is not None and ep.token in self._publishing):
                return
            if s.excluded is not None:
                ep.error = f"the worker is excluded: {s.excluded}"
                s.orphan_job = ep.job_id
                self._over(ep, "error", None)
                return
            outcome = end = None
            if last.get("job_id") == ep.job_id and last.get("task") == ep.task:
                if isinstance(last.get("episode"), int):
                    ep.episode = last["episode"]
                outcome = last.get("outcome")
                end = ep.end
                if end is None:
                    end = "ended" if outcome == "saved" and ep.ready_at is not None else "error"
                if outcome == "failed":
                    ep.error = last.get("error") or "the sim failed the episode"
                    if last.get("void") is True:
                        outcome = "void"
            elif ep.end is not None and (ep.ready_at is None or ep.abort_sent) and \
                    st.settled and st.job_id != ep.job_id:
                outcome, end = "cancelled", ep.end
            elif st.job_id != ep.job_id and (ep.ready_at is not None
                                             or st.pending_id != ep.job_id):
                ep.error = "the worker no longer has the episode's job"
                s.orphan_job = ep.job_id
                outcome, end = "lost", "error"
            elif ep.ready_at is None:
                if st.ready_for(ep.job_id, ep.task):
                    self._ready(s, ep, st, now)
                elif ep.end is None and now - ep.claimed_at >= (limit := ep.load_timeout_s
                                                                 or self.load_timeout_s):
                    ep.error = f"not ready after {limit:.0f} s"
                    self._decide(ep, "error")
            elif ep.end is None:
                if ep.official is not None:
                    acts = ep.official.settle(
                        self.mono(), st.motion or self._exec_inflight.get(ep.token, 0) > 0)
                if st.moving or self._waiting(ep):
                    ep.last_active = now
                if self._orphaned(ep, now):
                    ep.error = (f"no runner polled or started it within "
                                f"{self.orphan_grace_s:.0f} s of ready")
                    ep.discard = True
                    emit(self.tracer(ep), "gateway.orphan", {"grace_s": self.orphan_grace_s},
                         lvl="warn")
                    self._decide(ep, "error")
                elif ep.end is None and now - ep.last_active >= self.idle_timeout_s:
                    self._decide(ep, "idle")
                elif ep.end is None and now - ep.ready_at >= self.episode_timeout_s:
                    self._decide(ep, "timeout")
        if ep.official is not None and ep.official.void and outcome not in (None, "saved"):
            outcome = "void"
        if outcome is not None:
            record = self._run_record(s, ep) if outcome == "saved" else None
            with self._lock:
                self._over(ep, end, record, outcome)
            return
        self._official_do(s, ep, acts)
        self._act(s, ep)

    def _orphaned(self, ep: Episode, now: float) -> bool:
        """Under the lock: ready past the grace with no sign of a runner or an agent."""
        c = ep.counters
        return not ep.seen and ep.end is None and ep.ready_at is not None \
            and now - max(ep.ready_at, self._up_at) >= self.orphan_grace_s \
            and not c.moved and not c.requests \
            and not self._inflight.get(ep.token)

    def _ready(self, s: Slot, ep: Episode, st: SlotState, now: float) -> None:
        """Under the lock: the slot has loaded the episode; refuse it in the wrong info mode."""
        if st.info_mode != ep.info_mode:
            ep.error = f"the slot is in info mode {st.info_mode!r}, the harness needs " \
                       f"{ep.info_mode!r}"
            ep.discard = True
            self._decide(ep, "error")
            return
        ep.ready_at = ep.last_active = now
        ep.episode = (st.job or {}).get("episode", ep.episode)
        log.info("%s: episode %s ready after %.2f s", s.name, ep.episode_id, now - ep.claimed_at)
        emit(self.tracer(ep), "gateway.ready", {"load_s": round(now - ep.claimed_at, 3),
                                                "episode": ep.episode})
        if ep.finish_requested and ep.end is None:
            self._decide(ep, "finished")
        self._save()

    def _act(self, s: Slot, ep: Episode) -> None:
        """Send the publish (or the discarding release) a decided end needs, once."""
        with self._lock:
            if ep.over or ep.end is None:
                return
            o = ep.official
            regrade = None
            if ep.ready_at is not None and not ep.discard and not ep.publish_sent:
                path, body = "/api/episode/publish", {}
                if o is not None and o.fired == OUTSIDE_BLOCK:
                    body = {"verdict": o.verdict()}
                    regrade = o.rescue
                ep.publish_sent = True
                if o is not None:
                    self._publishing.add(ep.token)
            elif (ep.ready_at is None or ep.discard) and not ep.abort_sent:
                path, body = "/api/episode/release", {"discard": True}
                ep.abort_sent = True
            else:
                return
            force = (o is not None and o.force) or ep.abandoned
        if regrade is not None:
            ok = self._official_regrade(s, ep, regrade)
            with self._lock:
                o.regrade_done(ok)
                if ok:
                    body = {"verdict": o.verdict()}
                else:
                    ep.discard, ep.error, ep.publish_sent = True, o.void_reason, False
                    self._publishing.discard(ep.token)
                self._save()
            if not ok:
                return self._act(s, ep)
        kind = "gateway.publish" if path.endswith("publish") else "gateway.abort"
        emit(self.tracer(ep), kind, {"end": ep.end, "discard": ep.discard})
        if path.endswith("release"):
            done, status, msg = self._discard(s, idle_ok=True, force=force)
            if done:
                return
        else:
            try:
                status, reply = self._call(s, path, body)
                msg = reply.get("message") if isinstance(reply, dict) else None
                if status == 200 and "verdict" in body and not (isinstance(reply, dict)
                                                                 and "verdict" in reply):
                    with self._lock:
                        ep.official.verdict_ignored = True
                        self._save()
                    emit(self.tracer(ep), "gateway.official.verdict_ignored", {}, lvl="warn")
            finally:
                with self._lock:
                    self._publishing.discard(ep.token)
            if status == 200 or (status == 409 and "no episode is running" in (msg or "")):
                return
            log.warning("%s: %s answered %s: %s; retrying at the next poll", s.name, path,
                        status, msg)
        emit(self.tracer(ep), kind + ".retry", {"status": status, "message": msg}, lvl="warn")
        with self._lock:
            if path.endswith("publish"):
                ep.publish_sent = False
            else:
                ep.abort_sent = False

    def _call(self, s: Slot, path: str, body: dict, quiet: bool = False):
        """POST to the slot: (status, reply); quiet leaves a transport error to the caller's log."""
        try:
            return s.request("POST", path, body, timeout=CALL_TIMEOUT_S)
        except (OSError, http.client.HTTPException) as e:
            if quiet:
                return None, {"message": f"{type(e).__name__}: {e}"}
            log.warning("%s: %s failed: %s", s.name, path, e)
            return None, None

    def _run_record(self, s: Slot, ep: Episode) -> dict | None:
        if not isinstance(ep.episode, int):
            return None
        return self._record_of(s, ep.run_id, ep.task, ep.episode)

    def _record_of(self, s: Slot, run_id: str, task: str, k: int) -> dict | None:
        """The robot server's saved record of (run_id, task, k), None if it has none or is silent."""
        try:
            run = f"{quote(run_id, safe='')}/" if run_id else ""
            status, reply = s.request("GET", f"/api/runs/{run}{quote(task, safe='')}/{k}",
                                      timeout=CALL_TIMEOUT_S)
        except (OSError, http.client.HTTPException) as e:
            log.warning("%s: reading the record of %s %s failed: %s", s.name, task, k, e)
            return None
        rec = reply.get("record") if status == 200 and isinstance(reply, dict) else None
        return rec if isinstance(rec, dict) else None

    def _over(self, ep: Episode, end: str, record: dict | None, outcome: str | None = None):
        """Under the lock: the episode is over for good; write its ledger row, free the slot."""
        if ep.over:
            return
        ep.over, ep.end, ep.outcome, ep.ended_at = True, end, outcome, self.clock()
        self._claim_keys.pop(ep.token, None)
        with self._grade_cv:
            self._grade_cv.notify_all()
            self._wake(ep, every=True)
            self._turns.pop(ep.token, None)
        if ep.end_at is None:
            ep.end_at = ep.ended_at
        row = episode_row(ep, record)
        self.ledger.append(row)
        tr = self.tracer(ep)
        emit(tr, "gateway.over", row)
        close_tracer(tr)
        self.tracers[ep.episode_id] = NullTracer("gateway")
        self.catalogs.pop(ep.worker, None)
        self._save()
        log.info("%s: episode %s of %s over: end %s, outcome %s, success %s", ep.worker,
                 ep.episode_id, ep.task, end, outcome, row["success"])

    def _save(self) -> None:
        """Under the lock: persist live and recent episodes (with their tokens: mode 600)."""
        self._saved_at = time.monotonic()
        now = self.clock()
        for t in [t for t, e in self.episodes.items()
                  if e.over and now - (e.ended_at or now) >= ENDED_KEEP_S]:
            self.tracers.pop(self.episodes.pop(t).episode_id, None)
            self._born.pop(t, None)
            self._inflight.pop(t, None)
            self._owed_since.pop(t, None)
        data = {"format": STATE_FORMAT, "saved_at": now,
                "orphans": {s.name: s.orphan_job for s in self.slots if s.orphan_job},
                "release_owed": sorted(s.name for s in self.slots if s.release_owed),
                "episodes": [_saved(e) for e in self.episodes.values()]}
        path = self.state_file
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            write_json_atomic(path, data, mode=0o600, indent=1)
            _fsync_dir(path.parent)
        except OSError as e:
            log.error("could not write %s: %s", path, e)

    def _load(self) -> None:
        if not self.state_file.is_file():
            return
        try:
            data = json.loads(self.state_file.read_text())
            if data.get("format") != STATE_FORMAT:
                raise ValueError(f"format {data.get('format')!r}")
            rows = data["episodes"]
        except (OSError, ValueError, KeyError, AttributeError) as e:
            log.warning("ignoring %s: %s", self.state_file, e)
            return
        for name, job in (data.get("orphans") or {}).items():
            if name in self.by_name:
                self.by_name[name].orphan_job = job
        for name in data.get("release_owed") or ():
            if name in self.by_name:
                self.by_name[name].release_owed = True
        for row in rows:
            try:
                ep = Episode.from_dict(row)
            except TypeError:
                continue
            self.episodes[ep.token] = ep
        mono, wall = self.mono(), self.clock()
        for ep in self.episodes.values():
            o = ep.official
            if o is not None and o.deadline is not None:
                due_wall = o.deadline + o.mono_to_wall
                o.mono_to_wall = wall - mono
                o.deadline = due_wall - o.mono_to_wall
        for ep in list(self.episodes.values()):
            if not ep.over and ep.worker not in self.by_name:
                ep.error = f"worker {ep.worker!r} is not among this gateway's slots"
                self._over(ep, "error", None)
        for ep in self.episodes.values():
            self._resume_grades(ep)
        live = sum(1 for e in self.episodes.values() if not e.over)
        log.info("%s: %d live episode(s) restored", self.state_file, live)

    def _resume_grades(self, ep: Episode) -> None:
        """After a restart no grade worker survives: resubmit a live episode's pending grades."""
        o = ep.official
        if o is None or not o.grading:
            return
        live = not ep.over and ep.end is None and ep.ready_at is not None \
            and ep.worker in self.by_name
        pending, o.grading = list(o.grading), []
        if not live:
            return
        s = self.by_name[ep.worker]
        for n in pending:
            o.grading.append(n)
            self._graders[s.name].submit(self._official_grade, s, ep, n)


def _saved(ep: Episode) -> dict:
    d = asdict(ep)
    if d["official"] is None:
        del d["official"]
    return d


def _fsync_dir(d: Path) -> None:
    """Make a rename into d durable; amp.fs.atomic fsyncs the file but not its directory."""
    fd = os.open(d, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)
