#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The agent loop: one episode's tool-use conversation between a model and the robot harness."""

from __future__ import annotations

import contextlib
import copy
import errno
import gzip
import hashlib
import inspect
import json
import logging
import re
import threading
import time
import traceback
import uuid
from pathlib import Path

from amp.agents.context import ContextWindow
from amp.agents.episode import EpisodeOver, GatewayError
from amp.agents.llm import ContextOverflow, LLMError, Observer, converter
from amp.agents.llm.base import Aborted
from amp.agents.official import (
    DEADLINE_ERROR,
    INTERRUPTED_NOTE,
    AttemptOver,
    deadline_text,
)
from amp.agents.recording import RequestLog, blobbed
from amp.agents_text import tools as T
from amp.agents_text.surface import (
    AGENT_SURFACE_VERSION,
    VDM_NOTES,
    VDM_PROMPTS,
    VDM_SURFACE_VERSION,
    episode_over_text,
)
from amp.contracts.embodiment import DROID
from amp.contracts.trace import NullTracer
from amp.fs.safe_files import is_safe_file, rel_parts

log = logging.getLogger("amp.agents.loop")

CAMERAS = ("ego_l", "ego_r", "wrist")
REPORT_TURNS = 3
NUDGES = 3
OVER_POLL_S = 180.0
POLL_S = 1.0
KEEPALIVE_S = 60.0
OVERFLOW_RETRIES = 3
REPORT_GRACE_S = 900.0
IMAGE_400 = re.compile(r"tool_result|image", re.IGNORECASE)
FIRST_USER_ZERO_SHOT = (
    "The episode is ready. Your instruction and step budget are not in this message: read them "
    'from the episode with `obs = ep.observe(); print(obs["instruction"], ep.budget())`, then '
    "begin by looking at the scene.")
TOTAL_KEYS = ("cost_usd", "failed_cost_usd", "input_tokens", "output_tokens", "thinking_tokens",
              "cache_read_tokens", "cache_write_tokens", "model_s", "tool_s")
VDM_TOTAL_KEYS = ("vdm_calls", "vdm_cost_usd", "vdm_input_tokens", "vdm_output_tokens", "vdm_s")


def system_sha256(system: list[dict]) -> str:
    return hashlib.sha256("\n".join(b["text"] for b in system).encode()).hexdigest()


def turn1_sha256(system: list[dict], tools: list[dict], first_user: list[dict]) -> str:
    """SHA-256 of turn 1's input: system blocks, tools and the first user message."""
    raw = json.dumps({"system": system, "tools": tools, "first_user": first_user},
                     sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(raw.encode()).hexdigest()


def _span() -> str:
    return uuid.uuid4().hex[:16]


class _Observer(Observer):
    def __init__(self, agent: Agent):
        self.agent = agent

    def request(self, body, payload, attempt):
        self.agent.record_request(body, payload, attempt)

    def retry(self, err):
        self.agent.totals["cost_usd"] += err.get("cost_usd") or 0.0
        self.agent.totals["failed_cost_usd"] += err.get("cost_usd") or 0.0
        self.agent.ev("agent.llm.retry", {"turn": self.agent.turn + 1, **err}, lvl="warn")


class _VdmObserver(Observer):
    def __init__(self, agent: Agent):
        self.agent = agent

    def retry(self, err):
        self.agent.totals["cost_usd"] += err.get("cost_usd") or 0.0
        self.agent.totals["failed_cost_usd"] += err.get("cost_usd") or 0.0
        self.agent.ev("agent.vdm.retry", {"turn": self.agent.turn, **err}, lvl="warn")


def _text(text: str) -> dict:
    return {"type": "text", "text": text}


class Agent:
    """One episode's tool-use loop."""

    def __init__(self, episode_dir: Path, *, llm, kernel_factory, episode, system_text: str,
                 tracer=None, tools: list[dict] | None = None,
                 preloaded: tuple[str, ...] = T.PRELOADED, zero_shot: bool = True,
                 max_turns: int = 300, wall_s: float = 7200, max_cost_usd: float = 80.0,
                 meta: dict | None = None,
                 cameras: tuple[str, ...] = CAMERAS, judged_by_self: bool = False,
                 official=None, vdm_llm=None):
        self.dir = Path(episode_dir).resolve()
        self.info = json.loads((self.dir / "episode.json").read_text())
        self.llm, self.kernel_factory, self.ep = llm, kernel_factory, episode
        self.tracer = tracer or NullTracer("agent")
        self.system_text = system_text
        self.preloaded = tuple(preloaded)
        self.judged_by_self = judged_by_self
        self.vdm = vdm_llm
        self.vision = "vdm_strict" if vdm_llm is not None else "image"
        self.tools = (T.tools_for(self.preloaded, judged_by_self, vision=self.vision)
                      if tools is None else tools)
        self.zero_shot = zero_shot
        self.meta = dict(meta or {})
        self.cameras = tuple(cameras)
        self.robot_cameras: tuple[str, ...] | None = None
        self.provider = llm.provider
        self.ctx = ContextWindow(getattr(llm, "context_tokens", None),
                                 getattr(llm, "estimate_image_tokens", None))
        self.context_window: int | None = getattr(llm, "context_window", None)
        self.limits = {"max_turns": max_turns, "wall_s": wall_s, "max_cost_usd": max_cost_usd}
        self.kernel = None
        self.messages: list[dict] = []
        self.system: list[dict] = []
        self.first_user: list[dict] = []
        self.turn = 0
        self.tool_calls = 0
        self.totals = dict.fromkeys(TOTAL_KEYS, 0)
        self.totals.update(cost_usd=0.0, failed_cost_usd=0.0, model_s=0.0, tool_s=0.0)
        self.t0 = time.monotonic()
        self.phase = "work"
        self.over_status: dict | None = None
        self.last_obs: dict = {}
        self.last_step: int | None = None
        self.max_steps: int | None = None
        self.report: dict | None = None
        self.end_reason: str | None = None
        self.restarts = 0
        self.keepalives = 0
        self.run_span = _span()
        self.turn_span: str | None = None
        self.requests = RequestLog()
        self.report_t0: float | None = None
        self.instruction: str | None = None
        self._verbatim = "verbatim" in inspect.signature(self.tracer.event).parameters
        self._rich_blob = "verbatim" in inspect.signature(self.tracer.blob).parameters
        self._begun = False
        self._ended = False
        self.official = official
        self.attempt_over = False
        self.kernel_failed_at = 0.0
        if hasattr(llm, "observer"):
            llm.observer = _Observer(self)
        self.frames: dict[str, bytes] = {}
        self.before: dict[str, bytes] = {}
        self.frames_step: int | None = None
        self.before_step: int | None = None
        self.vdm_pending: list[str] = []
        if self.vdm is not None:
            self.totals.update(dict.fromkeys(VDM_TOTAL_KEYS, 0), vdm_cost_usd=0.0, vdm_s=0.0)
            self.meta["vdm_surface_version"] = VDM_SURFACE_VERSION
            if hasattr(self.vdm, "observer"):
                self.vdm.observer = _VdmObserver(self)

    def ev(self, kind: str, data: dict, *, lvl: str = "info", blobs=(),
           verbatim: bool = False) -> None:
        kw = {"verbatim": True} if verbatim and self._verbatim else {}
        self.tracer.event(kind, data, lvl=lvl, blobs=blobs, span=self.turn_span or self.run_span,
                          **kw)

    def blob(self, data: bytes, mime: str, role: str, *, compress: bool = False,
             verbatim: bool = False):
        if self._rich_blob:
            return self.tracer.blob(data, mime=mime, role=role, compress=compress,
                                    verbatim=verbatim)
        if compress:
            return self.tracer.blob(gzip.compress(data, mtime=0), mime="application/gzip",
                                    role=role)
        return self.tracer.blob(data, mime=mime, role=role)

    @property
    def kernel_uid(self) -> int | None:
        return getattr(self.kernel, "kernel_uid", None)

    def begin(self, instruction: str | None) -> None:
        if self._begun:
            return
        self._begun = True
        self.ev("agent.run.begin", {
            **self.meta, "episode_id": self.info.get("episode_id"), "instruction": instruction,
            "provider": self.provider, "model": getattr(self.llm, "model", None),
            "max_tokens": getattr(self.llm, "max_tokens", None),
            "context_tokens": self.ctx.context_tokens, "context_window": self.context_window,
            "compaction": None, "zero_shot": self.zero_shot, "image_policy": T.IMAGE_POLICY,
            **self.guard_meta(), "limits": self.limits,
            "tools_sha256": T.tools_sha256(self.tools),
            "agent_surface_version": AGENT_SURFACE_VERSION,
            "system_sha256": system_sha256(self.system) if self.system else None,
            "first_user": self.first_user[0]["text"] if self.first_user else None,
            "turn1_sha256": (turn1_sha256(self.system, self.tools, self.first_user)
                             if self.system and self.first_user else None)})

    def guard_meta(self) -> dict:
        """The client's resample budgets: degenerate_resamples, Gemini's block and safety ones."""
        n = (getattr(self.llm, "resamples", None) or {}).get("degenerate")
        out = {} if n is None else {"degenerate_resamples": n}
        for k in ("max_prompt_block_retries", "max_safety_retries"):
            v = getattr(self.llm, k, None)
            if isinstance(v, int) and not isinstance(v, bool):
                out[k] = v
        return out

    def end(self) -> None:
        if self._ended:
            return
        self._ended = True
        self.begin(self.info.get("instruction"))
        t = self.totals
        totals = {"turns": self.turn, "tool_calls": self.tool_calls,
                  **{k: t[k] for k in TOTAL_KEYS},
                  "cost_usd": round(t["cost_usd"], 6), "model_s": round(t["model_s"], 3),
                  "tool_s": round(t["tool_s"], 3), "wall_s": round(time.monotonic() - self.t0, 3)}
        if self.vdm is not None:
            totals.update({k: t[k] for k in VDM_TOTAL_KEYS},
                          vdm_cost_usd=round(t["vdm_cost_usd"], 6), vdm_s=round(t["vdm_s"], 3))
        self.turn_span = None
        self.ev("agent.run.end", {"end_reason": self.end_reason or "unknown", "totals": totals})

    def record_request(self, body: dict, payload: bytes, attempt: int) -> None:
        """agent.llm.request for a body as sent; called by the client before each send."""
        _, keys = converter(self.provider)
        data = {"turn": self.turn + 1, "attempt": attempt, "provider": self.provider,
                "keys": list(body), "params": {k: v for k, v in body.items() if k not in keys},
                "system_sha256": system_sha256(self.system) if self.system else None,
                "tools_sha256": T.tools_sha256(self.tools)}
        if self.requests.changed("system", self.system):
            ref = self.blob(json.dumps(self.system).encode(), "application/json",
                            "system_prompt", verbatim=True)
            data["system"] = {"ref": ref.ref, "size": ref.size}
        if self.requests.changed("tools", self.tools):
            ref = self.blob(json.dumps(self.tools).encode(), "application/json", "tools",
                            verbatim=True)
            data["tools"] = {"ref": ref.ref, "size": ref.size}
        base, new = self.requests.delta(self.messages)
        data.update(base=base, messages=blobbed(new, self.tracer),
                    n_messages=len(self.messages),
                    body_sha256=hashlib.sha256(payload).hexdigest(), body_bytes=len(payload))
        self.ev("agent.llm.request", data, verbatim=True)

    def build_system(self) -> None:
        if self.zero_shot:
            self.system = [{"type": "text", "text": self.system_text}]
            return
        obs = self.last_obs
        max_steps, dt = obs.get("max_steps"), obs.get("control_dt") or 1 / 15
        instruction = obs.get("instruction") or self.info.get("instruction") or ""
        budget = (f"{max_steps} steps ({max_steps * dt:.0f} s of motion at {1 / dt:.0f} Hz); "
                  f"{obs.get('steps_left')} left now" if max_steps else "unknown")
        episode = f"## Your episode\n\nInstruction: {instruction}\n\nStep budget: {budget}.\n"
        self.system = [{"type": "text", "text": self.system_text},
                       {"type": "text", "text": episode}]

    def first_user_content(self, obs: dict, instruction: str | None) -> list[dict]:
        if self.zero_shot:
            return [{"type": "text", "text": FIRST_USER_ZERO_SHOT}]
        first = (f"The episode is ready. Instruction: {instruction}\n"
                 f"Step budget: {obs.get('max_steps')} steps, {obs.get('steps_left')} left. "
                 "Begin by looking at the scene.")
        return [{"type": "text", "text": first}]

    def observe(self) -> dict | None:
        """The loop's own observation through the gateway; None once the episode is over."""
        try:
            obs = self.ep.observe()
        except EpisodeOver:
            return None
        except (GatewayError, OSError) as e:
            log.warning("observe failed: %s", e)
            return self.last_obs or {}
        if obs.get("mode") != "episode_over":
            self.last_obs = obs
        if obs.get("max_steps") is not None:
            self.max_steps = obs["max_steps"]
        return obs

    @contextlib.contextmanager
    def polling(self, interval_s: float = POLL_S):
        """Keep last_obs fresh while a call runs, so the step at the episode's end is known."""
        stop = threading.Event()
        client = self.ep.copy()

        def loop():
            while not stop.wait(interval_s):
                try:
                    obs = client.observe()
                except (GatewayError, OSError):
                    return
                if obs.get("mode") != "episode_over":
                    self.last_obs = obs

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        try:
            yield
        finally:
            stop.set()
            th.join(timeout=interval_s + 5)

    @contextlib.contextmanager
    def keepalive(self, interval_s: float | None = None):
        """POST a no-op wait every interval_s while the model thinks, so the episode stays live."""
        interval_s = KEEPALIVE_S if interval_s is None else interval_s
        stop = threading.Event()
        client = self.ep.copy()

        def loop():
            while not stop.wait(interval_s):
                try:
                    client.keepalive()
                    self.keepalives += 1
                except EpisodeOver:
                    return
                except (GatewayError, OSError) as e:
                    log.warning("keep-alive failed: %s", e)

        th = threading.Thread(target=loop, daemon=True)
        th.start()
        try:
            yield
        finally:
            stop.set()
            th.join(timeout=5)

    def storyboard(self, obs: dict) -> None:
        """One frame per camera into the trace whenever the step advanced."""
        step = obs.get("step")
        if step is None or (self.last_step is not None and step <= self.last_step):
            return
        self.last_step = step
        cams, blobs, frames = [], [], {}
        for cam in self.storyboard_cameras(obs):
            try:
                raw = self.ep.frame(cam)
            except Exception as e:  # noqa: BLE001
                log.warning("storyboard %s failed: %s", cam, e)
                continue
            blobs.append(self.tracer.blob(raw, mime="image/jpeg", role=f"rgb:{cam}"))
            cams.append(cam)
            frames[cam] = raw
        if self.vdm is not None:
            self.before, self.before_step = self.frames, self.frames_step
            self.frames = frames
            self.frames_step = step
        if cams:
            self.ev("agent.storyboard", {"step": step, "cams": cams}, blobs=blobs)

    def storyboard_cameras(self, obs: dict) -> list[str]:
        """The robot's reported cameras when it declares a non-DROID embodiment, else DROID's."""
        emb = obs.get("embodiment")
        if isinstance(emb, dict) and emb.get("name") not in (None, DROID):
            self.robot_cameras = tuple(str(c["name"]) for c in emb.get("cameras") or ()
                                       if isinstance(c, dict) and c.get("name"))
        if self.robot_cameras is None:
            return [c for c in self.cameras if c in (obs.get("cameras") or self.cameras)]
        have = obs.get("cameras") or {}
        return [c for c in self.robot_cameras if not have or c in have]

    def start_kernel(self):
        self.kernel = self.kernel_factory()
        self.kernel.start()

    def wait_over(self, timeout_s: float | None = None) -> dict | None:
        """The gateway's episode status once it says over (None if it never does in time)."""
        deadline = time.monotonic() + (OVER_POLL_S if timeout_s is None else timeout_s)
        while True:
            try:
                st = self.ep.status()
            except Exception as e:  # noqa: BLE001
                log.warning("status failed: %s", e)
                st = {}
            if st.get("state") == "over":
                return st
            if time.monotonic() >= deadline:
                return None
            time.sleep(1.0)

    def ended_by_itself(self) -> bool:
        """end "ended" with steps to spare at the last observation (the sim is paused between
        motions, so only the motion running then could have used them up)."""
        if (self.over_status or {}).get("end") != "ended":
            return False
        o = self.last_obs or {}
        left = o.get("steps_left")
        if left is None:
            return False
        ex = o.get("execution") or {}
        running = o.get("mode") == "executing"
        in_flight = max(0, (ex.get("n_steps") or 0) - (ex.get("index") or 0)) if running else 0
        return left - in_flight > 5

    def mark_over(self, how: str, status: dict | None) -> str:
        self.phase = "report"
        self.over_status = status or {"state": "over", "end": None}
        o = self.last_obs or {}
        self.ev("agent.episode.over", {
            "how": how, "end": self.over_status.get("end"), "last_step": o.get("step"),
            "steps_left": o.get("steps_left"), "max_steps": self.max_steps,
            "ended_by_itself": self.ended_by_itself()})
        end = self.over_status.get("end")
        return episode_over_text(end, o, benchmark=self.robot_cameras is not None,
                                 judged_by_self=self.judged_by_self, vdm=self.vdm is not None)

    def run_kernel(self, code: str, timeout_s: float, deadline: bool = False):
        try:
            if deadline:
                return self.kernel.run(code, timeout_s=timeout_s,
                                       deadline_error=DEADLINE_ERROR), None
            return self.kernel.run(code, timeout_s=timeout_s), None
        except Exception as e:  # noqa: BLE001
            self.kernel_failed_at = time.monotonic()
            self.restarts += 1
            self.ev("agent.kernel.restart", {"error": f"{type(e).__name__}: {e}",
                                             "restarts": self.restarts}, lvl="warn")
            with contextlib.suppress(Exception):
                self.kernel.close()
            if self.restarts > 3:
                return None, "The Python kernel failed repeatedly and was not restarted."
            try:
                self.start_kernel()
            except Exception as e2:  # noqa: BLE001
                return None, f"The Python kernel crashed and could not be restarted: {e2}"
            return None, ("The Python kernel crashed and was restarted: your variables are "
                          f"gone; {T.preloaded_phrase(self.preloaded)} are loaded again. Error: "
                          f"{type(e).__name__}: {e}")

    def tool_python(self, inp: dict):
        code = inp.get("code") or ""
        timeout_s = max(1.0, min(float(inp.get("timeout_s") or 600), 1800.0))
        if self.official is not None:
            return self.official_cell(code, timeout_s)
        with self.polling():
            res, crash = self.run_kernel(code, timeout_s)
        return self.python_reply(res, crash, timeout_s)

    def official_cell(self, code: str, timeout_s: float):
        """tool_python between the gateway's begin and end, capped at the trial's deadline."""
        try:
            began = self.official.begin()
            t_reply = time.monotonic()
        except AttemptOver as e:
            self.attempt_over = True
            self.ev("agent.official.cell", {"op": "begin", "official_timeout": str(e)})
            msg = "The trial's time limit ended this attempt; the cell was not run."
            return [{"type": "text", "text": msg}], True, {"error": msg}
        except (GatewayError, OSError) as e:
            self.ev("agent.official.cell", {"op": "begin", "error": str(e)[:500]}, lvl="warn")
            msg = f"The cell was not run: the gateway did not start it ({e})."
            return [{"type": "text", "text": msg}], True, {"error": msg}
        cell, left = began.get("cell"), began.get("interrupt_in_s")
        earlier = self.official.surface(began)
        deadline_at = None if left is None else t_reply + float(left)
        capped = left is not None and float(left) < timeout_s
        self.ev("agent.official.cell", {"op": "begin", "cell": cell, "interrupt_in_s": left,
                                        "surfaced": earlier is not None})
        with self.polling():
            res, crash = self.run_kernel(
                code, max(0.0, deadline_at - time.monotonic()) if capped else timeout_s,
                deadline=capped)
        late = deadline_at is not None and time.monotonic() > deadline_at
        if res is None:
            interrupted = deadline_at is not None and self.kernel_failed_at >= deadline_at
        else:
            interrupted = res.timed_out and (capped or late)
        if interrupted and res is not None and not capped:
            res.error = deadline_text(res.error)
        err = self.official.end(cell, interrupted)
        self.ev("agent.official.cell", {"op": "end", "cell": cell, "interrupted": interrupted,
                                        "delivered": err is None, **({"error": err} if err else {})},
                lvl="warn" if err else "info")
        blocks, is_error, rec = self.python_reply(res, crash, timeout_s,
                                                  INTERRUPTED_NOTE if interrupted else None)
        if earlier is not None:
            blocks[0]["text"] = earlier + "\n" + blocks[0]["text"]
        return blocks, is_error, rec

    def python_reply(self, res, crash: str | None, timeout_s: float, note: str | None = None):
        if res is None:
            return [{"type": "text", "text": crash}], True, {"error": crash}
        if (res.error or "").startswith("KernelRestart"):
            self.ev("agent.kernel.restart", {"error": res.error[:2000], "by": "kernel"},
                    lvl="warn")
        blocks, is_error = T.python_result(res, timeout_s, self.dir, self.kernel_uid, note,
                                           attach=self.vdm is None)
        if self.vdm is None:
            blocks += self.camera_blocks(res.images or [])
        rec = {"stdout": res.stdout, "stderr": res.stderr, "error": res.error,
               "duration_s": res.duration_s, "timed_out": res.timed_out,
               "shown": list(res.images or [])}
        return blocks, is_error, rec

    def camera_blocks(self, shown: list[dict]) -> list[dict]:
        """[label, frame] for each show(camera=...) of an image harness configuration, within
        MAX_SHOWN in all."""
        room = max(0, T.MAX_SHOWN - sum(1 for s in shown if s.get("path")))
        blocks = []
        for s in [s for s in shown if s.get("camera") and not s.get("path")][:room]:
            cam = str(s["camera"])
            label = f"camera {cam}" + (f": {s['caption']}" if s.get("caption") else "")
            try:
                block, _, _ = T.encode_image(self.camera_frame(cam))
            except Exception as e:  # noqa: BLE001
                blocks.append(_text(f"{label} (could not be attached: {e})"))
                continue
            blocks += [_text(label), block]
        return blocks

    def tool_view_image(self, inp: dict):
        rel = str(inp.get("path") or "")
        try:
            rel = "/".join(rel_parts(self.dir / rel, self.dir))
            T.read_image(self.dir, rel, self.kernel_uid)
        except (OSError, ValueError) as e:
            msg = f"no image file {rel!r} in your working directory"
            if isinstance(e, OSError) and e.errno == errno.ELOOP:
                msg += f" ({e.strerror})"
            return [{"type": "text", "text": msg}], True, {"error": msg}
        blocks = T.image_blocks(self.dir, rel, None, self.kernel_uid)
        is_error = len(blocks) == 1
        return blocks, is_error, {"error": blocks[0]["text"] if is_error else None}

    def tool_finish(self, inp: dict):
        res, crash = self.run_kernel("ep.finish()\nprint('finish requested')", 120)
        err = crash or (res.error if res else None)
        if err:
            try:
                self.ep.finish()
            except Exception as e:  # noqa: BLE001
                log.warning("finish through the loop's client failed: %s", e)
        msg = self.mark_over("finish_episode", self.wait_over())
        rec = {"stdout": res.stdout if res else "", "stderr": res.stderr if res else "",
               "error": err, "reason": inp.get("reason")}
        return [{"type": "text", "text": msg}], False, rec

    def tool_report(self, inp: dict):
        if self.phase == "work":
            msg = ("The episode is still running. Keep working, or call finish_episode to end "
                   "it as it stands; submit_report comes after the episode is over.")
            return [{"type": "text", "text": msg}], True, {"error": msg}
        err = T.validate_report(inp, self.ended_by_itself(), self.judged_by_self)
        if err:
            return [{"type": "text", "text": f"report refused: {err}"}], True, {"error": err}
        self.report = dict(inp)
        missing = [k.get("path") for k in inp.get("key_images") or []
                   if not self.image_ok(str(k.get("path")))]
        self.ev("agent.report", {"report": copy.deepcopy(self.report),
                                 "missing_images": missing})
        note = "Report received." + (f" (missing image files: {missing})" if missing else "")
        return [{"type": "text", "text": note}], False, {"error": None}

    def image_ok(self, rel: str) -> bool:
        try:
            return is_safe_file(self.dir / rel, self.dir, kernel_uid=self.kernel_uid)
        except (OSError, ValueError):
            return False

    def dispatch(self, tu: dict):
        """(content, is_error, record) for one tool_use."""
        name, inp = tu.get("name"), tu.get("input") or {}
        if self.report is not None:
            msg = "The report was already submitted."
            return [{"type": "text", "text": msg}], True, {"error": msg}
        if self.phase == "report" and name in ("python", "finish_episode"):
            msg = "The episode is over: only view_image and submit_report are available now."
            if self.vdm is not None:
                msg = VDM_NOTES["report_only"]
            return [{"type": "text", "text": msg}], True, {"error": msg}
        if not inp and tu.get("_cut"):
            msg = "Your tool call was cut off at the output limit; send it again, shorter."
            return [{"type": "text", "text": msg}], True, {"error": msg}
        fn = {"python": self.tool_python, "view_image": self.tool_view_image,
              "finish_episode": self.tool_finish, "submit_report": self.tool_report}.get(name)
        if self.vdm is not None and name == "view_image":
            fn = None
        if fn is None:
            msg = f"unknown tool {name!r}"
            return [{"type": "text", "text": msg}], True, {"error": msg}
        return fn(inp)

    def limit_hit(self) -> str | None:
        if self.turn >= self.limits["max_turns"]:
            return f"turn limit ({self.limits['max_turns']} turns)"
        if time.monotonic() - self.t0 >= self.limits["wall_s"]:
            return f"wall-clock limit ({self.limits['wall_s']:.0f} s)"
        if self.totals["cost_usd"] >= self.limits["max_cost_usd"]:
            return f"cost limit (${self.limits['max_cost_usd']:.2f})"
        return None

    def end_for_limit(self, why: str) -> str:
        self.ev("agent.limit", {"limit": why, "turn": self.turn,
                                "cost_usd": round(self.totals["cost_usd"], 4),
                                "wall_s": round(time.monotonic() - self.t0, 1)})
        try:
            self.ep.finish()
        except Exception as e:  # noqa: BLE001
            log.warning("finish at the limit failed: %s", e)
        msg = self.mark_over("limit", self.wait_over())
        return f"You reached the {why}; the episode has been ended as it stands. " + msg

    def abort_reason(self) -> str | None:
        """Why an in-flight call must stop: the cost limit, during the work phase."""
        if self.phase == "work" and self.totals["cost_usd"] >= self.limits["max_cost_usd"]:
            return f"cost limit (${self.limits['max_cost_usd']:.2f})"
        return None

    def arm_limits(self) -> None:
        """Bound the next call: the wall limit while working, REPORT_GRACE_S per report call."""
        if not hasattr(self.llm, "deadline"):
            return
        if self.phase == "work":
            self.llm.deadline = self.t0 + self.limits["wall_s"]
        else:
            self.llm.deadline = time.monotonic() + REPORT_GRACE_S
        self.llm.should_abort = self.abort_reason

    def prompt_tokens(self, usage: dict | None) -> int | None:
        """The prompt the server counted for a call, cache included, as the client defines it."""
        fn = getattr(self.llm, "prompt_tokens", None)
        if callable(fn):
            return fn(usage or {})
        n = (usage or {}).get("input_tokens")
        return n if isinstance(n, int) else None

    def call_model(self):
        self.arm_limits()
        with self.keepalive():
            turn = self.llm.create(system=self.system, messages=self.messages, tools=self.tools)
        self.turn += 1
        u = turn.usage or {}
        t = self.totals
        thinking = (u.get("output_tokens_details") or {}).get("thinking_tokens") or 0
        prompt, window = self.prompt_tokens(u), self.context_window
        t["cost_usd"] += turn.cost_usd or 0.0
        t["input_tokens"] += u.get("input_tokens") or 0
        t["output_tokens"] += u.get("output_tokens") or 0
        t["thinking_tokens"] += thinking
        t["cache_read_tokens"] += u.get("cache_read_input_tokens") or 0
        t["cache_write_tokens"] += u.get("cache_creation_input_tokens") or 0
        t["model_s"] += turn.latency_s
        blobs = []
        if getattr(turn, "raw", b""):
            mime = "application/json" if turn.raw.lstrip()[:1] == b"{" else "text/event-stream"
            blobs.append(self.blob(turn.raw, mime, "llm_raw_response", compress=True,
                                   verbatim=True))
        self.ev("agent.llm.response", {
            "turn": self.turn, "request_id": turn.request_id, "message_id": turn.message_id,
            "stop_reason": turn.stop_reason, "content": copy.deepcopy(turn.content),
            "native": copy.deepcopy(getattr(turn, "native", None))}, blobs=blobs, verbatim=True)
        self.ev("agent.llm.usage", {
            "turn": self.turn, "input_tokens": u.get("input_tokens") or 0,
            "output_tokens": u.get("output_tokens") or 0, "thinking_tokens": thinking,
            "cache_read_tokens": u.get("cache_read_input_tokens") or 0,
            "cache_write_tokens": u.get("cache_creation_input_tokens") or 0,
            "prompt_tokens": prompt, "context_window": window,
            "context_pct": round(100 * prompt / window, 2) if prompt is not None and window
            else None,
            "latency_s": turn.latency_s, "ttft_s": getattr(turn, "ttft_s", None),
            "cost_usd": turn.cost_usd, "cost_source": turn.cost_source,
            "attempts": turn.attempts, "usage": u,
            "ratelimit": {f"{k}_remaining": v for k, v in (turn.ratelimit or {}).items()}})
        return turn

    def llm_error(self, e: LLMError, fatal: bool, **extra) -> None:
        self.ev("agent.llm.error", {"turn": self.turn + 1, "fatal": fatal, "status": e.status,
                                    "error": str(e)[:2000], "attempts": e.attempts, **extra},
                lvl="error" if fatal else "warn")

    def call_model_recovering(self):
        """call_model; on a 400 about a tool_result or image, strip the newest user turn's
        images and retry once."""
        try:
            return self.call_model()
        except LLMError as e:
            if isinstance(e, ContextOverflow) or e.status != 400 or not IMAGE_400.search(
                    f"{e} {e.body or ''}"):
                raise
            n = self.ctx.strip_newest(self.messages)
            if not n:
                raise
            self.llm_error(e, False, recovery=f"stripped {n} image(s) from the newest user "
                                              "turn; retrying once")
        return self.call_model()

    def call_model_fitting(self):
        """call_model_recovering on a fitted context; on ContextOverflow shrink the budget and
        retry, OVERFLOW_RETRIES times at most."""
        for attempt in range(OVERFLOW_RETRIES + 1):
            trim = self.ctx.fit(self.system, self.tools, self.messages)
            if trim is not None:
                self.ev("agent.context.trim", {"turn": self.turn + 1, **trim})
            raw = self.ctx.estimate(self.system, self.tools, self.messages) \
                if self.ctx.context_tokens else 0
            try:
                turn = self.call_model_recovering()
            except ContextOverflow as e:
                if not self.ctx.context_tokens:
                    raise
                self.ctx.calibrate(raw, e.prompt_tokens, overflow=True)
                last = attempt == OVERFLOW_RETRIES
                frac = self.ctx.budget_fraction
                self.llm_error(e, False, context_overflow=True, prompt_tokens=e.prompt_tokens,
                               estimated_tokens=self.ctx.estimate(self.system, self.tools,
                                                                  self.messages),
                               fraction=round(frac, 4),
                               recovery="none left" if last else
                               f"budget fraction to {frac * 0.8:.4f}; retrying")
                if last:
                    raise
                self.ctx.shrink()
                continue
            self.ctx.calibrate(raw, self.prompt_tokens(turn.usage))
            return turn
        raise AssertionError("unreachable")

    def budget_note(self) -> str:
        """`[step S of M, L left; wall W min of X left]`: W the minutes left of X in all."""
        o = self.last_obs or {}

        def q(v):
            return "?" if v is None else v

        wall = self.limits["wall_s"]
        left = max(0.0, wall - (time.monotonic() - self.t0))
        return (f"[step {q(o.get('step'))} of {q(o.get('max_steps') or self.max_steps)}, "
                f"{q(o.get('steps_left'))} left; wall {left / 60:.0f} min of {wall / 60:.0f} "
                "left]")

    def after_motion(self, name: str, rec: dict, blocks: list[dict]) -> dict | None:
        """Observe after a sim-touching call: storyboard on a new step, detect the end."""
        if self.phase == "report":
            return None
        obs = self.observe()
        if obs:
            self.storyboard(obs)
        over = obs is None or (obs or {}).get("mode") == "episode_over" or \
            "EpisodeOver" in (rec.get("error") or "")
        if over:
            st = self.wait_over(None if obs is None or obs.get("mode") == "episode_over"
                                else 10.0)
            if st is not None or obs is None:
                blocks.append({"type": "text", "text": self.mark_over(name, st)})
        return obs

    def vdm_note(self, rec: dict, moved: bool) -> str | None:
        """What the vision model says about one python call: the motion it made, or
        the last motion again through the views it show()ed; None when it neither moved nor
        showed anything, and no note is pending."""
        notes, self.vdm_pending = self.vdm_pending, []
        shown = [s for s in rec.get("shown") or [] if s.get("path") or s.get("camera")]
        dropped = sum(1 for s in shown if s.get("path"))
        shown = [s for s in shown if not s.get("path")]
        shown = shown[:T.MAX_SHOWN]
        known = set(self.episode_cameras())
        unknown = [s["camera"] for s in shown if s.get("camera") and s["camera"] not in known]
        shown = [s for s in shown if not s.get("camera") or s["camera"] in known]
        if self.frames and self.before and (moved or shown):
            notes.append(self.describe("diff" if moved else "look", shown))
        elif shown:
            notes.append(self.describe("scene", shown))
        notes += [VDM_NOTES["no_camera"].format(cam=c, cams=", ".join(sorted(known)))
                  for c in unknown]
        if dropped:
            notes.append(VDM_NOTES["camera_only"].format(n=dropped))
        return "\n\n".join(notes) or None

    def episode_cameras(self) -> list[str]:
        """The cameras show(camera=...) may name: those the storyboard records."""
        return self.storyboard_cameras(self.last_obs or {})

    def camera_frame(self, cam: str) -> bytes:
        """A camera's current frame; ValueError for a camera the episode does not have."""
        if cam not in self.episode_cameras():
            raise ValueError(f"no camera {cam!r}")
        return self.frames.get(cam) or self.ep.frame(cam)

    def describe(self, mode: str, shown: list[dict]) -> str:
        """One vision-model call, "scene" (the cameras now) or "diff" / "look" (just before the
        last motion against now), plus the shown views; the note for the model."""
        P = VDM_PROMPTS
        content = [_text(P["task"].format(instruction=self.instruction or ""))]
        if mode == "scene":
            cams = list(self.frames)
            system, content = P["initial_system"], [*content, _text(P["initial_user"])]
            for c in cams:
                content += [_text(P["view"].format(cam=c)), self.vdm_image(self.frames[c])]
        else:
            cams = [c for c in self.frames if c in self.before]
            system, content = P["diff_system"], [*content, _text(P["diff_user"])]
            for c in cams:
                content += [_text(P["previous"].format(cam=c)), self.vdm_image(self.before[c]),
                            _text(P["current"].format(cam=c)), self.vdm_image(self.frames[c])]
        views, captions, missing = [], [], []
        for s in shown:
            cap = P["caption"].format(caption=s["caption"]) if s.get("caption") else ""
            cam = str(s["camera"]) if s.get("camera") else None
            if cam is None and s.get("caption"):
                captions.append(str(s["caption"]))
            try:
                if cam is None:
                    raw = T.read_image(self.dir, str(s["path"]), self.kernel_uid)
                    add = [_text(P["agent_view"].format(path=s["path"], caption=cap)),
                           self.vdm_image(raw)]
                else:
                    add = [_text(P["agent_camera"].format(cam=cam, caption=cap))]
                    if cam not in cams:
                        add.append(self.vdm_image(self.camera_frame(cam)))
            except Exception as e:
                if cam is None and not isinstance(e, (OSError, ValueError)):
                    raise
                if cam is None:
                    log.warning("vdm view %s unreadable: %s", s.get("path"), e)
                    continue
                log.warning("vdm camera %s unavailable: %s", cam, e)
                missing.append(VDM_NOTES["no_camera"].format(
                    cam=cam, cams=", ".join(self.episode_cameras())))
                continue
            content += add
            if cam is None:
                views.append(str(s["path"]))
            else:
                views.append(VDM_NOTES["camera_view"].format(cam=cam))
                if s.get("caption"):
                    captions.append(str(s["caption"]))
        if captions:
            content.append(_text(P["focus"].format(captions='"; "'.join(captions))))
        seen = ", ".join(cams)
        if views:
            seen = VDM_NOTES["views"].format(cams=seen, paths=", ".join(views))
        info = {"mode": mode, "cams": cams, "views": views, "before_step": self.before_step
                if mode != "scene" else None, "after_step": self.frames_step}
        system = f"{system} {P['qualitative']}"
        content.append(_text(P["qualitative"]))
        text, err = self.vdm_ask(system, content, info)
        if text is None:
            note = VDM_NOTES["unavailable"].format(error=err)
        else:
            note = VDM_NOTES[mode].format(before=self.before_step, after=self.frames_step,
                                          seen=seen, text=text.strip())
        return "\n\n".join([note, *missing])

    def vdm_image(self, data: bytes) -> dict:
        block, _, _ = T.encode_image(data)
        return block

    def vdm_ask(self, system: str, content: list[dict], info: dict) -> tuple[str | None, str]:
        """(the vision model's text, "") or (None, why): traced as agent.vdm.*, its cost counted
        in the totals and against the cost limit, the episode kept alive meanwhile."""
        n = self.totals["vdm_calls"] = self.totals["vdm_calls"] + 1
        self.ev("agent.vdm.request", {"turn": self.turn, "call": n, **info, "system": system,
                                      "content": blobbed(content, self.tracer)}, verbatim=True)
        if hasattr(self.vdm, "deadline"):
            self.vdm.deadline = self.t0 + self.limits["wall_s"]
            self.vdm.should_abort = self.abort_reason
        t0 = time.monotonic()
        try:
            with self.keepalive():
                turn = self.vdm.create(system=[_text(system)],
                                       messages=[{"role": "user", "content": content}], tools=[])
        except LLMError as e:
            self.totals["vdm_s"] += time.monotonic() - t0
            self.ev("agent.vdm.error", {"turn": self.turn, "call": n, "status": e.status,
                                        "error": str(e)[:2000], "attempts": e.attempts},
                    lvl="warn")
            return None, f"{type(e).__name__}: {str(e)[:300]}"
        u, t = turn.usage or {}, self.totals
        t["cost_usd"] += turn.cost_usd or 0.0
        t["vdm_cost_usd"] += turn.cost_usd or 0.0
        t["vdm_input_tokens"] += u.get("input_tokens") or 0
        t["vdm_output_tokens"] += u.get("output_tokens") or 0
        t["vdm_s"] += turn.latency_s or (time.monotonic() - t0)
        text = "".join(b.get("text") or "" for b in turn.content or [] if b.get("type") == "text")
        blobs = []
        if getattr(turn, "raw", b""):
            mime = "application/json" if turn.raw.lstrip()[:1] == b"{" else "text/event-stream"
            blobs.append(self.blob(turn.raw, mime, "vdm_raw_response", compress=True,
                                   verbatim=True))
        self.ev("agent.vdm.response", {
            "turn": self.turn, "call": n, "text": text, "stop_reason": turn.stop_reason,
            "request_id": turn.request_id, "latency_s": turn.latency_s,
            "cost_usd": turn.cost_usd, "cost_source": turn.cost_source,
            "attempts": turn.attempts, "usage": u}, blobs=blobs, verbatim=True)
        if not text.strip():
            return None, "the vision model answered no text"
        return text, ""

    def handle_tools(self, content: list[dict], stop_reason: str) -> list[dict]:
        results = []
        tool_uses = [b for b in content if b.get("type") == "tool_use"]
        for i, tu in enumerate(tool_uses):
            if stop_reason == "max_tokens" and i == len(tool_uses) - 1 and not tu.get("input"):
                tu = {**tu, "_cut": True}
            name = tu.get("name")
            self.ev("agent.tool.call", {"turn": self.turn, "id": tu.get("id"), "name": name,
                                        "input": copy.deepcopy(tu.get("input"))})
            t0 = time.monotonic()
            step0 = self.last_step
            blocks, is_error, rec = self.dispatch(tu)
            dt = time.monotonic() - t0
            self.tool_calls += 1
            self.totals["tool_s"] += dt
            obs = None
            ran = not (rec.get("error") or "").startswith("The episode is over")
            if name in ("python", "finish_episode") and not self.attempt_over and ran:
                obs = self.after_motion(name, rec, blocks)
            if self.vdm is not None and name == "python" and ran:
                note = self.vdm_note(rec, moved=self.last_step != step0)
                if note:
                    blocks.insert(1, _text(note))
            o = obs if obs is not None else self.last_obs
            if is_error and any(b.get("type") == "image" for b in blocks):
                is_error = False
            if self.report is None:
                note = self.budget_note()
                if blocks and blocks[-1].get("type") == "text":
                    blocks[-1]["text"] += "\n" + note
                else:
                    blocks.append({"type": "text", "text": note})
            refs: list = []
            sent = blobbed(blocks, self.tracer, refs)
            self.ev("agent.tool.result", {
                "turn": self.turn, "id": tu.get("id"), "name": name, "is_error": is_error,
                "duration_s": round(rec.get("duration_s") or dt, 3),
                "stdout": rec.get("stdout"), "stderr": rec.get("stderr"),
                "error": rec.get("error"), "timed_out": rec.get("timed_out"),
                "shown": rec.get("shown", []),
                "obs": {"step": o.get("step"), "steps_left": o.get("steps_left"),
                        "mode": "episode_over" if self.phase == "report" else o.get("mode")},
                "content": sent})
            if self.report is not None or self.attempt_over:
                break
            results.append({"type": "tool_result", "tool_use_id": tu.get("id"),
                            "content": blocks, "is_error": is_error})
        return results

    def run(self) -> int:
        """The episode; 0 when the model submitted its report, else 1."""
        try:
            return self._run()
        except (KeyboardInterrupt, SystemExit):
            self.end_reason = self.end_reason or "killed"
            raise
        except Exception:  # noqa: BLE001
            self.end_reason = self.end_reason or "crash"
            self.begin(self.info.get("instruction"))
            self.ev("agent.crash", {"error": traceback.format_exc()[-4000:]}, lvl="error")
            return 1
        finally:
            self.end()
            if self.kernel is not None:
                with contextlib.suppress(Exception):
                    self.kernel.close()

    def _run(self) -> int:
        try:
            self.start_kernel()
        except Exception as e:  # noqa: BLE001
            self.begin(self.info.get("instruction"))
            self.ev("agent.kernel.restart", {"error": f"start failed: {type(e).__name__}: {e}",
                                             "fatal": True}, lvl="error")
            self.end_reason = "kernel_error"
            return 1
        obs = self.observe()
        if obs is None:
            self.begin(self.info.get("instruction"))
            self.ev("agent.episode.over", {"how": "before_start", "end": None})
            self.end_reason = "episode_over_before_start"
            return 1
        self.build_system()
        instruction = obs.get("instruction") or self.info.get("instruction")
        self.instruction = instruction
        self.first_user = self.first_user_content(obs, instruction)
        self.begin(instruction)
        self.storyboard(obs)
        if self.vdm is not None and self.frames:
            self.vdm_pending.append(self.describe("scene", []))
        self.messages.append({"role": "user", "content": copy.deepcopy(self.first_user)})
        nudges = 0
        report_turns = 0
        while True:
            if self.phase == "work":
                why = self.limit_hit()
                if why:
                    note = self.end_for_limit(why)
                    self.messages[-1]["content"].append({"type": "text", "text": note})
            if self.phase == "report":
                report_turns += 1
                if report_turns > REPORT_TURNS:
                    self.end_reason = "no_report"
                    return 1
            dropped = self.ctx.trim_images()
            if dropped:
                self.ev("agent.context.images", {"turn": self.turn + 1, "dropped": dropped})
            self.turn_span = _span()
            self.ev("agent.turn.begin", {"turn": self.turn + 1, "phase": self.phase})
            try:
                turn = self.call_model_fitting()
            except ContextOverflow as e:
                if self.phase == "work" and self.ctx.context_tokens:
                    self.ev("agent.turn.end", {"turn": self.turn + 1, "stop_reason": None,
                                               "tool_calls": 0, "phase": self.phase})
                    self.turn_span = None
                    note = self.end_for_limit("context window full")
                    self.messages[-1]["content"].append({"type": "text", "text": note})
                    continue
                self.llm_error(e, True, context_overflow=True)
                self.end_reason = "api_error"
                return 1
            except Aborted as e:
                self.llm_error(e, self.phase != "work")
                if self.phase == "work":
                    self.ev("agent.turn.end", {"turn": self.turn + 1, "stop_reason": None,
                                               "tool_calls": 0, "phase": self.phase})
                    self.turn_span = None
                    note = self.end_for_limit(self.limit_hit() or str(e))
                    self.messages[-1]["content"].append({"type": "text", "text": note})
                    continue
                self.end_reason = "api_error"
                return 1
            except LLMError as e:
                if self.official is not None and self.phase == "work" and e.timeout:
                    self.ev("agent.official.model_timeout",
                            {"turn": self.turn + 1, "sent": self.official.model_timeout()})
                self.llm_error(e, True)
                self.end_reason = "api_error"
                return 1
            content = copy.deepcopy(turn.content)
            self.messages.append({"role": "assistant", "content": content})
            results = self.handle_tools(content, turn.stop_reason)
            self.ev("agent.turn.end", {"turn": self.turn, "stop_reason": turn.stop_reason,
                                       "tool_calls": len(results), "phase": self.phase})
            self.turn_span = None
            if self.report is not None:
                self.end_reason = "report"
                return 0
            if self.attempt_over:
                self.end_reason = "official_timeout"
                return 1
            if results:
                nudges = 0
                user = results
            else:
                nudges += 1
                if self.phase == "work" and nudges > NUDGES:
                    note = self.end_for_limit(f"limit of {NUDGES} replies without a tool call")
                    user = [{"type": "text", "text": note}]
                elif self.phase == "report":
                    user = [{"type": "text", "text": "Call submit_report now."}]
                else:
                    user = [{"type": "text", "text": (
                        "The episode is still running. Continue with the tools: python to act "
                        "and look, finish_episode to end it.")}]
            self.ctx.register(user)
            self.messages.append({"role": "user", "content": user})
