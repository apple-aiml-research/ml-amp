#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Per-turn console lines for a running episode, read from the agent's trace tail: names, counts
and durations only (no model text, tool arguments or images), so a console log shows the agent is
working, and what it waits on when it is not."""

from __future__ import annotations

import json
import logging
import math
import time
from collections import Counter
from pathlib import Path

from amp.contracts.trace import TRACE_DIR, stream_name

AGENT_STREAM = stream_name("agent")
READ_MAX = 4 << 20
LINE_MAX = 8 << 20
ERROR_CHARS = 100

log = logging.getLogger(__name__)


def _num(v) -> float:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v) else 0.0


def _first_line(v) -> str:
    text = v if isinstance(v, str) else ""
    lines = [ln for ln in text.strip().splitlines() if ln.strip()]
    return (lines[-1] if lines else "")[:ERROR_CHARS]


class TurnTail:
    """Counters and finished turns over the agent's trace stream, read incrementally."""

    def __init__(self, ep_dir: Path):
        self.path = Path(ep_dir) / TRACE_DIR / AGENT_STREAM
        self.pos = 0
        self.buf = b""
        self.turn = 0
        self.phase: str | None = None
        self.tool_calls = 0
        self.tool_errors = 0
        self.cost_usd = 0.0
        self.input_tokens = 0
        self.output_tokens = 0
        self.step: int | None = None
        self.steps_left: int | None = None
        self.llm_errors = 0
        self.end_reason: str | None = None
        self.last_event_t: float | None = None
        self.running: tuple[str, float] | None = None
        self.done: list[dict] = []
        self._cur: dict = {}

    def _fresh(self, turn: int) -> dict:
        return {"turn": turn, "tools": Counter(), "tool_s": 0.0, "errors": 0, "error": "",
                "model_s": 0.0}

    def poll(self) -> bool:
        """Read what the agent has appended since the last poll; whether any event was new."""
        try:
            with open(self.path, "rb") as f:
                f.seek(self.pos)
                chunk = f.read(READ_MAX)
        except OSError:
            return False
        if not chunk:
            return False
        self.pos += len(chunk)
        *lines, self.buf = (self.buf + chunk).split(b"\n")
        if len(self.buf) > LINE_MAX:
            self.buf = b""
        new = False
        for raw in lines:
            if raw.strip():
                new |= self._apply(raw)
        return new

    def _apply(self, raw: bytes) -> bool:
        try:
            ev = json.loads(raw)
        except ValueError:
            return False
        if not isinstance(ev, dict) or not isinstance(ev.get("kind"), str):
            return False
        kind, d = ev["kind"], ev.get("data")
        d = d if isinstance(d, dict) else {}
        t = _num(ev.get("t"))
        if t:
            self.last_event_t = t
        cur = self._cur or self._fresh(self.turn)
        if kind == "agent.turn.begin":
            self.turn, self.phase = int(_num(d.get("turn"))), d.get("phase")
            cur = self._fresh(self.turn)
        elif kind == "agent.tool.call":
            name = d.get("name") if isinstance(d.get("name"), str) else "?"
            self.tool_calls += 1
            cur["tools"][name] += 1
            self.running = (name, t)
        elif kind == "agent.tool.result":
            self.running = None
            cur["tool_s"] += _num(d.get("duration_s"))
            if d.get("is_error"):
                self.tool_errors += 1
                cur["errors"] += 1
                cur["error"] = _first_line(d.get("error") or d.get("stderr"))
            obs = d.get("obs") if isinstance(d.get("obs"), dict) else {}
            self.step = obs.get("step", self.step)
            self.steps_left = obs.get("steps_left", self.steps_left)
        elif kind == "agent.llm.usage":
            self.cost_usd += _num(d.get("cost_usd"))
            self.input_tokens += int(_num(d.get("input_tokens")))
            self.output_tokens += int(_num(d.get("output_tokens")))
            cur["model_s"] += _num(d.get("latency_s"))
        elif kind == "agent.llm.error":
            self.llm_errors += 1
            cur["error"] = cur["error"] or f"llm error {d.get('status')}"
        elif kind == "agent.turn.end":
            self.done.append({**cur, "phase": d.get("phase") or self.phase,
                              "stop_reason": d.get("stop_reason")})
            cur = {}
        elif kind == "agent.run.end":
            self.end_reason = d.get("end_reason")
        else:
            return False
        self._cur = cur
        return True


def turn_line(info: dict, tail: TurnTail, t: dict) -> str:
    tools = " ".join(f"{k}x{n}" if n > 1 else k for k, n in t["tools"].items()) or "no tool"
    steps = "" if tail.step is None else f" step {tail.step}"
    bad = f" ERR {t['errors']}: {t['error']}" if t["errors"] else ""
    return (f"episode {info.get('task')} #{info.get('episode')} turn {t['turn']} ({t['phase']}): "
            f"{tools} model {t['model_s']:.0f}s tools {t['tool_s']:.0f}s{steps} "
            f"cost ${tail.cost_usd:.2f}{bad}")


def still_line(info: dict, tail: TurnTail, now: float, wall_s) -> str:
    since = "never" if tail.last_event_t is None else f"{max(0.0, now - tail.last_event_t):.0f}s"
    if tail.running is not None:
        doing = f"running {tail.running[0]} for {max(0.0, now - tail.running[1]):.0f}s"
    elif tail.turn == 0:
        doing = "waiting for the first turn"
    else:
        doing = f"waiting on the model (turn {tail.turn})"
    return (f"episode {info.get('task')} #{info.get('episode')} alive: {doing}; last agent event "
            f"{since} ago; turns {tail.turn} tools {tail.tool_calls} (errors {tail.tool_errors}) "
            f"cost ${tail.cost_usd:.2f} wall {wall_s}s")


class EpisodeProgress:
    """Logs each finished turn, and a still-running line when nothing else was logged for
    `heartbeat_s`."""

    def __init__(self, ep_dir: Path, info: dict, *, every_s: float, heartbeat_s: float,
                 clock=time.time):
        self.tail = TurnTail(ep_dir)
        self.info, self.every_s, self.heartbeat_s, self.clock = info, every_s, heartbeat_s, clock
        self.last_poll = self.last_log = 0.0

    def tick(self, status: dict | None = None, *, final: bool = False) -> None:
        now = self.clock()
        if self.every_s <= 0 or (not final and now - self.last_poll < self.every_s):
            return
        self.last_poll = now
        try:
            self.tail.poll()
            for t in self.tail.done:
                log.info(turn_line(self.info, self.tail, t))
                self.last_log = now
            self.tail.done.clear()
            if final:
                log.info("episode %s #%s agent ended (%s): turns %d tools %d (errors %d) "
                         "cost $%.2f", self.info.get("task"), self.info.get("episode"),
                         self.tail.end_reason or "no run.end", self.tail.turn,
                         self.tail.tool_calls, self.tail.tool_errors, self.tail.cost_usd)
            elif now - self.last_log >= self.heartbeat_s:
                self.last_log = now
                log.info(still_line(self.info, self.tail, now, (status or {}).get("wall_s")))
        except Exception as e:  # noqa: BLE001
            log.debug("episode progress: %r", e)
