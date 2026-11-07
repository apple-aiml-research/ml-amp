#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Per-episode request accounting, the episode summary row, and the append-only ledger file."""

from __future__ import annotations

import fcntl
import json
import os
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

LEDGER_FILE = "ledger.jsonl"


def iso(t: float | None) -> str | None:
    """UTC ISO-8601 with a Z."""
    return None if t is None else time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))


@dataclass
class Counters:
    """What one episode's agent asked for: forwarded, blocked, filtered, and moves that landed."""

    requests: dict = field(default_factory=dict)
    blocked: dict = field(default_factory=dict)
    filtered: dict = field(default_factory=dict)
    executes: int = 0
    executes_inflight: int = 0

    def block(self, label: str) -> None:
        self.blocked[label] = self.blocked.get(label, 0) + 1

    def begin(self, key: str, executes: bool) -> None:
        self.requests[key] = self.requests.get(key, 0) + 1
        if executes:
            self.executes_inflight += 1

    def done(self, key: str, status: int | None, filtered: bool, executes: bool) -> None:
        if filtered:
            self.filtered[key] = self.filtered.get(key, 0) + 1
        if executes:
            self.executes_inflight -= 1
            if status is not None and status < 300:
                self.executes += 1

    @property
    def moved(self) -> bool:
        return self.executes > 0 or self.executes_inflight > 0


def episode_row(ep, record: dict | None) -> dict:
    """The summary row of an episode that is over: what the eval layer reads for the unit."""
    rec = record or {}
    c = ep.counters
    return {"episode_id": ep.episode_id, "harness": ep.harness, "code_ref": ep.code_ref,
            "run_id": ep.run_id,
            "task": ep.task,
            "episode": ep.episode, "job_id": ep.job_id, "agent": ep.agent, "worker": ep.worker,
            "claimed_at": iso(ep.claimed_at), "ready_at": iso(ep.ready_at),
            "ended_at": iso(ep.ended_at), "end": ep.end, "outcome": ep.outcome,
            "success": rec.get("success") if record else None,
            "score": rec.get("score") if record else None,
            "episode_step": rec.get("episode_step") if record else None,
            "instruction_type": rec.get("instruction_type") if record else None,
            "requests": dict(c.requests), "blocked": dict(c.blocked),
            "filtered": dict(c.filtered), "executes": c.executes, "attempted": c.moved,
            "released": ep.released and not ep.discard, "discarded": ep.discard,
            "error": ep.error, **({"sim_hung": True} if ep.sim_hung else {}),
            **({"official": official_row(o)} if (o := getattr(ep, "official", None)) else {})}


def official_row(o) -> dict:
    """An official episode's clock and blocks, times in ISO."""
    row = o.row()
    return {**row, **{k: iso(row[k]) for k in ("started_at", "in_block_at", "fired_at")}}


class Ledger:
    """<state_dir>/ledger.jsonl: one row per finished episode, appended under an flock."""

    def __init__(self, state_dir: str | Path):
        self.path = Path(state_dir) / LEDGER_FILE
        self._lock = threading.Lock()

    def append(self, row: dict) -> None:
        data = (json.dumps(row, allow_nan=False) + "\n").encode()
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            fd = os.open(self.path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o644)
            try:
                fcntl.flock(fd, fcntl.LOCK_EX)
                os.fchmod(fd, 0o644)
                os.write(fd, data)
            finally:
                os.close(fd)

    def rows(self) -> list[dict]:
        try:
            text = self.path.read_text()
        except OSError:
            return []
        out = []
        for line in text.splitlines():
            try:
                row = json.loads(line)
            except ValueError:
                continue
            if isinstance(row, dict):
                out.append(row)
        return out

    def find(self, episode_id: str) -> dict | None:
        return next((r for r in reversed(self.rows()) if r.get("episode_id") == episode_id), None)
