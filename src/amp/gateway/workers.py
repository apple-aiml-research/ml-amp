#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The slot robot servers a gateway fronts: where they are, their health, what their state says."""

from __future__ import annotations

import http.client
import json
from urllib.parse import urlsplit

STATE_TIMEOUT_S = 3.0
DOWN_AFTER_FAILURES = 3


class SlotState:
    """One robot-server GET api/state reply, read through the fields the gateway relies on."""

    def __init__(self, raw: dict | None):
        self.raw = raw if isinstance(raw, dict) else {}
        b = self.raw.get("benchmark")
        self.has_catalog = isinstance(b, dict)
        b = b if isinstance(b, dict) else {}
        self.job = b.get("job") if isinstance(b.get("job"), dict) else None
        self.pending = b.get("pending") if isinstance(b.get("pending"), dict) else None
        self.last_end = b.get("last_end") if isinstance(b.get("last_end"), dict) else {}
        self.info_mode = self.raw.get("info_mode")
        self.publishing = bool(self.raw.get("publishing"))

    @property
    def job_id(self):
        return (self.job or {}).get("job_id")

    @property
    def pending_id(self):
        return (self.pending or {}).get("job_id")

    @property
    def settled(self) -> bool:
        """No job and no pending job: a released slot may be handed out."""
        return self.job is None and self.pending is None

    @property
    def moving(self) -> bool:
        """The robot is executing or the assistive planner is working: the agent is not idle."""
        assist = self.raw.get("assist") or {}
        return self.raw.get("execution") is not None or bool(assist.get("busy"))

    @property
    def motion(self) -> bool:
        """Motion in flight for an official block: a chunk running or a steps chain executing."""
        m = self.raw.get("motion")
        if isinstance(m, dict):
            return m.get("in_flight") is True
        plan = (self.raw.get("assist") or {}).get("plan") or {}
        return self.moving or bool(plan.get("executing"))

    @property
    def features(self) -> tuple:
        """What the slot's linked backend advertised in its hello (e.g. "grade")."""
        f = (self.raw.get("link") or {}).get("features")
        return tuple(x for x in f if isinstance(x, str)) if isinstance(f, list) else ()

    def ready_for(self, job_id, task) -> bool:
        """The slot has loaded job_id of task and its first frame and perception are in."""
        return (self.job_id == job_id and (self.job or {}).get("task") == task
                and self.raw.get("mode") == "awaiting_plan"
                and (self.raw.get("frame_seq") or 0) > 0
                and self.raw.get("perception") is not None)


class Slot:
    """One slot's robot server and what the gateway last learned about it."""

    def __init__(self, name: str, url: str):
        u = urlsplit(url)
        if u.scheme != "http" or not u.hostname:
            raise ValueError(f"slot URL must be http://host:port, got {url!r}")
        self.name = name
        self.url = url
        self.host = u.hostname
        self.port = u.port or 80
        self.state = SlotState(None)
        self.answered = False
        self.failures = 0
        self.excluded: str | None = "not answering yet"
        self.orphan_job: int | None = None
        self.release_owed = False
        self.release_refusals = 0
        self.releasing = False

    @property
    def unreachable(self) -> bool:
        return not self.answered or self.failures >= DOWN_AFTER_FAILURES

    def observe(self, raw: dict | None, ok: bool) -> str | None:
        """Record one api/state poll; returns the new exclusion reason if it changed, else None."""
        if ok:
            self.state, self.failures, self.answered = SlotState(raw), 0, True
            why = None if self.state.has_catalog else "no task catalog yet"
        else:
            self.failures += 1
            why = "not answering" if self.unreachable else self.excluded
        changed = why != self.excluded
        self.excluded = why
        return (why or "serves agents") if changed else None

    def request(self, method: str, path: str, body=None, timeout: float = STATE_TIMEOUT_S,
                headers: dict | None = None):
        """(status, parsed JSON or None) for one small request to this slot's server."""
        conn = http.client.HTTPConnection(self.host, self.port, timeout=timeout)
        try:
            sent, data = dict(headers or {}), None
            if body is not None:
                data = json.dumps(body).encode()
                sent.update({"Content-Type": "application/json", "Content-Length": str(len(data))})
            conn.request(method, path, body=data, headers=sent)
            resp = conn.getresponse()
            raw = resp.read()
            try:
                return resp.status, json.loads(raw) if raw else None
            except ValueError:
                return resp.status, None
        finally:
            conn.close()

    def fetch_state(self) -> tuple[bool, dict | None]:
        try:
            status, state = self.request("GET", "/api/state")
        except (OSError, http.client.HTTPException):
            return False, None
        return status == 200 and isinstance(state, dict), state
