#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""L1 of the assist layers on the manual SDK (_manual, unchanged): planned collision-free moves of
the TCP, and execute of an assist plan."""

from __future__ import annotations

import re
from collections.abc import Mapping

import numpy as np

from .._manual import _trace, mobile
from .._manual import client as _client
from .._manual.client import HarnessError, _tcp, explain

CALLS = {"plan_move": "plan", "execute": "execute", "execute_both": "execute", "run": "execute",
         "move": "execute"}


def _trace_calls(cls, calls: dict) -> None:
    """Trace the methods cls itself defines, by CALLS kind."""
    for name, kind in calls.items():
        setattr(cls, name, _trace.traced(cls.__dict__[name], kind))


def _register_outcome(hook) -> None:
    """Add a trace outcome hook once: a re-imported layer replaces its earlier hook in place."""
    key = (hook.__module__, hook.__qualname__)
    for i, h in enumerate(_trace.OUTCOMES):
        if (getattr(h, "__module__", None), getattr(h, "__qualname__", None)) == key:
            _trace.OUTCOMES[i] = hook
            return
    _trace.OUTCOMES.append(hook)


def _register_failure(pattern: str, code: str, before: str) -> None:
    """Add a failure code once, ahead of the code `before` (or last)."""
    if any(c == code for _, c in _trace.FAILURES):
        return
    i = next((i for i, (_, c) in enumerate(_trace.FAILURES) if c == before), len(_trace.FAILURES))
    _trace.FAILURES.insert(i, (re.compile(pattern, re.IGNORECASE), code))


# The manual's checks and plans never name the table (table_obstacle false); assist plans still do.
_register_failure(r"below the table", "BELOW_TABLE", "TOO_WIDE")


class Episode(mobile.Episode):
    """The manual Episode plus planned collision-free moves."""

    _ee_routes = _client.EE_ROUTES | {"api/assist/plan"}

    def _plan_kind(self, body: dict, options=None, **params) -> dict:
        """Plan JSON; params (height_m, retreat_m) go top level and into options."""
        opts = dict(options or {})
        for k, v in params.items():
            if v is not None:
                body[k] = opts[k] = float(v)
        if opts:
            body["options"] = opts
        return self._post("api/assist/plan", body, check=False)

    def plan_move(self, tcp, options=None) -> dict:
        """Collision-free transit of the TCP to a pose, holding what is held (checked with it)."""
        return self._plan_kind({"kind": "move", "tcp": _tcp(tcp)}, options)

    def run(self, preview, wait=True, timeout_s=300) -> dict:
        """Execute an assist preview, or raise HarnessError naming its error issues."""
        if not preview.get("ok") or preview.get("plan_id") is None:
            raise HarnessError(explain(preview, preview.get("message") or "the plan is not valid"),
                               None, "api/assist/plan", preview)
        r = self.execute(int(preview["plan_id"]), wait=wait, timeout_s=timeout_s)
        return {**r, "plan": preview}

    def move(self, tcp, options=None, wait=True) -> dict:
        """plan_move + run: a collision-free move to tcp while holding."""
        return self.run(self.plan_move(tcp, options), wait)

    def execute(self, plan_or_plan_id, wait=True, timeout_s=300) -> dict:
        """Run a plan: an assist plan_id / assist preview, or a manual preview, waypoints / Plan
        JSON. With wait, returns after the motion with the reply's "observation" and
        "last_chunk" filled in."""
        p = plan_or_plan_id
        if isinstance(p, (int, np.integer)):
            r = self._post("api/assist/execute", {"plan_id": int(p)})
        elif isinstance(p, dict) and p.get("plan_id") is not None and "waypoints" not in p:
            r = self._post("api/assist/execute", {"plan_id": int(p["plan_id"])})
        else:
            return super().execute(p, wait=wait, timeout_s=timeout_s)
        return self._after_execute(r, wait, timeout_s)

    def execute_both(self, plan, wait=True, timeout_s=300) -> dict:
        """execute() of a coordinated move: {"ee_waypoints": {ee: [...]}, "interp"} (or just the
        {ee: [...]} dict); an assist plan_id or a one-arm preview is run as in execute()."""
        if isinstance(plan, (int, np.integer)) or isinstance(plan, Mapping) and "plan_id" in plan:
            return self.execute(plan, wait=wait, timeout_s=timeout_s)
        return super().execute_both(plan, wait=wait, timeout_s=timeout_s)


_trace_calls(Episode, CALLS)


def _outcome(result, args: dict | None = None) -> dict | None:
    """The trace outcome of plan and verify replies: their plan_id, why and wrist points."""
    jsonable = _trace.jsonable
    if not isinstance(result, dict) or not {"plan_id", "plan", "why", "wrist_points"} & set(result):
        return None
    out = _trace.base_outcome(result, args)
    out.update({k: jsonable(result[k]) for k in ("plan_id", "why", "wrist_points") if k in result})
    plan = result.get("plan")
    if isinstance(plan, dict):
        out["plan"] = {k: jsonable(plan.get(k)) for k in ("ok", "plan_id", "n_steps")}
    return out


_register_outcome(_outcome)


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = mobile.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
