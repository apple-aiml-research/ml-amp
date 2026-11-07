#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What a plan shows: one combined trajectory of its planned prefix, and the plan JSON."""

from __future__ import annotations

import numpy as np

from amp.planning.assist.steps import PlannedStep, pose7_json
from amp.planning.trajectory import Issue, PlanResult

_CONCAT = ("actions", "q", "tcp_planned", "tcp_fk", "gripper", "links", "pos_err", "rot_err",
           "vel_ratio")


def layout(steps: list[PlannedStep]) -> tuple[list[PlannedStep], list[tuple[int, int]]]:
    """(steps in the combined preview, their (start, end)); stops after the first bad step."""
    included, ranges, n = [], [], 0
    for s in steps:
        if s.result is None:
            break
        m = len(s.result.t)
        included.append(s)
        ranges.append((n, n + m - 1))
        n += m
        if not s.ok:
            break
    return included, ranges


def planner_summary(planners: list[dict]) -> dict:
    """The steps' planner infos as one: backend, method, totals and the tightest clearance."""
    methods = [p.get("method") for p in planners if p.get("method")]
    clear = [p["min_clearance_m"] for p in planners if p.get("min_clearance_m") is not None]
    return {
        "backend": "builtin",
        "method": "rrt" if "rrt" in methods else (methods[0] if methods else "direct"),
        "time_s": round(sum(float(p.get("time_s") or 0.0) for p in planners), 3),
        "checks": int(sum(int(p.get("checks") or 0) for p in planners)),
        "min_clearance_m": min(clear) if clear else None,
        "message": "; ".join(p["message"] for p in planners if p.get("message")),
    }


def preview(steps: list[PlannedStep], dt: float) -> dict | None:
    """Combined preview JSON (kind "assist") of the planned prefix; None if nothing is planned."""
    included, ranges = layout(steps)
    if not included:
        return None
    dt = float(dt)
    for s in included:
        if abs(float(s.result.dt) - dt) > 1e-9:
            raise ValueError(f"step {s.spec.id} was planned at dt {s.result.dt}, not {dt}")
    rs = [s.result for s in included]
    cat = {k: np.concatenate([np.asarray(getattr(r, k)) for r in rs]) for k in _CONCAT}
    n_total = len(cat["q"])
    issues, segments, planners = [], [], []
    for k, (s, (a, _)) in enumerate(zip(included, ranges, strict=True)):
        label = f"{k + 1} {s.spec.kind} {s.spec.object}"
        for i in s.result.issues:
            step = int(i.step) + a
            issues.append(Issue(step, (step + 1) * dt, i.kind, i.severity,
                                f"step {label}: {i.message}"))
        for sg in s.result.extra.get("segments", []):
            segments.append({"name": f"{label}: {sg['name']}", "start": sg["start"] + a,
                             "end": sg["end"] + a})
        planners.append(s.result.extra.get("planner") or {})
    nxt = len(included)
    if nxt < len(steps) and included[-1].ok and not steps[nxt].ok:
        s = steps[nxt]
        issues.append(Issue(n_total - 1, n_total * dt, "infeasible", "error",
                            f"step {nxt + 1} {s.spec.kind} {s.spec.object}: {s.message}"))
    first = rs[0]
    result = PlanResult(
        ok=not any(i.severity == "error" for i in issues),
        kind="assist", dt=dt, t=(np.arange(n_total) + 1) * dt, actions=cat["actions"],
        q=cat["q"], tcp_planned=cat["tcp_planned"], tcp_fk=cat["tcp_fk"],
        gripper=cat["gripper"], links=cat["links"], pos_err=cat["pos_err"],
        rot_err=cat["rot_err"], vel_ratio=cat["vel_ratio"], issues=issues,
        min_duration=float(sum(r.min_duration for r in rs)), start_q=first.start_q,
        start_tcp=first.start_tcp, start_gripper=first.start_gripper,
        settle_steps=int(rs[-1].settle_steps),
        extra={"segments": segments, "planner": planner_summary(planners)},
    )
    return result.to_json()


def _issue_json(i, offset, dt) -> dict:
    step = int(i.step) + offset
    return {"step": step, "t": round((step + 1) * dt, 6), "kind": i.kind,
            "severity": i.severity, "message": i.message}


def step_json(s: PlannedStep, where: tuple[int, int] | tuple[None, None]) -> dict:
    """One step's JSON; where is its (start, end) in the preview, or (None, None)."""
    spec, res = s.spec, s.result
    a, b = where
    return {
        "id": spec.id, "kind": spec.kind, "object": spec.object, "target": spec.target,
        "point": None if spec.point is None else [float(v) for v in spec.point],
        "tcp": None if s.tcp is None else pose7_json(s.tcp),
        "ok": bool(s.ok), "message": s.message,
        "issues": [] if res is None else [_issue_json(i, a, res.dt) for i in res.issues],
        "planner": {} if res is None else dict(res.extra.get("planner") or {}),
        "variant": None if res is None else res.extra.get("variant"),
        "start": a, "end": b,
        "segments": [] if res is None else [
            {"name": sg["name"], "start": sg["start"] + a, "end": sg["end"] + a}
            for sg in res.extra.get("segments", [])
        ],
        "after": None if s.after is None else s.after.to_json(),
    }


def plan_json(steps: list[PlannedStep], *, version: int) -> dict:
    """The plan JSON without preview and draft."""
    included, ranges = layout(steps)
    where = {id(s): r for s, r in zip(included, ranges, strict=True)}
    return {"ok": True, "version": version, "valid": all(s.ok for s in steps),
            "steps": [step_json(s, where.get(id(s), (None, None))) for s in steps]}
