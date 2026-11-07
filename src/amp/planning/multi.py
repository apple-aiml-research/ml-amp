#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Coordinated chunks for several arms: per-arm IK on one time grid, arm checked against arm."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from amp.geometry.collision.checker import CLEARANCE_CAP, moving_spheres
from amp.geometry.collision.peers import BODY, PeerSet, own_spheres, pair_path_clearance
from amp.geometry.kinematics import pose7, quat_to_matrix
from amp.planning.robot import check_tool
from amp.planning.trajectory import (
    Issue,
    PlanResult,
    Waypoint,
    finish,
    step_range,
    waypoint_path,
)

SCHEDULES = ("auto", "simultaneous", "serial")
PEER_MARGIN_M = 0.01
PEER_KIND = "peer_collision"
PEER_START_TOL_M = 0.005
SELF_KIND = "self_collision"


@dataclass
class ArmPlan:
    """One end effector's waypoints; empty = hold its arm at the start (hold) q."""

    ee: str
    arm: str | None = None
    waypoints: list[Waypoint] = field(default_factory=list)


@dataclass
class MultiPlanResult(PlanResult):
    """Every arm's plan on one time grid: q is full, the TCP and gripper fields the lead ee's."""

    arms: dict[str, PlanResult] = field(default_factory=dict)
    q_full: np.ndarray | None = None
    grip_by_ee: dict[str, np.ndarray] = field(default_factory=dict)
    tcp_planned_by_ee: dict[str, np.ndarray] = field(default_factory=dict)
    schedule: str = "simultaneous"
    composed: bool = False

    def to_json(self) -> dict:
        out = super().to_json()
        out["ee"] = self.ee
        out["schedule"] = self.schedule
        out["arms"] = {ee: r.to_json() for ee, r in self.arms.items()}
        return out


@dataclass
class _Path:
    ee: str
    arm: str
    tool: object
    t: np.ndarray
    tcp: np.ndarray
    gripper: np.ndarray
    q: np.ndarray
    pos_err: np.ndarray
    rot_err: np.ndarray


def _starts(base, q_start_by_arm) -> dict[str, np.ndarray]:
    if not isinstance(q_start_by_arm, Mapping):
        q_start_by_arm = base.split(q_start_by_arm)
    out = {}
    for a in base.spec.arms:
        if a.name not in q_start_by_arm:
            raise ValueError(f"{base.name}: no start q for arm {a.name}")
        q = np.asarray(q_start_by_arm[a.name], dtype=np.float64).reshape(-1)
        if len(q) != len(a.joints):
            raise ValueError(f"arm {a.name}: start q has {len(q)} values, not {len(a.joints)}")
        out[a.name] = q
    return out


def _moving(base, plans: Sequence[ArmPlan]) -> list[ArmPlan]:
    seen: dict[str, str] = {}
    out = []
    for p in plans:
        if p.ee not in base.end_effectors:
            raise ValueError(f"{base.name}: unknown end effector {p.ee!r}")
        arm = base.arm_of_ee(p.ee)
        if p.arm is not None and p.arm != arm:
            raise ValueError(f"end effector {p.ee} is on arm {arm}, not {p.arm}")
        if arm in seen:
            raise ValueError(f"arm {arm} has two plans ({seen[arm]}, {p.ee})")
        seen[arm] = p.ee
        if p.waypoints:
            out.append(p)
    if not out:
        raise ValueError("no end effector has waypoints: hold instead")
    return out


def _held(path: _Path, n: int, before: int, q0, tcp0, g0) -> tuple:
    """path's rows placed at [before, before + len), holding its start before, its end after."""
    m = len(path.q)
    after = n - before - m

    def pad(first, rows, last):
        return np.concatenate([np.repeat(first[None], before, axis=0), rows,
                               np.repeat(last[None], after, axis=0)])

    return (pad(q0, path.q, path.q[-1]), pad(tcp0, path.tcp, path.tcp[-1]),
            pad(np.float64(g0), path.gripper, path.gripper[-1]),
            pad(np.float64(0.0), path.pos_err, path.pos_err[-1]),
            pad(np.float64(0.0), path.rot_err, path.rot_err[-1]))


def _arm_results(base, paths, offsets, n, q0, g0, tools, dt, kw) -> dict[str, PlanResult]:
    """Every ee's validated plan over n steps: paths at their offsets, idle arms at q0."""
    t = (np.arange(n) + 1) * dt
    out = {}
    by_ee = {p.ee: p for p in paths}
    for e in base.spec.end_effectors:
        tool, q_s = tools[e.name], q0[e.arm]
        tcp0 = pose7(tool.fk_tcp(q_s))
        p = by_ee.get(e.name)
        if p is None:
            mover = next((x for x in paths if x.arm == e.arm), None)
            q = np.tile(q_s, (n, 1)) if mover is None else _held(
                mover, n, offsets[mover.ee], q_s, tcp0, g0[e.name])[0]
            tcp = np.array([pose7(tool.fk_tcp(qi)) for qi in q])
            out[e.name] = finish("hold", dt, t, q, tcp, np.full(n, g0[e.name]), tool, q_s,
                                 g0[e.name], speed_scale=kw["speed_scale"], check_table=False,
                                 settle_s=kw["settle_s"])
        else:
            q, tcp, grip, pe, re = _held(p, n, offsets[p.ee], q_s, tcp0, g0[e.name])
            out[e.name] = finish("plan", dt, t, q, tcp, grip, tool, q_s, g0[e.name], pos_err=pe,
                                 rot_err=re, speed_scale=kw["speed_scale"],
                                 table_z=kw["table_z"], check_table=kw["check_table"],
                                 settle_s=kw["settle_s"])
        out[e.name].ee = e.name
    return out


def _arm_rows(base, results) -> dict[str, np.ndarray]:
    """Each arm's q rows (n, arm.n), from the plan of its first end effector."""
    out = {}
    for e in base.spec.end_effectors:
        out.setdefault(e.arm, results[e.name].q)
    return out


def _peer_contact(base, a, b, qa, qb, margin) -> tuple[str, str, float]:
    """(arm a's link, arm b's "<arm>:<link>", clearance) of the closest pair at one step."""
    sa, sb = base.spheres(a), base.spheres(b)
    ma, mb = moving_spheres(sa), own_spheres(sb, base.arm(b), base)
    Pa = sa.world_batch(np.asarray(qa)[None])[0][ma]
    Pb = sb.world_batch(np.asarray(qb)[None])[0][mb]
    peers = PeerSet(Pb, sb.radius[mb], tuple(f"{b}:{sb.link_names[i]}" for i in sb.link[mb]))
    clr = peers.clearance(Pa, sa.radius[ma] + margin, 0, CLEARANCE_CAP)
    j = int(np.argmin(clr))
    return (f"{a}:{sa.link_names[sa.link[ma][j]]}", peers.link_at(Pa[j], 0), float(clr[j]))


def _bad_runs(mask) -> list[tuple[int, int]]:
    """(first, last) of each run of True."""
    d = np.diff(np.concatenate([[0], np.asarray(mask, dtype=np.int8), [0]]))
    return list(zip(np.flatnonzero(d == 1).tolist(), (np.flatnonzero(d == -1) - 1).tolist()))


def _peer_issues(base, rows, moving_arms, t, margin) -> list[Issue]:
    """peer_collision errors per run of steps where two arms come within margin of each other."""
    issues = []
    names = list(rows)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if a not in moving_arms and b not in moving_arms:
                continue
            ab = pair_path_clearance(base, rows[a], rows[b], a, b, margin)[0]
            ba = pair_path_clearance(base, rows[b], rows[a], b, a, margin)[0]
            issues += _run_issues(base, {a: rows[a], b: rows[b]}, (a, b), ab, ba,
                                  np.minimum(ab, ba) < 0, t, margin)
    return issues


def _run_issues(base, rows, pair, ab, ba, bad, t, margin) -> list[Issue]:
    """One peer_collision error per run of bad steps of arms pair, at its closest step."""
    a, b = pair
    clr = np.minimum(ab, ba)
    out = []
    for lo, hi in _bad_runs(bad):
        k = lo + int(np.argmin(clr[lo:hi + 1]))
        x, y = (a, b) if ab[k] <= ba[k] else (b, a)
        lx, ly, c = _peer_contact(base, x, y, rows[x][k], rows[y][k], margin)
        out.append(Issue(lo, float(t[lo]), PEER_KIND, "error", (
            f"{lx} hits {ly} at {step_range(lo, hi)} (clearance {c * 1e3:.0f} mm "
            f"at step {k}, margin {margin * 1e3:.0f} mm)")))
    return out


def peer_path_issues(embodiment, arm: str, q_start, q, peer_q, t, margin: float = PEER_MARGIN_M,
                     tol: float = PEER_START_TOL_M) -> list[Issue]:
    """peer_collision errors of arm's path q_start -> q against every other arm held at peer_q."""
    base = getattr(embodiment, "base", embodiment)
    qa = np.vstack([np.asarray(q_start, dtype=np.float64).reshape(1, -1),
                    np.asarray(q, dtype=np.float64).reshape(len(t), -1)])
    held = _starts(base, peer_q)
    issues = []
    for b in (x.name for x in base.spec.arms if x.name != arm):
        qb = np.tile(held[b], (len(qa), 1))
        ab = pair_path_clearance(base, qa, qb, arm, b, margin)[0]
        ba = pair_path_clearance(base, qb, qa, b, arm, margin)[0]
        clr = np.minimum(ab, ba)
        bad = clr[1:] < 0
        if clr[0] < 0:
            sep = np.flatnonzero(~bad)
            end = int(sep[0]) if len(sep) else len(bad)
            bad[:end] = clr[1:end + 1] < clr[0] - tol
        issues += _run_issues(base, {arm: qa[1:], b: qb[1:]}, (arm, b), ab[1:], ba[1:], bad,
                              np.asarray(t, dtype=np.float64), margin)
    return issues


def body_path_issues(embodiment, arm: str, q_start, q, t, margin: float = PEER_MARGIN_M,
                     tol: float = PEER_START_TOL_M) -> list[Issue]:
    """self_collision errors of arm's path against the robot's own body at the held q."""
    base = getattr(embodiment, "base", embodiment)
    if not getattr(base, "whole_body", False):
        return []
    C, R, names, mount = base.body_peer(arm)
    sa = base.spheres(arm)
    ma = moving_spheres(sa)
    qa = np.vstack([np.asarray(q_start, dtype=np.float64).reshape(1, -1),
                    np.asarray(q, dtype=np.float64).reshape(len(t), -1)])
    if not len(R) or not ma.any():
        return []
    body = PeerSet(C, R, tuple(f"{BODY}:{n}" for n in names), mount=mount)
    Pa, ra = sa.world_batch(qa)[:, ma], sa.radius[ma] + float(margin)
    links = np.array([sa.link_names[i] for i in sa.link[ma]], dtype=object)
    per = np.empty(Pa.shape[:2])
    for lk in np.unique(links):
        cols = np.flatnonzero(links == lk)
        per[:, cols] = body.clearance(Pa[:, cols].reshape(-1, 3), np.tile(ra[cols], len(Pa)), 0,
                                      CLEARANCE_CAP, link=lk).reshape(len(Pa), len(cols))
    names_l = np.unique(links)
    L = np.stack([per[:, links == lk].min(axis=1) for lk in names_l], axis=1)
    bad, Lr, part = L[1:] < 0, L.copy(), {}
    for i in np.flatnonzero(L[0] < 0):
        bad[:, i], Lr[1:, i], part[i] = _start_contact_by_part(
            body, np.array(names), mount, Pa, ra, qa, links == names_l[i], names_l[i], tol)
    t = np.asarray(t, dtype=np.float64)
    out = []
    for lo, hi in _bad_runs(bad.any(axis=1)):
        masked = np.where(bad[lo:hi + 1], Lr[1 + lo:2 + hi], np.inf)
        kr, i = np.unravel_index(int(np.argmin(masked)), masked.shape)
        k, link = lo + int(kr), names_l[i]
        cols = np.flatnonzero(links == link)
        j = int(cols[np.argmin(per[k + 1, cols])])
        where = body if i not in part else part[i][k]
        out.append(Issue(lo, float(t[lo]), SELF_KIND, "error", (
            f"{arm}:{link} hits {where.link_at(Pa[k + 1][j], 0, link=link)} at "
            f"{step_range(lo, hi)} (clearance {Lr[k + 1, i] * 1e3:.0f} mm at step {k}, margin "
            f"{margin * 1e3:.0f} mm)")))
    return out


def _start_contact_by_part(body, names, mount, Pa, ra, qa, cols, link, tol):
    """(bad, clearance, part per step) of a link touching the body at the start, per body part."""
    n = len(qa)
    parts = list(dict.fromkeys(names.tolist()))
    sets = []
    for p in parts:
        sel = names == p
        sets.append(PeerSet(body.centres[sel], body.radii[sel],
                            tuple(n for n, k in zip(body.names, sel) if k),
                            mount={k: m[sel] for k, m in mount.items()}))
    pts, rr = Pa[:, cols].reshape(-1, 3), np.tile(ra[cols], n)
    C = np.stack([ps.clearance(pts, rr, 0, CLEARANCE_CAP, link=link).reshape(n, -1).min(axis=1)
                  for ps in sets], axis=1)
    bad = C[1:] < 0
    for q in np.flatnonzero(C[0] < 0):
        sep = np.flatnonzero(~bad[:, q])
        end = int(sep[0]) if len(sep) else len(bad)
        bad[:end, q] = C[1:end + 1, q] < C[0, q] - tol
    deepest = np.argmin(np.where(bad, C[1:], np.inf), axis=1)
    return bad.any(axis=1), np.where(bad, C[1:], np.inf).min(axis=1), [sets[d] for d in deepest]


def grip_drift(tcp_a, tcp_b, start_a, start_b) -> tuple[np.ndarray, np.ndarray]:
    """(n,) metres and radians by which b's pose in a's frame differs from its start, per step."""
    def frame(p):
        p = np.asarray(p, dtype=np.float64).reshape(-1, 7)
        R = np.array([quat_to_matrix(q) for q in p[:, 3:]])
        return R, p[:, :3]

    Ra, pa = frame(tcp_a)
    Rb, pb = frame(tcp_b)
    R0a, p0a = frame(start_a)
    R0b, p0b = frame(start_b)
    rel_p = np.einsum("nji,nj->ni", Ra, pb - pa)
    rel_R = np.einsum("nji,njk->nik", Ra, Rb)
    d_p = np.linalg.norm(rel_p - R0a[0].T @ (p0b[0] - p0a[0]), axis=1)
    twist = np.einsum("nji,jk->nik", rel_R, R0a[0].T @ R0b[0])
    cos = np.clip((np.trace(twist, axis1=1, axis2=2) - 1.0) / 2.0, -1.0, 1.0)
    return d_p, np.arccos(cos)


def _compose(base, results, issues, schedule, lead, q0, g0, dt) -> MultiPlanResult:
    ees = [e.name for e in base.spec.end_effectors]
    rows = _arm_rows(base, results)
    q_full = np.hstack([rows[a.name] for a in base.spec.arms])
    grip = {ee: results[ee].gripper.copy() for ee in ees}
    prim = results[lead]
    own = [Issue(i.step, i.t, i.kind, i.severity, f"{ee}: {i.message}")
           for ee in ees for i in results[ee].issues]
    issues = sorted([*own, *issues], key=lambda i: (i.step, i.severity != "error"))
    actions = np.empty((len(q_full), base.dof + len(ees)), dtype=np.float32)
    actions[:, :base.dof] = q_full
    for k, ee in enumerate(ees):
        actions[:, base.dof + k] = grip[ee]
    return MultiPlanResult(
        ok=not any(i.severity == "error" for i in issues), kind="plan", dt=float(dt),
        t=prim.t, actions=actions, q=q_full, tcp_planned=prim.tcp_planned, tcp_fk=prim.tcp_fk,
        gripper=prim.gripper, links=prim.links, pos_err=prim.pos_err, rot_err=prim.rot_err,
        vel_ratio=np.max([r.vel_ratio for r in results.values()], axis=0), issues=issues,
        min_duration=max(r.min_duration for r in results.values()),
        start_q=np.concatenate([q0[a.name] for a in base.spec.arms]), start_tcp=prim.start_tcp,
        start_gripper=float(g0[lead]), settle_steps=prim.settle_steps, ee=lead, arms=results,
        q_full=q_full, grip_by_ee=grip,
        tcp_planned_by_ee={ee: results[ee].tcp_planned for ee in ees}, schedule=schedule)


def _attempt(base, paths, order, q0, g0, tools, dt, kw, margin):
    """(results, peer issues, schedule) with paths simultaneous (order None) or one by one."""
    if order is None:
        n = max(len(p.q) for p in paths)
        offsets = {p.ee: 0 for p in paths}
    else:
        n, offsets = 0, {}
        for p in order:
            offsets[p.ee] = n
            n += len(p.q)
    results = _arm_results(base, paths, offsets, n, q0, g0, tools, dt, kw)
    t = (np.arange(n) + 1) * dt
    rows = {a: r[:n] for a, r in _arm_rows(base, results).items()}
    peer = _peer_issues(base, rows, {p.arm for p in paths}, t, margin)
    for a in dict.fromkeys(p.arm for p in paths):
        peer += body_path_issues(base, a, q0[a], rows[a], t, margin)
    return results, peer, "simultaneous" if order is None else "serial"


def plan_chunk_multi(q_start_by_arm, grip_start_by_ee, tools_by_ee, plans: Sequence[ArmPlan],
                     dt, *, interp="minjerk", speed_scale=0.8, table_z=0.0, check_table=True,
                     settle_s=0.0, embodiment, schedule: str = "auto",
                     margin: float = PEER_MARGIN_M) -> MultiPlanResult:
    """Every arm's chunk on one time grid, each checked against the others' planned spheres."""
    if schedule not in SCHEDULES:
        raise ValueError(f"schedule must be one of {list(SCHEDULES)}, got {schedule!r}")
    base = getattr(embodiment, "base", embodiment)
    q0 = _starts(base, q_start_by_arm)
    g0 = {}
    for e in base.spec.end_effectors:
        if e.name not in grip_start_by_ee:
            raise ValueError(f"{base.name}: no start gripper command for {e.name}")
        g0[e.name] = float(grip_start_by_ee[e.name])
    tools = {e.name: (tools_by_ee or {}).get(e.name) or base.tool(e.name)
             for e in base.spec.end_effectors}
    kw = {"speed_scale": speed_scale, "table_z": table_z, "check_table": check_table,
          "settle_s": settle_s}
    paths = []
    for p in _moving(base, plans):
        arm = check_tool(base.view(p.ee), tools[p.ee])
        t, tcp, grip, q, pe, re = waypoint_path(q0[arm.name], g0[p.ee], tools[p.ee],
                                                p.waypoints, dt, interp, arm)
        paths.append(_Path(p.ee, arm.name, tools[p.ee], t, tcp, grip, q, pe, re))

    def run(order):
        return _attempt(base, paths, order, q0, g0, tools, dt, kw, margin)

    lead = paths[0].ee
    if schedule == "serial":
        return _compose(base, *run(paths), lead, q0, g0, dt)
    results, peer, sched = run(None)
    if not peer or schedule == "simultaneous" or len(paths) < 2:
        return _compose(base, results, peer, sched, lead, q0, g0, dt)
    for order in (paths, paths[::-1]):
        res_s, peer_s, sched_s = run(order)
        if not peer_s:
            why = Issue(0, float(res_s[order[0].ee].t[0]), PEER_KIND, "warn", (
                "together the arms collide (" + "; ".join(i.message for i in peer) +
                "): planned one after another, " + ", then ".join(p.ee for p in order)))
            return _compose(base, res_s, [why], sched_s, order[0].ee, q0, g0, dt)
    return _compose(base, results, peer, sched, lead, q0, g0, dt)
