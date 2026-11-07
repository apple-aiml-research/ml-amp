#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Place candidates: release poses over a target or a table point, with their descend paths."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from amp.geometry.embodiment import Embodiment
from amp.planning import clock
from amp.planning.candidates import (
    MAX_INFEASIBLE,
    CandidateIK,
    candidate_flags,
    joint_terms,
    solve_at,
    solve_best,
    weighted_score,
)
from amp.planning.poses import (
    EZ,
    as_pose,
    as_q,
    make_frame,
    pose7_rounded,
    rotz,
    rounded,
    unique_in_order,
)
from amp.planning.robot import arm_of, check_tool
from amp.planning.scene_geom import (
    CLEAR_CAP,
    MARGIN,
    POINT_DROP,
    ROBOTIQ_BODY,
    ArmOk,
    GripperBody,
    GripperModel,
    ObjectGeom,
    Obstacles,
    SceneContext,
    gripper_contacts,
    lowest_point,
    table_note,
)

PLACE_DROP = 0.02
PLACE_YAWS_DEG = (0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 90.0, -90.0, 180.0)
PLACE_OFFSET_YAWS_DEG = (0.0, 30.0, -30.0, 90.0, -90.0)
PLACE_OFFSET_FRACS = (0.3, 0.55)
PLACE_OFFSET_RADII = (0.03, 0.06)
PLACE_APPROACH = 0.10
PLACE_REACH_MARGIN = 0.02
PLACE_INSET = 0.01
PLACE_GRID_STEP = 0.04
PLACE_COLUMN_CLEAR = 0.06
PLACE_COLUMN_GAP = 0.02
PLACE_RISE_MAX = 0.15
PLACE_SHELF_CLEAR = 0.02
PLACE_SLIDE_LIFT = 0.02
PLACE_SHORT_FRACS = (0.6, 0.3)
PLACE_OPEN_HALF = 0.07
PLACE_BACK_CLEAR = 0.05
PLACE_BACK_MIN = 0.04
PLACE_BACK_MAX = 0.15
PLACE_ON_FRAC = 0.7
PLACE_BUDGET_S = 1.0
PLACE_SWEEP_CLEAR = 0.02
PLACE_SWEEP_WEIGHT = 0.2
PLACE_SWEEP_STEPS = (0.25, 0.5, 0.75)
PLACE_KEEP_FRAC = 0.5
PLACE_KEEP_TRIES = 12
PLACE_KEEP_BUDGET = 0.25


@dataclass
class PlaceCandidate:
    """A release pose of the held object with its descend, height, score and notes."""

    tcp: np.ndarray
    object_pose: np.ndarray
    release_height: float
    score: float
    q: np.ndarray | None
    reachable: bool
    collision_free: bool
    notes: list[str] = field(default_factory=list)
    column_clear: bool = True
    object_center: np.ndarray | None = None

    @property
    def ok(self) -> bool:
        return bool(self.reachable and self.collision_free)

    def to_json(self, id: int) -> dict:
        return {
            "id": int(id),
            "tcp": pose7_rounded(self.tcp),
            "object_pose": pose7_rounded(self.object_pose),
            "object_center": None if self.object_center is None else rounded(self.object_center, 5),
            "release_height": round(float(self.release_height), 5),
            "score": round(float(self.score), 4),
            "q": None if self.q is None else rounded(self.q, 6),
            "reachable": bool(self.reachable),
            "collision_free": bool(self.collision_free),
            "ok": self.ok,
            "notes": list(self.notes),
        }


@dataclass
class _Goal:
    xy: np.ndarray | None
    z: float | None
    label: str


def _place_goal(ctx: SceneContext, target, point) -> _Goal:
    if target is not None and point is not None:
        raise ValueError("give exactly one of target or point")
    if target is not None:
        if target == ctx.name:
            raise ValueError("the target cannot be the held object")
        g = next((g for g in ctx.obst.geoms if g.name == target), None)
        if g is None:
            raise KeyError(f"unknown object {target!r}")
        return _Goal(0.5 * (g.lo[:2] + g.hi[:2]), float(g.hi[2]) + PLACE_DROP, str(target))
    if point is not None:
        p = np.asarray(point, dtype=np.float64).reshape(-1)
        if p.shape[0] < 2:
            raise ValueError("point needs at least x and y")
        return _Goal(p[:2].copy(), ctx.table_z + POINT_DROP, "the point")
    return _Goal(None, None, "")


def _support_under(H: ObjectGeom, obst: Obstacles, lowest: float, table_z: float):
    """Highest obstacle point (or the table) under the held object's footprint."""
    best, name = table_z, "the table"
    if obst.tree is None:
        return best, name
    xy = H.pts[:, :2]
    lo, hi = xy.min(0) - _UNDER_R, xy.max(0) + _UNDER_R
    idx = obst.near_xy(lo, hi)
    P = obst.pts[idx]
    m = (P[:, 2] <= lowest + 1e-3) & np.all((P[:, :2] >= lo) & (P[:, :2] <= hi), axis=1)
    m &= P[:, 2] > best
    k = _highest_covered(P[m], xy, idx[m])
    if k is not None:
        j = idx[m][k]
        best, name = float(obst.pts[j, 2]), obst.geoms[obst.owner[j]].name
    return best, name


_UNDER_R = 0.006


def _highest_covered(C: np.ndarray, xy: np.ndarray, key: np.ndarray,
                     r: float = _UNDER_R) -> int | None:
    """Position in C of its highest point (of equal ones, the least key) within r in xy of a point
    of xy, as cKDTree(xy).query(C[:, :2], distance_upper_bound=r) decides; else None."""
    if not len(C):
        return None
    o = xy.min(0) - 2.0 * r
    sure = maybe = None
    for cell, grow in ((r * (1.0 + 1e-6), True), (r / math.sqrt(2.0) * (1.0 - 1e-6), False)):
        k = np.floor((xy - o) / cell).astype(np.int64)
        n = k.max(0) + 3
        img = np.zeros(tuple(n), dtype=bool)
        img[k[:, 0], k[:, 1]] = True
        if grow:
            img[1:] |= img[:-1].copy()
            img[:-1] |= img[1:].copy()
            img[:, 1:] |= img[:, :-1].copy()
            img[:, :-1] |= img[:, 1:].copy()
        kq = np.floor((C[:, :2] - o) / cell)
        ok = np.all((kq >= 0) & (kq < n), axis=1)
        kq = kq[ok].astype(np.int64)
        hit = np.zeros(len(C), dtype=bool)
        hit[ok] = img[kq[:, 0], kq[:, 1]]
        if grow:
            maybe = hit
        else:
            sure = hit
    z = C[:, 2]
    best = None
    if sure.any():
        s = np.flatnonzero(sure)
        s = s[z[s] == z[s].max()]
        best = int(s[np.argmin(key[s])])
    u = np.flatnonzero(maybe & ~sure)
    if best is not None:
        u = u[(z[u] > z[best]) | ((z[u] == z[best]) & (key[u] < key[best]))]
    u = u[np.lexsort((key[u], -z[u]))]
    r2 = r * r
    for a in range(0, len(u), 32):
        v = u[a:a + 32]
        d2 = ((C[v, None, 0] - xy[None, :, 0]) ** 2 + (C[v, None, 1] - xy[None, :, 1]) ** 2).min(1)
        near = d2 < r2 * (1.0 - 1e-9)
        amb = ~near & (d2 <= r2 * (1.0 + 1e-9))
        stop = np.flatnonzero(near | amb)
        if len(stop):
            if amb[stop[0]]:
                break
            return int(v[stop[0]])
    else:
        return best
    d, _ = cKDTree(xy).query(C[:, :2], distance_upper_bound=r)
    under = np.flatnonzero(np.isfinite(d))
    if not len(under):
        return None
    under = under[z[under] == z[under].max()]
    return int(under[np.argmin(key[under])])


PLACE_INSIDE_WEIGHT = 0.4
PLACE_CONTENTS_WEIGHT = 0.4
PLACE_ON_TOP_M = 0.03
PLACE_NO_CLEAR = 0.5
PLACE_LOWER_MARGIN = 0.02
PLACE_ALIGN_BONUS = 0.1
_PLACE_WEIGHTS = {"keep_yaw": 0.35, "clear": 0.25, "centred": 0.15, "limits": 0.15, "seed": 0.10}


def _top_under(P: np.ndarray, obst: Obstacles, below: float = math.inf,
               skip: str | None = None) -> float | None:
    """Highest obstacle point (lower than `below`, not of `skip`) under the footprint of points
    P, or None."""
    if obst.tree is None:
        return None
    lo, hi = P[:, :2].min(0) - _UNDER_R, P[:, :2].max(0) + _UNDER_R
    idx = obst.near_xy(lo, hi)
    Q = obst.pts[idx]
    m = np.all((Q[:, :2] >= lo) & (Q[:, :2] <= hi), axis=1) & (Q[:, 2] < below)
    if skip is not None:
        m &= np.array([g.name != skip for g in obst.geoms])[obst.owner[idx]]
    k = _highest_covered(Q[m], P[:, :2], idx[m])
    return None if k is None else float(Q[m][k, 2])


def _descend_ik(T_tcp, q, tool, approach: float, arm_ok=None) -> CandidateIK | None:
    """The first failing IK (or arm collision) at the top of a descend onto T_tcp (or back out),
    or None."""
    tops = [(approach + PLACE_REACH_MARGIN) * EZ]
    back = -(approach + PLACE_REACH_MARGIN) * T_tcp[:3, 2]
    if np.linalg.norm(back - tops[0]) > 0.01:
        tops.append(back)
    for d in tops:
        T = T_tcp.copy()
        T[:3, 3] += d
        ik = solve_at(T, q, tool)
        if not ik.good or (arm_ok is not None and not arm_ok(ik.q)):
            return ik
    return None


def descend_length(T_tcp, q, tool, approach: float, arm_ok=None) -> float | None:
    """The longest of approach and its PLACE_SHORT_FRACS whose descend is reachable (and, given
    arm_ok, collision-free at its top), or None."""
    for f in (1.0,) + PLACE_SHORT_FRACS:
        if _descend_ik(T_tcp, q, tool, f * approach, arm_ok) is None:
            return f * approach
    return None


def _descend_reach(T_tcp, q, tool, approach: float) -> str | None:
    """Why the arm cannot reach the top of the descend onto T_tcp (or back out), or None."""
    ik = _descend_ik(T_tcp, q, tool, approach)
    if ik is None:
        return None
    why = ik.note("the top of the descend") or ""
    return f"the descend from {approach * 100:.0f} cm above is out of reach ({why})"


def _assess_place(
    ctx: SceneContext, T_obj0, T_obj, T_tcp, goal: _Goal, yaw: float, q_seed, tool, arm_ok,
    search: bool = False, approach: float | None = None, target: str | None = None,
) -> PlaceCandidate:
    H = ctx.tgt.moved(T_obj @ np.linalg.inv(T_obj0))
    problems = []
    lowest = float(H.pts[:, 2].min())
    if lowest - ctx.table_z < MARGIN:
        problems.append(table_note(ctx.name, lowest - ctx.table_z))
    held_clear = CLEAR_CAP
    if ctx.obst.tree is not None:
        near = ctx.obst.near_mask(H.pts)
        d = np.full(len(H.pts), np.inf)
        i = np.full(len(H.pts), len(ctx.obst.pts), dtype=np.int64)
        if near.any():
            d[near], i[near] = ctx.obst.tree.query(H.pts[near], distance_upper_bound=CLEAR_CAP)
        if np.isfinite(d).any():
            held_clear = float(d.min())
            for j in unique_in_order(ctx.obst.owner[i[d < MARGIN]].tolist()):
                problems.append(f"{ctx.name} hits {ctx.obst.geoms[j].name}")
        mid = H.samples.mean(0, keepdims=True)
        for gi in ctx.obst.solid_under(mid):
            g = ctx.obst.geoms[gi]
            if g.inside(mid)[0]:
                problems.append(f"{ctx.name} inside {g.name}")
    body = ctx.gripper.body
    grip, grip_clear = gripper_contacts(T_tcp, ctx.obst, body=body)
    problems += grip
    zmin, part = lowest_point(T_tcp, body)
    if zmin - ctx.table_z < MARGIN:
        problems.append(table_note(part, zmin - ctx.table_z))
    support, support_name = _support_under(H, ctx.obst, lowest, ctx.table_z)
    release = lowest - support
    g_target = None if target is None else next(
        (g for g in ctx.obst.geoms if g.name == target), None)
    if g_target is not None and _off_target(H, g_target, lowest):
        problems.append(f"{ctx.name} would not rest on {target}: its footprint leaves it")
    stacked = target is not None and support_name not in (target, "the table")

    ik = solve_best(T_tcp, q_seed, tool)
    q = ik.q if ik.converged else None
    reachable = bool(ik.good)
    above = column = slide = None
    path = None if approach is None or target is None else place_path(
        ctx, H, T_tcp, target, approach, approach)
    if path is not None and path.side is not None and reachable and not problems:
        slide = f"slides in and out along {_axis_name(path.side)}, under {target}"
        above = _path_reach(path, ik.q, tool)
        reachable = above is None
        if is_down(T_tcp) and abs(float(T_tcp[:3, 0] @ path.side)) > 0.7:
            column = "the fingers would drag the object out: they close along the slide"
        elif not is_down(T_tcp) and float(T_tcp[:3, 2] @ path.side) > -0.7:
            column = f"the gripper would not back out from under {target}: it does not point in"
    elif approach is not None and reachable and not problems:
        under = path is not None and math.isfinite(path.ceil)
        length = (descend_length(T_tcp, ik.q, tool, approach, arm_ok) if under
                  else None if _descend_ik(T_tcp, ik.q, tool, approach) else approach)
        if length is None:
            above = _descend_reach(T_tcp, ik.q, tool, approach)
            reachable = False
        elif length < approach:
            slide = f"descends only {length * 100:.0f} cm: from higher is out of reach"
            column = "a full-height descend is preferred"
            approach = length
        extra, who = _column_rise(H, ctx.obst, approach, goal.label)
        if column is None and extra > 0 and (extra > PLACE_RISE_MAX or _descend_reach(
                T_tcp, ik.q, tool, approach + extra) is not None):
            column = (f"{who} stands {extra * 100:.0f} cm above the start of the descend, "
                      "too high to rise over")
    arm = arm_ok is not None and q is not None and (
        not search or (reachable and not problems))
    if arm and not arm_ok(q):
        problems.append("arm collides at the release")
    problems = unique_in_order(problems)

    off = 0.0
    if goal.xy is not None:
        off = float(np.linalg.norm(0.5 * (H.lo[:2] + H.hi[:2]) - goal.xy))
    limits, seed = joint_terms(q, q_seed, arm_of(tool))
    terms = {
        "keep_yaw": 1.0 - min(1.0, abs(yaw) / math.pi),
        "clear": min(1.0, min(held_clear, grip_clear) / CLEAR_CAP),
        "centred": 1.0 - min(1.0, off / 0.03),
        "limits": limits,
        "seed": seed,
    }
    deg = math.degrees(yaw)
    notes = ["keeps the current yaw" if abs(deg) < 0.5 else f"turned {deg:+.0f} deg about z"]
    if goal.label:
        over = off < 0.005
        notes.append(f"over {goal.label}" if over else f"{off * 100:.1f} cm off {goal.label}")
    notes.append(f"release {release * 100:.1f} cm above {support_name}")
    notes += problems
    ik_note = ik.note("release")
    if ik_note:
        notes.append(ik_note)
    if slide is not None:
        notes.append(slide)
    if above is not None:
        notes.append(above)
    if stacked and column is None:
        column = f"rests on {support_name}, not on {target}"
    if column is not None:
        notes.append(column)
    return PlaceCandidate(
        tcp=T_tcp.copy(),
        object_pose=T_obj.copy(),
        release_height=float(release),
        score=weighted_score(terms, _PLACE_WEIGHTS),
        q=None if q is None else q.copy(),
        reachable=reachable,
        collision_free=not problems,
        notes=notes,
        column_clear=column is None,
        object_center=0.5 * (H.lo + H.hi),
    )


@dataclass
class _PlacePath:
    """The top of the descend and the retreat waypoints; side is the horizontal direction out of
    a shelf when the approach slides in under a level, else None (straight down)."""

    pre: np.ndarray
    out: list
    side: np.ndarray | None = None
    ceil: float = math.inf


def _axis_name(v) -> str:
    k = int(np.argmax(np.abs(v[:2])))
    return ("+" if v[k] > 0 else "-") + "xy"[k]


def _ceiling(H: ObjectGeom, g: ObjectGeom, above: float, grip_r: float) -> float:
    """Lowest horizontal surface of g above z `above` over H's footprint and the gripper."""
    lo, hi = H.lo[:2] - grip_r, H.hi[:2] + grip_r
    h = np.abs(g.normals[:, 2]) > 0.9
    P = g.samples[h]
    m = np.all((P[:, :2] >= lo) & (P[:, :2] <= hi), axis=1) & (P[:, 2] > above)
    return float(P[m, 2].min()) if m.any() else math.inf


def _open_side(g: ObjectGeom, xy) -> tuple[np.ndarray, float] | None:
    """The horizontal unit normal of g's AABB face toward the robot base (the origin), and that
    face's coordinate along it; None when the base is inside g's footprint."""
    toward = -np.asarray(xy[:2], dtype=np.float64)
    toward /= max(float(np.linalg.norm(toward)), 1e-9)
    best = None
    for k in (0, 1):
        for sgn, face in ((1.0, float(g.hi[k])), (-1.0, float(g.lo[k]))):
            if sgn * face >= 0:
                continue
            n = np.zeros(3)
            n[k] = sgn
            if best is None or n[:2] @ toward > best[0][:2] @ toward:
                best = (n, sgn * face)
    return best


def place_path(ctx: SceneContext, H: ObjectGeom, T_tcp, target, approach: float,
                retreat: float) -> _PlacePath:
    """Straight down onto T_tcp and back along the TCP axis; under a level of target that the
    gripper cannot descend past, in and out horizontally through its face toward the robot."""
    pre = T_tcp.copy()
    pre[2, 3] += approach
    down = _PlacePath(pre, [_shifted(T_tcp, -retreat * T_tcp[:3, 2])])
    g = next((g for g in ctx.obst.geoms if g.name == target), None)
    if g is None:
        return down
    b = ctx.gripper.body
    ceil = _ceiling(H, g, float(H.hi[2]) - 0.005, b.grip_r)
    down.ceil = ceil
    body = _body_top(T_tcp, b)
    if ceil >= body + approach + PLACE_SHELF_CLEAR:
        return down
    face = _open_side(g, T_tcp[:2, 3])
    room = ceil - PLACE_SHELF_CLEAR - max(body, float(H.hi[2]))
    if face is None or room < 0:
        return down
    side, at = face
    lift = min(PLACE_SLIDE_LIFT, room)
    held = at + PLACE_SHELF_CLEAR - float((H.pts @ side).min())
    grip = at + PLACE_SHELF_CLEAR + b.grip_r - float(T_tcp[:3, 3] @ side)
    pre = _shifted(T_tcp, max(held, grip) * side + lift * EZ)
    if is_down(T_tcp):
        up = _shifted(T_tcp, min(retreat, room) * EZ)
        return _PlacePath(pre, [up, _shifted(up, max(grip, 0.0) * side)], side, ceil)
    back = _shifted(T_tcp, -back_off(H, T_tcp, b) * T_tcp[:3, 2])
    up = _shifted(back, min(retreat, room) * EZ)
    rest = grip - float((back[:3, 3] - T_tcp[:3, 3]) @ side)
    return _PlacePath(pre, [back, up, _shifted(up, max(rest, 0.0) * side)], side, ceil)


def is_down(T_tcp) -> bool:
    """A top-down TCP (approach within ~18 deg of straight down), as most grasps are."""
    return float(T_tcp[2, 2]) < -0.95


def _body_top(T_tcp, b: GripperBody = ROBOTIQ_BODY) -> float:
    """Highest point of the gripper body at T_tcp."""
    if is_down(T_tcp):
        return float(T_tcp[2, 3]) + b.grip_top
    r, top = b.grip_r, b.grip_top
    c = np.array([[x, y, z] for x in (-r, r) for y in (-r, r) for z in (-top, 0.0)])
    return float(T_tcp[2, 3] + (c @ T_tcp[:3, :3].T)[:, 2].max())


def back_off(H: ObjectGeom, T_tcp, b: GripperBody = ROBOTIQ_BODY) -> float:
    """How far to back a side grasp's open fingers along -approach to clear the object."""
    P = (H.pts - T_tcp[:3, 3]) @ T_tcp[:3, :3]
    m = (np.abs(P[:, 0]) < max(PLACE_OPEN_HALF, b.pad_inner_x + MARGIN)) & (
        np.abs(P[:, 1]) < b.grip_r)
    rear = -float(P[m, 2].min()) if m.any() else 0.0
    return float(np.clip(rear + PLACE_BACK_CLEAR, PLACE_BACK_MIN, PLACE_BACK_MAX))


def _shifted(T, d) -> np.ndarray:
    out = T.copy()
    out[:3, 3] += d
    return out


def _path_reach(path: _PlacePath, q, tool) -> str | None:
    """Why the arm cannot reach the path's waypoints (seeded from the release q), or None."""
    for what, T in [("the start of the slide in", path.pre)] + [
            ("the retreat", T) for T in path.out]:
        ik = solve_at(T, q, tool)
        if not ik.good:
            return f"{what} is out of reach ({ik.note(what)})"
        q = ik.q
    return None


def _off_target(H: ObjectGeom, g: ObjectGeom, lowest: float) -> bool:
    """Whether the held object's footprint leaves g's surface: g has no sample below its centre,
    or below less than PLACE_ON_FRAC of a grid over the middle 80% of it."""
    if getattr(g, "_surf", None) is None:
        d, _ = cKDTree(g.samples).query(g.samples, k=2)
        g._surf = (cKDTree(g.samples[:, :2]), max(0.008, 2.5 * float(np.median(d[:, 1]))))
    tree, r = g._surf
    c, h = 0.5 * (H.lo[:2] + H.hi[:2]), 0.4 * (H.hi[:2] - H.lo[:2])
    t = np.linspace(-1.0, 1.0, 5)
    probes = c + np.stack(np.meshgrid(t, t), -1).reshape(-1, 2) * h
    hit = np.array([bool(idx) and bool((g.samples[idx, 2] <= lowest + 1e-3).any())
                    for idx in tree.query_ball_point(probes, r)])
    return not (hit[12] and hit.mean() >= PLACE_ON_FRAC)


def _column_rise(H: ObjectGeom, obst: Obstacles, rise: float, target: str) -> tuple[float, str]:
    """How much higher the descend must start to clear the tallest neighbour beside it, and it."""
    if obst.tree is None:
        return 0.0, ""
    lo, hi = H.lo[:2] - PLACE_COLUMN_CLEAR, H.hi[:2] + PLACE_COLUMN_CLEAR
    start = float(H.lo[2]) + rise
    idx = obst.near_xy(lo, hi)
    P = obst.pts[idx]
    m = np.all((P[:, :2] >= lo) & (P[:, :2] <= hi), axis=1)
    m &= P[:, 2] > start - PLACE_COLUMN_GAP
    z, own = P[m, 2], obst.owner[idx[m]]
    best, name = 0.0, ""
    for j in np.unique(own).tolist():
        g = obst.geoms[j]
        over = float(z[own == j].max()) + PLACE_COLUMN_GAP - start
        if g.name != target and over > best:
            best, name = over, g.name
    return best, name


def _footprint_terms(ctx: SceneContext, cand: PlaceCandidate,
                     target) -> tuple[float, list[str], float]:
    """Score, notes and the most of the held footprint over one object already in or on the
    target: the footprint inside the target's, clear of its contents."""
    g = next(g for g in ctx.obst.geoms if g.name == target)
    T = cand.object_pose @ np.linalg.inv(ctx.T_obj0)
    P = ctx.tgt.pts @ T[:3, :3].T + T[:3, 3]
    inside = np.all((P[:, :2] >= g.lo[:2]) & (P[:, :2] <= g.hi[:2]), axis=1).mean()
    adj, notes = PLACE_INSIDE_WEIGHT * (float(inside) - 1.0), []
    if inside < 0.999:
        notes.append(f"{100 * (1 - inside):.0f} % of the footprint outside {target}")
    lo, hi = P[:, :2].min(0), P[:, :2].max(0)
    worst = 0.0
    for o in ctx.obst.geoms:
        if o.name == target:
            continue
        c = 0.5 * (o.lo + o.hi)
        within = bool(np.all((c[:2] >= g.lo[:2]) & (c[:2] <= g.hi[:2])))
        on = bool(np.all(o.lo[:2] < g.hi[:2]) and np.all(o.hi[:2] > g.lo[:2])
                  and abs(float(o.lo[2] - g.hi[2])) <= PLACE_ON_TOP_M)
        if not (within or on):
            continue
        ov = np.clip(np.minimum(hi, o.hi[:2]) - np.maximum(lo, o.lo[:2]), 0.0, None)
        area = min(1.0, float(np.prod(ov)) / max(float(np.prod(hi - lo)), 1e-6))
        if area > 0.05:
            adj -= PLACE_CONTENTS_WEIGHT * area
            worst = max(worst, area)
            where = "in" if within and not on else "on"
            notes.append(f"over {o.name}, already {where} {target} ({100 * area:.0f} %)")
    return adj, notes, worst


def _long_axis_yaws(ctx: SceneContext, target, T_obj0) -> list[float]:
    """Yaws (deg) that turn a long held object along a long target's major xy axis."""
    g = next(g for g in ctx.obst.geoms if g.name == target)
    sh = ctx.shape
    if not sh.elongated:
        return []
    D = g.pts[:, :2] - g.pts[:, :2].mean(0)
    w, v = np.linalg.eigh(D.T @ D)
    if w[1] < 1.5 ** 2 * max(w[0], 1e-9):
        return []
    a_t = math.degrees(math.atan2(v[1, 1], v[0, 1]))
    a_h = math.degrees(math.atan2(sh.v1[1], sh.v1[0]))
    d = (a_t - a_h + 90.0) % 180.0 - 90.0
    return [d, d + 180.0 if d < 0 else d - 180.0]


def _sweep_terms(ctx: SceneContext, cand: PlaceCandidate, yaw: float) -> tuple[float, str | None]:
    """Penalty and note for a turn whose sweep brings the held geometry within
    PLACE_SWEEP_CLEAR of anything beside it (not what lies under it) at the release height."""
    if abs(yaw) < math.radians(0.5) or ctx.obst.tree is None:
        return 0.0, None
    T = cand.object_pose @ np.linalg.inv(ctx.T_obj0)
    P = ctx.tgt.pts @ T[:3, :3].T + T[:3, 3]
    p = cand.tcp[:3, 3]
    reach = float(np.linalg.norm(P[:, :2] - p[:2], axis=1).max()) + PLACE_SWEEP_CLEAR
    idx = ctx.obst.near_xy(p[:2] - reach, p[:2] + reach)
    band = float(P[:, 2].min()) - 0.5 * PLACE_DROP
    idx = idx[(ctx.obst.pts[idx, 2] > band) & (ctx.obst.pts[idx, 2] < float(P[:, 2].max())
                                                + PLACE_SWEEP_CLEAR)]
    if not len(idx):
        return 0.0, None
    tree = cKDTree(ctx.obst.pts[idx])
    hits, who, closest = 0, [], math.inf
    for t in PLACE_SWEEP_STEPS:
        Q = (P - p) @ rotz(-(1.0 - t) * yaw).T + p
        d, i = tree.query(Q, distance_upper_bound=PLACE_SWEEP_CLEAR)
        m = np.isfinite(d)
        if m.any():
            hits += 1
            closest = min(closest, float(d[m].min()))
            who += [ctx.obst.geoms[j].name for j in ctx.obst.owner[idx[i[m]]].tolist()]
    if not hits:
        return 0.0, None
    names = ", ".join(unique_in_order(who))
    note = (f"turning {math.degrees(yaw):+.0f} deg sweeps {ctx.name} within "
            f"{closest * 100:.1f} cm of {names}")
    return -PLACE_SWEEP_WEIGHT * hits / len(PLACE_SWEEP_STEPS), note


def propose_places(
    scene,
    held,
    T_tcp_obj,
    *,
    target=None,
    point=None,
    q_seed,
    tool,
    max_candidates=8,
    arm_ok: ArmOk | None = None,
    seed=0,
    approach=PLACE_APPROACH,
    time_budget_s: float = PLACE_BUDGET_S,
    gripper: GripperModel | None = None,
    embodiment: Embodiment | None = None,
    proposals: str = "v1",
) -> list[PlaceCandidate]:
    """Release poses for the held object over `target` or at a table `point`, best first."""
    arm = check_tool(embodiment, tool)
    ctx = SceneContext(scene, held, seed, gripper=gripper, embodiment=embodiment)
    goal = _place_goal(ctx, target, point)
    if goal.xy is None:
        raise ValueError("give exactly one of target or point")
    q_seed = as_q(q_seed, arm)
    T_tcp_obj = as_pose(T_tcp_obj)
    T_obj_tcp = np.linalg.inv(T_tcp_obj)
    T_obj0 = np.asarray(scene.objects[held].pose, dtype=np.float64)
    ctx.T_obj0 = T_obj0
    v2 = proposals == "v2"
    keep_ids = set()
    c0 = 0.5 * (ctx.tgt.lo + ctx.tgt.hi)
    feasible, infeasible, occupied = [], [], {}

    def place(xy, deg):
        yaw = math.radians(deg)
        Rz = rotz(yaw)
        P = (ctx.tgt.pts - c0) @ Rz.T + c0
        lo, hi = P.min(0), P.max(0)
        z = goal.z
        shift = np.r_[xy - 0.5 * (lo[:2] + hi[:2]), z - lo[2]]
        if target is not None:
            top = _top_under(P + shift, ctx.obst)
            z = (ctx.table_z if top is None else top) + PLACE_DROP
            if approach is None or goal.z - PLACE_DROP - z > approach - PLACE_LOWER_MARGIN:
                z = max(z, goal.z)
            shift[2] = z - lo[2]
        T_obj = make_frame(Rz, c0 - Rz @ c0 + shift) @ T_obj0
        cand = _assess_place(
            ctx, T_obj0, T_obj, T_obj @ T_obj_tcp, goal, yaw, q_seed, tool, arm_ok, search=True,
            approach=approach, target=target,
        )
        below_top = target is not None and z < goal.z - 1e-6
        if below_top and cand.ok and (cand.q is None or _descend_reach(
                cand.tcp, cand.q, tool, approach) is not None):
            cand.reachable = False
            cand.notes.append(_descend_reach(cand.tcp, cand.q, tool, approach)
                              if cand.q is not None else "no IK at the release")
        if target is not None:
            adj, notes, occupied[id(cand)] = _footprint_terms(ctx, cand, target)
            if deg in long_yaws:
                adj += PLACE_ALIGN_BONUS
                notes.append(f"along {target}'s long axis")
            cand.score += adj
            cand.notes += notes
        if v2:
            adj, note = _sweep_terms(ctx, cand, yaw)
            cand.score += adj
            if note is not None:
                cand.notes.append(note)
            if abs(deg) < 0.5:
                keep_ids.add(id(cand))
        (feasible if cand.ok else infeasible).append(cand)
        return cand

    def clear():
        return sum(c.column_clear and occupied.get(id(c), 0.0) <= PLACE_NO_CLEAR
                   for c in feasible)

    long_yaws = [] if target is None else _long_axis_yaws(ctx, target, T_obj0)
    for deg in [*long_yaws, *PLACE_YAWS_DEG]:
        place(goal.xy, deg)
    if not clear():
        until = clock.now() + time_budget_s
        for xy in _place_offsets(ctx, goal, target):
            for deg in PLACE_OFFSET_YAWS_DEG:
                place(xy, deg)
            if clear() >= 2 or clock.now() > until:
                break
    if v2 and not any(id(c) in keep_ids for c in feasible):
        _keep_yaw_search(ctx, goal, target, place, keep_ids, feasible, time_budget_s)
    max_candidates = max(1, int(max_candidates))
    if feasible:
        out = sorted(feasible, key=lambda c: (not c.column_clear, -c.score))[:max_candidates]
    else:
        infeasible.sort(key=lambda c: (-candidate_flags(c), -c.score))
        out = infeasible[: min(max_candidates, MAX_INFEASIBLE)]
    if v2:
        _with_keep_yaw(out, feasible or infeasible, keep_ids,
                       max_candidates if feasible else min(max_candidates, MAX_INFEASIBLE))
    if out and all(occupied.get(id(c), 0.0) > PLACE_NO_CLEAR for c in out):
        for c in out:
            c.notes.append(f"no clear area on {target}")
    return out


def _keep_yaw_search(ctx, goal: _Goal, target, place, keep_ids, feasible, budget: float):
    """Keep-yaw releases over the target's centre region (or about the point), nearest the
    centre first, until one is feasible."""
    if target is not None:
        g = next(g for g in ctx.obst.geoms if g.name == target)
        lim = PLACE_KEEP_FRAC * 0.5 * (g.hi[:2] - g.lo[:2])
    else:
        lim = np.full(2, max(PLACE_OFFSET_RADII))
    xs = [p for p in _place_offsets(ctx, goal, target)
          if np.all(np.abs(p - goal.xy) <= lim + 1e-9)]
    xs.sort(key=lambda p: float(np.linalg.norm(p - goal.xy)))
    until = clock.now() + PLACE_KEEP_BUDGET * budget
    for xy in xs[:PLACE_KEEP_TRIES]:
        if place(xy, 0.0).ok or clock.now() > until:
            return


def _with_keep_yaw(out: list, pool: list, keep_ids: set, limit: int) -> None:
    """Make sure out holds the best keep-yaw candidate of pool, in its last slot if full; a
    single slot keeps the top-ranked candidate."""
    if not out or limit < 2 or any(id(c) in keep_ids for c in out):
        return
    keep = [c for c in pool if id(c) in keep_ids]
    if not keep:
        return
    best = max(keep, key=lambda c: (c.column_clear, c.score))
    if len(out) >= limit:
        out[-1] = best
    else:
        out.append(best)


def _grid(lo, hi, centre) -> list[np.ndarray]:
    axes = []
    for a, b, c in zip(lo, hi, centre):
        n = math.ceil((b - a) / PLACE_GRID_STEP) + 1 if b > a else 1
        axes.append(np.linspace(a, b, n) if n > 1 else np.array([c]))
    return [np.array([x, y]) for x in axes[0] for y in axes[1]]


def _place_offsets(ctx: SceneContext, goal: _Goal, target) -> list[np.ndarray]:
    """Release xy's around the goal, nearest the robot first (on a plate or shelf top, nearest
    its middle): rings and a grid over the target's interior (inset by the held object's
    footprint), or a few cm about a point."""
    out = []
    if target is not None:
        g = next(g for g in ctx.obst.geoms if g.name == target)
        half = 0.5 * (g.hi[:2] - g.lo[:2])
        rings = [(f * half[0], f * half[1]) for f in PLACE_OFFSET_FRACS]
        foot = 0.5 * (ctx.tgt.hi[:2] - ctx.tgt.lo[:2])
        inset = foot + PLACE_INSET
        out += _grid(g.lo[:2] + inset, g.hi[:2] - inset, goal.xy)
    else:
        rings = [(r, r) for r in PLACE_OFFSET_RADII]
    for rx, ry in rings:
        for k in range(8):
            a = math.radians(45.0 * k)
            out.append(goal.xy + np.array([rx * math.cos(a), ry * math.sin(a)]))
    if target is not None:
        others = [(o.lo[:2], o.hi[:2]) for o in ctx.obst.geoms if o.name != target]
        foot = 0.5 * (ctx.tgt.hi[:2] - ctx.tgt.lo[:2])

        def overlap(p):
            return sum(float(np.prod(np.clip(np.minimum(hi, p + foot) - np.maximum(lo, p - foot),
                                             0.0, None))) for lo, hi in others)

        out.sort(key=lambda p: (overlap(p), float(np.linalg.norm(p - goal.xy))))
    else:
        out.sort(key=lambda p: float(np.linalg.norm(p)))
    kept = []
    for p in out:
        if np.linalg.norm(p - goal.xy) > 0.005 and all(
                np.linalg.norm(p - k) > 0.01 for k in kept):
            kept.append(p)
    return kept


def evaluate_place(
    scene,
    held,
    T_tcp_obj,
    T_tcp,
    *,
    target=None,
    point=None,
    q_seed,
    tool,
    arm_ok: ArmOk | None = None,
    gripper: GripperModel | None = None,
    embodiment: Embodiment | None = None,
) -> PlaceCandidate:
    """Exactly this release pose, with the grasp transform held fixed."""
    arm = check_tool(embodiment, tool)
    ctx = SceneContext(scene, held, 0, gripper=gripper, embodiment=embodiment)
    goal = _place_goal(ctx, target, point)
    T = as_pose(T_tcp)
    T_obj0 = np.asarray(scene.objects[held].pose, dtype=np.float64)
    T_obj = T @ as_pose(T_tcp_obj)
    R_d = T_obj[:3, :3] @ T_obj0[:3, :3].T
    yaw = math.atan2(R_d[1, 0] - R_d[0, 1], R_d[0, 0] + R_d[1, 1])
    return _assess_place(ctx, T_obj0, T_obj, T, goal, yaw, as_q(q_seed, arm), tool, arm_ok,
                         target=target)
