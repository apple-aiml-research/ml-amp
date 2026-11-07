#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Grasp candidates: proposal, closure and hold model, scoring and ranking."""

from __future__ import annotations

import functools
import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

from amp.geometry.embodiment import Embodiment
from amp.geometry.gripper import profile
from amp.planning import clock
from amp.planning.candidates import (
    MAX_INFEASIBLE,
    candidate_flags,
    joint_terms,
    solve_at,
    solve_best,
    weighted_score,
    wrist_options,
)
from amp.planning.poses import (
    EZ,
    FLIP,
    as_pose,
    as_q,
    make_frame,
    pose7_rounded,
    rotz,
    rounded,
    to_tcp_frame,
    unique_in_order,
)
from amp.planning.robot import arm_of, check_tool
from amp.planning.scene_geom import (
    CLEAR_CAP,
    MARGIN,
    MAX_WIDTH,
    MIN_WIDTH,
    PAD_Z,
    ROBOTIQ_BODY,
    ArmOk,
    GripperBody,
    GripperModel,
    ObjectShape,
    Obstacles,
    SceneContext,
    body_of,
    gripper_contacts,
    lowest_point,
    table_note,
)

STROKE = 0.085
DEDUP_POS = 0.015
DEDUP_ROT = math.radians(20.0)
TALL = 0.08
MANIP_GOOD = 0.06
TRACK_DEV = 0.5
RAISES = (0.01, 0.02, 0.03)
TILTS_DEG = (25.0, -25.0, 45.0, -45.0)
YAW_NUDGE_DEG = (7.5, -7.5)
SHELF_GAP = 0.01
SHELF_COVER = 0.3
SHELF_XY = 0.03
SHELF_SIDE_MIN_H = 0.04
TIP_CLEAR = 0.005
SQUEEZE = math.radians(2.0)
SHELF_CLEAR = 0.02
SHELF_SIDE_FRACS = (0.3, 0.5, 0.7)
SHELF_TILTS_DEG = (50.0, 35.0, 20.0, 0.0)
PLACE_CHECKS, PLACE_CHECK_S = 3, 4.0
PLACE_FAMILY_ROT, PLACE_FAMILY_POS = math.radians(20.0), 0.03
GRAVITY = 9.81
TALL_RATIO = 2.0
TOP_FRAC = 0.75
TOP_PENALTY = 0.30
SIDE_FRACS = (0.3, 0.4, 0.5)
NEAR_WEIGHT = 3.0
SLIP_LEVER_WEIGHT = 0.15
SLIP_HIGH_WEIGHT = 0.40
SLIP_TILT_WEIGHT = 0.30
SLIP_TILT_NOTE_DEG = 10.0
SLIP_WEDGE_BAND = 0.015
SLIP_WEDGE_WEIGHT = 0.15
PAD_FRICTION = 2.0
GRIP_FORCE = 4.0
HOLD_ACCEL = 3.0
HOLD_FLOOR = 1.0
_CONTACT_TOL = 0.004
_PATCH_MIN = 0.002
_SLAB_X = 0.10
_WIDTH_X = 0.15
_GAP = 0.01
_MIN_PAD_POINTS = 5
_MIN_COVERAGE = 0.15
GRASP_BUDGET_S = 1.0
_MIN_FEASIBLE = 2
_VARIANT_SEEDS = 10
GRASP_VARIANT_BUDGET_S = 0.8
_FAR_ERR, _FAR_RUN = 0.05, 12
_TLO, _THI = ROBOTIQ_BODY.tlo, ROBOTIQ_BODY.thi


@dataclass
class GraspCandidate:
    """A grasp pose with its IK, width, closure, hold margin, score and notes."""

    tcp: np.ndarray
    pregrasp: np.ndarray
    width: float
    score: float
    q: np.ndarray | None
    q_pregrasp: np.ndarray | None
    reachable: bool
    collision_free: bool
    notes: list[str] = field(default_factory=list)
    closure: float | None = None
    hold: float | None = None
    stroke: float = field(default=STROKE, repr=False)

    @property
    def approach(self) -> np.ndarray:
        return self.tcp[:3, 2].copy()

    @property
    def ok(self) -> bool:
        return bool(
            self.reachable and self.collision_free and MIN_WIDTH <= self.width <= self.stroke
        )

    def to_json(self, id: int) -> dict:
        return {
            "id": int(id),
            "tcp": pose7_rounded(self.tcp),
            "pregrasp": pose7_rounded(self.pregrasp),
            "approach": rounded(self.approach, 6),
            "width": round(float(self.width), 5),
            "score": round(float(self.score), 4),
            "q": None if self.q is None else rounded(self.q, 6),
            "q_pregrasp": None if self.q_pregrasp is None else rounded(self.q_pregrasp, 6),
            "reachable": bool(self.reachable),
            "collision_free": bool(self.collision_free),
            "ok": self.ok,
            "closure": None if self.closure is None else round(float(self.closure), 3),
            "notes": list(self.notes),
        } | ({} if self.hold is None else {"hold": round(float(self.hold), 3)})


def grasp_frame(approach, x) -> np.ndarray:
    """The rotation with z along approach and x along x (made orthogonal)."""
    z = np.asarray(approach, dtype=np.float64)
    z = z / np.linalg.norm(z)
    x = np.asarray(x, dtype=np.float64)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def _backoff(T, dist: float) -> np.ndarray:
    out = T.copy()
    out[:3, 3] -= T[:3, 2] * dist
    return out


def _target_hits(L, b: GripperBody = ROBOTIQ_BODY) -> list[int]:
    m = np.all((L >= b.tlo_min) & (L <= b.thi_max), axis=1)
    Q = L[m]
    if len(Q):
        Q = Q[(np.abs(Q[:, 0]) >= b.gap_x) | (Q[:, 2] <= b.gap_z)]
    if not len(Q):
        return []
    inside = np.all((Q[:, None] >= b.tlo[None]) & (Q[:, None] <= b.thi[None]), axis=2)
    return [int(j) for j in np.nonzero(inside.any(0))[0]]


@dataclass
class _GraspGeo:
    width: float
    offset: float
    coverage: float
    n_pad: int
    problems: list[str]
    clear: float
    theta: float = 0.0
    tip_clear: float = CLEAR_CAP
    hold: _Hold | None = None
    closed_angle: float = 1.0
    max_width: float = MAX_WIDTH
    angular: bool = True

    @property
    def closure(self) -> float:
        return self.theta / self.closed_angle

    @property
    def ok(self) -> bool:
        return (
            not self.problems
            and MIN_WIDTH <= self.width <= self.max_width
            and self.n_pad >= _MIN_PAD_POINTS
            and self.coverage >= _MIN_COVERAGE
        )


@dataclass
class _Hold:
    """How well the pads hold the object: slip and twist loads over their friction limits."""

    mass: float
    slide: float
    twist: float
    lever: float

    @property
    def need(self) -> float:
        return math.hypot(self.slide, self.twist)

    @property
    def margin(self) -> float:
        return 1.0 / max(self.need, 1e-9)

    def note(self) -> str | None:
        if self.margin >= HOLD_FLOOR:
            return None
        why = (f"twists: centre of mass {self.lever * 100:.1f} cm off the pads"
               if self.twist > self.slide else "slides")
        return (f"may slip: {self.mass:.2g} kg needs ~{self.need:.1f}x the grip of this grasp "
                f"({why})")


def _hold(T, L, width: float, offset: float, theta: float, ctx: SceneContext) -> _Hold:
    """Friction limit surface of two pad patches against gravity plus lift/carry acceleration."""
    gp = ctx.gripper.profile
    b = body_of(gp)
    z0, z1 = _band(gp, theta)
    C = _pad_cols(L, b)
    C = C[(C[:, 2] >= z0) & (C[:, 2] <= z1) & (np.abs(C[:, 1]) <= b.pad_y)]
    rho, centres = [], []
    for face in (offset - 0.5 * width, offset + 0.5 * width):
        P = C[np.abs(C[:, 0] - face) <= _CONTACT_TOL][:, 1:]
        if len(P):
            c = P.mean(0)
            centres.append(c)
            rho.append(float(np.linalg.norm(P - c, axis=1).mean()))
    yz = np.mean(centres, axis=0) if centres else np.array([0.0, 0.5 * (z0 + z1)])
    r = to_tcp_frame(T, ctx.com[None])[0] - np.r_[offset, yz]
    g = T[:3, :3].T @ np.array([0.0, 0.0, -1.0])
    load = GRAVITY + HOLD_ACCEL
    lever = float(np.hypot(r[1], r[2]))
    force = ctx.mass * (load * float(np.hypot(g[1], g[2])) + HOLD_ACCEL)
    moment = ctx.mass * (load * abs(float(r[1] * g[2] - r[2] * g[1])) + HOLD_ACCEL * lever)
    mu = PAD_FRICTION if ctx.friction is None else 0.5 * (PAD_FRICTION + ctx.friction)
    cap = 2.0 * mu * GRIP_FORCE
    patch = max(_PATCH_MIN, float(np.mean(rho)) if rho else _PATCH_MIN)
    return _Hold(ctx.mass, force / cap, moment / (cap * patch), lever)


def _clusters(xs) -> tuple[np.ndarray, np.ndarray]:
    br = np.nonzero(np.diff(xs) > _GAP)[0]
    return xs[np.r_[0, br + 1]], xs[np.r_[br, len(xs) - 1]]


def _pad_cols(L, b: GripperBody = ROBOTIQ_BODY) -> np.ndarray:
    """The points of L in the fingers' slab, at any depth."""
    return L[(np.abs(L[:, 1]) <= b.pad_y + MARGIN) & (np.abs(L[:, 0]) <= _WIDTH_X)]


def _pad_stats(L, band=PAD_Z, b: GripperBody = ROBOTIQ_BODY) -> tuple[int, float, float, float]:
    """(points between the pads, width across the fingers, its centre offset, pad coverage)."""
    return _col_stats(_pad_cols(L, b), band, b)


def _col_stats(L, band, b: GripperBody = ROBOTIQ_BODY) -> tuple[int, float, float, float]:
    z0, z1 = band
    slab = (L[:, 2] >= z0) & (L[:, 2] <= z1)
    inner = slab & (np.abs(L[:, 0]) < b.pad_inner_x)
    n = int(inner.sum())
    if n == 0:
        return 0, 0.0, 0.0, 0.0
    lo, hi = _clusters(np.sort(L[slab, 0]))
    keep = (hi > -b.pad_inner_x) & (lo < b.pad_inner_x)
    a, b = float(lo[keep].min()), float(hi[keep].max())
    cov = float(np.ptp(L[inner, 2])) / (z1 - z0)
    return n, b - a, 0.5 * (a + b), cov


def _band(gp, theta: float) -> tuple[float, float]:
    s = gp.pad_span(theta)
    return s.z_lo, s.z_hi


def _closure(gp, L) -> tuple[float, tuple[int, float, float, float]]:
    """Contact angle and the pad stats in the band the pads grip at it (which depends on it)."""
    b = body_of(gp)
    L = _pad_cols(L, b)
    stats = _col_stats(L, b.pad_z, b)
    if stats[0] == 0:
        stats = _col_stats(L, _band(gp, gp.closed_angle), b)
        if stats[0] == 0:
            return gp.closed_angle, stats
    theta = float(gp.theta_for_gap(stats[1]))
    for _ in range(6):
        stats = _col_stats(L, _band(gp, theta), b)
        if stats[0] == 0:
            return gp.closed_angle, stats
        new = float(gp.theta_for_gap(stats[1]))
        if abs(new - theta) < 1e-4:
            return new, stats
        theta = new
    return theta, stats


def _sweep_to(gp, theta: float) -> float:
    return min(theta + getattr(gp, "squeeze", SQUEEZE), gp.closed_angle)


def _finger_hull(gp, theta: float) -> np.ndarray:
    """The vertices of the convex hull of finger_points(theta): where any linear function of
    them is least."""
    return _hull_at(gp, int(np.argmin(np.abs(gp.theta - gp._clip(theta)))))


@functools.cache
def _hull_at(gp, k: int) -> np.ndarray:
    P = gp.finger_points(float(gp.theta[k]))
    try:
        return P[np.sort(ConvexHull(P).vertices)]
    except QhullError:
        return P


def finger_low(T, theta: float, gp=None) -> float:
    """Lowest world z of the fingers at theta."""
    hull = _finger_hull(profile() if gp is None else gp, theta)
    return float(T[2, 3] + (hull @ T[2, :3]).min())


def _sweep_low(gp, T, theta: float) -> float:
    """Lowest world z of the fingers over the closing sweep from open to theta."""
    return min(finger_low(T, th, gp) for th in gp.sweep_thetas(theta))


def _sweep_hits(gp, T, theta: float, obst: Obstacles) -> list[str]:
    """Other objects the fingers touch while closing from open to theta."""
    if obst.tree is None:
        return []
    W, c, rad, bc, bh = _sweep_points(gp, theta)
    R, t = T[:3, :3], T[:3, 3]
    if not np.isfinite(obst.tree.query(R @ c + t, distance_upper_bound=rad)[0]):
        return []
    cw, h = R @ bc + t, np.abs(R) @ bh + 1e-6
    idx = obst.near_xy(cw[:2] - h[:2], cw[:2] + h[:2])
    L = to_tcp_frame(T, obst.pts[idx[np.abs(obst.pts[idx, 2] - cw[2]) <= h[2]]])
    L = L[np.all(np.abs(L - bc) <= bh, axis=1)]
    if not len(L):
        return []
    lo, hi = L.min(0) - MARGIN - 1e-6, L.max(0) + MARGIN + 1e-6
    W = W[np.all((W >= lo) & (W <= hi), axis=1)]
    if not len(W):
        return []
    W = W @ R.T + t
    d, j = obst.tree.query(W, distance_upper_bound=MARGIN)
    hit = np.isfinite(d)
    owners = unique_in_order(obst.owner[j[hit]].tolist())
    return [f"closing: finger hits {obst.geoms[o].name}" for o in owners]


@functools.lru_cache(maxsize=4096)
def _sweep_points(gp, theta: float) -> tuple:
    """The (coarse) finger points over the closing sweep to theta in the TCP frame; a ball
    (centre, radius) holding every point within MARGIN of them; their box (centre, half-size)
    grown by MARGIN."""
    ths = np.clip(gp.sweep_thetas(theta), 0.0, gp.closed_angle)
    key = tuple(np.argmin(np.abs(gp.theta[None] - ths[:, None]), axis=1).tolist())
    return _sweep_of(gp, key)


@functools.lru_cache(maxsize=64)
def _sweep_of(gp, key: tuple) -> tuple:
    W = np.vstack([gp.finger_points(float(gp.theta[k]), coarse=True) for k in key])
    c = 0.5 * (W.min(0) + W.max(0))
    rad = float(np.linalg.norm(W - c, axis=1).max()) * (1 + 1e-9) + 2 * MARGIN
    return W, c, rad, c, 0.5 * (W.max(0) - W.min(0)) + MARGIN + 1e-6


def _recentre(T, P, band=PAD_Z, b: GripperBody = ROBOTIQ_BODY) -> np.ndarray:
    n, _, off, _ = _pad_stats(to_tcp_frame(T, P), band, b)
    if n:
        T = T.copy()
        T[:3, 3] += T[:3, 0] * off
    return T


def _widest_depth(Lc) -> float:
    """Depth along the approach of the object's widest section, near its middle."""
    z = Lc[:, 2]
    z0, z1 = float(z.min()), float(z.max())
    mid = 0.5 * (z0 + z1)
    if z1 - z0 < 0.008:
        return mid
    edges = np.arange(z0, z1 + 1e-9, 0.004)
    best = []
    for a in edges:
        c = a + 0.002
        if abs(c - mid) > 0.25 * (z1 - z0):
            continue
        m = (z >= a) & (z < a + 0.004)
        if m.sum() >= 2:
            best.append((float(np.ptp(Lc[m, 0])), c))
    if not best:
        return mid
    top = max(w for w, _ in best)
    return min((c for w, c in best if w >= top - 0.0015), key=lambda c: abs(c - mid))


def _table_need(gp, T, P, table_z: float) -> float:
    theta = _closure(gp, to_tcp_frame(T, P))[0]
    tips = table_z + TIP_CLEAR - _sweep_low(gp, T, _sweep_to(gp, theta))
    return max(table_z + MARGIN - lowest_point(T, body_of(gp))[0], tips)


def _fit_tcp(gp, R, p0, P, table_z: float) -> np.ndarray | None:
    """Slide a TCP through p0 so the pads, at contact, straddle the object's widest section."""
    b = body_of(gp)
    L = (P - p0) @ R
    slab = (np.abs(L[:, 1]) <= b.pad_y + MARGIN) & (np.abs(L[:, 0]) <= _SLAB_X)
    if not slab.any():
        return None
    lo, hi = _clusters(np.sort(L[slab, 0]))
    k = int(np.argmin(np.maximum(0.0, np.maximum(lo, -hi))))
    off = np.array([0.5 * (lo[k] + hi[k]), 0.0, 0.0])
    col = slab & (np.abs(L[:, 0] - off[0]) < b.pad_inner_x)
    z_star = _widest_depth(L[col])
    cap = float(L[col, 2].min()) - b.depth_limit
    theta = float(gp.theta_for_gap(np.ptp(L[col, 0])))
    T = make_frame(R, p0 + R @ off)
    for _ in range(4):
        off[2] = min(z_star - gp.pad_span(theta).z_mid, cap)
        T = make_frame(R, p0 + R @ off)
        new = _closure(gp, to_tcp_frame(T, P))[0]
        if abs(new - theta) < 1e-4:
            break
        theta = new
    T = _recentre(T, P, _band(gp, theta), b)
    if R[2, 2] < -0.3:
        for _ in range(4):
            need = _table_need(gp, T, P, table_z)
            if need <= 0:
                break
            T[:3, 3] -= R[:, 2] * (need / -R[2, 2] + 5e-4)
            T = _recentre(T, P, _band(gp, _closure(gp, to_tcp_frame(T, P))[0]), b)
    return T


def _assess_grasp(T, ctx: SceneContext, full: bool = True) -> _GraspGeo:
    """The grasp's geometry and problems; not `full` stops at the first costly check that fails."""
    gp = ctx.gripper.profile
    b = body_of(gp)
    L = to_tcp_frame(T, ctx.tgt.pts)
    theta, (n, width, offset, cov) = _closure(gp, L)
    if _pad_stats(L, b.pad_z, b)[0] == 0 and ctx.tgt.inside(T[:3, 3])[0]:
        n, width, offset, cov = 0, float(np.ptp(L[:, 0])), 0.0, 1.0
        theta = float(gp.theta_for_gap(width))
    problems = [f"{b.parts[j]} hits {ctx.name}" for j in _target_hits(L, b)]
    inside = ctx.tgt.inside(b.cen @ T[:3, :3].T + T[:3, 3])
    problems += [f"{b.parts[j]} inside {ctx.name}" for j in np.nonzero(inside)[0]]
    zmin, part = lowest_point(T, b)
    if zmin - ctx.table_z < MARGIN:
        problems.append(table_note(part, zmin - ctx.table_z))
    tip = finger_low(T, theta, gp) - ctx.table_z
    sweep = _sweep_low(gp, T, _sweep_to(gp, theta)) - ctx.table_z
    if sweep < TIP_CLEAR:
        problems.append(_tip_note(sweep))
    clear = CLEAR_CAP
    if full or not problems:
        other, clear = gripper_contacts(T, ctx.obst, body=b)
        problems += other
    if full or not problems:
        problems += _sweep_hits(gp, T, _sweep_to(gp, theta), ctx.obst)
    hold = None if ctx.mass is None or not n else _hold(T, L, width, offset, theta, ctx)
    return _GraspGeo(width, offset, cov, n, unique_in_order(problems), clear, theta, tip, hold,
                     gp.closed_angle, ctx.gripper.max_width, getattr(gp, "angular", True))


def _tip_note(clear: float) -> str:
    if clear < 0:
        return f"closing: fingertips {-clear * 1000:.0f} mm below the table"
    return (
        f"closing: fingertips only {clear * 1000:.1f} mm above the table "
        f"(need {TIP_CLEAR * 1000:.0f} mm)"
    )


def _sweep_problems(T, dist: float, ctx: SceneContext) -> list[str]:
    """Collisions along the straight approach from the pregrasp (target, others, table)."""
    b = ctx.gripper.body
    out = []
    for f in (1.0 / 3.0, 2.0 / 3.0, 1.0):
        Ts = _backoff(T, dist * f)
        out += [f"approach: {b.parts[j]} hits {ctx.name}" for j in _target_hits(
            to_tcp_frame(Ts, ctx.tgt.pts), b)]
        out += gripper_contacts(Ts, ctx.obst, prefix="approach: ", body=b)[0]
        zmin, part = lowest_point(Ts, b)
        if zmin - ctx.table_z < MARGIN:
            out.append(table_note(part, zmin - ctx.table_z, prefix="approach: "))
    return unique_in_order(out)


def _manipulability(q, tool) -> float:
    J = arm_of(tool).jacobian(q, tool.T_flange_tcp)
    return math.sqrt(max(float(np.linalg.det(J @ J.T)), 0.0))


@dataclass
class _Reach:
    """How well the arm executes a grasp: joint margins, manipulability, approach tracking."""

    limits: float = 0.5
    dexterity: float = 0.5
    track: float = 0.5


def _reach(q, q_pre, q_mid, tool) -> _Reach:
    if q is None or q_pre is None:
        return _Reach(0.0, 0.0, 0.0)
    arm = arm_of(tool)
    limits = min(joint_terms(q, q, arm)[0], joint_terms(q_pre, q_pre, arm)[0])
    dex = min(1.0, min(_manipulability(q, tool), _manipulability(q_pre, tool)) / MANIP_GOOD)
    track = 0.0
    if q_mid is not None:
        dev = float(np.max(np.abs(q_mid - 0.5 * (q + q_pre))))
        track = 1.0 - min(1.0, dev / TRACK_DEV)
    return _Reach(limits, dex, track)


_GRASP_WEIGHTS = {
    "vertical": 0.20,
    "middle": 0.15,
    "width": 0.12,
    "contact": 0.08,
    "centred": 0.05,
    "clear": 0.10,
    "limits": 0.10,
    "seed": 0.05,
    "dexterity": 0.10,
    "track": 0.05,
}


def _grasp_terms(T, geo: _GraspGeo, sh: ObjectShape, q, q_seed, reach=None,
                 arm=None) -> dict[str, float]:
    a, p = T[:3, 2], T[:3, 3]
    reach = _Reach() if reach is None else reach
    seed = 0.5 if q_seed is None else joint_terms(q, q_seed, arm)[1]
    return {
        "vertical": max(0.0, -float(a[2])),
        "middle": 1.0 - min(1.0, abs(float((p - sh.centroid) @ sh.u1)) / max(0.5 * sh.L1, 1e-3)),
        "width": float(np.clip((geo.max_width - geo.width) / 0.04, 0.0, 1.0)),
        "contact": min(1.0, geo.coverage),
        "centred": 1.0 - min(1.0, abs(geo.offset) / 0.02),
        "clear": min(1.0, geo.clear / CLEAR_CAP),
        "limits": reach.limits,
        "seed": seed,
        "dexterity": reach.dexterity,
        "track": reach.track,
    }


def _family(a) -> str:
    down = -float(a[2])
    if down > 0.94:
        return "top-down"
    if abs(down) < 0.26:
        return "side"
    return f"tilted {math.degrees(math.asin(np.clip(down, -1, 1))):.0f} deg"


def _grasp_notes(T, geo: _GraspGeo, sh: ObjectShape, stroke: float) -> list[str]:
    notes = [_family(T[:3, 2])]
    if sh.elongated:
        c = abs(float(T[:3, 0] @ sh.u1))
        if c < 0.5:
            notes.append("across the minor axis")
        elif c > 0.87:
            notes.append("along the major axis")
    d = abs(float((T[:3, 3] - sh.centroid) @ sh.u1))
    notes.append("at the middle" if d < 0.01 else f"{d * 100:.1f} cm from the middle")
    if geo.n_pad == 0 and geo.width == 0.0:
        notes.append("nothing between the fingers")
        return notes
    notes.append(f"width {geo.width * 100:.1f} cm")
    notes.append(f"closes to {geo.closure:.2f} ({math.degrees(geo.theta):.0f} deg)"
                 if geo.angular else f"closes to {geo.closure:.2f}")
    if geo.tip_clear < CLEAR_CAP:
        notes.append(f"fingertips {geo.tip_clear * 1000:.0f} mm above the table at contact")
    if geo.width > stroke:
        notes.append(f"too wide: {geo.width * 100:.1f} cm > the {stroke * 100:.1f} cm stroke")
    elif geo.width > geo.max_width:
        notes.append(f"too wide: {geo.width * 100:.1f} cm leaves under 5 mm of opening")
    elif geo.width < MIN_WIDTH:
        notes.append(f"too thin: {geo.width * 1000:.1f} mm between the pads")
    if 0 < geo.n_pad < _MIN_PAD_POINTS or (geo.n_pad and geo.coverage < _MIN_COVERAGE):
        notes.append("grazes an edge: little of the object between the pads")
    if abs(geo.offset) > 0.005:
        notes.append(f"object {abs(geo.offset) * 100:.1f} cm off centre between the pads")
    if geo.hold is not None and (warn := geo.hold.note()):
        notes.append(warn)
    return notes


def _tall_thin(ctx: SceneContext) -> bool:
    sh = ctx.shape
    v2 = np.array([-sh.v1[1], sh.v1[0], 0.0])
    narrow = min(sh.Lxy, float(np.ptp(ctx.tgt.pts[:, :2] @ v2[:2])))
    return bool(sh.zmax - sh.zmin > TALL_RATIO * max(narrow, 1e-3))


def _shape_terms(T, geo: _GraspGeo, ctx: SceneContext) -> tuple[float, list[str]]:
    """Score adjustments and notes for tall, thin objects."""
    adj, notes = 0.0, []
    sh = ctx.shape
    h = sh.zmax - sh.zmin
    if h > 1e-3 and _tall_thin(ctx):
        frac = (float(T[2, 3]) - sh.zmin) / h
        if frac > TOP_FRAC:
            adj -= TOP_PENALTY
            notes.append(f"contact in the top {100 * (1 - TOP_FRAC):.0f} % of a "
                         f"tall, thin object ({100 * frac:.0f} % of its height)")
    return adj, notes


def _contact_centre(T, geo: _GraspGeo, ctx: SceneContext) -> np.ndarray:
    """World centre of the target's points between the pads; the TCP when there are none."""
    gp = ctx.gripper.profile
    b = body_of(gp)
    z0, z1 = _band(gp, geo.theta)
    C = _pad_cols(to_tcp_frame(T, ctx.tgt.pts), b)
    C = C[(C[:, 2] >= z0) & (C[:, 2] <= z1) & (np.abs(C[:, 1]) <= b.pad_y)
          & (np.abs(C[:, 0] - geo.offset) <= 0.5 * geo.width + _CONTACT_TOL)]
    return T[:3, :3] @ C.mean(0) + T[:3, 3] if len(C) else T[:3, 3].copy()


def _slip_terms(T, geo: _GraspGeo, ctx: SceneContext) -> tuple[float, list[str]]:
    """Score adjustments and notes for slip-prone grasps, from the perceived geometry alone."""
    if geo.n_pad == 0 and geo.width == 0.0:
        return 0.0, []
    adj, notes = 0.0, []
    sh, P = ctx.shape, ctx.tgt.pts
    c = _contact_centre(T, geo, ctx)
    r = sh.centroid - c
    lever = abs(float(T[:3, 0] @ np.cross(r, (0.0, 0.0, -1.0))))
    rim = max(float(np.linalg.norm(P[:, :2] - sh.centroid[:2], axis=1).max()), 1e-3)
    frac = min(1.0, lever / rim)
    adj -= SLIP_LEVER_WEIGHT * frac
    if frac > 0.5:
        notes.append(f"contact {lever * 100:.1f} cm off the centroid: gravity twists it out")
    h = sh.zmax - sh.zmin
    narrow = min(sh.Lxy, float(np.ptp(P[:, :2] @ np.array([-sh.v1[1], sh.v1[0]]))))
    if h > TALL and h > narrow:
        up = float(np.clip((c[2] - sh.centroid[2]) / (0.5 * h), 0.0, 1.0))
        adj -= SLIP_HIGH_WEIGHT * up
        if up > 0.5:
            notes.append(f"contact high on a tall object ({100 * (c[2] - sh.zmin) / h:.0f} % "
                         "of its height): lower holds better")
    e = math.degrees(math.asin(float(np.clip(-T[2, 2], -1.0, 1.0))))
    tilt = min(abs(e), 90.0 - abs(e))
    adj -= SLIP_TILT_WEIGHT * tilt / 45.0
    if tilt > SLIP_TILT_NOTE_DEG:
        notes.append(f"approach tilted {tilt:.0f} deg off vertical/horizontal: pads may slip")
    spare = geo.max_width - geo.width
    if spare < SLIP_WEDGE_BAND:
        adj -= SLIP_WEDGE_WEIGHT * min(1.0, 1.0 - spare / SLIP_WEDGE_BAND)
        notes.append(f"will wedge, not pinch: {geo.width * 100:.1f} cm of the "
                     f"{geo.max_width * 100:.1f} cm max opening")
    return adj, notes


def _finish_grasp(
    T, geo: _GraspGeo, ctx: SceneContext, q_seed, tool, approach_dist, arm_ok, *, search: bool,
    proposals: str = "v1",
) -> tuple[GraspCandidate, float]:
    """IK, arm check, approach sweep, score and notes for one grasp pose; also its IK error."""
    if search:
        flips = [f for f, _ in wrist_options(T[:3, :3], q_seed, tool)]
    else:
        flips = [False]
    best = None
    for flip in flips:
        Tc = T @ FLIP if flip else T
        Tp = _backoff(Tc, approach_dist)
        ik = solve_best(Tc, q_seed, tool)
        ikp = solve_at(Tp, ik.q, tool)
        if not ikp.good:
            alt = solve_best(Tp, q_seed, tool)
            ikp = alt if (alt.good, -alt.pos_err) > (ikp.good, -ikp.pos_err) else ikp
        trial = (Tc, Tp, ik, ikp)
        if best is None or (ik.good and ikp.good) > (best[2].good and best[3].good):
            best = trial
        if (ik.good and ikp.good) or ik.pos_err > 0.02:
            break
    assert best is not None
    Tc, Tp, ik, ikp = best
    reachable = ik.good and ikp.good
    q = ik.q if ik.converged else None
    q_pre = ikp.q if ikp.converged else None
    problems = list(geo.problems) + _sweep_problems(Tc, approach_dist, ctx)
    q_mid = None
    if q is not None and q_pre is not None:
        ikm = solve_at(_backoff(Tc, 0.5 * approach_dist), 0.5 * (q + q_pre), tool)
        q_mid = ikm.q if ikm.converged else None
    arm = arm_ok is not None and (not search or (reachable and not problems))
    if arm and q is not None and not arm_ok(q):
        problems.append("arm collides at the grasp")
    arm = arm and (not search or not problems)
    if arm and q_pre is not None and not arm_ok(q_pre):
        problems.append("arm collides at the pregrasp")
    arm = arm and (not search or not problems)
    if arm and q_mid is not None and not arm_ok(q_mid):
        problems.append("arm collides on the approach")
    problems = unique_in_order(problems)
    notes = _grasp_notes(Tc, geo, ctx.shape, ctx.gripper.stroke) + problems
    notes += [n for n in (ik.note("grasp"), ikp.note("pregrasp")) if n]
    reach = _reach(q, q_pre, q_mid, tool)
    score = weighted_score(_grasp_terms(Tc, geo, ctx.shape, q, q_seed, reach, arm_of(tool)),
                           _GRASP_WEIGHTS)
    adj, more = _shape_terms(Tc, geo, ctx)
    score += adj
    notes += more
    if proposals == "v2":
        adj, more = _slip_terms(Tc, geo, ctx)
        score += adj
        notes += more
    cand = GraspCandidate(
        tcp=Tc,
        pregrasp=Tp,
        width=float(geo.width),
        score=score,
        q=None if q is None else q.copy(),
        q_pregrasp=None if q_pre is None else q_pre.copy(),
        reachable=bool(reachable),
        collision_free=not problems,
        notes=notes,
        closure=float(geo.closure),
        hold=None if geo.hold is None else geo.hold.margin,
        stroke=ctx.gripper.stroke,
    )
    return cand, float(ik.pos_err)


def _grasp_frames(ctx: SceneContext, far: bool = False,
                  near=None) -> list[tuple[np.ndarray, np.ndarray]]:
    """(R, p0) seeds: top-down along the long axis, middle first; side/tilted for tall objects."""
    sh, P = ctx.shape, ctx.tgt.pts
    out = []
    dirs = np.radians(np.arange(0.0, 180.0, 5.0))
    xy = P[:, :2] - sh.centroid[:2]
    base = float(dirs[np.argmin(np.ptp(xy @ np.stack([np.cos(dirs), np.sin(dirs)]), axis=0))])
    proj = (P[:, :2] - sh.centroid[:2]) @ sh.v1[:2]
    step = max(0.02, 0.12 * sh.Lxy)
    offsets = [0.0]
    for k in range(1, int(0.36 * sh.Lxy // step) + 1):
        offsets += [k * step, -k * step]
    if far:
        reach = 0.5 * sh.Lxy - 0.012
        inner = max(offsets) + 0.5 * step
        offsets = [s * t for t in np.arange(inner, reach + 1e-9, 0.012) for s in (1.0, -1.0)]
    if near is not None:
        t = float((np.asarray(near, dtype=np.float64)[:2] - sh.centroid[:2]) @ sh.v1[:2])
        lim = max(0.0, 0.5 * sh.Lxy - 0.006)
        offsets = [float(np.clip(t, -lim, lim))] + offsets
    for t in offsets:
        band = P[np.abs(proj - t) <= 0.006]
        if not len(band):
            continue
        p0 = np.r_[0.5 * (band[:, :2].min(0) + band[:, :2].max(0)), band[:, 2].max()]
        for k in range(12):
            yaw = base + math.radians(15.0 * k)
            out.append((grasp_frame((0, 0, -1), (math.cos(yaw), math.sin(yaw), 0)), p0))
    height = sh.zmax - sh.zmin
    thin = _tall_thin(ctx)
    if height > TALL or (thin and height > SHELF_SIDE_MIN_H):
        az0 = math.atan2(sh.centroid[1], sh.centroid[0])
        level = (*SIDE_FRACS, 0.65) if thin else (0.5, 0.65, 0.8)
        for dphi in (0.0, 30.0, -30.0, 60.0, -60.0):
            phi = az0 + math.radians(dphi)
            x = (-math.sin(phi), math.cos(phi), 0.0)
            for tilt, fracs in ((0.0, level), (45.0, (0.5, 0.65) if thin else (0.6, 0.8))):
                ct, st = math.cos(math.radians(tilt)), math.sin(math.radians(tilt))
                R = grasp_frame((math.cos(phi) * ct, math.sin(phi) * ct, -st), x)
                for f in fracs:
                    zl = sh.zmin + f * height
                    band = P[np.abs(P[:, 2] - zl) <= 0.01]
                    if len(band):
                        xy = 0.5 * (band[:, :2].min(0) + band[:, :2].max(0))
                        out.append((R, np.r_[xy, zl]))
    return out


def _over(ctx: SceneContext, xy, z: float, r: float | None = None) -> float:
    """The lowest obstacle surface above z within r (the gripper's radius) of xy, or inf."""
    r = ctx.gripper.body.grip_r if r is None else r
    P = ctx.obst.pts
    m = (P[:, 2] > z) & (np.linalg.norm(P[:, :2] - np.asarray(xy)[:2], axis=1) <= r)
    return float(P[m, 2].min()) if m.any() else math.inf


def _overhang_frames(ctx: SceneContext, approach: float) -> list[tuple[np.ndarray, np.ndarray]]:
    """(R, p0) seeds for a target under an overhang (a shelf board above it): top-down on the
    parts that stick out from under it, and horizontal through the side facing the robot."""
    sh, P = ctx.shape, ctx.tgt.pts
    grip_top = ctx.gripper.body.grip_top
    c = sh.centroid
    above = ctx.obst.pts
    above = above[(above[:, 2] > sh.zmax + SHELF_GAP)
                  & (above[:, 2] < sh.zmax + grip_top + approach)]
    if not len(above):
        return []
    d, _ = cKDTree(above[:, :2]).query(P[:, :2], distance_upper_bound=SHELF_XY)
    if np.mean(d <= SHELF_XY) < SHELF_COVER:
        return []
    out = []
    v1 = sh.v1
    v2 = np.array([-v1[1], v1[0], 0.0])
    proj = (P[:, :2] - c[:2]) @ v1[:2]
    for t in np.arange(proj.min() + 0.012, proj.max() - 0.012 + 1e-9, 0.012):
        band = P[np.abs(proj - t) <= 0.006]
        if not len(band):
            continue
        p0 = np.r_[0.5 * (band[:, :2].min(0) + band[:, :2].max(0)), band[:, 2].max()]
        if _over(ctx, p0, p0[2]) < p0[2] + grip_top + SHELF_CLEAR:
            continue
        for deg in (0.0, 15.0, -15.0):
            a = math.radians(deg)
            R = grasp_frame((0, 0, -1), math.cos(a) * v2 + math.sin(a) * v1)
            out += [(R @ _rotx(math.radians(t)), p0) for t in (0.0, 25.0, -25.0)]
    toward = -c[:2] / max(float(np.linalg.norm(c[:2])), 1e-9)
    k = int(np.argmax(np.abs(toward)))
    a = np.zeros(3)
    a[k] = -math.copysign(1.0, toward[k])
    side = np.cross(EZ, a)
    height = sh.zmax - sh.zmin
    across = P @ side
    for f in SHELF_SIDE_FRACS if height >= SHELF_SIDE_MIN_H else ():
        zl = sh.zmin + f * height
        for s in np.arange(across.min() + 0.01, across.max() - 0.01 + 1e-9, 0.015):
            band = P[(np.abs(P[:, 2] - zl) <= 0.012) & (np.abs(across - s) <= 0.02)]
            if not len(band):
                continue
            p0 = band.mean(0)
            for deg in SHELF_TILTS_DEG:
                t = math.radians(deg)
                out.append((grasp_frame(math.cos(t) * a - math.sin(t) * EZ, side), p0))
    return out


def _neck_frames(ctx: SceneContext) -> list[tuple[np.ndarray, np.ndarray]]:
    """(R, p0) seeds from the side, level or tilted down, across the upper part of a tall object
    where it is narrow enough to close on (a jug's neck or handle): the wrist then stays about
    as high as the grasp, for a place under a board."""
    sh, P = ctx.shape, ctx.tgt.pts
    height = sh.zmax - sh.zmin
    if height <= TALL:
        return []
    az0 = math.atan2(sh.centroid[1], sh.centroid[0])
    out = []
    for zl in np.arange(sh.zmin + 0.55 * height, sh.zmax - 0.015 + 1e-9, 0.015):
        band = P[np.abs(P[:, 2] - zl) <= 0.008]
        if len(band) < 3:
            continue
        for dphi in (0.0, 30.0, -30.0, 60.0, -60.0):
            phi = az0 + math.radians(dphi)
            x = np.array([-math.sin(phi), math.cos(phi), 0.0])
            if float(np.ptp(band @ x)) > ctx.gripper.max_width - 0.01:
                continue
            p0 = np.r_[0.5 * (band[:, :2].min(0) + band[:, :2].max(0)), zl]
            for deg in (0.0, 20.0):
                t = math.radians(deg)
                a = (math.cos(phi) * math.cos(t), math.sin(phi) * math.cos(t), -math.sin(t))
                out.append((grasp_frame(a, x), p0))
    return out


def _rotx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _variants(T, ctx: SceneContext) -> list[np.ndarray]:
    """Nearby grasps for a blocked one: higher on the object, tilted about the closing line."""
    gp = ctx.gripper.profile
    P, R, p = ctx.tgt.pts, T[:3, :3], T[:3, 3]
    out = []
    for d in RAISES:
        Tr = _backoff(T, d)
        out.append(_recentre(Tr, P, _band(gp, _closure(gp, to_tcp_frame(Tr, P))[0]),
                             body_of(gp)))
    for deg in TILTS_DEG:
        Tt = _fit_tcp(gp, R @ _rotx(math.radians(deg)), p, P, ctx.table_z)
        if Tt is not None:
            out.append(Tt)
            out.append(_backoff(Tt, RAISES[0]))
    for deg in YAW_NUDGE_DEG:
        Ty = _fit_tcp(gp, R @ rotz(math.radians(deg)), p, P, ctx.table_z)
        if Ty is not None:
            out.append(Ty)
    return out


def _rot_angle(Ra, Rb) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.acos(float(np.clip(c, -1.0, 1.0)))


def _is_dup(T, others) -> bool:
    for U in others:
        if np.linalg.norm(T[:3, 3] - U[:3, 3]) > DEDUP_POS:
            continue
        a = min(_rot_angle(T[:3, :3], U[:3, :3]), _rot_angle(T[:3, :3] @ FLIP[:3, :3], U[:3, :3]))
        if a < DEDUP_ROT:
            return True
    return False


def propose_grasps(
    scene,
    name,
    *,
    q_seed,
    tool,
    max_candidates=8,
    approach_dist=0.10,
    arm_ok: ArmOk | None = None,
    seed=0,
    near=None,
    gripper: GripperModel | None = None,
    time_budget_s: float = GRASP_BUDGET_S,
    variant_budget_s: float = GRASP_VARIANT_BUDGET_S,
    embodiment: Embodiment | None = None,
    proposals: str = "v1",
) -> list[GraspCandidate]:
    """Feasible grasps of `name`, best first; else the best few infeasible ones, flagged."""
    t0 = clock.now()
    arm = check_tool(embodiment, tool)
    ctx = SceneContext(scene, name, seed, gripper, embodiment)
    gp = ctx.gripper.profile
    near = None if near is None else np.asarray(near, dtype=np.float64).reshape(3)
    q_seed = as_q(q_seed, arm)
    max_candidates = max(1, int(max_candidates))

    def assess(Ts, full=True, until=None):
        out = []
        for T in Ts:
            if until is not None and clock.now() > until:
                break
            geo = _assess_grasp(T, ctx, full)
            pre = weighted_score(_grasp_terms(T, geo, ctx.shape, None, None), _GRASP_WEIGHTS)
            pre += _shape_terms(T, geo, ctx)[0]
            if proposals == "v2":
                pre += _slip_terms(T, geo, ctx)[0]
            if near is not None:
                pre -= NEAR_WEIGHT * float(np.linalg.norm(T[:3, 3] - near))
            out.append((geo.ok, pre, T, geo))
        out.sort(key=lambda r: (not r[0], -r[1]))
        return out

    fits = (_fit_tcp(gp, R, p0, ctx.tgt.pts, ctx.table_z)
            for R, p0 in _grasp_frames(ctx, **({} if near is None else {"near": near}))
            + _neck_frames(ctx))
    raw = assess([T for T in fits if T is not None])
    fits = (_fit_tcp(gp, R, p0, ctx.tgt.pts, ctx.table_z)
            for R, p0 in _overhang_frames(ctx, approach_dist))
    shelf = [r for r in assess([T for T in fits if T is not None]) if r[0]]

    feasible: list[GraspCandidate] = []
    infeasible: list[GraspCandidate] = []
    deadline = clock.now() + time_budget_s
    hard_stop = t0 + 4 * time_budget_s
    far_run = 0

    def consider(items, flagged=True):
        nonlocal far_run
        for good, _, T, geo in items:
            now = clock.now()
            if len(feasible) >= 2 * max_candidates:
                return
            if now > hard_stop and (feasible or infeasible):
                return
            if now > deadline and (feasible or len(infeasible) >= MAX_INFEASIBLE):
                return
            if not good and (not flagged or feasible or len(infeasible) >= 4 * MAX_INFEASIBLE):
                return
            if _is_dup(T, [c.tcp for c in feasible]):
                continue
            cand, err = _finish_grasp(
                T, geo, ctx, q_seed, tool, approach_dist, arm_ok, search=True,
                proposals=proposals,
            )
            if cand.ok and cand.width <= ctx.gripper.max_width:
                feasible.append(cand)
            elif flagged and all(np.linalg.norm(cand.tcp[:3, 3] - c.tcp[:3, 3]) > 2 * DEDUP_POS
                                 for c in infeasible):
                infeasible.append(cand)
            far_run = far_run + 1 if err > _FAR_ERR else 0
            if not feasible and far_run >= _FAR_RUN:
                return

    consider(shelf, flagged=False)
    far_run = 0
    consider([r for r in raw if r[0]])
    if len(feasible) < _MIN_FEASIBLE and far_run < _FAR_RUN:
        until = clock.now() + variant_budget_s
        deadline = max(deadline, until + time_budget_s)
        blocked = sorted(raw, key=lambda r: (r[0], len(r[3].problems), -r[1]))
        seeds: list[np.ndarray] = []
        for _, _, T, _ in blocked:
            if len(seeds) >= _VARIANT_SEEDS:
                break
            if not _is_dup(T, seeds):
                seeds.append(T)
        Ts = [V for T in seeds for V in _variants(T, ctx)]
        Ts += [V for R, p0 in _grasp_frames(ctx, far=True)
               if (V := _fit_tcp(gp, R, p0, ctx.tgt.pts, ctx.table_z)) is not None]
        consider([r for r in assess(Ts, full=False, until=until) if r[0]], flagged=False)
    if not feasible:
        consider([r for r in raw if not r[0]])
    if near is not None:
        for c in feasible + infeasible:
            d = float(np.linalg.norm(c.tcp[:3, 3] - near))
            c.score -= NEAR_WEIGHT * d
            c.notes.append(f"{d * 100:.1f} cm from the requested point")
    if feasible:
        return sorted(feasible, key=lambda c: -c.score)[:max_candidates]
    infeasible.sort(key=lambda c: (-candidate_flags(c), -c.score))
    return infeasible[: min(max_candidates, MAX_INFEASIBLE)]


def rank_for_place(cands, has_place: Callable[[GraspCandidate], bool], where: str, *,
                   checks: int = PLACE_CHECKS, budget_s: float = PLACE_CHECK_S):
    """Feasible grasps re-ranked so the first one with a feasible place at `where` leads."""
    t0, n = clock.now(), 0
    blocked: list[GraspCandidate] = []
    for c in cands:
        if not c.ok:
            break
        if any(_same_family(c, b) for b in blocked):
            c.notes.append(f"no feasible place at {where}: like a grasp that has none")
            blocked.append(c)
            continue
        if n >= checks or (n and clock.now() - t0 > budget_s):
            break
        n += 1
        if has_place(c):
            break
        c.notes.append(f"no feasible place at {where} with this grasp")
        blocked.append(c)
    out = {id(c) for c in blocked}
    return [c for c in cands if id(c) not in out] + blocked


def _same_family(a: GraspCandidate, b: GraspCandidate) -> bool:
    return (float(a.tcp[:3, 2] @ b.tcp[:3, 2]) > math.cos(PLACE_FAMILY_ROT)
            and float(np.linalg.norm(a.tcp[:3, 3] - b.tcp[:3, 3])) < PLACE_FAMILY_POS)


def evaluate_grasp(
    scene, name, T_tcp, *, q_seed, tool, approach_dist=0.10, arm_ok: ArmOk | None = None,
    gripper: GripperModel | None = None, embodiment: Embodiment | None = None,
    proposals: str = "v1",
) -> GraspCandidate:
    """Exactly this grasp pose, not re-centred: for live feedback while the human drags it."""
    arm = check_tool(embodiment, tool)
    ctx = SceneContext(scene, name, 0, gripper, embodiment)
    T = as_pose(T_tcp)
    geo = _assess_grasp(T, ctx)
    return _finish_grasp(
        T, geo, ctx, as_q(q_seed, arm), tool, approach_dist, arm_ok, search=False,
        proposals=proposals,
    )[0]


def gripper_problems(scene, name, T_tcp, *, approach_dist=0.10, gripper: GripperModel | None = None,
                     embodiment: Embodiment | None = None) -> list[str]:
    """What the grasp model finds wrong with the gripper at T_tcp and along its straight approach."""
    ctx = SceneContext(scene, name, 0, gripper=gripper, embodiment=embodiment)
    T = as_pose(T_tcp)
    return unique_in_order(_assess_grasp(T, ctx).problems + _sweep_problems(T, approach_dist, ctx))


def closure_at(scene, name, T_tcp, gripper: GripperModel | None = None,
               embodiment: Embodiment | None = None) -> dict:
    """The closing model at exactly T_tcp: contact angle, closure, width, fingertip clearances."""
    ctx = SceneContext(scene, name, 0, gripper=gripper, embodiment=embodiment)
    gp = ctx.gripper.profile
    T = as_pose(T_tcp)
    theta, (_, width, _, _) = _closure(gp, to_tcp_frame(T, ctx.tgt.pts))
    return {
        "theta": theta,
        "closure": theta / gp.closed_angle,
        "width": width,
        "tip_clear": finger_low(T, theta, gp) - ctx.table_z,
        "sweep_clear": _sweep_low(gp, T, _sweep_to(gp, theta)) - ctx.table_z,
    }


def held_geometry(name, obj, T_world_obj, T_grasp) -> dict | None:
    """A picked object's perceived mesh held fixed in the TCP frame of the planned grasp."""
    if obj is None or T_grasp is None or T_world_obj is None:
        return None
    T_world_obj = np.asarray(T_world_obj, dtype=np.float64)
    T_tcp_obj = np.linalg.inv(np.asarray(T_grasp, dtype=np.float64)) @ T_world_obj
    v = np.asarray(obj.verts, dtype=np.float64).reshape(-1, 3)
    return {"object": name, "verts": obj.verts, "faces": obj.faces, "color": obj.color,
            "T_tcp_obj": T_tcp_obj,
            "c0": (v @ T_world_obj[:3, :3].T + T_world_obj[:3, 3]).mean(0)}
