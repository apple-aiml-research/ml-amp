#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Configuration-space search: edge checks, RRT-Connect, Cartesian stages, IK branches."""

from __future__ import annotations

import copy
import itertools
import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from amp.geometry.collision import HELD_PREFIX, CollisionWorld, RobotSpheres
from amp.geometry.embodiment import DHArm
from amp.geometry.kinematics import (
    Q_MAX,
    Q_MIN,
    QD_MAX,
    IKResult,
    limit_margin,
    make_pose,
    pose7,
    quat_slerp,
    rot_error,
    solve_ik,
    solve_ik_batch,
)
from amp.planning import clock, retime, trajectory
from amp.planning.robot import arm_of, lims, plan_arm

__all__ = [
    "ACCEL_MAX",
    "CART_SPEED",
    "IK_POS_OK",
    "IK_ROT_OK",
    "LIMIT_MARGIN",
    "MARGIN",
    "MIN_TRANSIT_S",
    "PREFER_CLEARANCE",
    "REST_GAIN",
    "STAGE_SPEED_SCALE",
    "VEL_FRACTION",
    "HeldMargin",
    "Piece",
    "Rule",
    "Setup",
    "Stage",
    "Variant",
    "collision_issues",
    "contact_rule",
    "gripper_reach",
    "lift_target",
    "make_rule",
    "pieces_free",
    "plan_search",
    "run_stages",
    "sliding_rules",
]

MARGIN = 0.01
PREFER_CLEARANCE = 0.02
LIMIT_MARGIN = 0.02
EDGE_STEP_M = 0.01
CART_SPEED = 0.08
CART_ROT_SPEED = 0.6
ACCEL_MAX = 3.0
STAGE_SPEED_SCALE = 1.0
_RAMP_UP_S, _RAMP_MID_S, _RAMP_DOWN_S = 0.3, 0.5, 0.6
DENSE_M, DENSE_RAD, DENSE_DQ = 0.002, 0.02, 0.02
JUMP_M, JUMP_RAD = 1e-4, 1e-3
_DENSE_MAX_S = 0.1
_CAP_DT = 0.005
VEL_FRACTION = 0.85
MIN_TRANSIT_S = 0.2
SLIDE_TOL_M = 0.003
PUSH_TOL_M = 0.005
LIFT_DRIFT_M = (0.01, 0.02, 0.03, 0.05)
LIFT_DRIFT_DIRS = 8
DRIFT_CAP_M = 0.02
CARRY_RATE = 0.5
RRT_STEP_M = 0.25
RRT_MAX_ITERS = 20_000
SHORTCUT_ROUNDS = 60
N_RANDOM_SEEDS = 8
N_PERTURB_SEEDS = 4
PERTURB_RAD = 0.4
J7_SWEEP = 5
J1_OFFSETS = (-0.5, 0.5)
IK_DISTINCT_RAD = 0.3
IK_LIMIT_GAINS = (0.3, 0.0)
IK_CHUNK = 32
MAX_BRANCHES = 10
VARIANT_BRANCHES = 5
VARIANT_SEED_BRANCHES = 3
LIMIT_COMFORT_RAD = 0.05
LIMIT_STEP_RAD = 0.05
DIST_STEP = 0.05
ERR_STEP_M = 0.005
DIRECT_DETOUR = 1.5
BRANCH_TIME_FRACTION = 0.5
VARIANT_RESERVE_FRACTION = 0.1
FALLBACK_RESERVE_FRACTION = 0.1
VARIANT_IK_GIVE_UP_M = 0.02
SWING_J7_RAD = math.pi / 2
SWING_TRAVEL_RAD = 6.0
SWING_GAIN_RAD = 1.0
SWING_TIME_FRACTION = 0.15
SWING_BRANCHES = 5
MAX_SUBSTEPS = 64
MAX_REPORT_SUBSTEPS = 3000
REPORT_RESERVE_S = 0.75
FREE_CHUNK = 32
IK_POS_OK = 1e-3
IK_ROT_OK = math.radians(0.5)
REST_GAIN = 0.02
_LO = Q_MIN + LIMIT_MARGIN
_HI = Q_MAX - LIMIT_MARGIN
Rule = tuple


def _ik(arm):
    return solve_ik if isinstance(arm, DHArm) else arm.ik


def _ik_batch(arm):
    if isinstance(arm, DHArm):
        return solve_ik_batch

    def rows(T, seeds, T_flange_tool=None, **kw):
        seeds = np.asarray(seeds, dtype=np.float64)
        per = {k: np.broadcast_to(v, (len(seeds),)) for k, v in kw.items() if np.ndim(v)}
        same = {k: v for k, v in kw.items() if k not in per}
        Ts = np.broadcast_to(np.asarray(T, dtype=np.float64), (len(seeds), 4, 4))
        return [arm.ik(Ts[i], seeds[i], T_flange_tool, **same,
                       **{k: float(v[i]) for k, v in per.items()}) for i in range(len(seeds))]

    return rows


def _qd_max(arm) -> np.ndarray:
    return QD_MAX if isinstance(arm, DHArm) else arm.qd_max


def _margin(arm, q) -> float:
    return limit_margin(q) if isinstance(arm, DHArm) else arm.limit_margin(q)


def _range(arm) -> tuple[np.ndarray, np.ndarray]:
    """The joint range RRT samples and seeds draw from: limits shrunk by LIMIT_MARGIN."""
    return (_LO, _HI) if isinstance(arm, DHArm) else lims(arm, LIMIT_MARGIN)


def _qlim(arm) -> tuple[np.ndarray, np.ndarray]:
    return (Q_MIN, Q_MAX) if isinstance(arm, DHArm) else (arm.q_min, arm.q_max)


def make_rule(world: CollisionWorld, table=(), ignore=()) -> Rule:
    """A hashable collision rule: (world, table links ignored, ignore pairs)."""
    return (world, tuple(table), tuple(tuple(p) for p in ignore))


def _evaluate(rule: Rule, qs, tool):
    world, table, ignore = rule
    return world.evaluate(qs, tool, table_links_ignored=table, ignore=ignore)


def _minjerk(tau):
    return tau**3 * (10 - 15 * tau + 6 * tau * tau)


def _label(link: str) -> str:
    return "held " + link[len(HELD_PREFIX):] if link.startswith(HELD_PREFIX) else link


class _Checker:
    """Dense edge checks under one rule; sub-steps move no checked point more than EDGE_STEP_M."""

    def __init__(self, rule: Rule, tool):
        self.rule, self.tool = rule, tool
        self.bounds = np.maximum(rule[0].motion_bounds(tool), 0.05)

    def clearance(self, qs) -> np.ndarray:
        return _evaluate(self.rule, qs, self.tool)[0]

    def dist(self, a, b) -> float:
        return float(np.abs(np.asarray(b) - a) @ self.bounds)

    def samples(self, a, b) -> np.ndarray:
        n = max(1, math.ceil(self.dist(a, b) / EDGE_STEP_M))
        return a + ((np.arange(n) + 1) / n)[:, None] * (b - a)

    def edge_min(self, a, b) -> float:
        qs = self.samples(a, b)
        if len(qs) <= EDGE_STRIDE:
            return float(self.clearance(qs).min())
        idx = _coarse_index(len(qs))
        c = self.clearance(qs[idx])
        low = float(c.min())
        rest = _uncovered(idx, c, len(qs), self.dist(a, b) / len(qs), max(low, 0.0))
        if len(rest):
            low = min(low, float(self.clearance(qs[rest]).min()))
        return low

    def edge_ok(self, a, b, thresh: float) -> bool:
        qs = self.samples(a, b)
        if len(qs) <= EDGE_STRIDE or thresh < 0:
            return bool(self.clearance(qs).min() >= thresh)
        idx = _coarse_index(len(qs))
        c = self.clearance(qs[idx])
        if c.min() < thresh:
            return False
        rest = _uncovered(idx, c, len(qs), self.dist(a, b) / len(qs), thresh)
        return not len(rest) or bool(self.clearance(qs[rest]).min() >= thresh)


EDGE_STRIDE = 8


def _coarse_index(n: int) -> np.ndarray:
    return np.unique(np.r_[np.arange(EDGE_STRIDE // 2, n, EDGE_STRIDE), n - 1])


def _uncovered(idx, c, n, step, floor) -> np.ndarray:
    """Samples not in idx whose clearance the evaluated ones cannot prove to be >= floor >= 0."""
    k = np.arange(n)
    lb = (c[None, :] - np.abs(k[:, None] - idx[None, :]) * step).max(axis=1)
    lb[idx] = np.inf
    return np.flatnonzero(lb < floor + 1e-9)


class _Tree:
    def __init__(self, root, w):
        self.q = np.empty((256, len(root)))
        self.parent = np.empty(256, dtype=np.int64)
        self.n = 0
        self.w = w
        self.add(root, -1)

    def add(self, q, parent: int) -> int:
        if self.n == len(self.q):
            self.q = np.vstack([self.q, np.empty_like(self.q)])
            self.parent = np.concatenate([self.parent, np.empty_like(self.parent)])
        self.q[self.n], self.parent[self.n] = q, parent
        self.n += 1
        return self.n - 1

    def nearest(self, q) -> int:
        return int(np.argmin(np.abs(self.q[: self.n] - q) @ self.w))

    def path(self, i: int) -> list[np.ndarray]:
        out = []
        while i >= 0:
            out.append(self.q[i].copy())
            i = int(self.parent[i])
        return out


def _step_toward(chk, q, target):
    d = chk.dist(q, target)
    if d <= RRT_STEP_M:
        return target.copy(), True
    return q + (target - q) * (RRT_STEP_M / d), False


def _rrt_connect(chk, q_start, q_goal, thresh, rng, deadline) -> list[np.ndarray] | None:
    """RRT-Connect in joint space (limits shrunk by LIMIT_MARGIN); None when out of time."""
    ta, tb = _Tree(q_start, chk.bounds), _Tree(q_goal, chk.bounds)
    a_is_start = True
    for _ in range(RRT_MAX_ITERS):
        if clock.now() > deadline:
            return None
        q_rand = rng.uniform(*_range(arm_of(chk.tool)))
        i = ta.nearest(q_rand)
        q_new, _ = _step_toward(chk, ta.q[i], q_rand)
        if chk.edge_ok(ta.q[i], q_new, thresh):
            i_new = ta.add(q_new, i)
            target = ta.q[i_new].copy()
            j = tb.nearest(target)
            while clock.now() <= deadline:
                q_next, reached = _step_toward(chk, tb.q[j], target)
                if not chk.edge_ok(tb.q[j], q_next, thresh):
                    break
                j = tb.add(q_next, j)
                if reached:
                    path = ta.path(i_new)[::-1] + tb.path(j)[1:]
                    return path if a_is_start else path[::-1]
        ta, tb = tb, ta
        a_is_start = not a_is_start
    return None


def _greedy(chk, path, thresh) -> list[np.ndarray]:
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not chk.edge_ok(path[i], path[j], thresh):
            j -= 1
        out.append(path[j])
        i = j
    return out


def _shortcut(chk, path, thresh, rng, deadline) -> list[np.ndarray]:
    path = _greedy(chk, path, thresh)
    for _ in range(SHORTCUT_ROUNDS):
        if len(path) < 3 or clock.now() > deadline:
            break
        seg = np.array([chk.dist(a, b) for a, b in itertools.pairwise(path)])
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        s1, s2 = np.sort(rng.uniform(0.0, cum[-1], 2))
        i = min(int(np.searchsorted(cum, s1, side="right")) - 1, len(seg) - 1)
        j = min(int(np.searchsorted(cum, s2, side="right")) - 1, len(seg) - 1)
        if i == j:
            continue
        p1 = path[i] + (s1 - cum[i]) / max(seg[i], 1e-12) * (path[i + 1] - path[i])
        p2 = path[j] + (s2 - cum[j]) / max(seg[j], 1e-12) * (path[j + 1] - path[j])
        if chk.edge_ok(p1, p2, thresh):
            path = path[: i + 1] + [p1, p2] + path[j + 1:]
    return _greedy(chk, path, thresh)


def _segment_duration(dq, speed_scale, accel=ACCEL_MAX, qd_max=None) -> float:
    """Shortest min-jerk duration keeping peak |qd| and |qdd| inside the limits."""
    a = np.abs(dq)
    v = (QD_MAX if qd_max is None else qd_max) * speed_scale * VEL_FRACTION
    return float(max(np.max(1.875 * a / v), np.max(np.sqrt(5.7735 * a / accel))))


def _time_path(path, dt, speed_scale, accel=ACCEL_MAX, qd_max=None) -> np.ndarray:
    """(n,dof) targets at the end of every step: min-jerk along each segment, at rest at
    vertices."""
    path = [np.asarray(p, dtype=np.float64) for p in path]
    if len(path) == 1:
        path = path * 2
    durs = np.array([
        _segment_duration(b - a, speed_scale, accel, qd_max) for a, b in itertools.pairwise(path)
    ])
    durs = np.maximum(durs, 1e-6)
    total = max(float(durs.sum()), MIN_TRANSIT_S)
    n = max(1, math.ceil(total / dt - 1e-9))
    durs *= n * dt / durs.sum()
    cum = np.concatenate([[0.0], np.cumsum(durs)])
    t = (np.arange(n) + 1) * dt
    k = np.clip(np.searchsorted(cum, t, side="left"), 1, len(durs))
    tau = np.clip((t - cum[k - 1]) / durs[k - 1], 0.0, 1.0)
    A, B = np.array(path[:-1]), np.array(path[1:])
    q = A[k - 1] + _minjerk(tau)[:, None] * (B[k - 1] - A[k - 1])
    q[-1] = path[-1]
    return q


def _polish_deadline(deadline: float) -> float:
    return max(deadline, clock.now() + 0.3)


@dataclass
class _Transit:
    path: list | None
    method: str
    message: str
    chain: _Chain | None = None


def _transit(chk, q_start, q_goal, rng, deadline, margin) -> _Transit:
    """Direct joint-space segment if clear enough, else RRT-Connect plus shortcutting."""
    pref = max(0.0, PREFER_CLEARANCE - margin)
    c = chk.clearance(np.array([q_start, q_goal]))
    if c[0] < 0:
        return _Transit(None, "direct", "the start configuration is in collision")
    if c[1] < 0:
        return _Transit(None, "direct", "the goal configuration is in collision")
    want = float(min(pref, c[0], c[1]))
    direct = chk.edge_min(q_start, q_goal)
    if direct >= want:
        return _Transit([q_start, q_goal], "direct", "")
    now = clock.now()
    left = max(0.0, deadline - now)
    tight = ""
    if direct >= 0:
        tight = f"direct path clearance {1e3 * (direct + margin):.0f} mm is below the preferred"
        path = _rrt_connect(chk, q_start, q_goal, want, rng, now + min(0.25 * left, 1.0))
        if path is None:
            return _Transit([q_start, q_goal], "direct", tight)
        return _Transit(_shortcut(chk, path, want, rng, _polish_deadline(deadline)), "rrt", "")
    path = _rrt_connect(chk, q_start, q_goal, want, rng, now + 0.5 * left)
    used = want
    if path is None and want > 0:
        path, used = _rrt_connect(chk, q_start, q_goal, 0.0, rng, deadline), 0.0
    if path is None:
        return _Transit(None, "rrt", "no collision-free transit found in the time budget")
    msg = "" if used >= want else "clearance below the preferred 20 mm on the found path"
    return _Transit(_shortcut(chk, path, used, rng, _polish_deadline(deadline)), "rrt", msg)


@dataclass
class _Chain:
    q: np.ndarray
    tcp: np.ndarray
    pos_err: np.ndarray
    rot_err: np.ndarray
    vel_ratio: float

    @property
    def ok(self) -> bool:
        return bool(
            self.pos_err.max() <= trajectory.UNREACHABLE_POS_M
            and self.rot_err.max() <= trajectory.UNREACHABLE_ROT_RAD
            and self.vel_ratio <= 1.0
        )


def _ramp(t, T, va, vb):
    """Distance along a cosine velocity ramp va -> vb of duration T, at time t in [0, T]."""
    return va * t + 0.5 * (vb - va) * (t - T / math.pi * np.sin(math.pi * t / T))


def _phases(length, v_peak, slow) -> list[tuple[float, float, float]]:
    """(duration, v_from, v_to) cosine-ramped phases: slow start, v_peak, rest; [] if too short."""
    d_slow, v1 = slow if slow is not None else (0.0, v_peak)
    v1 = min(v1, v_peak)
    d_up = 0.5 * v1 * _RAMP_UP_S
    if v_peak > v1:
        d0 = max(d_slow, d_up)
        room = length - d0 - 0.5 * v1 * _RAMP_MID_S
        v2 = min(v_peak, room / (0.5 * (_RAMP_MID_S + _RAMP_DOWN_S)))
        if v2 > v1:
            rest = length - d0 - 0.5 * (v1 + v2) * _RAMP_MID_S - 0.5 * v2 * _RAMP_DOWN_S
            return [(_RAMP_UP_S, 0.0, v1), ((d0 - d_up) / v1, v1, v1), (_RAMP_MID_S, v1, v2),
                    (max(0.0, rest) / v2, v2, v2), (_RAMP_DOWN_S, v2, 0.0)]
    cruise = length - d_up - 0.5 * v1 * _RAMP_DOWN_S
    if cruise < 0:
        return []
    return [(_RAMP_UP_S, 0.0, v1), (cruise / v1, v1, v1), (_RAMP_DOWN_S, v1, 0.0)]


def _profile_s(length, ang, dt, v_peak, slow, stretch, rot_speed=None) -> np.ndarray:
    """Path fraction at the end of every step for a Cartesian segment."""
    rot_speed = CART_ROT_SPEED if rot_speed is None else rot_speed
    ph = _phases(length, v_peak, slow) if v_peak is not None and length > 1e-6 else []
    if not ph:
        v = CART_SPEED if v_peak is None else min([v_peak] + ([slow[1]] if slow else []))
        dur = max(1.875 * length / v, 1.875 * ang / rot_speed, 2 * dt) * stretch
        n = max(1, math.ceil(dur / dt - 1e-9))
        return _minjerk((np.arange(n) + 1) / n)
    total = sum(p[0] for p in ph)
    rot = ang * max(p[2] for p in ph) / length / rot_speed
    n = max(2, math.ceil(total * max(stretch, rot, 1.0) / dt - 1e-9))
    t = (np.arange(n) + 1) / n * total
    d, t0, acc = np.zeros(n), 0.0, 0.0
    for T, va, vb in ph:
        m = (t > t0) & (t <= t0 + T)
        if T > 0:
            d[m] = acc + _ramp(t[m] - t0, T, va, vb)
            acc += 0.5 * (va + vb) * T
        t0 += T
    d[t > t0] = acc
    return np.clip(d / acc, 0.0, 1.0)


def _speed_cap(length, ang, v_peak, slow, rot_speed=None, slow_end=None
               ) -> tuple[np.ndarray, np.ndarray]:
    """(s, top ds/dt) along the Cartesian profile, from `_profile_s` sampled finely."""
    if slow_end is not None:
        s, v = _speed_cap(length, ang, v_peak, slow_end, rot_speed)
        return 1.0 - s[::-1], v[::-1]
    s = np.concatenate([[0.0], _profile_s(length, ang, _CAP_DT, v_peak, slow, 1.0, rot_speed)])
    mid = 0.5 * (s[:-1] + s[1:])
    return np.concatenate([[0.0], mid, [1.0]]), np.concatenate([[0.0], np.diff(s) / _CAP_DT, [0.0]])


def _cartesian(tool, q0, T0, T1, dt, speed_scale, v_peak=None, slow=None,
               deadline=None, rot_speed=None, accel=None, q_rest=None,
               singular_push=True, slow_end=None) -> _Chain:
    """Straight TCP line T0 -> T1, IK'd densely, slowed only where joints need it; jump: inf."""
    arm = arm_of(tool)
    p0, p1 = T0[:3, 3], T1[:3, 3]
    q0_quat, q1_quat = pose7(T0)[3:], pose7(T1)[3:]
    ang = float(np.linalg.norm(rot_error(T1[:3, :3], T0[:3, :3])))
    length = float(np.linalg.norm(p1 - p0))
    T_ft = tool.T_flange_tcp
    q0 = np.asarray(q0, dtype=np.float64)
    accel = ACCEL_MAX if accel is None else accel
    q_end = q0 if q_rest is None else np.asarray(q_rest, dtype=np.float64)

    def pose(s):
        return p0 + s * (p1 - p0), quat_slerp(q0_quat, q1_quat, s)

    def frac(m, r):
        return min(m / length if length > 1e-9 else 1.0, r / ang if ang > 1e-9 else 1.0)

    h_max = min(_DENSE_MAX_S, frac(DENSE_M, DENSE_RAD))
    h_min = min(h_max, frac(JUMP_M, JUMP_RAD))
    cap = _speed_cap(length, ang, v_peak, slow, rot_speed, slow_end)

    def attempt(**ik):
        base, last = [0.0, q0], [0.0, None]

        def solve(s, seed):
            if seed is last[1]:
                base[:] = last
            T = make_pose(*pose(s))
            guess = seed + (s - base[0]) * (q_end - q0)
            q = _ik(arm)(T, guess, T_ft, q_rest=q0 + s * (q_end - q0), rest_gain=REST_GAIN,
                         singular_push=singular_push, **ik).q
            last[:] = [s, q]
            return q

        s, q, jump = retime.march(solve, q0, h_max, h_min, DENSE_DQ, deadline)
        g, x, t = retime.timing(
            s, q, *cap, _qd_max(arm) * speed_scale * VEL_FRACTION, accel, free=jump,
        )
        s_out = retime.resample(g, x, t, dt)
        q_out = np.stack([np.interp(s_out, s, q[:, j]) for j in range(arm.n)], axis=1)
        tcp = np.empty((len(s_out), 7))
        pe, re = np.empty(len(s_out)), np.empty(len(s_out))
        for i, si in enumerate(s_out):
            tcp[i, :3], tcp[i, 3:] = pose(si)
            T = tool.fk_tcp(q_out[i])
            pe[i] = np.linalg.norm(T[:3, 3] - tcp[i, :3])
            re[i] = np.linalg.norm(rot_error(make_pose(tcp[i, :3], tcp[i, 3:])[:3, :3], T[:3, :3]))
        qd = np.diff(np.vstack([q0, q_out]), axis=0) / dt
        vr = math.inf if jump.any() else float(np.max(np.abs(qd) / (_qd_max(arm) * speed_scale)))
        return _Chain(q_out, tcp, pe, re, vr)

    first = attempt()
    if first.vel_ratio < math.inf or (deadline is not None and clock.now() > deadline):
        return first
    steady = attempt(limit_gain=0.0)
    return max(first, steady, key=lambda c: (
        c.vel_ratio < math.inf, c.ok, min(_margin(arm, q) for q in c.q), -len(c.q)))


@dataclass
class Stage:
    """One Cartesian stage of a step plan: a target (or a hold), its speed profile and rules."""

    name: str
    grip: float
    rules: Callable
    T: np.ndarray | None = None
    hold_s: float = 0.0
    v_peak: float | None = None
    slow: tuple[float, float] | None = None
    slow_end: tuple[float, float] | None = None
    accel: float | None = None


@dataclass
class Piece:
    """A stage once solved: joint targets, TCP poses, gripper and residuals per step."""

    name: str
    q: np.ndarray
    tcp: np.ndarray
    grip: float
    pos_err: np.ndarray
    rot_err: np.ndarray
    rules: list


def run_stages(stages, q0, T0, tool, dt, speed_scale, deadline=None
                ) -> tuple[list[Piece], bool]:
    """Solve stages from q0 at T0 into pieces; (pieces, whether every stage solved)."""
    pieces, kin_ok = [], True
    q_prev, T_prev = np.asarray(q0, dtype=np.float64), T0
    pe_prev, re_prev = 0.0, 0.0
    for st in stages:
        q_before = q_prev
        if st.T is None:
            n = round(st.hold_s / dt)
            if n == 0:
                continue
            q = np.tile(q_prev, (n, 1))
            tcp = np.tile(pose7(T_prev), (n, 1))
            pe, re = np.full(n, pe_prev), np.full(n, re_prev)
        else:
            ch = _cartesian(tool, q_prev, T_prev, st.T, dt, speed_scale, st.v_peak, st.slow,
                            deadline, accel=st.accel, slow_end=st.slow_end)
            kin_ok &= ch.ok
            q, tcp, pe, re = ch.q, ch.tcp, ch.pos_err, ch.rot_err
            T_prev = st.T
        q_prev, pe_prev, re_prev = q[-1], pe[-1], re[-1]
        pieces.append(Piece(st.name, q, tcp, st.grip, pe, re, st.rules(tcp, q, q_before)))
    return pieces, kin_ok


def pieces_free(pieces, tool, deadline=None) -> tuple[bool | None, bool]:
    """(collision-free, or None if the deadline cut it short; whether a TCP-rigid part hit)."""
    for p in pieces:
        for rule in set(p.rules):
            idx = [i for i, r in enumerate(p.rules) if r == rule]
            for k in range(0, len(idx), FREE_CHUNK):
                if deadline is not None and clock.now() > deadline:
                    return None, False
                c, links, obs = _evaluate(rule, p.q[idx[k:k + FREE_CHUNK]], tool)
                if c.min() < 0:
                    j = int(np.argmin(c))
                    link = rule[0].contact(c[j], links[j], obs[j]).link or ""
                    return False, _rigid(rule[0], link)
    return True, False


def _rigid(world, link: str) -> bool:
    """Whether a contact link moves rigidly with the TCP: the gripper or the held object."""
    if link.startswith(HELD_PREFIX):
        return True
    if getattr(world, "arm", None) is None:
        return link == "gripper"
    return link in world.gripper_links


def _start_contacts(world, q, tool, *, table=(), ignore=(), allowed=None) -> list[tuple]:
    """Gripper/held contacts already present at q (with allowed obstacles): (link, obstacle)."""
    table, ignore = list(table), list(ignore)
    grip_links = getattr(world, "gripper_links", None)
    if grip_links is None:
        sp = world.spheres
        grip_links = {n for n, f in zip(sp.link_names, sp.link_frames) if f == "ee"}
    pairs = []
    for _ in range(8):
        c = world.check(q, tool, table_links_ignored=table, ignore=ignore)
        if c.ok or c.link is None:
            break
        if not (c.link in grip_links or c.link.startswith(HELD_PREFIX)):
            break
        if allowed is not None and c.obstacle not in allowed:
            break
        pairs.append((c.link, c.obstacle))
        if c.obstacle == "table":
            table.append(c.link)
        else:
            ignore.append((c.link, c.obstacle))
    return pairs


def _forgiving(world, table, ignore, pairs) -> Rule:
    """The rule (table, ignore) with the (link, obstacle) pairs also ignored."""
    table, ignore = list(table), list(ignore)
    for link, obstacle in pairs:
        if obstacle == "table":
            table.append(link)
        else:
            ignore.append((link, obstacle))
    return make_rule(world, table, ignore)


def contact_rule(world, q, tool, *, table=(), ignore=(), allowed=None) -> Rule:
    """The rule plus every gripper/held contact already present at q (with allowed obstacles)."""
    pairs = _start_contacts(world, q, tool, table=table, ignore=ignore, allowed=allowed)
    return _forgiving(world, table, ignore, pairs)


def _pair_clearance(world, qs, tool, link, obstacle) -> np.ndarray:
    """(N,) clearance of one (link, obstacle) pair alone at every configuration."""
    robot = [n for n in dict.fromkeys(world.link_names) if not n.startswith(HELD_PREFIX)]
    mine = ["*"] if link.startswith(HELD_PREFIX) else ["held", *(n for n in robot if n != link)]
    objs = [o for o in dict.fromkeys(world.obstacle_names[:-1]) if o != obstacle]
    ignore = [(s, o) for o in objs for s in ("*", "held")]
    if obstacle == "table":
        table = mine
    else:
        table = ["*", "held"]
        ignore += [(s, obstacle) for s in mine]
    return world.evaluate(qs, tool, table_links_ignored=table, ignore=ignore)[0]


def sliding_rules(world, q0, qs, rise, window, tool, *, table=(), carried=None) -> list[Rule]:
    """One rule per step of a vertical rise from q0 (rise: (N,) height above it)."""
    pairs = _start_contacts(world, q0, tool, table=table)
    if not pairs:
        return [make_rule(world, table)] * len(qs)
    rise = np.asarray(rise, dtype=np.float64)
    ends = []
    for link, obstacle in pairs:
        c = _pair_clearance(world, np.vstack([q0[None], qs]), tool, link, obstacle)
        c0, c = c[0], c[1:]
        sep = np.flatnonzero(c >= 0)
        tol = PUSH_TOL_M if len(sep) else SLIDE_TOL_M
        deep = np.flatnonzero(c < c0 - tol)
        end = min(int(sep[0]) + 1 if len(sep) else len(qs), int(deep[0]) if len(deep) else len(qs))
        ends.append(end)
        hit = (np.arange(len(c)) >= end) & (rise >= window) & (c < 0)
        if carried is not None and len(deep) and hit.any() and link.startswith(HELD_PREFIX):
            d = int(deep[0])
            if c0 - c[d] >= CARRY_RATE * rise[d]:
                carried.append((link[len(HELD_PREFIX):], obstacle))
    rules: dict = {}
    out = []
    for i, h in enumerate(rise):
        keep = tuple(p for p, e in zip(pairs, ends) if h < window or i < e)
        if keep not in rules:
            rules[keep] = _forgiving(world, table, (), keep)
        out.append(rules[keep])
    return out


class HeldMargin:
    """world with the held object checked at margin `held`; the robot keeps world's margin."""

    def __init__(self, world, held: float):
        self._w = world
        self._h = copy.copy(world)
        self._h.margin = float(held)

    def __getattr__(self, name):
        return getattr(self._w, name)

    def evaluate(self, qs, tool, *, table_links_ignored=(), ignore=()):
        t, ig = tuple(table_links_ignored), tuple(tuple(p) for p in ignore)
        c1, l1, o1 = self._w.evaluate(qs, tool, table_links_ignored=t + ("held",),
                                      ignore=ig + (("held", "*"),))
        c2, l2, o2 = self._h.evaluate(qs, tool, table_links_ignored=t + ("*",),
                                      ignore=ig + (("*", "*"),))
        m = c2 < c1
        return np.where(m, c2, c1), np.where(m, l2, l1), np.where(m, o2, o1)

    def check(self, q, tool, *, table_links_ignored=(), ignore=()):
        c, lk, ob = self.evaluate(q, tool, table_links_ignored=table_links_ignored, ignore=ignore)
        return self._w.contact(c[0], lk[0], ob[0])


def _held_path_clearance(world, T_tcps, margin, only=None) -> np.ndarray:
    """(N, obstacles) clearance of world's held object alone at each TCP pose (no robot); only the
    obstacle indices in only, if given."""
    pts = world._att_pts
    r = float(world._r[world._kind == 1][0])
    T_tcps = np.asarray(T_tcps, dtype=np.float64)
    P = pts @ T_tcps[:, :3, :3].transpose(0, 2, 1) + T_tcps[:, None, :3, 3]
    out = np.full((len(P), len(world.obstacle_names) - 1), DRIFT_CAP_M)
    n_mesh = len(world._obs)
    for g in range(len(world.obstacle_names) - 1 - n_mesh):
        if only is None or n_mesh + g in only:
            out[:, n_mesh + g] = world.peers.clearance(
                P.reshape(-1, 3), r + margin, g, DRIFT_CAP_M, hand=n_mesh + g not in world.co_held
            ).reshape(P.shape[:2]).min(axis=1)
    for k, o in enumerate(world._obs):
        if only is not None and k not in only:
            continue
        L = (P - o.t) @ o.R
        box = np.linalg.norm(np.maximum(np.maximum(o.field.lo - L, L - o.field.hi), 0), axis=2)
        cand = box - r - margin < DRIFT_CAP_M
        if not cand.any():
            continue
        Lc = L[cand]
        cap = DRIFT_CAP_M + r + margin
        d = np.minimum(o.field.tree.query(Lc, distance_upper_bound=cap)[0], cap)
        clr = np.full(cand.shape, DRIFT_CAP_M)
        clr[cand] = np.where(o.field.inside(Lc), -d, d) - r - margin
        out[:, k] = clr.min(axis=1)
    return out


def _lift_ok(c) -> bool:
    """Whether a held path's clearances (N, obstacles) pass the sliding rule, roughly."""
    c0 = c[0]
    if (c[:, c0 >= 0] < 0).any():
        return False
    for k in np.flatnonzero(c0 < 0):
        sep = np.flatnonzero(c[:, k] >= 0)
        end = int(sep[0]) if len(sep) else len(c)
        if (c[:end, k] < c0[k] - PUSH_TOL_M).any() or (c[end:, k] < 0).any():
            return False
    return True


def lift_target(world, T_g, rise, margin) -> np.ndarray:
    """T_g raised by rise; drifted sideways (least first) when the held object would otherwise
    come within margin of a neighbour it did not start touching."""
    s = np.linspace(0.0, 1.0, 26)

    def path(d):
        T = np.repeat(T_g[None], len(s), axis=0)
        T[:, :2, 3] += s[:, None] * d
        T[:, 2, 3] += s * rise
        return T

    T_up = T_g.copy()
    T_up[2, 3] += rise
    if world._att_pts is None:
        return T_up
    c = _held_path_clearance(world, path(np.zeros(2)), margin)
    if _lift_ok(c):
        return T_up
    near = set(np.flatnonzero(c.min(axis=0) < DRIFT_CAP_M).tolist())
    for r in LIFT_DRIFT_M:
        cands = []
        for a in np.arange(LIFT_DRIFT_DIRS) * (2 * np.pi / LIFT_DRIFT_DIRS):
            d = r * np.array([np.cos(a), np.sin(a)])
            c = _held_path_clearance(world, path(d), margin, near)
            if _lift_ok(c):
                cands.append((float(c.min()), a, d))
        for _, _, d in sorted(cands, key=lambda x: (-x[0], x[1])):
            if _lift_ok(_held_path_clearance(world, path(d), margin)):
                T = T_up.copy()
                T[:2, 3] += d
                return T
    return T_up


def gripper_reach(spheres: RobotSpheres, tool, embodiment=None) -> float:
    """How far the gripper spheres extend past the TCP along the approach (tcp +z)."""
    arm = getattr(spheres, "arm", None)
    if arm is None:
        grip = np.array([f == "ee" for f in spheres.link_frames])[spheres.link]
        if not grip.any():
            return 0.0
        p = spheres.local[grip] @ tool.T_tcp_ee[:3, :3].T + tool.T_tcp_ee[:3, 3]
        return max(0.0, float((p[:, 2] + spheres.radius[grip]).max()))
    from amp.geometry.collision.checker import gripper_links

    names = gripper_links(spheres, embodiment)
    grip = np.array([n in names for n in spheres.link_names])[spheres.link]
    if not grip.any():
        return 0.0
    q = arm.q_home
    F = spheres.frames_batch(q)[0]
    T_tcp = arm.fk_flange(q) @ tool.T_flange_tcp
    W = np.einsum("sij,sj->si", F[spheres.frame[grip], :3, :3], spheres.local[grip])
    W += F[spheres.frame[grip], :3, 3]
    p = (W - T_tcp[:3, 3]) @ T_tcp[:3, :3]
    return max(0.0, float((p[:, 2] + spheres.radius[grip]).max()))


def _ik_seeds(q_start, rng, extra=(), arm=None) -> list[np.ndarray]:
    """q_start, home, given seeds, last-joint sweeps, joint-1 offsets, perturbations, random;
    arm None = DROID's."""
    arm = plan_arm(None) if arm is None else arm
    lo, hi = _range(arm)
    w = arm.n - 1
    q_start = np.asarray(q_start, dtype=np.float64)
    seeds = [q_start, arm.q_home, *extra]
    for base in (q_start, arm.q_home):
        for v in np.linspace(lo[w], hi[w], J7_SWEEP):
            seeds.append(np.concatenate([base[:w], [v]]))
    for d in J1_OFFSETS:
        s = q_start.copy()
        s[0] += d
        seeds.append(s)
    seeds += [q_start + rng.normal(0.0, PERTURB_RAD, arm.n) for _ in range(N_PERTURB_SEEDS)]
    seeds += [rng.uniform(lo, hi) for _ in range(N_RANDOM_SEEDS)]
    return [np.clip(s, *_qlim(arm)) for s in seeds]


def _ik_solutions(T, q_start, tool, rng, weights, extra=(), deadline=None, give_up_m=None):
    """Distinct IK solutions from many seeds (nearest q_start first) and the best try."""
    arm = arm_of(tool)
    sols, best = [], None
    seeds = _ik_seeds(q_start, rng, extra, arm)
    head = 2 + len(extra)
    chunks = [seeds[:head]] + [seeds[i:i + IK_CHUNK] for i in range(head, len(seeds), IK_CHUNK)]
    for c, chunk in enumerate(chunks):
        if deadline is not None and sols and clock.now() > deadline:
            break
        if c == 1 and give_up_m is not None and not sols and best.pos_err > give_up_m:
            break
        rows = np.repeat(np.array(chunk), len(IK_LIMIT_GAINS), axis=0)
        gains = np.tile(IK_LIMIT_GAINS, len(chunk))
        for r in _ik_batch(arm)(T, rows, tool.T_flange_tcp, max_iters=150, limit_gain=gains):
            if best is None or r.pos_err + 0.1 * r.rot_err < best.pos_err + 0.1 * best.rot_err:
                best = r
            if r.pos_err <= IK_POS_OK and r.rot_err <= IK_ROT_OK and all(
                    np.max(np.abs(r.q - o.q)) > IK_DISTINCT_RAD for o in sols):
                sols.append(r)
    sols.sort(key=lambda r: float(np.abs(r.q - q_start) @ weights))
    return sols, best


@dataclass
class Setup:
    """One requested (or variant) grasp/release TCP and everything planned from it."""

    tcp: np.ndarray
    T_pre: np.ndarray
    stages: list
    transit_rule: Rule
    segment_issues: list


@dataclass
class Variant:
    """A flipped or yawed alternative to the requested TCP pose of a step."""

    flip: bool
    yaw_deg: float
    make: Callable

    @property
    def exact(self) -> bool:
        return not self.flip and self.yaw_deg == 0.0

    @property
    def twist(self) -> float:
        return math.radians(self.yaw_deg) + (math.pi if self.flip else 0.0)

    def info(self) -> dict | None:
        return None if self.exact else {"flip": self.flip, "yaw_deg": float(self.yaw_deg)}


@dataclass
class _Branch:
    r: object
    pieces: list
    goal_free: bool
    kin_ok: bool
    margin: float
    dist: float
    duration: float
    err: float
    free: bool | None = None
    rigid: bool = False
    singular: bool = False

    def check(self, tool, deadline=None) -> None:
        """Run the stages' collision check unless the deadline passes first (free stays None)."""
        if self.free is None:
            self.free, self.rigid = pieces_free(self.pieces, tool, deadline)

    @property
    def clean(self) -> bool:
        return bool(self.goal_free and self.kin_ok and self.free and not self.singular)

    def key(self) -> tuple:
        """Larger is better: clean, then joint-limit margin, then distance, then duration."""
        free = self.goal_free if self.free is None else self.free
        err = 0 if self.kin_ok else -math.ceil(self.err / ERR_STEP_M)
        margin = math.floor(min(self.margin, LIMIT_COMFORT_RAD) / LIMIT_STEP_RAD + 1e-9)
        ok = self.goal_free and self.kin_ok and not self.singular
        return (ok and free, self.goal_free, self.kin_ok, not self.singular, free, err, margin,
                -math.floor(self.dist / DIST_STEP), -self.duration)


@dataclass
class _Search:
    setup: Setup
    sols: list
    best_ik: object
    branches: list
    chosen: _Branch | None = None
    transit: _Transit | None = None
    notes: list = field(default_factory=list)


def _branch(r, setup, chk, goal_free, q_start, tool, dt, speed_scale, tscale, taccel,
            deadline=None) -> _Branch:
    arm = arm_of(tool)
    pieces, kin_ok = run_stages(setup.stages, r.q, setup.T_pre, tool, dt, STAGE_SPEED_SCALE,
                                 deadline)
    qs = np.vstack([r.q[None]] + [p.q for p in pieces])
    err = max([float(p.pos_err.max()) for p in pieces if len(p.pos_err)] + [0.0])
    dur = sum(len(p.q) for p in pieces) * dt + _segment_duration(r.q - q_start, tscale, taccel,
                                                                  _qd_max(arm))
    dist = float(np.abs(r.q - q_start) @ chk.bounds)
    singular = not arm.singular_ok(r.q) or any(
        h for _, _, h in trajectory.singular_runs(qs, dt, arm))
    return _Branch(r, pieces, goal_free, bool(kin_ok), _margin(arm, qs), dist, float(dur),
                   err, None if goal_free else False, singular=singular)


def _reaches(setup, q, tool) -> bool:
    """Whether IK follows the stages' Cartesian targets from q: a cheap filter before the chain."""
    for st in setup.stages:
        if st.T is None:
            continue
        r = _ik(arm_of(tool))(st.T, q, tool.T_flange_tcp, q_rest=q, rest_gain=REST_GAIN)
        if r.pos_err > IK_POS_OK or r.rot_err > IK_ROT_OK:
            return False
        q = r.q
    return True


def _direct(chk, q_start, q_goal, margin) -> _Transit | None:
    """The straight joint-space transit when it is as clear as _transit would want it."""
    c = chk.clearance(np.array([q_start, q_goal]))
    if c.min() < 0:
        return None
    want = float(min(max(0.0, PREFER_CLEARANCE - margin), c[0], c[1]))
    if not chk.edge_ok(q_start, q_goal, want):
        return None
    return _Transit([q_start, q_goal], "direct", "")


TRANSIT_SPEED = 1.0
TRANSIT_ROT_SPEED = 4.0
CART_TRANSIT_S = 0.6
CART_TRANSIT_FRACTION = 0.15
OVER_MIN_LEG_M = 0.001


def _cart_until(deadline: float, slices: float = 1.0) -> float:
    now = clock.now()
    return now + slices * min(CART_TRANSIT_S, CART_TRANSIT_FRACTION * max(0.0, deadline - now))


def _over_height(world, tool, q, T0, T1) -> float:
    """TCP height at which the hand and held object clear all in the start-goal XY corridor."""
    P = world.points(q[None], tool)[0]
    hand = world._grip.copy()
    if world.attached_name is not None:
        hand |= world._link == len(world.link_names) - 1
    local = (P[hand] - T0[:3, 3]) @ T0[:3, :3]
    rad = world._r[hand]
    q0, q1 = pose7(T0)[3:], pose7(T1)[3:]
    drop, reach = 0.0, 0.0
    for f in (0.0, 0.5, 1.0):
        w = local @ make_pose(np.zeros(3), quat_slerp(q0, q1, f))[:3, :3].T
        drop = max(drop, float(np.max(rad - w[:, 2])))
        reach = max(reach, float(np.max(np.linalg.norm(w[:, :2], axis=1) + rad)))
    a, b = T0[:3, 3], T1[:3, 3]
    d = b[:2] - a[:2]
    top = world.table_z
    for o in world._obs:
        V = o.field.samples @ o.R.T + o.t
        u = np.clip((V[:, :2] - a[:2]) @ d / max(float(d @ d), 1e-12), 0.0, 1.0)
        near = np.linalg.norm(V[:, :2] - a[:2] - u[:, None] * d, axis=1) <= reach + PREFER_CLEARANCE
        if near.any():
            top = max(top, float(V[near, 2].max()))
    if world.peers is not None:
        C, R = world.peers.centres, world.peers.radii
        u = np.clip((C[:, :2] - a[:2]) @ d / max(float(d @ d), 1e-12), 0.0, 1.0)
        off = np.linalg.norm(C[:, :2] - a[:2] - u[:, None] * d, axis=1) - R
        near = off <= reach + PREFER_CLEARANCE
        if near.any():
            top = max(top, float((C[near, 2] + R[near]).max()))
    return max(top + drop + PREFER_CLEARANCE + EDGE_STEP_M, float(a[2]), float(b[2]))


def _over_poses(T0, T1, z) -> list[np.ndarray]:
    """Up to height z, across, then down to T1, the orientation turning with the distance."""
    p0, p1 = T0[:3, 3], T1[:3, 3]
    pts = [np.r_[p0[:2], z], np.r_[p1[:2], z], p1]
    lens = np.linalg.norm(np.diff(np.vstack([p0] + pts), axis=0), axis=1)
    frac = np.cumsum(lens) / max(float(lens.sum()), 1e-12)
    q0, q1 = pose7(T0)[3:], pose7(T1)[3:]
    out = [make_pose(p, quat_slerp(q0, q1, f)) for p, f, n in zip(pts[:-1], frac, lens)
           if n > OVER_MIN_LEG_M]
    return out + [T1]


def _transit_chain(tool, q_start, T0, poses, dt, tscale, taccel, until, q_rest, push
                   ) -> _Chain | None:
    """Straight TCP legs through poses, each from rest to rest; None when out of time."""
    legs, q, T = [], q_start, T0
    for T_next in poses:
        ch = _cartesian(tool, q, T, T_next, dt, tscale, v_peak=TRANSIT_SPEED * tscale,
                        deadline=until, rot_speed=TRANSIT_ROT_SPEED * tscale, accel=taccel,
                        q_rest=q_rest, singular_push=push)
        if clock.now() > until:
            return None
        legs.append(ch)
        q, T = ch.q[-1], T_next
    if len(legs) == 1:
        return legs[0]
    return _Chain(*(np.concatenate([getattr(c, k) for c in legs])
                    for k in ("q", "tcp", "pos_err", "rot_err")),
                  max(c.vel_ratio for c in legs))


def _samples_clear(chk, qs, thresh) -> bool:
    """Whether every sub-step between consecutive qs clears thresh >= 0 (as _Checker.edge_ok)."""
    subs = np.vstack([chk.samples(a, b) for a, b in itertools.pairwise(qs)])
    idx = _coarse_index(len(subs))
    c = chk.clearance(subs[idx])
    if c.min() < thresh:
        return False
    rest = _uncovered(idx, c, len(subs), EDGE_STEP_M, thresh)
    return not len(rest) or bool(chk.clearance(subs[rest]).min() >= thresh)


def _cartesian_transit(setup, chk, q_start, tool, dt, speed_scale, tscale, taccel, until,
                       q_rest=None, push=False, over=False) -> tuple[_Branch, _Transit] | str:
    """A straight TCP line (or up, over, down) to the pre-pose and its branch if clean, else why."""
    c0 = float(chk.clearance(q_start[None])[0])
    if c0 < 0:
        return "start"
    if q_rest is None and not over:
        r = _ik(arm_of(tool))(setup.T_pre, q_start, tool.T_flange_tcp, q_rest=q_start,
                              rest_gain=REST_GAIN)
        if r.pos_err > IK_POS_OK or r.rot_err > IK_ROT_OK:
            return "reach"
    T0 = tool.fk_tcp(q_start)
    poses, msg = [setup.T_pre], ""
    if over:
        z = _over_height(setup.transit_rule[0], tool, q_start, T0, setup.T_pre)
        poses = _over_poses(T0, setup.T_pre, z)
        for T in poses[:-1]:
            r = _ik(arm_of(tool))(T, q_start, tool.T_flange_tcp, q_rest=q_start,
                                  rest_gain=REST_GAIN)
            if r.pos_err > IK_POS_OK or r.rot_err > IK_ROT_OK:
                return "reach"
        msg = f"went over the obstacles in the way, up to {z:.2f} m"
    ch = _transit_chain(tool, q_start, T0, poses, dt, tscale, taccel, until, q_rest, push)
    if ch is None:
        return "time"
    if not ch.vel_ratio < math.inf:
        return "jump"
    if not ch.ok or ch.pos_err[-1] > IK_POS_OK or ch.rot_err[-1] > IK_ROT_OK:
        return "reach"
    qs = np.vstack([q_start[None], ch.q])
    if any(h and i > 0 for i, _, h in trajectory.singular_runs(qs, dt, arm_of(tool))):
        return "singular"
    c1 = float(chk.clearance(ch.q[-1:])[0])
    want = min(max(0.0, PREFER_CLEARANCE - setup.transit_rule[0].margin), c0, c1)
    if want < 0:
        return "goal"
    if not _samples_clear(chk, qs, want):
        return "collision"
    q_end = ch.q[-1].copy()
    r = IKResult(q_end, float(ch.pos_err[-1]), float(ch.rot_err[-1]), True, 0,
                 arm_of(tool).limit_margin(q_end))
    b = _branch(r, setup, chk, True, q_start, tool, dt, speed_scale, tscale, taccel, until)
    b.check(tool, until)
    if not b.clean:
        if clock.now() > until:
            return "time"
        return "singular" if b.kin_ok and b.free and b.singular else "stages"
    return b, _Transit([q_start, r.q], "cartesian", msg, ch)


@dataclass
class _Ext:
    """An external motion provider and what it plans in."""

    provider: object
    scene: object
    held: tuple | None


EXT_GOAL_TOL_RAD = 1e-3


def _ext_ik(ext, setup, q_start, tool, n, deadline) -> list:
    """The provider's IK solutions of setup's pre-pose, their errors from our FK."""
    T_flange = setup.T_pre @ np.linalg.inv(tool.T_flange_tcp)
    qs = ext.provider.ik(T_flange, q_start, scene=ext.scene, held=ext.held, n=n,
                         deadline=deadline)
    out = []
    for q in qs or ():
        q = np.asarray(q, dtype=np.float64).reshape(arm_of(tool).n)
        if not np.all(np.isfinite(q)):
            continue
        T = tool.fk_tcp(q)
        pe = float(np.linalg.norm(T[:3, 3] - setup.T_pre[:3, 3]))
        re = float(np.linalg.norm(rot_error(setup.T_pre[:3, :3], T[:3, :3])))
        out.append(IKResult(q, pe, re, pe <= IK_POS_OK and re <= IK_ROT_OK, 0,
                            arm_of(tool).limit_margin(q)))
    return out


def _ext_transit(ext, q_start, q_goal, tool, dt, tscale, taccel, T_pre, r, deadline
                 ) -> _Transit:
    """The provider's timed transit q_start -> q_goal as a pre-timed _Transit (path None: why)."""
    name = ext.provider.name
    path, info = ext.provider.transit(q_start, q_goal, scene=ext.scene, held=ext.held, dt=dt,
                                      speed_scale=tscale, accel_scale=taccel / ACCEL_MAX,
                                      deadline=deadline)
    info = info or {}
    if path is None:
        why = info.get("message") or f"{name} found no transit (status {info.get('status')})"
        return _Transit(None, name, str(why))
    q = np.array(path, dtype=np.float64)
    if (q.ndim != 2 or q.shape[1] != len(q_goal) or not len(q) or not np.all(np.isfinite(q))
            or np.max(np.abs(q[-1] - q_goal)) > EXT_GOAL_TOL_RAD):
        return _Transit(None, name, f"{name} returned an invalid transit (shape {q.shape})")
    q[-1] = q_goal
    tcp = np.array([pose7(tool.fk_tcp(qi)) for qi in q])
    tcp[-1] = pose7(T_pre)
    pe, re = np.zeros(len(q)), np.zeros(len(q))
    pe[-1], re[-1] = r.pos_err, r.rot_err
    qd = np.diff(np.vstack([q_start, q]), axis=0) / dt
    vr = float(np.max(np.abs(qd) / (_qd_max(arm_of(tool)) * tscale)))
    return _Transit([np.asarray(q_start, dtype=np.float64), q_goal], name, "",
                    _Chain(q, tcp, pe, re, vr))


def _ext_search(setup, chk, ext, *, q_start, tool, dt, speed_scale, deadline, tscale, taccel,
                max_branches, clean_only) -> _Search:
    """_search with the provider's IK and transit; our stages, checks and branch order."""
    now = clock.now()
    eval_until = now + BRANCH_TIME_FRACTION * max(0.0, deadline - now)
    cands = _ext_ik(ext, setup, q_start, tool, max_branches, eval_until)
    sols = sorted((r for r in cands if r.converged),
                  key=lambda r: float(np.abs(r.q - q_start) @ chk.bounds))
    if cands:
        best_ik = min(cands, key=lambda r: r.pos_err + 0.1 * r.rot_err)
    else:
        best_ik = _ik(arm_of(tool))(setup.T_pre, q_start, tool.T_flange_tcp, max_iters=150)
    out = _Search(setup, sols, best_ik, [])
    if not sols:
        return out
    goal_free = chk.clearance(np.array([r.q for r in sols])) >= 0
    reach = [bool(g) and _reaches(setup, r.q, tool) for r, g in zip(sols, goal_free)]
    order = sorted(range(len(sols)), key=lambda i: (not reach[i], not goal_free[i]))
    if clean_only:
        order = [i for i in order if reach[i]]
    for i in order[:max_branches]:
        if out.branches and clock.now() > eval_until:
            break
        out.branches.append(_branch(sols[i], setup, chk, bool(goal_free[i]), q_start, tool, dt,
                                    speed_scale, tscale, taccel))
    out.branches.sort(key=_Branch.key, reverse=True)
    for b in out.branches:
        if clock.now() > deadline:
            break
        b.check(tool, deadline)
        if b.rigid and b.kin_ok:
            for o in out.branches:
                if o.free is None and o.kin_ok:
                    o.free, o.rigid = False, True
        if not b.clean:
            continue
        tr = _ext_transit(ext, q_start, b.r.q, tool, dt, tscale, taccel, setup.T_pre, b.r,
                          deadline)
        if tr.path is not None:
            out.chosen, out.transit = b, tr
            return out
        out.notes.append(tr.message)
    for b in out.branches:
        b.check(tool, deadline)
    out.branches.sort(key=lambda b: (b.free is not None, b.key()), reverse=True)
    return out


def _search(setup, *, q_start, tool, dt, speed_scale, rng, deadline, tscale, taccel,
            seeds=(), max_branches=MAX_BRANCHES, clean_only=False, ext=None
            ) -> tuple[_Search, _Checker]:
    """Every IK branch of setup's pre-pose through its stages, best first, with a transit."""
    chk = _Checker(setup.transit_rule, tool)
    if ext is not None:
        return _ext_search(setup, chk, ext, q_start=q_start, tool=tool, dt=dt,
                           speed_scale=speed_scale, deadline=deadline, tscale=tscale,
                           taccel=taccel, max_branches=max_branches, clean_only=clean_only), chk
    args = (setup, chk, q_start, tool, dt, speed_scale, tscale, taccel)
    cart = _cartesian_transit(*args, _cart_until(deadline))
    if cart == "collision":
        over = _cartesian_transit(*args, _cart_until(deadline, 2.0), over=True)
        cart = cart if isinstance(over, str) else over
    if not isinstance(cart, str):
        b, tr = cart
        return _Search(setup, [b.r], b.r, [b], b, tr), chk
    now = clock.now()
    eval_until = now + BRANCH_TIME_FRACTION * max(0.0, deadline - now)
    sols, best_ik = _ik_solutions(setup.T_pre, q_start, tool, rng, chk.bounds, seeds, eval_until,
                                  VARIANT_IK_GIVE_UP_M if clean_only else None)
    out = _Search(setup, sols, best_ik, [])
    if clean_only and not sols:
        return out, chk
    if not sols and not _reaches(setup, best_ik.q, tool):
        return out, chk
    cands = sols or [best_ik]
    goal_free = chk.clearance(np.array([r.q for r in cands])) >= 0
    reach = [bool(g) and _reaches(setup, r.q, tool) for r, g in zip(cands, goal_free)]
    order = sorted(range(len(cands)), key=lambda i: (not reach[i], not goal_free[i]))
    if clean_only:
        order = [i for i in order if reach[i]]
    for i in order[:max_branches]:
        if (out.branches or clean_only) and clock.now() > eval_until:
            break
        if out.branches and not reach[i] and any(b.clean for b in out.branches):
            break
        out.branches.append(_branch(cands[i], setup, chk, bool(goal_free[i]), q_start, tool, dt,
                                    speed_scale, tscale, taccel))
    out.branches.sort(key=_Branch.key, reverse=True)
    pending, clean, direct = list(out.branches), [], {}
    margin = setup.transit_rule[0].margin

    def clean_at(k):
        while len(clean) <= k and pending:
            if clock.now() > deadline:
                break
            b = pending.pop(0)
            b.check(tool, deadline)
            if b.rigid and b.kin_ok:
                for o in out.branches:
                    if o.free is None and o.kin_ok:
                        o.free, o.rigid = False, True
            if b.clean:
                clean.append(b)
        return clean[k] if k < len(clean) else None

    def direct_of(b):
        if id(b) not in direct:
            direct[id(b)] = _direct(chk, q_start, b.r.q, margin)
        return direct[id(b)]

    best = clean_at(0)
    k = 0
    while (b := clean_at(k)) is not None:
        if b.key()[6] < best.key()[6] or b.dist > DIRECT_DETOUR * best.dist + DIST_STEP:
            break
        if direct_of(b) is not None:
            out.chosen, out.transit = b, direct_of(b)
            _retry_cartesian(out, cart, args, deadline, b)
            return out, chk
        k += 1
    if best is not None and _retry_cartesian(out, cart, args, deadline, best):
        return out, chk
    k = 0
    while (b := clean_at(k) if clock.now() <= deadline else _at(clean, k)) is not None:
        tr = direct_of(b)
        if tr is None and clock.now() <= deadline:
            tr = _transit(chk, q_start, b.r.q, rng, deadline, margin)
            if tr.path is None:
                out.notes.append(tr.message)
        if tr is not None and tr.path is not None:
            out.chosen, out.transit = b, tr
            break
        k += 1
    if out.chosen is None:
        for b in out.branches:
            b.check(tool, deadline)
        out.branches.sort(key=lambda b: (b.free is not None, b.key()), reverse=True)
    return out, chk


_REDUNDANCY = ("jump", "reach", "singular")


def _retry_cartesian(out, why, args, deadline, toward: _Branch) -> bool:
    """A Cartesian transit with its redundancy drawn toward a branch; chosen if it is clean."""
    if why not in _REDUNDANCY or clock.now() > deadline:
        return False
    until = _cart_until(deadline)
    for push in (False, True):
        cart = _cartesian_transit(*args, until, q_rest=toward.r.q, push=push)
        if not isinstance(cart, str):
            out.chosen, out.transit = cart
            out.branches.insert(0, cart[0])
            return True
        if cart not in _REDUNDANCY or clock.now() > until:
            return False
    return False


def _at(items, k):
    return items[k] if k < len(items) else None


def _variant_seeds(branches, twist) -> list[np.ndarray]:
    """The exact request's branches with the last joint turned by the variant's twist, both
    ways."""
    out = []
    for b in branches[:VARIANT_SEED_BRANCHES]:
        for sgn in (1.0, -1.0):
            q = b.r.q.copy()
            q[-1] += sgn * twist
            out.append(q)
    return out


def _variant_message(kind, v: Variant, exact: _Search) -> str:
    parts = []
    if v.flip:
        parts.append("180° about the gripper axis")
    if v.yaw_deg:
        parts.append(f"{v.yaw_deg:+.0f}° about vertical")
    b = exact.branches[0] if exact.branches else None
    if not exact.sols:
        why = "to reach the pose"
    elif b is not None and not b.goal_free:
        why = "to avoid a collision at the approach pose"
    elif b is not None and not b.kin_ok:
        why = "to stay within reach and joint limits"
    elif b is not None and b.singular:
        why = "to keep joint 2 off the shoulder singularity"
    elif b is not None and not b.free:
        why = "to avoid a collision"
    else:
        why = "to find a collision-free path"
    verb = "grasped" if kind == "pick" else "released"
    return f"{verb} rotated {' and '.join(parts)} {why}; the requested pose has no clean plan"


def collision_issues(pieces, q_start, tool, dt, deadline=None):
    """Collision issues per run of bad steps, checked at every step and every sub-step."""
    q = np.concatenate([p.q for p in pieces])
    rules = [r for p in pieces for r in p.rules]
    prev = np.vstack([q_start, q[:-1]])
    n = len(q)
    clr = np.full(n, np.inf)
    label: list = [None] * n
    bounds: dict = {}
    for rule in rules:
        if id(rule[0]) not in bounds:
            bounds[id(rule[0])] = np.maximum(rule[0].motion_bounds(tool), 0.05)
    dq = np.array([float(np.abs(q[i] - prev[i]) @ bounds[id(r[0])]) for i, r in enumerate(rules)])
    ms = np.minimum(np.maximum(1, np.ceil(dq / EDGE_STEP_M)), MAX_SUBSTEPS)
    coarse = deadline is not None and clock.now() >= deadline
    cap = len(ms) if coarse else MAX_REPORT_SUBSTEPS
    if ms.sum() > cap:
        ms = np.maximum(1, np.floor(ms * cap / ms.sum()))
    ms = ms.astype(int)
    for rule in dict.fromkeys(rules):
        w = rule[0]
        steps = [i for i, r in enumerate(rules) if r == rule]
        subs, owner = [], []
        for i, m in zip(steps, ms[steps]):
            subs.append(prev[i] + ((np.arange(m) + 1) / m)[:, None] * (q[i] - prev[i]))
            owner += [i] * m
        c, lk, ob = _evaluate(rule, np.vstack(subs), tool)
        owner = np.array(owner)
        for i in steps:
            sel = np.flatnonzero(owner == i)
            k = sel[np.argmin(c[sel])]
            if c[k] < clr[i]:
                clr[i] = c[k]
                label[i] = (w.link_names[lk[k]], w.obstacle_names[ob[k]]) if ob[k] >= 0 else None
    t = (np.arange(n) + 1) * dt
    issues, i = [], 0
    while i < n:
        if not clr[i] < 0:
            i += 1
            continue
        j = i
        while j + 1 < n and clr[j + 1] < 0 and label[j + 1] == label[i]:
            j += 1
        link, obstacle = label[i]
        worst = float(clr[i:j + 1].min())
        issues.append(trajectory.Issue(
            i, float(t[i]), "collision", "error",
            f"{_label(link)} hits {obstacle} at {trajectory.step_range(i, j)} "
            f"(clearance {worst * 1e3:.0f} mm)",
        ))
        i = j + 1
    if coarse and not issues:
        return collision_issues(pieces, q_start, tool, dt)
    return issues, clr


def _travel(transit: _Transit) -> np.ndarray:
    """(dof,) radians each joint turns along the transit's path (a Cartesian one's dense
    steps)."""
    path = np.asarray(transit.path, dtype=np.float64)
    if transit.chain is not None:
        path = np.vstack([path[:1], transit.chain.q])
    return np.abs(np.diff(path, axis=0)).sum(axis=0)


def _swing_cost(transit: _Transit) -> float:
    t = _travel(transit)
    return float(t.sum() + t[-1])


def plan_search(
    kind, *, scene, tool, dt, q_start, gripper_start, variants, transit_grip, speed_scale,
    settle_s, time_budget_s, seed, t_begin, transit_scale=None, transit_accel=ACCEL_MAX,
    prefix=(), swing=None, provider=None, held=None,
) -> trajectory.PlanResult:
    """Plan variants[0] (the exact request); the others only when it has no clean plan."""
    prefix = list(prefix)
    q_from = prefix[-1].q[-1] if prefix else q_start
    rng = np.random.default_rng(seed)
    hard = t_begin + float(time_budget_s)
    deadline = hard - min(REPORT_RESERVE_S, 0.25 * float(time_budget_s))
    tscale = transit_scale or speed_scale
    common = {"q_start": q_from, "tool": tool, "dt": dt, "speed_scale": speed_scale, "rng": rng,
              "tscale": tscale, "taccel": transit_accel}
    ext = None if provider is None else _Ext(provider, scene, held)
    if ext is not None:
        common["ext"] = ext
    exact, chk = _search(variants[0].make(), deadline=deadline, **common)
    used, used_v, used_chk, tried, quiet = exact, variants[0], chk, 1, False
    if exact.chosen is None and not any(b.clean for b in exact.branches):
        reserve = FALLBACK_RESERVE_FRACTION if not exact.branches else VARIANT_RESERVE_FRACTION
        v_deadline = deadline - reserve * float(time_budget_s)
        for v in variants[1:]:
            if clock.now() > v_deadline:
                break
            setup = v.make()
            if setup is None:
                continue
            tried += 1
            s, c = _search(setup, deadline=deadline, max_branches=VARIANT_BRANCHES,
                           seeds=_variant_seeds(exact.branches, v.twist), clean_only=True,
                           **common)
            if s.chosen is not None:
                used, used_v, used_chk = s, v, c
                break
    elif swing is not None and exact.transit is not None and exact.transit.path is not None:
        t = _travel(exact.transit)
        now = clock.now()
        if (t[-1] > SWING_J7_RAD or t.sum() > SWING_TRAVEL_RAD) and now < deadline:
            setup = swing.make()
            if setup is not None:
                tried += 1
                s, c = _search(setup, deadline=min(deadline, now + SWING_TIME_FRACTION
                                                   * float(time_budget_s)),
                               max_branches=SWING_BRANCHES, clean_only=True,
                               seeds=_variant_seeds(exact.branches, swing.twist), **common)
                if (s.chosen is not None and _swing_cost(s.transit)
                        < _swing_cost(exact.transit) - SWING_GAIN_RAD):
                    used, used_v, used_chk, quiet = s, swing, c, True
    if not used.branches:
        r = used.best_ik
        goal_free = bool(chk.clearance(r.q[None])[0] >= 0)
        b = _branch(r, used.setup, chk, goal_free, q_from, tool, dt, speed_scale, tscale,
                    transit_accel, deadline)
        b.check(tool, deadline)
        used.branches.append(b)
    setup, chk = used.setup, used_chk
    T_pre, transit_rule = setup.T_pre, setup.transit_rule
    segment_issues = list(setup.segment_issues)
    if not used_v.exact and not quiet:
        segment_issues.append(("close" if kind == "pick" else "release", "variant", "warn",
                               _variant_message(kind, used_v, exact)))
    transit, notes, sols = used.transit, used.notes, used.sols

    issues = []
    if used.chosen is None:
        b = used.branches[0]
        r, pieces = b.r, b.pieces
        score = (b.clean, b.goal_free, b.kin_ok, b.free)
        no_path = False
        if not sols:
            reason = "no accurate IK solution for the pre-pose"
        elif not score[1]:
            c = _evaluate(transit_rule, r.q[None], tool)
            contact = transit_rule[0].contact(c[0][0], c[1][0], c[2][0])
            reason = (
                f"every IK solution of the pre-pose collides ({_label(contact.link or '')} "
                f"hits {contact.obstacle}, clearance {contact.clearance * 1e3:.0f} mm)"
            )
            no_path = True
        elif not score[2]:
            reason = "no IK branch reaches through the Cartesian segments"
        elif b.singular:
            reason = "every IK branch holds joint 2 near 0 (the shoulder singularity)"
        elif score[3] is None:
            reason = "the time budget ran out checking the Cartesian segments for collisions"
        elif not score[3]:
            reason = "every IK branch collides during the Cartesian segments"
        else:
            reason = notes[-1] if notes else "no collision-free transit found"
            no_path = True
        fallback = "direct" if ext is None else ext.provider.name
        method = transit.method if transit is not None else fallback
        if score[1] and not score[0]:
            if ext is None:
                transit = _transit(chk, q_from, r.q, rng, deadline, transit_rule[0].margin)
            else:
                transit = _ext_transit(ext, q_from, r.q, tool, dt, tscale, transit_accel, T_pre,
                                       r, deadline)
            method = transit.method
            if transit.path is None:
                reason, no_path = f"{reason}; {transit.message}", True
        if transit is None or transit.path is None:
            transit = _Transit([q_from, r.q], method, reason)
        else:
            transit = _Transit(transit.path, transit.method, reason)
        if no_path:
            issues.append(trajectory.Issue(
                0, dt, "no_path", "error", f"no collision-free transit: {reason}"
            ))
    else:
        r, pieces = used.chosen.r, used.chosen.pieces

    if transit.chain is not None:
        ch = transit.chain
        q_tr, tcp_tr, pe_tr, re_tr = ch.q, ch.tcp, ch.pos_err, ch.rot_err
    else:
        q_tr = _time_path(transit.path, dt, tscale, transit_accel, _qd_max(arm_of(tool)))
        tcp_tr = np.array([pose7(tool.fk_tcp(qi)) for qi in q_tr])
        tcp_tr[-1] = pose7(T_pre)
        pe_tr, re_tr = np.zeros(len(q_tr)), np.zeros(len(q_tr))
        pe_tr[-1], re_tr[-1] = r.pos_err, r.rot_err
    pieces = prefix + [Piece("transit", q_tr, tcp_tr, transit_grip, pe_tr, re_tr,
                              [transit_rule] * len(q_tr))] + pieces

    col_issues, clr = collision_issues(pieces, q_start, tool, dt, hard)
    q = np.concatenate([p.q for p in pieces])
    tcp = np.concatenate([p.tcp for p in pieces])
    grip = np.concatenate([np.full(len(p.q), p.grip) for p in pieces])
    pos_err = np.concatenate([p.pos_err for p in pieces])
    rot_err = np.concatenate([p.rot_err for p in pieces])
    n_settle = trajectory.settle_step_count(settle_s, dt)
    segments, start = [], 0
    for p in pieces:
        end = start + len(p.q) - 1
        if segments and segments[-1]["name"] == p.name:
            segments[-1]["end"] = end
        else:
            segments.append({"name": p.name, "start": start, "end": end})
        start = end + 1
    segments[-1]["end"] += n_settle
    first = {}
    for sg in segments:
        first.setdefault(sg["name"], sg["start"])
    for name, kind_, severity, message in segment_issues:
        if name in first:
            i = first[name]
            issues.append(trajectory.Issue(i, (i + 1) * dt, kind_, severity, message))
    worlds = {id(rule[0]): rule[0] for p in pieces for rule in p.rules}
    extra = {
        "segments": segments,
        "planner": {
            "method": transit.method,
            "time_s": round(clock.now() - t_begin, 3),
            "checks": int(sum(w.checks for w in worlds.values())),
            "min_clearance_m": round(float(np.min(clr) + MARGIN), 4),
            "message": transit.message,
        },
        "variant": used_v.info(),
        "tcp_used": [float(x) for x in pose7(setup.tcp)],
        "search": {"branches": len(used.branches), "solutions": len(sols), "variants": tried},
    }
    t = (np.arange(len(q)) + 1) * dt
    return trajectory.finish(
        kind, dt, t, q, tcp, grip, tool, q_start, gripper_start,
        pos_err=pos_err, rot_err=rot_err, speed_scale=STAGE_SPEED_SCALE,
        table_z=float(getattr(scene, "table_z", 0.0)), settle_s=settle_s,
        extra_issues=issues + col_issues, extra=extra, singular_hold_error=True,
        singular_moving=_moving_span(segments),
    )


def _moving_span(segments) -> list[tuple[int, int]]:
    """The leading escape and transit steps: joint-space motion, which the branch cannot steer."""
    span = [(sg["start"], sg["end"]) for sg in segments if sg["name"] in ("escape", "transit")]
    return [(span[0][0], span[-1][1])] if span else []
