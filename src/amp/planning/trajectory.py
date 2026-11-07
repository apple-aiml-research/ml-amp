#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Joint trajectories: PlanResult, waypoint chunks, densify, validation, hold and home."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from amp.geometry.embodiment import DHArm, Embodiment
from amp.geometry.kinematics import (
    ToolModel,
    make_pose,
    pose7,
    pose_parts,
    quat_normalize,
    quat_slerp,
    solve_ik,
)
from amp.planning.robot import arm_of, check_tool, plan_arm

MAX_DURATION_S = 60.0
UNREACHABLE_POS_M = 0.005
UNREACHABLE_ROT_RAD = math.radians(3.0)
JOINT_LIMIT_MARGIN = 0.02
SINGULAR_HOLD_S = 1.0
SINGULAR_CROSS_SPEED = 0.1
VEL_ERROR = 1.0
VEL_WARN = 0.9
VEL_MARGIN = 1.3
TABLE_TCP_ERROR = -0.005
TABLE_TCP_WARN = 0.015
TABLE_LINK_ERROR = -0.02
TABLE_CONTACT_ERROR = -0.01
INTERPS = ("minjerk", "linear")
_REST_GAIN = 0.02


@dataclass
class Waypoint:
    """A TCP pose and gripper command at time t."""

    t: float
    pos: np.ndarray
    quat: np.ndarray
    gripper: float


@dataclass
class Issue:
    """A validation finding at a step: kind, severity and message."""

    step: int
    t: float
    kind: str
    severity: str
    message: str


def _r(a, digits: int) -> list:
    return np.round(np.asarray(a, dtype=np.float64), digits).tolist()


def _pose7_json(p: np.ndarray) -> list:
    p = np.asarray(p, dtype=np.float64)
    return np.concatenate(
        [np.round(p[..., :3], 5), np.round(p[..., 3:], 6)], axis=-1
    ).tolist()


@dataclass
class PlanResult:
    """A validated joint trajectory: actions, poses, gripper, residuals and issues."""

    ok: bool
    kind: str
    dt: float
    t: np.ndarray
    actions: np.ndarray
    q: np.ndarray
    tcp_planned: np.ndarray
    tcp_fk: np.ndarray
    gripper: np.ndarray
    links: np.ndarray
    pos_err: np.ndarray
    rot_err: np.ndarray
    vel_ratio: np.ndarray
    issues: list[Issue]
    min_duration: float
    start_q: np.ndarray
    start_tcp: np.ndarray
    start_gripper: float
    settle_steps: int = 0
    extra: dict = field(default_factory=dict)
    ee: str | None = None

    def to_json(self) -> dict:
        """The preview JSON, plain python types only."""
        out = {
            "ok": bool(self.ok),
            "kind": self.kind,
            "dt": float(self.dt),
            "n_steps": len(self.t),
            "duration": round(float(self.t[-1]), 6) if len(self.t) else 0.0,
            "start": {
                "pos": _r(self.start_tcp[:3], 5),
                "quat": _r(self.start_tcp[3:], 6),
                "gripper": float(self.start_gripper),
                "q": _r(self.start_q, 6),
            },
            "t": _r(self.t, 6),
            "tcp_planned": _pose7_json(self.tcp_planned),
            "tcp_fk": _pose7_json(self.tcp_fk),
            "gripper": _r(self.gripper, 6),
            "q": _r(self.q, 6),
            "links": _r(self.links, 4),
            "pos_err_mm": _r(self.pos_err * 1e3, 3),
            "rot_err_deg": _r(np.degrees(self.rot_err), 3),
            "vel_ratio": _r(self.vel_ratio, 4),
            "issues": [
                {
                    "step": int(i.step),
                    "t": round(float(i.t), 6),
                    "kind": i.kind,
                    "severity": i.severity,
                    "message": i.message,
                }
                for i in self.issues
            ],
            "min_duration": round(float(self.min_duration), 4),
            "settle_steps": int(self.settle_steps),
        }
        out.update(self.extra)
        return out


def _num(x, name: str) -> float:
    if isinstance(x, (bool, np.bool_)) or not isinstance(x, (int, float, np.integer, np.floating)):
        raise ValueError(f"{name} must be a number, got {x!r}")  # noqa: TRY004
    v = float(x)
    if not math.isfinite(v):
        raise ValueError(f"{name} must be finite, got {x!r}")
    return v


def _vec(x, n: int, name: str) -> np.ndarray:
    if not isinstance(x, (list, tuple, np.ndarray)) or len(x) != n:
        raise ValueError(f"{name} must be a list of {n} numbers, got {x!r}")
    return np.array([_num(v, f"{name}[{i}]") for i, v in enumerate(x)])


def parse_plan(plan_json: dict) -> tuple[list[Waypoint], str, float | None]:
    """Validate a plan JSON; returns (waypoints, interp, speed_scale)."""
    if not isinstance(plan_json, dict):
        raise ValueError("plan must be a JSON object")  # noqa: TRY004
    raw = plan_json.get("waypoints")
    if not isinstance(raw, list) or not raw:
        raise ValueError("plan needs a non-empty 'waypoints' list")
    interp = plan_json.get("interp", "minjerk")
    if interp not in INTERPS:
        raise ValueError(f"interp must be one of {list(INTERPS)}, got {interp!r}")
    speed_scale = plan_json.get("speed_scale")
    if speed_scale is not None:
        speed_scale = _num(speed_scale, "speed_scale")
        if not 0.0 < speed_scale <= 1.0:
            raise ValueError(f"speed_scale must be in (0, 1], got {speed_scale}")

    waypoints: list[Waypoint] = []
    t_prev = 0.0
    for i, w in enumerate(raw):
        name = f"waypoints[{i}]"
        if not isinstance(w, dict):
            raise ValueError(f"{name} must be an object")  # noqa: TRY004
        for key in ("t", "pos", "quat", "gripper"):
            if key not in w:
                raise ValueError(f"{name} is missing '{key}'")
        t = _num(w["t"], f"{name}.t")
        if t <= 0:
            raise ValueError(f"{name}.t must be > 0, got {t}")
        if i > 0 and t <= t_prev:
            raise ValueError(f"{name}.t = {t} must be greater than the previous t = {t_prev}")
        pos = _vec(w["pos"], 3, f"{name}.pos")
        quat = _vec(w["quat"], 4, f"{name}.quat")
        try:
            quat = quat_normalize(quat)
        except ValueError:
            raise ValueError(f"{name}.quat must be a non-zero quaternion") from None
        g = w["gripper"]
        if isinstance(g, bool):
            gripper = float(g)
        else:
            gripper = 1.0 if _num(g, f"{name}.gripper") > 0.5 else 0.0
        waypoints.append(Waypoint(t=t, pos=pos, quat=quat, gripper=gripper))
        t_prev = t
    if t_prev > MAX_DURATION_S:
        raise ValueError(f"plan duration {t_prev} s exceeds the {MAX_DURATION_S} s maximum")
    return waypoints, interp, speed_scale


def _n_steps(duration: float, dt: float) -> int:
    if not dt > 0:
        raise ValueError(f"dt must be > 0, got {dt}")
    return max(1, round(duration / dt))


def densify(
    start_pos, start_quat, start_gripper, waypoints, dt, interp="minjerk"
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t (N,), tcp pose7 (N,7), gripper (N,)) sampled at the end of every control step."""
    if not waypoints:
        raise ValueError("densify needs at least one waypoint")
    if interp not in INTERPS:
        raise ValueError(f"interp must be one of {list(INTERPS)}, got {interp!r}")
    times = np.array([0.0] + [float(w.t) for w in waypoints])
    pos = np.vstack([np.asarray(start_pos, dtype=np.float64)] + [w.pos for w in waypoints])
    quats = [quat_normalize(start_quat)] + [quat_normalize(w.quat) for w in waypoints]
    grips = np.array([float(w.gripper) for w in waypoints])
    duration = times[-1]
    n = _n_steps(duration, dt)
    t = (np.arange(n) + 1) * dt
    te = np.minimum(t, duration)
    te[-1] = duration

    tcp = np.empty((n, 7))
    gripper = np.empty(n)
    for i, ti in enumerate(te):
        k = int(np.clip(np.searchsorted(times, ti, side="left"), 1, len(times) - 1))
        tau = (ti - times[k - 1]) / (times[k] - times[k - 1])
        tau = min(max(tau, 0.0), 1.0)
        s = tau if interp == "linear" else tau**3 * (10 - 15 * tau + 6 * tau * tau)
        tcp[i, :3] = pos[k - 1] + s * (pos[k] - pos[k - 1])
        tcp[i, 3:] = quat_slerp(quats[k - 1], quats[k], s)
        reached = np.nonzero(times[1:] <= ti + 1e-9)[0]
        gripper[i] = grips[reached[-1]] if len(reached) else float(start_gripper)
    tcp[-1, :3] = pos[-1]
    tcp[-1, 3:] = quats[-1]
    return t, tcp, gripper


def step_range(a: int, b: int) -> str:
    """'step a' or 'steps a-b'."""
    return f"step {a}" if a == b else f"steps {a}-{b}"


def _runs(sev: np.ndarray):
    """Yield (start, end, severity) for runs of equal non-zero severity."""
    i, n = 0, len(sev)
    while i < n:
        if sev[i] == 0:
            i += 1
            continue
        j = i
        while j + 1 < n and sev[j + 1] == sev[i]:
            j += 1
        yield i, j, int(sev[i])
        i = j + 1


_SEVERITY = {1: "warn", 2: "error"}


def _validate(
    t, q, tcp_fk, links, pos_err, rot_err, vel_ratio, *, table_z, check_table=True, arm=None,
    table_contact=False,
) -> list[Issue]:
    arm = plan_arm(None) if arm is None else arm
    issues: list[Issue] = []

    def emit(kind, sev, message_of):
        for a, b, s in _runs(sev):
            issues.append(Issue(a, float(t[a]), kind, _SEVERITY[s], message_of(a, b, s)))

    bad = (pos_err > UNREACHABLE_POS_M) | (rot_err > UNREACHABLE_ROT_RAD)
    emit(
        "unreachable",
        np.where(bad, 2, 0),
        lambda a, b, s: (
            f"unreachable at {step_range(a, b)} (worst {pos_err[a:b + 1].max() * 1e3:.1f} mm, "
            f"{math.degrees(rot_err[a:b + 1].max()):.1f} deg)"
        ),
    )

    margin = np.minimum(q - arm.q_min, arm.q_max - q)

    def joint_msg(a, b, s):
        m = margin[a:b + 1]
        i, j = np.unravel_index(np.argmin(m), m.shape)
        return (
            f"joint {j + 1} near its limit at {step_range(a, b)} "
            f"(closest {max(m[i, j], 0.0):.3f} rad)"
        )

    emit("joint_limit", np.where(margin.min(axis=1) < JOINT_LIMIT_MARGIN, 1, 0), joint_msg)

    vel_sev = np.where(vel_ratio > VEL_ERROR, 2, np.where(vel_ratio > VEL_WARN, 1, 0))

    def vel_msg(a, b, s):
        worst = f"worst {vel_ratio[a:b + 1].max():.2f}x the allowed speed"
        if s != 2:
            return f"near the joint velocity limit at {step_range(a, b)} ({worst})"
        md = float(t[-1] * vel_ratio.max())
        need, safe = (math.ceil(x * 100.0 - 1e-6) / 100.0 for x in (md, md * VEL_MARGIN))
        return (f"too fast at {step_range(a, b)} ({worst}): needs at least {need:.2f} s "
                f"(min_duration, {safe:.2f} s with margin), this one is {float(t[-1]):.2f} s")

    emit("velocity", vel_sev, vel_msg)

    if check_table:
        tcp_z = tcp_fk[:, 2] - table_z
        link_z = links[:, 1:-1, 2].min(axis=1) - table_z
        tab_sev = np.where(
            (tcp_z < TABLE_TCP_ERROR) | (link_z < TABLE_LINK_ERROR),
            2,
            np.where(tcp_z < TABLE_TCP_WARN, 1, 0),
        )
        if table_contact:
            arm_z = links[:, 1:-2, 2].min(axis=1) - table_z
            ee_z = np.minimum(tcp_z, links[:, -2, 2] - table_z)
            tab_sev = np.where((arm_z < TABLE_LINK_ERROR) | (ee_z < TABLE_CONTACT_ERROR), 2, 0)

        def table_msg(a, b, s):
            if table_contact:
                az, ez = arm_z[a:b + 1].min(), ee_z[a:b + 1].min()
                if az < TABLE_LINK_ERROR:
                    return (f"arm link below the table at {step_range(a, b)} "
                            f"(lowest {az * 1e3:.1f} mm)")
                return (f"tool pressed into the table at {step_range(a, b)} "
                        f"(lowest {ez * 1e3:.1f} mm, at most {-TABLE_CONTACT_ERROR * 1e3:.0f} mm)")
            tz, lz = tcp_z[a:b + 1].min(), link_z[a:b + 1].min()
            if s == 2 and lz < TABLE_LINK_ERROR and not tz < TABLE_TCP_ERROR:
                return f"arm link below the table at {step_range(a, b)} (lowest {lz * 1e3:.1f} mm)"
            what = "TCP below the table" if s == 2 else "TCP close to the table"
            return f"{what} at {step_range(a, b)} (lowest {tz * 1e3:.1f} mm)"

        emit("table", tab_sev, table_msg)

    issues.sort(key=lambda i: (i.step, i.severity != "error"))
    return issues


def _singular_bands(q, dt, arm) -> list[tuple[int, int, int, bool]]:
    """(joint, first, last, held) per run of steps within a declared singularity band."""
    q = np.asarray(q)
    out = []
    for sing in arm.singularities:
        qj = q[:, sing.joint]
        for a, b, _ in _runs((np.abs(qj) < sing.min_abs).astype(np.int64)):
            dur = (b - a + 1) * dt
            crossed = 0 < a and b + 1 < len(qj) and qj[a - 1] * qj[b + 1] < 0 and (
                abs(qj[b + 1] - qj[a - 1]) / (dur + dt) >= SINGULAR_CROSS_SPEED)
            out.append((sing.joint, a, b, dur >= SINGULAR_HOLD_S and not crossed))
    return out


def singular_runs(q, dt, arm=None) -> list[tuple[int, int, bool]]:
    """(first, last, held) for each run of steps with a joint inside its singular band."""
    arm = plan_arm(None) if arm is None else arm
    return [(a, b, held) for _, a, b, held in _singular_bands(q, dt, arm)]


def _singular_issues(t, q, dt, hold_error, moving=(), arm=None) -> list[Issue]:
    """hold_error: a stay is an error, unless the plan starts in the band (it can only leave) or
    the stay lies within the `moving` step ranges (joint-space transits: warned)."""
    arm = plan_arm(None) if arm is None else arm
    zone = {s.joint: s.min_abs for s in arm.singularities}
    issues = []
    for j, a, b, held in _singular_bands(q, dt, arm):
        dur = (b - a + 1) * dt
        in_motion = any(lo <= a and b <= hi for lo, hi in moving)
        error = held and hold_error and a > 0 and not in_motion
        what = "starts" if a == 0 else "stays" if held else "passes"
        if isinstance(arm, DHArm):
            why = ("the shoulder singularity, where joint 1 can drift off its target; "
                   f"keep joint {j + 1} further from 0")
        else:
            why = f"a singularity of the arm; keep joint {j + 1} further from 0"
        issues.append(Issue(a, float(t[a]), "singular", "error" if error else "warn", (
            f"joint {j + 1} {what} within {zone[j]:.2f} rad of 0 at {step_range(a, b)} "
            f"({dur:.1f} s): {why}")))
    return issues


def _vel_ratio(q: np.ndarray, start_q: np.ndarray, dt: float, speed_scale: float,
               qd_max=None) -> np.ndarray:
    qd_max = plan_arm(None).qd_max if qd_max is None else qd_max
    qd = np.diff(np.vstack([start_q, q]), axis=0) / dt
    return np.max(np.abs(qd) / (qd_max * speed_scale), axis=1)


def settle_step_count(settle_s, dt) -> int:
    """How many dt steps settle_s seconds of settling hold."""
    s = _num(settle_s, "settle_s")
    if s < 0:
        raise ValueError(f"settle_s must be >= 0, got {s}")
    return round(s / dt)


def finish(
    kind, dt, t, q, tcp_planned, gripper, tool, start_q, start_gripper, *,
    pos_err=None, rot_err=None, speed_scale=0.8, table_z=0.0, check_table=True, settle_s=0.0,
    extra_issues=(), extra=None, singular_hold_error=False, singular_moving=(),
    table_contact=False,
) -> PlanResult:
    """Validate a planned joint trajectory and assemble its PlanResult (the tool's arm's)."""
    arm = arm_of(tool)
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError(f"speed_scale must be in (0, 1], got {speed_scale}")
    n_move = len(t)
    pos_err = np.zeros(n_move) if pos_err is None else pos_err
    rot_err = np.zeros(n_move) if rot_err is None else rot_err
    n_settle = settle_step_count(settle_s, dt)
    if n_settle:
        t = np.concatenate([t, t[-1] + (np.arange(n_settle) + 1) * dt])
        q, tcp_planned, gripper, pos_err, rot_err = (
            np.concatenate([a, np.repeat(np.asarray(a)[-1:], n_settle, axis=0)])
            for a in (q, tcp_planned, gripper, pos_err, rot_err)
        )
    n = len(t)
    tcp_fk = np.array([pose7(tool.fk_tcp(qi)) for qi in q])
    links = np.array([tool.link_points(qi) for qi in q])
    vel_ratio = _vel_ratio(q, start_q, dt, speed_scale, arm.qd_max)
    m = n_move
    issues = _validate(
        t[:m], q[:m], tcp_fk[:m], links[:m], pos_err[:m], rot_err[:m], vel_ratio[:m],
        table_z=table_z, check_table=check_table, arm=arm, table_contact=table_contact,
    )
    issues += _singular_issues(t, q, dt, singular_hold_error, singular_moving, arm)
    issues = sorted([*issues, *extra_issues], key=lambda i: (i.step, i.severity != "error"))
    actions = np.empty((n, arm.n + 1), dtype=np.float32)
    actions[:, :arm.n] = q
    actions[:, arm.n] = gripper
    return PlanResult(
        ok=not any(i.severity == "error" for i in issues),
        kind=kind,
        dt=float(dt),
        t=t,
        actions=actions,
        q=q,
        tcp_planned=tcp_planned,
        tcp_fk=tcp_fk,
        gripper=np.asarray(gripper, dtype=np.float64),
        links=links,
        pos_err=pos_err,
        rot_err=rot_err,
        vel_ratio=vel_ratio,
        issues=issues,
        min_duration=float(t[m - 1] * vel_ratio[:m].max()),
        start_q=start_q.copy(),
        start_tcp=pose7(tool.fk_tcp(start_q)),
        start_gripper=float(start_gripper),
        settle_steps=n_settle,
        extra=dict(extra or {}),
    )


def plan_chunk(
    q_start, gripper_start, tool: ToolModel, waypoints, dt, *,
    interp="minjerk", speed_scale=0.8, table_z=0.0, check_table=True, settle_s=0.0,
    embodiment: Embodiment | None = None, table_contact=False,
) -> PlanResult:
    """Densify waypoints from the current TCP pose, solve IK per step, and validate."""
    arm = check_tool(embodiment, tool)
    if table_contact and (embodiment is None or embodiment.gripper() is not None):
        raise ValueError("table_contact is only for an end effector without a gripper")
    q_start = np.asarray(q_start, dtype=np.float64)
    t, tcp, gripper, q, pos_err, rot_err = waypoint_path(
        q_start, gripper_start, tool, waypoints, dt, interp, arm)
    return finish(
        "plan", dt, t, q, tcp, gripper, tool, q_start, gripper_start,
        pos_err=pos_err, rot_err=rot_err, speed_scale=speed_scale, table_z=table_z,
        check_table=check_table, settle_s=settle_s, table_contact=table_contact,
    )


def waypoint_path(q_start, gripper_start, tool: ToolModel, waypoints, dt, interp, arm):
    """plan_chunk's unvalidated path: (t, tcp, gripper, q, pos_err, rot_err), IK per step."""
    q_start = np.asarray(q_start, dtype=np.float64)
    start_pos, start_quat = pose_parts(tool.fk_tcp(q_start))
    t, tcp, gripper = densify(start_pos, start_quat, gripper_start, waypoints, dt, interp)
    n = len(t)
    T_ft = tool.T_flange_tcp
    q = np.empty((n, arm.n))
    pos_err = np.empty(n)
    rot_err = np.empty(n)
    q_prev = q_start
    solve = solve_ik if isinstance(arm, DHArm) else arm.ik
    for i in range(n):
        res = solve(
            make_pose(tcp[i, :3], tcp[i, 3:]), q_prev, T_ft, q_rest=q_start, rest_gain=_REST_GAIN,
            singular_push=False,
        )
        q[i], pos_err[i], rot_err[i] = res.q, res.pos_err, res.rot_err
        q_prev = res.q
    return t, tcp, gripper, q, pos_err, rot_err


def _check_duration(duration) -> float:
    d = _num(duration, "duration")
    if not 0.0 < d <= MAX_DURATION_S:
        raise ValueError(f"duration must be in (0, {MAX_DURATION_S}] s, got {d}")
    return d


def _stationary(kind, q_start, gripper, tool, duration, dt) -> PlanResult:
    q_start = np.asarray(q_start, dtype=np.float64)
    n = _n_steps(_check_duration(duration), dt)
    t = (np.arange(n) + 1) * dt
    q = np.tile(q_start, (n, 1))
    tcp = np.tile(pose7(tool.fk_tcp(q_start)), (n, 1))
    g = np.full(n, float(gripper))
    return finish(kind, dt, t, q, tcp, g, tool, q_start, gripper, check_table=False)


def plan_hold(q_start, gripper, tool: ToolModel, duration, dt, *,
              embodiment: Embodiment | None = None) -> PlanResult:
    """Hold the joint targets and the gripper command."""
    check_tool(embodiment, tool)
    return _stationary("hold", q_start, gripper, tool, duration, dt)


def plan_gripper(q_start, closed, tool: ToolModel, duration, dt, *,
                 embodiment: Embodiment | None = None) -> PlanResult:
    """Hold the arm and command the gripper closed (1) or open (0) from the first step."""
    check_tool(embodiment, tool)
    return _stationary("gripper", q_start, 1.0 if closed else 0.0, tool, duration, dt)


def plan_home(
    q_start, tool: ToolModel, duration, dt, *, q_goal=None, speed_scale=0.8, settle_s=0.0,
    embodiment: Embodiment | None = None, check_table=True,
) -> PlanResult:
    """Joint-space min-jerk move to q_goal (the arm's home when None) with the gripper open,
    then settle_s of holding it; check_table=False drops the table checks."""
    arm = check_tool(embodiment, tool)
    q_start = np.asarray(q_start, dtype=np.float64)
    q_goal = arm.q_home if q_goal is None else q_goal
    q_goal = np.clip(np.asarray(q_goal, dtype=np.float64), arm.q_min, arm.q_max)
    d = _check_duration(duration)
    n = _n_steps(d, dt)
    t = (np.arange(n) + 1) * dt
    tau = np.minimum(t / (n * dt), 1.0)
    tau[-1] = 1.0
    s = tau**3 * (10 - 15 * tau + 6 * tau * tau)
    q = q_start + s[:, None] * (q_goal - q_start)
    tcp = np.array([pose7(tool.fk_tcp(qi)) for qi in q])
    return finish(
        "home", dt, t, q, tcp, np.zeros(n), tool, q_start, 0.0, speed_scale=speed_scale,
        settle_s=settle_s, check_table=check_table,
    )
