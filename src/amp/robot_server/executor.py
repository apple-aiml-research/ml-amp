#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Execution: plans become chunks of link actions, with lead tracking, holds, stop and accounting."""

from __future__ import annotations

import math
import re
import time
from typing import NamedTuple

import numpy as np

from amp.geometry import kinematics as kin
from amp.link.obs import env_error_logged
from amp.link.wire import jsonable
from amp.perception.facts import fact_strings, scene_facts
from amp.perception.held import (
    GRIP_SETTLED,
    HELD_GRIPPER_SAMPLES,
    grip_lost_step,
    grip_state,
)
from amp.perception.params import embodiment_or_droid
from amp.perception.tools import HeldCheck
from amp.robot_server import embodiment as rt
from amp.robot_server.errors import SessionConflict

HELD_CLOSED_READING = 0.96
HELD_JAM_READING = 0.03
TRACK_FIT_MIN_RAD = 2e-3
TRACK_FIT_MAX_RAD = 0.25
TRACK_MIN_SAMPLES = 30
DRIVE_TAU_S = 0.2
TRACK_TAU_BAND_S = (0.16, 0.23)
TRACK_INLIER_FRAC = 0.2
TRACK_ABORT_STEPS = 3
LEAD_MAX_RAD = 0.3
HOLD_OFFSET_MAX_RAD = np.array([0.03, 0.03, 0.03, 0.03, 0.2, 0.2, 0.2])
HOLD_RELEASE_DECAY = 0.5
STALE = "the robot moved since this observation; preview again"
ABANDONING = "the episode is being abandoned"


def held_back_message(mm: float, who: str = "", proposals: str = "v1") -> str:
    """The stop reason of an opening held back with the arm mm off the planned release."""
    msg = f"{who}release held back: {mm:.0f} mm off target"
    if proposals != "v2":
        return msg
    return (msg + "; the arm is not where the plan releases, likely held up by contact: "
            "re-plan the place from where the arm is now rather than opening here")


class Tracker:
    """The sim's joint drive as a first-order lag q' = a q + (1 - a) u, fitted online, inverted."""

    def __init__(self, embodiment=None):
        self.dt = None
        self.reset(None)
        self.configure(embodiment)

    def configure(self, embodiment) -> None:
        """The drive priors and joint limits of an embodiment runtime (None: DROID's)."""
        self.group_tau = None
        if embodiment is None:
            self.tau_s, self.tau_band = DRIVE_TAU_S, TRACK_TAU_BAND_S
            self.hold_max = HOLD_OFFSET_MAX_RAD
            self.q_min, self.q_max = kin.Q_MIN, kin.Q_MAX
            self.step_clamp = None
        else:
            d = embodiment.spec.drive
            lim = rt.limits(embodiment)
            self.tau_s, self.tau_band = d.tau_s, d.tau_band_s
            self.q_min, self.q_max = lim["q_min"], lim["q_max"]
            if embodiment.spec.joint_groups:
                self._configure_groups(embodiment)
            else:
                self.hold_max = np.array(d.hold_offset_max or (0.0,) * embodiment.dof)
                self.step_clamp = np.array(d.step_clamp) if d.step_clamp else None

    def _configure_groups(self, embodiment) -> None:
        """Per-joint priors over q = arms then joint groups: each group's own, unknown ones safe."""
        spec = embodiment.spec
        d = spec.drive
        by = {g.target: g for g in d.groups}
        arm_hold = d.hold_offset_max or (0.0,) * spec.arm_dof
        hold, clamp, tau = list(arm_hold), list(d.step_clamp or (np.inf,) * spec.arm_dof), []
        for g in spec.joint_groups:
            gd = by.get(g.name)
            n = len(g.joints)
            hold += list(gd.hold_offset_max) if gd is not None and gd.hold_offset_max else [0.0] * n
            if gd is not None and gd.step_clamp:
                clamp += list(gd.step_clamp)
            else:
                clamp += [v * spec.control_dt for v in g.qd_max]
            tau += [gd.tau_s if gd is not None and gd.tau_s is not None else None] * n
        self.hold_max = np.array(hold, dtype=np.float64)
        self.step_clamp = np.array(clamp, dtype=np.float64)
        self.group_tau = (spec.arm_dof, tau)

    def reset(self, dt) -> None:
        self.dt = float(dt) if dt else None
        self.sxx = self.sxy = 0.0
        self.samples = self.rejected = 0

    def _pole(self, tau) -> float:
        return float(np.exp(-self.dt / tau))

    @property
    def alpha_fit(self):
        if self.samples < TRACK_MIN_SAMPLES or self.sxx <= 0.0:
            return None
        return 1.0 - self.sxy / self.sxx

    @property
    def alpha(self):
        """The pole the lead uses: the fit clamped to the drive's band, or None before a fit."""
        a = self.alpha_fit
        if a is None or self.dt is None:
            return None
        lo, hi = (self._pole(t) for t in self.tau_band)
        return float(np.clip(a, lo, hi))

    def observe(self, u, q_prev, q_now, mask=None) -> None:
        """Fit one step: target u sent from q_prev, q_now observed after it (joints in mask)."""
        if self.dt is None:
            return
        x = np.asarray(u, dtype=np.float64) - q_prev
        y = np.asarray(q_now, dtype=np.float64) - q_prev
        ax = np.abs(x)
        m = (ax > TRACK_FIT_MIN_RAD) & (ax < TRACK_FIT_MAX_RAD)
        if mask is not None:
            m &= mask
        inlier = np.abs(y - (1.0 - self._pole(self.tau_s)) * x) <= TRACK_INLIER_FRAC * ax
        self.rejected += int((m & ~inlier).sum())
        m &= inlier
        if m.any():
            self.sxx += float(x[m] @ x[m])
            self.sxy += float(x[m] @ y[m])
            self.samples += int(m.sum())

    def lead(self, p, p_prev):
        """The target that makes the modelled arm reach p after one step from p_prev."""
        a = self.alpha
        p = np.asarray(p, dtype=np.float64)
        if self.group_tau is not None and p_prev is not None:
            return self._lead_groups(p, p_prev, a)
        if not a or p_prev is None:
            return p
        off = np.clip(a / (1.0 - a) * (p - p_prev), -LEAD_MAX_RAD, LEAD_MAX_RAD)
        return np.clip(p + off, self.q_min, self.q_max)

    def _lead_groups(self, p, p_prev, a):
        """lead with the fitted pole on the arms and each group's declared one (none if unknown)."""
        k, taus = self.group_tau
        poles = np.zeros(len(p))
        poles[:k] = a or 0.0
        if self.dt is not None:
            for i, tau in enumerate(taus):
                if tau is not None:
                    poles[k + i] = self._pole(tau)
        m = poles > 0.0
        if not m.any():
            return p
        out = p.copy()
        off = np.clip(poles[m] / (1.0 - poles[m]) * (p[m] - p_prev[m]), -LEAD_MAX_RAD, LEAD_MAX_RAD)
        out[m] = np.clip(p[m] + off, self.q_min[m], self.q_max[m])
        return out

    def clamp_step(self, u, q):
        """A target led at most step_clamp ahead of the observed q."""
        if self.step_clamp is None or q is None:
            return u
        q = np.asarray(q, dtype=np.float64)
        c = self.step_clamp
        return np.clip(u, _inside32(q - c, q, c), _inside32(q + c, q, c))


def _readings(measured) -> list[float]:
    """A chunk's gripper readings without the frames that had none."""
    return [float(g) for g in measured if g is not None]


def _base_pose(obs) -> np.ndarray | None:
    """An observation's T_world_base, None for a robot without a base."""
    T = getattr(obs, "T_world_base", None)
    return None if T is None else np.array(T, dtype=np.float64)


def planar(T) -> tuple[float, float, float]:
    """(x, y, yaw) of a pose about world z."""
    T = np.asarray(T, dtype=np.float64)
    return float(T[0, 3]), float(T[1, 3]), float(np.arctan2(T[1, 0], T[0, 0]))


def planar_pose(x: float, y: float, yaw: float, z: float = 0.0) -> np.ndarray:
    T = np.eye(4)
    c, s = np.cos(yaw), np.sin(yaw)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = (x, y, z)
    return T


def base_goal(T_world_base, delta) -> np.ndarray:
    """Where (dx, dy, dyaw) in the base's own frame takes it: translate, then turn."""
    dx, dy, dyaw = (float(v) for v in delta)
    return np.asarray(T_world_base, dtype=np.float64) @ planar_pose(dx, dy, dyaw)


BASE_POSE_TOL = (0.001, 0.001, math.radians(0.1))
BASE_REST_STEP = (0.0001, 0.0001, math.radians(0.01))
BASE_TRACK_SHARE_FLOOR = 0.8
BASE_SETTLE_MARGIN_STEPS = 30
BASE_TRIP_LEAD_STEPS = 3
BASE_CONTACT_PUSH_MAX_M = 0.02
BASE_CONTACT_LEVER_M = 0.3
BASE_CONTACT_START_STEPS = 1
BASE_UNAVAILABLE = "base motion unavailable: contact sensing is unavailable"
ENV_ERROR = "env_error: the simulator raised on the latest action, so the move failed"


class BaseStep(NamedTuple):
    """One step of a base move: the (dx, dy, dyaw) to send; end None to go on, "settled" or
    "stopped" (reason says why, rule which net: "contact", "trip" or "missed")."""

    delta: np.ndarray
    end: str | None = None
    reason: str | None = None
    rule: str | None = None
    detail: dict | None = None


class BaseTrack:
    """A base move to a world-frame goal, judged on the measured pose."""

    def __init__(self, T_goal, step_max, tol=BASE_POSE_TOL, rest=BASE_REST_STEP,
                 max_extra=BASE_SETTLE_MARGIN_STEPS):
        self.T_goal = np.asarray(T_goal, dtype=np.float64)
        self.goal = planar(self.T_goal)
        self.step_max = np.asarray(step_max, dtype=np.float64)
        self.tol = np.asarray(tol, dtype=np.float64)
        self.rest = np.asarray(rest, dtype=np.float64)
        self.max_extra = int(max_extra)
        self.target: np.ndarray | None = None
        self.push_limit = 0.0
        self._origin: np.ndarray | None = None
        self._start_contacts: set[str] = set()
        self._seen: list[tuple] = []

    def error(self, T_now) -> np.ndarray:
        """The goal from T_now in T_now's own frame: (dx, dy, dyaw)."""
        return self._error(planar(T_now))

    def _error(self, pose) -> np.ndarray:
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        ex, ey = self.goal[0] - x, self.goal[1] - y
        dyaw = (self.goal[2] - yaw + np.pi) % (2.0 * np.pi) - np.pi
        return np.array([c * ex + s * ey, -s * ex + c * ey, dyaw])

    def arrived(self, T_now) -> bool:
        """Whether the pose alone is within the tolerance of the goal."""
        return bool((np.abs(self.error(T_now)) <= self.tol).all())

    def update(self, T_now, seq=None) -> None:
        """Note a measured pose; a frame seen again (same seq) counts once."""
        if self._seen and seq is not None and self._seen[-1][0] == seq:
            return
        self._seen = [*self._seen[-1:], (seq, np.array(planar(T_now)))]

    def settled(self) -> bool:
        """The last frame is within the tolerance of the goal and the base moved less than the
        rest step since the frame before."""
        if len(self._seen) < 2:
            return False
        (_, a), (_, b) = self._seen
        c, s = np.cos(a[2]), np.sin(a[2])
        w = b - a
        moved = np.array([c * w[0] + s * w[1], -s * w[0] + c * w[1],
                          (w[2] + np.pi) % (2.0 * np.pi) - np.pi])
        return bool((np.abs(self._error(b)) <= self.tol).all()
                    and (np.abs(moved) <= self.rest).all())

    def verdict(self, steps: int, planned: int) -> str | None:
        """After `steps` steps of a plan of `planned`: None to go on, "settled" or "missed"."""
        if steps < planned:
            return None
        if self.settled():
            return "settled"
        return "missed" if steps - planned >= self.max_extra else None

    def tripped(self, T_now) -> str | None:
        """Why the base is blocked (it trails the target it was sent by BASE_TRIP_LEAD_STEPS of
        steps), else None."""
        if self.target is None:
            return None
        cur = planar(T_now)
        xy = float(np.hypot(self.target[0] - cur[0], self.target[1] - cur[1]))
        yaw = abs((self.target[2] - cur[2] + np.pi) % (2.0 * np.pi) - np.pi)
        lim_xy = BASE_TRIP_LEAD_STEPS * float(np.hypot(*self.step_max[:2]) / np.sqrt(2.0))
        lim_yaw = BASE_TRIP_LEAD_STEPS * float(self.step_max[2])
        if xy <= lim_xy and yaw <= lim_yaw:
            return None
        return (f"base_blocked: the base trails its target by {xy * 100:.1f} cm, "
                f"{math.degrees(yaw):.1f} deg (limit {lim_xy * 100:.1f} cm, "
                f"{math.degrees(lim_yaw):.1f} deg)")

    def residual(self) -> np.ndarray:
        """The goal from the last measured pose, in that pose's frame (dx, dy, dyaw)."""
        return self._error(self._seen[-1][1])

    def miss_reason(self, steps: int) -> str:
        r = self.residual()
        return (f"the base did not reach its goal within {steps} steps: {r[0] * 1000:+.1f} mm, "
                f"{r[1] * 1000:+.1f} mm, {math.degrees(r[2]):+.2f} deg short")

    def _travel(self, pose) -> float:
        """How far a point BASE_CONTACT_LEVER_M from the root has moved since the move began."""
        o = self._origin
        w = np.asarray(pose) - o
        c, sn = np.cos(o[2]), np.sin(o[2])
        dyaw = (w[2] + np.pi) % (2.0 * np.pi) - np.pi
        return float(np.hypot(c * w[0] + sn * w[1], -sn * w[0] + c * w[1])
                     + BASE_CONTACT_LEVER_M * abs(dyaw))

    @staticmethod
    def _point_speed(step) -> float:
        return float(np.hypot(step[0], step[1]) + BASE_CONTACT_LEVER_M * abs(step[2]))

    @staticmethod
    def _start_step_cap() -> float:
        """The travel of a step out of a start contact: half the push limit."""
        return BASE_CONTACT_PUSH_MAX_M / 2.0

    def _push_limit(self, T_now) -> float:
        """BASE_CONTACT_PUSH_MAX_M less one step's travel, the travel past which a start contact stops."""
        e = self.error(T_now)
        step = e * min(1.0, float(np.min(self.step_max / np.maximum(np.abs(e), 1e-12))))
        one = self._point_speed(step)
        if self._start_contacts:
            one = min(one, self._start_step_cap())
        return max(0.0, BASE_CONTACT_PUSH_MAX_M - one)

    @staticmethod
    def _pair(entry: str) -> tuple[str, str]:
        """(link, object) of a "<link> touched <object>" entry; the link is "robot" if malformed."""
        link, sep, obj = entry.partition(" touched ")
        return (link, obj) if sep and re.fullmatch(r"[\w.\-]+", link) else ("robot", entry)

    def _contact_stop(self, T_now, steps: int, contacts) -> BaseStep | None:
        """A stop for a contact that is new, or that a start contact outlasted the push limit."""
        now = set(contacts)
        pose = np.array(planar(T_now))
        if self._origin is None:
            self._origin = pose
        if steps < BASE_CONTACT_START_STEPS:
            self._start_contacts |= now
            self.push_limit = self._push_limit(T_now)
            return None
        self._start_contacts &= now
        new = sorted(now - self._start_contacts)
        if new:
            link, obj = self._pair(new[0])
            more = f" (+{len(new) - 1} more)" if len(new) > 1 else ""
            return BaseStep(np.zeros(3), "stopped",
                            f"base_contact: the {link} touched an obstacle{more}", "contact",
                            {"rule": "contact", "start": False, "link": link, "object": obj,
                             "pairs": [dict(zip(("link", "object"), self._pair(e), strict=True))
                                       for e in new], "steps": steps})
        if self._start_contacts and self._travel(pose) >= self.push_limit:
            link, obj = self._pair(min(self._start_contacts))
            travel = self._travel(pose)
            return BaseStep(np.zeros(3), "stopped",
                            f"base_contact: the {link} is still touching an obstacle after "
                            f"{steps} steps, {travel * 100:.1f} cm", "contact",
                            {"rule": "contact", "start": True, "link": link, "object": obj,
                             "pairs": [dict(zip(("link", "object"), self._pair(e), strict=True))
                                       for e in sorted(self._start_contacts)],
                             "steps": steps, "travel_m": travel})
        return None

    def advance(self, T_now, seq, steps: int, planned: int, contacts=()) -> BaseStep:
        """The next BaseStep from the frame just measured (the executor calls this per step)."""
        self.update(T_now, seq)
        if (stop := self._contact_stop(T_now, steps, contacts)) is not None:
            return stop
        if steps > 0 and (why := self.tripped(T_now)):
            return BaseStep(np.zeros(3), "stopped", why, "trip")
        if steps < planned:
            return BaseStep(self._command(T_now))
        verdict = self.verdict(steps, planned)
        if verdict == "settled":
            return BaseStep(np.zeros(3), "settled")
        if verdict == "missed":
            return BaseStep(np.zeros(3), "stopped", self.miss_reason(steps), "missed")
        return BaseStep(np.zeros(3))

    def command(self, T_now, seq=None) -> np.ndarray:
        self.update(T_now, seq)
        return self._command(T_now)

    def _command(self, T_now) -> np.ndarray:
        if self.target is None:
            self.target = np.array(planar(T_now))
        t = self.target
        e = self._error(t)
        if (np.abs(e) <= 1e-9).all():
            return np.zeros(3)
        d = e * min(1.0, float(np.min(self.step_max / np.maximum(np.abs(e), 1e-12))))
        if self._start_contacts:
            d = d * min(1.0, self._start_step_cap() / max(self._point_speed(d), 1e-12))
        c, s = np.cos(t[2]), np.sin(t[2])
        t += np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
        return d


def _inside32(bound, q, c) -> np.ndarray:
    """bound as float32, one ulp toward q when rounding put it past q +- c."""
    b = np.asarray(bound, dtype=np.float32)
    out = np.abs(b.astype(np.float64) - q) > c
    return np.where(out, np.nextafter(b, q.astype(np.float32)), b).astype(np.float64)


class Holds:
    """A publish chunk's actions: one hold, grown a step at a time until the backend ends."""

    composed = True

    def __init__(self, hold, tcp, dt, tcp_by_ee=None):
        self.hold, self.tcp, self.dt = hold, tcp, dt
        self.actions, self.tcp_planned = [], []
        self.tcp_by_ee = tcp_by_ee or {}

    def grow(self, ch) -> None:
        self.actions.append(self.hold.copy())
        self.tcp_planned.append(self.tcp)
        ch.planned_tcp.append(self.tcp)
        if ch.ee_planned_tcp is not None:
            for e, tcp in self.tcp_by_ee.items():
                ch.ee_planned_tcp.setdefault(e, []).append(tcp)
        ch.total += 1


class Chunk:
    """One running plan: what was planned, what was sent, and what the arm did."""

    def __init__(self, chunk_id, kind, plan, result, start_step, assist=None, action_dim=8, *,
                 actions=None, start_q=None, ee=None, ees=None):
        self.chunk_id, self.kind, self.plan, self.result = chunk_id, kind, plan, result
        self.action_dim = action_dim
        self.actions = result.actions if actions is None else actions
        self.ee = ee
        self.total = len(self.actions)
        self.index = 0
        self.sent, self.offsets, self.actual_q, self.gripper_measured = [], [], [], []
        self.expected, self.held, self.track_err_mm, self.track_err_rad = [], [], [], []
        self.held_ees = []
        self.over = 0
        self.over_ee: dict[str, int] = {}
        self.stop_reason = None
        self.env_error: str | None = None
        if start_q is None:
            start_q = getattr(result, "start_q", None)
        self.p_prev = None if start_q is None else np.asarray(start_q, dtype=np.float64)
        self.actual_tcp = []
        self.start_step = start_step
        self.planned_tcp = None if result.tcp_planned is None else \
            np.asarray(result.tcp_planned, dtype=np.float64).tolist()
        self.assist = assist
        self.step_id = None
        self.dropped: str | None = None
        self.grip_held: str | None = None
        self.base_track = getattr(result, "base_track", None)
        self.base_planned = self.total
        self.base_miss: str | None = None
        self.base_detail: dict | None = None
        self.base_delta = np.zeros(3)
        self.ee_planned_tcp = self.ee_actual_tcp = self.ee_gripper_measured = None
        self.ee_track_err_mm = None
        if ees is not None:
            by_ee = getattr(result, "tcp_planned_by_ee", None)
            if by_ee:
                self.ee_planned_tcp = {e: np.asarray(v, dtype=np.float64).tolist()
                                       for e, v in by_ee.items()}
            else:
                self.ee_planned_tcp = {} if self.planned_tcp is None else {
                    ee: list(self.planned_tcp)}
            self.ee_actual_tcp = {e: [] for e in ees}
            self.ee_gripper_measured = {e: [] for e in ees}
            self.ee_track_err_mm = {e: [] for e in ees}

    def planned(self) -> list:
        return np.asarray(self.actions[: self.index], dtype=np.float64).tolist()

    def sent_actions(self) -> list:
        return np.asarray(self.sent[: self.index], dtype=np.float64).reshape(
            -1, self.action_dim).tolist()


class Executor:
    """Under the server lock: the running chunk, the commanded state, the episode's record."""

    def __init__(self, srv, tracking):
        self._srv = srv
        self.tracking = tracking
        self.chunk: Chunk | None = None
        self._base_unanchored = False
        self._select_pending: str | None = None
        self._select_deferred: str | None = None
        self.next_chunk_id = 1
        self.history: list[dict] = []
        self.gripper_cmds: dict[str, int] = {}
        self.last_action = None
        self.stale = False
        self._unsettled = None
        self._rebase_pending = False
        self.tracker = Tracker()
        self.set_embodiment(srv.embodiment)
        self._track_pending = None
        self._last_sent = None
        self.rec = None
        self.executes = 0
        self._env_streak = 0
        self._grip_seen: set = set()
        self._grip_close: dict = {}

    @property
    def gripper_cmd(self) -> int | None:
        """The primary end effector's last commanded gripper (0 open, 1 closed), or None."""
        return self.gripper_cmds.get(self.layout.primary)

    @gripper_cmd.setter
    def gripper_cmd(self, value) -> None:
        if value is None:
            self.gripper_cmds.pop(self.layout.primary, None)
        else:
            self.gripper_cmds[self.layout.primary] = int(value)

    def set_embodiment(self, emb) -> None:
        """The robot's action layout, drive priors and gripper; what was commanded is forgotten."""
        self.layout = rt.Layout(emb)
        self.J = self.layout.joints
        self.multi = len(self.layout.ees) > 1
        self.tracker.configure(None if rt.is_droid(emb) else emb)
        self._handoff = np.zeros(emb.dof)
        self.gripper_cmds = {}
        self._select_pending = self._select_deferred = None
        self.last_action = self._last_sent = None
        self._track_pending = None
        world = getattr(self._srv, "world", None)
        if world is not None:
            world.bind(self.layout.ees, self.layout.primary, self._srv.obs.observed_tcp,
                       self.layout.gripper_ees)

    def _grip(self, action) -> float:
        return self.layout.grip(action)

    def reset(self, control_dt) -> None:
        """A new episode."""
        self.drop("stopped")
        self._base_unanchored = False
        self._select_pending = self._select_deferred = None
        self.gripper_cmds = {}
        self.last_action = None
        self._rebase_pending = False
        self._track_pending = None
        self.tracker.reset(control_dt)
        self._handoff = np.zeros(self.layout.dof)
        self._last_sent = None
        self.settle(False)
        self.stale = False
        self.history = []
        self.rec = {"initial_state": None, "chunks": []}
        self.executes = 0
        self._env_streak = 0
        self._grip_seen = set()
        self._grip_close = {}

    def moved(self) -> bool:
        """Under the lock: a chunk has started this episode (the robot may have moved)."""
        return self.executes > 0

    def recorded(self) -> int:
        return len(self.rec["chunks"]) if self.rec is not None else 0

    def state_now(self, tcp=True) -> dict:
        srv = self._srv
        obs = srv.obs.obs
        st = {"q": np.asarray(obs.q, dtype=np.float64).tolist(),
              "gripper": float(obs.gripper) if self.layout.primary in self.layout.gripper_ees
              else None}
        if self.multi:
            st["grippers"] = {e: srv.obs.gripper_of(e) for e in self.layout.gripper_ees}
        if tcp:
            st["tcp"] = srv.obs.observed_tcp7().tolist()
        st["objects"] = {}
        return st

    def on_observation(self, obs) -> None:
        """A new observation: fit the drive, track the running chunk, settle a dropped one."""
        p = self._track_pending
        if p is not None and (p[0].any() or p[1].any()):
            fit, _, u, q_prev = p
            self._track_pending = None
            if obs.control_dt and self.tracker.dt != float(obs.control_dt):
                self.tracker.reset(obs.control_dt)
            if fit.any():
                q_now = np.asarray(obs.q, dtype=np.float64)
                self.tracker.observe(u, q_prev, q_now, None if fit.all() else fit)
        self.stale = False
        if self.rec is not None and self.rec["initial_state"] is None:
            self.rec["initial_state"] = self.state_now(tcp=False)
        for e in self.layout.gripper_ees:
            if e not in self.gripper_cmds and (g := self._srv.obs.gripper_of(e)) is not None:
                self.gripper_cmds[e] = round(g)
        if self._rebase_pending:
            self._rebase_to_observed()

    def _tcp7(self, ee):
        """ee's observed TCP as a pose7 list, or None when the observation has no pose for it."""
        try:
            return self._srv.obs.observed_tcp7(ee).tolist()
        except KeyError:
            return None

    def track(self, obs) -> None:
        """After calibration: record the running chunk's observed step and settle a dropped one."""
        srv = self._srv
        ch = self.chunk
        if ch is not None and len(ch.actual_tcp) < ch.index:
            ch.actual_tcp.append(srv.obs.observed_tcp7(ch.ee).tolist())
            ch.actual_q.append([float(x) for x in obs.q])
            ch.gripper_measured.append(srv.obs.gripper_of(ch.ee))
            if ch.ee_actual_tcp is not None:
                for e in self.layout.ees:
                    ch.ee_actual_tcp[e].append(self._tcp7(e))
                    ch.ee_gripper_measured[e].append(srv.obs.gripper_of(e))
            if len(ch.expected) > len(ch.track_err_mm):
                self._track_guard(ch, np.asarray(obs.q, dtype=np.float64))
        if self._unsettled is not None:
            self.settle(True)
        if getattr(obs, "env_error", None):
            self._env_error(obs.env_error)
        else:
            self._env_streak = 0

    def _env_error(self, error: str) -> None:
        """The simulator raised on the latest action: its move fails with the error; the episode goes on."""
        reason = f"{ENV_ERROR}: {error}"
        self._env_streak += 1
        ch, ex = self.chunk, None
        if ch is not None and ch.kind != "publish":
            ch.env_error, ch.stop_reason = error, reason
            _, ex = self._srv.stop_motion(rebase=True)
            what = f"chunk {ch.chunk_id} stopped: {reason}; the gripper stays as it was"
        else:
            h = self.history[-1] if ch is None and self.history else None
            if h is not None and h["n_steps"] > 0:
                h["status"] = "stopped"
                h["reason"] = f"{h['reason']}; {reason}" if h.get("reason") else reason
            what = reason
        n = self._env_streak
        if env_error_logged(n):
            self._srv.log("warn", what if n == 1 else f"{what} ({n} in a row)")
        if ex is not None:
            self._srv.log("info", "plan execution stopped; the steps not yet executed stay in "
                          "the plan")

    def _rebase_to_observed(self) -> None:
        """Make the observed q the last commanded arm target, so a hold freezes the arm there."""
        self._rebase_pending = False
        obs = self._srv.obs.obs
        if self.last_action is not None and obs is not None:
            self.last_action = self.last_action.copy()
            self.last_action[self.J] = np.asarray(obs.q, dtype=np.float64)

    def plan_context(self) -> dict:
        """The start state a new chunk is planned from: seq, q, hold_q, gripper(s), tool, dt, step."""
        srv = self._srv
        o = srv.obs
        obs = o.live()
        if o.calibrated_seq != o.frame_seq:
            o.calibrate()
        gripper = self.gripper_cmd if self.gripper_cmd is not None else 0
        hold_q = self.last_action[self.J] if self.last_action is not None else obs.q
        return {"seq": o.frame_seq, "q": np.asarray(obs.q, dtype=np.float64).copy(),
                "hold_q": np.asarray(hold_q, dtype=np.float64).copy(), "gripper": int(gripper),
                "grippers": {e: int(self.gripper_cmds.get(e) or 0) for e in self.layout.ees},
                "tool": o.tool_copy(), "dt": float(obs.control_dt),
                "step": srv.episode.step,
                "T_world_base": _base_pose(obs)}

    def start(self, kind, plan, make, assist=None, *, step_id=None, select=None) -> dict:
        """Plan with make(ctx) outside the lock and, if it has no errors, start dispensing it."""
        srv = self._srv
        ep = srv.episode

        def refuse(message, preview=None, conflict=True):
            body = {"ok": False, "chunk_id": None, "message": message, "preview": preview}
            if conflict:
                raise SessionConflict(message, body)
            return body

        with srv.cond:
            if ep.publishing is not None:
                refuse("the episode is being published")
            if ep.abort is not None:
                refuse(ABANDONING)
            if ep.mode != "awaiting_plan":
                self.note_call(select)
                refuse(f"cannot start a chunk in mode {ep.mode!r}")
            self._release_deferred()
            self.note_select(select)
            if self.stale:
                refuse(STALE)
            ctx = self.plan_context()
        result = make(ctx)
        preview = result.to_json()
        if not result.ok:
            errs = [i for i in preview.get("issues", []) if i.get("severity") == "error"]
            why = "; ".join(f"{i.get('kind')}: {i.get('message')}" for i in errs)
            return refuse(f"plan has {len(errs)} error issue(s); not executed: {why}", preview,
                          conflict=False)
        if len(result.actions) == 0:
            return refuse("plan has no steps", preview, conflict=False)
        with srv.cond:
            if ep.publishing is not None:
                refuse("the episode is being published", preview)
            if ep.abort is not None:
                refuse(ABANDONING, preview)
            if ep.mode != "awaiting_plan":
                refuse(f"cannot start a chunk in mode {ep.mode!r}", preview)
            if srv.obs.frame_seq != ctx["seq"] or self.stale:
                refuse(STALE, preview)
            chunk_id = self.begin_chunk(kind, jsonable(plan), result, assist, step_id=step_id,
                                        ee=self.plan_ee(result, ctx))
        srv.log("info", f"chunk {chunk_id} ({kind}) started: {len(result.actions)} steps")
        return {"ok": True, "chunk_id": chunk_id, "message": "executing", "preview": preview}

    def plan_ee(self, result, ctx=None) -> str:
        """The end effector a plan is for: result.ee, ctx["ee"], its tool's, else the primary."""
        lay = self.layout
        for ee in (getattr(result, "ee", None), (ctx or {}).get("ee")):
            if ee is not None:
                if ee not in lay.ee_arm:
                    raise ValueError(f"unknown end effector {ee!r}; the robot has "
                                     f"{list(lay.ees)}")
                return ee
        if self.multi:
            emb = self._srv.embodiment
            for tool in (getattr(result, "tool", None), (ctx or {}).get("tool")):
                arm = getattr(tool, "arm", None)
                for e in (lay.ees if arm is not None else ()):
                    if getattr(emb.tool(e), "arm", None) is arm:
                        return e
        return lay.primary

    def hold_q(self) -> np.ndarray:
        """The full q every arm a plan does not move holds: the last commanded target."""
        if self.last_action is not None:
            return np.asarray(self.last_action[self.J], dtype=np.float64).copy()
        obs = self._srv.obs.obs
        if obs is None:
            raise ValueError("no observation yet: nothing to hold the other arms at")
        return np.asarray(obs.q, dtype=np.float64).copy()

    def compose(self, result, ee):
        """(full-layout action rows, full start q or None) of a plan result for end effector ee."""
        lay = self.layout
        if getattr(result, "composed", False) is True or lay.plain(ee):
            return result.actions, None
        hold = self.hold_q()
        grips = {e: float(self.gripper_cmds.get(e) or 0) for e in lay.ees}
        actions = lay.compose_plan(result.actions, ee, hold, grips)
        start = getattr(result, "start_q", None)
        if start is not None:
            hold[lay.ee_q[ee]] = np.asarray(start, dtype=np.float64)
        return actions, None if start is None else hold

    def note_call(self, ees) -> None:
        """Under the lock: a call selects its arm now, or once a running chunk or steps plan ends."""
        srv = self._srv
        ep = srv.episode
        if ep.publishing is not None or ep.abort is not None:
            return
        if ep.mode == "awaiting_plan":
            self._release_deferred()
            self.note_select(ees)
        elif (self.chunk is not None or srv.planning.executing()) and \
                (arm := self._arm_of(ees)) is not None:
            self._select_deferred = arm

    def _release_deferred(self) -> None:
        """Under the lock: once nothing runs, the deferred selection becomes the pending one."""
        if self._select_deferred is None:
            return
        ch = self.chunk
        if (ch is not None and ch.kind != "publish") or self._srv.planning.executing():
            return
        if self._srv.episode.abort is None:
            self._select_pending = self._select_deferred
        self._select_deferred = None

    def _arm_of(self, ees) -> str | None:
        """The arm a call on ees selects: the first declared of several, whose order is not known."""
        lay = self.layout
        if not self.multi or not ees:
            return None
        arms = {lay.ee_arm[e] for e in ees}
        return next(a for a in (lay.ee_arm[e] for e in lay.ees) if a in arms)

    def note_select(self, ees) -> None:
        """Under the lock: a call that selects its arm was made; the next action names it."""
        arm = self._arm_of(ees)
        if arm is not None:
            self._select_pending = arm

    def _annotate(self, reply: dict) -> dict:
        """An action reply on a robot with several arms carries select_arm: the arm to select, or null."""
        if self.multi:
            self._release_deferred()
            reply["select_arm"], self._select_pending = self._select_pending, None
        return reply

    def begin_chunk(self, kind, plan, result, assist=None, *, step_id=None, ee=None) -> int:
        """Under the lock: make result the running chunk."""
        srv = self._srv
        ee = self.plan_ee(result) if ee is None else ee
        actions, start_q = self.compose(result, ee)
        chunk_id = self.next_chunk_id
        self.next_chunk_id += 1
        ch = Chunk(chunk_id, kind, plan, result, srv.episode.step, assist, self.layout.dim,
                   actions=actions, start_q=start_q, ee=ee,
                   ees=self.layout.ees if self.multi else None)
        ch.step_id = step_id
        self.chunk = ch
        self.executes += 1
        srv.episode.set_mode("executing")
        srv.trace("server.execute", {"chunk_id": chunk_id, "kind": kind, "status": "started",
                                     "n_steps": ch.total, "step_id": step_id})
        srv.cond.notify_all()
        return chunk_id

    def step_in_flight(self) -> None:
        """Under the lock: an observation is on its way; the held one is stale."""
        self.stale = True

    def link_reset(self) -> None:
        """Under the lock: the backend restarted or ended the episode; nothing runs."""
        self.drop("stopped")
        self.settle(False)
        self.stale = False

    def stop(self) -> dict:
        srv = self._srv
        with srv.cond:
            if srv.episode.publishing is not None:
                return {"ok": True}
            stopped = self._srv.stop_motion(rebase=True)
        self._srv.log_stopped(stopped)
        return {"ok": True}

    def stop_chunk(self, *, rebase: bool):
        """Under the lock: drop the running chunk; its history entry or None."""
        entry = self.drop("stopped")
        if entry is not None and rebase:
            self._rebase_pending = True
            if not self.stale:
                self._rebase_to_observed()
        return entry

    def log_stopped(self, entry, ex) -> None:
        if entry is not None:
            self._srv.log("info", f"chunk {entry['chunk_id']} stopped after {entry['n_steps']} "
                          "steps")
        if ex is not None:
            self._srv.log("info", "plan execution stopped; the steps not yet executed stay in "
                          "the plan")

    def next_action(self, rid, live, deadline) -> dict:
        """Under the lock, for /sim/step and /sim/poll: an action, an abort, a finish or a wait."""
        srv = self._srv
        ep = srv.episode
        while True:
            if not live():
                return {"request_id": rid, "wait": True}
            if ep.abort is not None:
                return {"request_id": rid, "abort": {"reason": ep.abort}}
            g = ep.grade_req
            if g is not None and not g["sent"]:
                g["sent"] = True
                return {"request_id": rid, "wait": True, "grade": {"cell": g["cell"]}}
            pub = ep.publishing
            if pub is not None and srv.obs.obs is not None and ep.mode != "episode_over":
                ch = self.chunk
                if ch is None or ch.kind != "publish":
                    self.drop("stopped")
                    ch = self._start_publish()
                if ch.index >= ch.total:
                    ch.result.grow(ch)
                finish = {"reason": pub["reason"]}
                if "verdict" in pub:
                    finish["verdict"] = dict(pub["verdict"])
                if "requested" in pub:
                    finish["verdict_requested"] = dict(pub["requested"])
                return self._annotate({"request_id": rid, "action": self._dispense(ch),
                                       "finish": finish})
            ch = self.chunk
            if ch is not None:
                if ch.index == 0 and self._base_unanchored:
                    self._base_unanchored = False
                    return {"request_id": rid, "wait": True}
                if ch.base_track is not None and not self._base_advance(ch):
                    self.drop("done")
                    continue
                if ch.index < ch.total:
                    action = self._dispense(ch)
                    if action is not None:
                        return self._annotate({"request_id": rid, "action": action})
                    continue
                self.drop("done")
                continue
            srv.planning.kick()
            srv.perception.kick()
            if (srv.obs.obs is not None and ep.mode not in ("episode_over", "awaiting_plan")
                    and not srv.planning.holding_turn()):
                ep.set_mode("awaiting_plan")
            left = deadline - time.monotonic()
            if left <= 0:
                return {"request_id": rid, "wait": True}
            srv.cond.wait(left)

    def _base_advance(self, ch) -> bool:
        """The base move's next step (BaseTrack.advance) from the newest frame: True with the
        delta in ch.base_delta (a hold step added when the plan is spent); False when the move
        ends, ch.total cut to the steps taken and ch.base_miss the reason it was stopped."""
        obs = self._srv.obs.obs
        T = _base_pose(obs)
        if T is None:
            step = BaseStep(np.zeros(3), "stopped", "the frame carries no T_world_base", "missed")
        elif getattr(obs, "contacts_unavailable", False):
            step = BaseStep(np.zeros(3), "stopped", BASE_UNAVAILABLE, "contacts_unavailable")
        else:
            step = ch.base_track.advance(T, self._srv.obs.frame_seq, ch.index, ch.base_planned,
                                         getattr(obs, "body_contacts", None) or ())
        if step.end is not None:
            ch.base_miss = step.reason
            ch.base_detail = step.detail
            ch.total = ch.index
            return False
        ch.base_delta = step.delta
        if ch.index >= ch.total:
            ch.actions = list(ch.actions)
            ch.actions.append(np.asarray(ch.actions[-1], dtype=np.float64).copy())
            if ch.planned_tcp is not None:
                ch.planned_tcp.append(ch.planned_tcp[-1])
            for tcps in (ch.ee_planned_tcp or {}).values():
                tcps.append(tcps[-1])
            ch.total += 1
        return True

    def _start_publish(self) -> Chunk:
        """With the last action observed: the publish chunk, holding still."""
        srv = self._srv
        obs = srv.obs.obs
        lay = self.layout
        if self.last_action is not None:
            hold = self.last_action.copy()
        else:
            grips = {e: float(self.gripper_cmds.get(e) or 0) for e in lay.ees}
            hold = lay.compose_rows(np.asarray(obs.q, dtype=np.float64)[None], grips)[0]
        tcp = np.asarray(kin.pose7(srv.obs.tool.fk_tcp(srv.obs.arm_q(hold[self.J]))),
                         dtype=np.float64).tolist()
        by_ee = None
        if self.multi:
            emb = srv.embodiment
            by_ee = {e: np.asarray(kin.pose7(emb.tool(e).fk_tcp(srv.obs.arm_q(
                hold[self.J], lay.ee_arm[e]))), dtype=np.float64).tolist() for e in lay.ees}
        plan = {"publish": {"reason": srv.episode.publishing["reason"]}}
        chunk_id = self.next_chunk_id
        self.next_chunk_id += 1
        ch = Chunk(chunk_id, "publish", plan, Holds(hold, tcp, float(obs.control_dt), by_ee),
                   srv.episode.step, action_dim=lay.dim, ee=lay.primary,
                   ees=lay.ees if self.multi else None)
        self.chunk = ch
        srv.log("info", f"chunk {chunk_id} (publish) started at step {srv.episode.step}: "
                "holding until the sim ends the episode")
        return ch

    def _arms_closed(self, g) -> np.ndarray:
        """Per joint of q: whether its arm's gripper is commanded closed in grips g."""
        lay = self.layout
        closed = np.zeros(lay.dof, dtype=bool)
        for e, ge in g.items():
            if ge >= 0.5:
                closed[lay.ee_q[e]] = True
        return closed

    def _dispense(self, ch):
        srv = self._srv
        tr = self.tracking
        obs = srv.obs.obs
        lay = self.layout
        J, tk = self.J, self.tracker
        planned = np.asarray(ch.actions[ch.index], dtype=np.float64)
        if planned.shape != (lay.dim,):
            raise ValueError(f"a plan step has {planned.size} values; {srv.embodiment.name}'s "
                             f"action has {lay.dim}")
        g = {e: lay.grip_of(planned, e) for e in lay.ees}
        if ch.kind != "publish":
            for e in lay.gripper_ees:
                if g[e] < 0.5 and self.gripper_cmds.get(e) == 1 and \
                        self._release_blocked(ch, planned, e):
                    return None
        action = planned.copy()
        if tr.lead:
            action[J] = tk.lead(planned[J], ch.p_prev)
        closed = self._arms_closed(g)
        self._handoff_step(ch, closed)
        if np.any(self._handoff):
            action[J] = np.clip(action[J] + self._handoff, tk.q_min, tk.q_max)
        if tk.step_clamp is not None and obs is not None:
            action[J] = tk.clamp_step(action[J], obs.q)
        if ch.base_track is not None and lay.base is not None:
            action[lay.base] = ch.base_delta
        moved = np.zeros(lay.dof, dtype=bool) if ch.p_prev is None else planned[J] != ch.p_prev
        ch.p_prev = planned[J].copy()
        ch.sent.append(action.copy())
        ch.offsets.append(self._handoff + 0.0)
        self._last_sent = action[J].copy()
        self._expect(ch, action, obs, planned)
        self._track_pending = None
        masks = self._learn_masks(g, moved) if obs is not None else None
        if masks is not None:
            self._track_pending = masks + (action[J].copy(), np.asarray(obs.q, dtype=np.float64))
        ch.index += 1
        self.stale = True
        self.last_action = planned.copy()
        if lay.base is not None:
            self.last_action[lay.base] = 0.0
        for e in lay.gripper_ees:
            self.gripper_cmds[e] = round(g[e])
        for e in lay.gripper_ees:
            if g[e] < 0.5:
                self._opened(ch, e, obs)
        srv.episode.set_mode("executing")
        return action.tolist()

    def _learn_masks(self, g, moved) -> tuple | None:
        """(fit, carry) joint masks of the arms whose gripper command is unchanged; else None."""
        lay = self.layout
        fit = np.zeros(lay.dof, dtype=bool)
        carry = np.zeros(lay.dof, dtype=bool)
        gated = False
        for arm, idx in lay.arm_q.items():
            ees = [e for e in lay.gripper_ees if lay.ee_arm[e] == arm]
            if any(self.gripper_cmds.get(e) != int(g[e] >= 0.5) for e in ees):
                continue
            gated = True
            if not any(g[e] >= 0.5 for e in ees):
                fit[idx] = True
            elif moved[idx].any():
                carry[idx] = True
        return (fit, carry) if gated else None

    def _opened(self, ch, ee, obs) -> None:
        """ee's gripper is commanded open: release what it holds (or hand it to the other)."""
        srv = self._srv
        world = srv.world
        if self._grip_tracking() and world.held_by(ee) is not None and self._slipped(ch, ee):
            return
        name = world.held_by(ee) if self.multi else None
        T = None
        if obs is not None:
            try:
                T = srv.obs.observed_tcp(ee)
            except KeyError:
                T = None
        released, mesh = world.gripper_opened(T, ee)
        if released is not None:
            srv.log("info", f"released {released} (chunk {ch.chunk_id} opened the gripper)")
            srv.perception.note_release(mesh)
        elif name is not None and world.held_name == name:
            srv.log("info", f"{ee} let go of {name}; {', '.join(world.held_ees)} holds it "
                    f"(chunk {ch.chunk_id})")

    def _expect(self, ch, action, obs, planned) -> None:
        """Advance the drive model's expected q by this step's offset-free target."""
        lay = self.layout
        u = action[self.J] - ch.offsets[-1]
        prev = ch.expected[-1] if ch.expected else (
            np.asarray(obs.q, dtype=np.float64) if obs is not None else planned[self.J])
        a = self.tracker.alpha or 0.0
        ch.expected.append(a * prev + (1.0 - a) * u)
        held = tuple(e for e in lay.gripper_ees
                     if lay.grip_of(action, e) >= 0.5 and self.gripper_cmds.get(e) == 1)
        ch.held_ees.append(held)
        ch.held.append(bool(held))

    def _track_error(self, q, q_expected, ee=None):
        """(TCP position error in mm, largest joint error in rad, its joint) of q vs q_expected,
        over ee's arm (None: the primary tool over every joint, as for a one-arm robot)."""
        o = self._srv.obs
        q = np.asarray(q, dtype=np.float64)
        if ee is None or not self.multi:
            tool = o.tool
            d = tool.fk_tcp(o.arm_q(q))[:3, 3] - tool.fk_tcp(o.arm_q(q_expected))[:3, 3]
            dq = np.abs(q - q_expected)
        else:
            idx = self.layout.ee_q[ee]
            tool = o.embodiment.tool(ee)
            qe = np.asarray(q_expected, dtype=np.float64)
            d = tool.fk_tcp(q[idx])[:3, 3] - tool.fk_tcp(qe[idx])[:3, 3]
            dq = np.abs(q[idx] - qe[idx])
        return float(np.linalg.norm(d) * 1000.0), float(dq.max()), int(dq.argmax()) + 1

    def _track_guard(self, ch, q) -> None:
        """Per observed step: stop a held object's chunk whose holding arm left its path."""
        tr = self.tracking
        lay = self.layout
        i = len(ch.track_err_mm)
        ees = lay.ees if self.multi else (lay.primary,)
        errs = {e: self._track_error(q, ch.expected[i], e if self.multi else None) for e in ees}
        mm, rad, _ = errs[ch.ee if self.multi else lay.primary]
        ch.track_err_mm.append(mm)
        ch.track_err_rad.append(rad)
        worst = None
        for e, (mm_e, rad_e, _) in errs.items():
            if ch.ee_track_err_mm is not None:
                ch.ee_track_err_mm[e].append(mm_e)
            over = e in ch.held_ees[i] and ((tr.abort_mm > 0 and mm_e > tr.abort_mm)
                                            or (tr.abort_rad > 0 and rad_e > tr.abort_rad))
            ch.over_ee[e] = ch.over_ee.get(e, 0) + 1 if over else 0
            if ch.over_ee[e] >= TRACK_ABORT_STEPS and worst is None:
                worst = e
        ch.over = max(ch.over_ee.values())
        if worst is not None and ch is self.chunk and ch.index < ch.total:
            mm, rad, joint = errs[worst]
            who = f"{worst}: " if self.multi else ""
            self._guard_stop(ch, f"{who}tracking error {mm:.0f} mm on joint {joint} "
                                 f"({rad:.2f} rad off)")

    def _release_blocked(self, ch, planned, ee=None) -> bool:
        """Hold back an opening that the arm is not where the plan put it for."""
        obs = self._srv.obs.obs
        if self.tracking.release_mm <= 0 or obs is None:
            return False
        ref = ch.expected[-1] if ch.expected else planned[self.J]
        mm, _, _ = self._track_error(np.asarray(obs.q, dtype=np.float64), ref,
                                     ee if self.multi else None)
        if mm <= self.tracking.release_mm:
            return False
        who = f"{ee}: " if self.multi else ""
        self._guard_stop(ch, held_back_message(mm, who, self._srv.settings.assist_proposals))
        return True

    def _guard_stop(self, ch, reason) -> None:
        ch.stop_reason = reason
        _, ex = self._srv.stop_motion(rebase=True)
        self._srv.log("warn", f"chunk {ch.chunk_id} stopped: {reason}; the gripper stays as it "
                      "was")
        if ex is not None:
            self._srv.log("info", "plan execution stopped; the steps not yet executed stay in "
                          "the plan")

    def _handoff_step(self, ch, closed) -> None:
        """Keep a held object where the last sent target left it when a new chunk starts."""
        tr = self.tracking
        lay = self.layout
        if not tr.handoff:
            self._handoff = np.zeros(lay.dof)
            return
        h = self._handoff.copy()
        for arm, idx in lay.arm_q.items():
            if not closed[idx].any():
                d = h[idx] * HOLD_RELEASE_DECAY
                d[np.abs(d) < 1e-3] = 0.0
                h[idx] = d
            elif (ch.index == 0 and self._last_sent is not None
                  and ch.p_prev is not None
                  and any(self.gripper_cmds.get(e) == 1 for e in lay.gripper_ees
                          if lay.ee_arm[e] == arm)):
                hm = self.tracker.hold_max
                h[idx] = np.clip(self._last_sent - ch.p_prev, -hm, hm)[idx]
        self._handoff = h

    def drop(self, status):
        """Finalize the running chunk (if any) as done/stopped; its history entry."""
        srv = self._srv
        ch = self.chunk
        if ch is None:
            return None
        self.chunk = None
        if ch.index >= ch.total:
            status = "stopped" if ch.base_miss or ch.env_error else "done"
            ch.stop_reason = ch.base_miss or ch.stop_reason
        if ch.base_miss or (ch.base_track is not None and status != "done"):
            self._base_unanchored = True
        self.settle(False)
        self._update_held(ch, status)
        if self._grip_tracking() and ch.kind != "publish" and ch.index > 0:
            self._track_grip(ch)
        srv.planning.on_chunk_done(ch, status)
        self._release_deferred()
        entry = {"chunk_id": ch.chunk_id, "kind": ch.kind, "n_steps": ch.index,
                 "total": ch.total, "duration": round(ch.index * float(ch.result.dt), 6),
                 "start_step": ch.start_step, "status": status, "final_tcp_err_mm": None,
                 "reason": ch.stop_reason if status == "stopped" else None}
        if self.multi:
            entry["ee"] = ch.ee
        if ch.base_track is not None:
            entry["base"] = {"T_world_goal": ch.base_track.T_goal.tolist()}
        if ch.dropped is not None:
            entry["dropped"] = ch.dropped
        self.history.append(entry)
        ep = srv.episode
        obs = srv.obs.obs
        if (ep.mode == "executing" and not (self.stale and obs is not None)
                and not srv.planning.holding_turn()):
            ep.set_mode("awaiting_plan" if obs is not None else "no_sim")
        if ch.index > 0:
            self._unsettled = (ch, entry)
            if not self.stale and obs is not None:
                self.settle(True)
        else:
            self._trace_chunk(ch, status, None)
        srv.planning.kick()
        srv.cond.notify_all()
        return entry

    def settle(self, observed: bool) -> None:
        """Record the last dropped chunk's final error once its last action has been observed."""
        if self._unsettled is None:
            return
        ch, entry = self._unsettled
        self._unsettled = None
        if observed:
            tcp7 = self._srv.obs.observed_tcp7(ch.ee)
            tcps = ch.planned_tcp if ch.base_track is not None else ch.result.tcp_planned
            planned = np.asarray(tcps[ch.index - 1], dtype=np.float64)
            entry["final_tcp_err_mm"] = float(np.linalg.norm(tcp7[:3] - planned[:3]) * 1000.0)
            if ch.ee_planned_tcp is not None:
                errs = {}
                for e, tcps in ch.ee_planned_tcp.items():
                    obs7 = self._tcp7(e)
                    if obs7 is not None and len(tcps) >= ch.index:
                        errs[e] = float(np.linalg.norm(np.asarray(obs7[:3]) - np.asarray(
                            tcps[ch.index - 1][:3], dtype=np.float64)) * 1000.0)
                entry["final_tcp_err_mm_by_ee"] = errs
            T = _base_pose(self._srv.obs.obs)
            if ch.base_track is not None and T is not None:
                e = ch.base_track.error(T)
                entry["base"] = {**entry.get("base", {}), "T_world_base": np.asarray(T).tolist(),
                                 "error": [float(v) for v in e],
                                 "arrived": ch.base_miss is None and ch.base_track.arrived(T)}
        self._record(ch, entry["status"])
        self._trace_chunk(ch, entry["status"], entry["final_tcp_err_mm"])

    def _record(self, ch, status) -> None:
        """Append a finished chunk to the episode's teleop record."""
        srv = self._srv
        if self.rec is None or ch.index == 0 or srv.obs.obs is None:
            return
        self.rec["chunks"].append({
            "chunk_id": ch.chunk_id, "kind": ch.kind, "start_step": ch.start_step,
            "n_steps": ch.index, "status": status, "plan": ch.plan, "replayed": False,
            "actions": ch.sent_actions(), "planned": ch.planned(), "end_state": self.state_now(),
            "reason": ch.stop_reason})
        srv.episode.plan_changed()
        srv.episode.save_draft()

    def _grip_trace(self, ee, rows, measured) -> dict:
        """ee's grip at the chunk end and the step a settled hold was lost, if it was."""
        emb = self._srv.embodiment
        readings = _readings(measured)
        cmds = [int(self.layout.grip_of(a, ee) >= 0.5) for a in rows]
        end = grip_state(self.gripper_cmds.get(ee), readings, embodiment=emb, ee=ee) \
            if readings else None
        if (self._grip_tracking() and end == "object"
                and float(np.median(readings[-HELD_GRIPPER_SAMPLES:])) <= HELD_JAM_READING):
            end = "jammed"
        return {"grip_end": end,
                "grip_lost_step": grip_lost_step(cmds, measured, embodiment=emb, ee=ee)}

    def _scene_trace(self) -> dict:
        """The perceived scene's facts with the held object, and how old that perception is."""
        srv = self._srv
        p = srv.perception.state.perception
        if p is None:
            return {}
        _, facts = scene_facts(p.objects.values(), held=srv.world.held_name)
        return {"facts": fact_strings(facts),
                "perception_age_s": round(time.time() - p.stamp, 3) if p.stamp else None}

    def _chunk_state(self, ch) -> dict:
        """What a finished chunk left: grip, held, whether it was checked against objects, facts."""
        srv = self._srv
        rows = ch.sent_actions()
        out = {"assist": ch.assist is not None, "held": srv.world.held_name,
               **self._grip_trace(ch.ee, rows, ch.gripper_measured)}
        if ch.ee_gripper_measured is not None:
            by_ee = {e: self._grip_trace(e, rows, ch.ee_gripper_measured[e])
                     for e in self.layout.ees}
            out["ee_grip_end"] = {e: g["grip_end"] for e, g in by_ee.items()}
            out["ee_grip_lost_step"] = {e: g["grip_lost_step"] for e, g in by_ee.items()}
            out["held_ees"] = list(srv.world.held_ees) if srv.world.held is not None else []
        if self._grip_tracking():
            out["dropped"], out["grip_held"] = ch.dropped, ch.grip_held
        out["forgotten"] = sorted(f["id"] for f in srv.perception.state.forgotten)
        return {**out, **self._scene_trace()}

    def _trace_chunk(self, ch, status, err) -> None:
        tr = self._srv.tracer_now()
        try:
            state = self._chunk_state(ch)
        except Exception as e:  # noqa: BLE001
            state = {"state_error": f"{type(e).__name__}: {e}"[:200]}
        rec = {"chunk_id": ch.chunk_id, "kind": ch.kind, "plan": ch.plan,
               "start_step": ch.start_step, "n_steps": ch.index, "status": status,
               "reason": ch.stop_reason, "final_tcp_err_mm": err, **state}
        arrays = {"actions": ch.sent_actions(), "planned": ch.planned(),
                  "hold_offset": np.asarray(ch.offsets[: ch.index], np.float64).tolist(),
                  "actual_q": ch.actual_q, "gripper_measured": ch.gripper_measured,
                  "track_err_mm": ch.track_err_mm, "planned_tcp": ch.planned_tcp,
                  "actual_tcp": ch.actual_tcp}
        if ch.base_detail is not None:
            rec["base_contact"] = ch.base_detail
        if ch.ee_actual_tcp is not None:
            rec["ee"] = ch.ee
            arrays.update(ee_planned_tcp=ch.ee_planned_tcp, ee_actual_tcp=ch.ee_actual_tcp,
                          ee_gripper_measured=ch.ee_gripper_measured,
                          ee_track_err_mm=ch.ee_track_err_mm)
        tr.event("server.execute", jsonable({**rec, "status": status}),
                 blobs=[tr.blob(jsonable(arrays), role="chunk", compress=True)])

    def _update_held(self, ch, status) -> None:
        """The held state: set by a pick that lifted (or closed on) its object, cleared by a
        finished place."""
        srv = self._srv
        if ch.assist is None or status != "done":
            return
        name, ee = ch.assist["object"], ch.ee
        if ee not in self.layout.gripper_ees:
            return
        if ch.kind == "place":
            placed = srv.world.placed(ee)
            if placed is not None:
                srv.log("info", f"placed {placed} (chunk {ch.chunk_id})")
            return
        if ch.kind != "pick":
            return
        obs = srv.obs.obs
        cmd_only = not srv.settings.measured_gripper
        measured = [] if cmd_only else _readings(
            ch.gripper_measured or ([srv.obs.gripper_of(ee)] if obs is not None else []))
        if not measured and (obs is not None or cmd_only):
            why = (f"not held: no measured gripper reading for {ee}" if cmd_only else
                   f"not held: the observation has no gripper reading for {ee}")
            srv.world.held_reason = why
            srv.log("warn", f"pick chunk {ch.chunk_id}: {why}")
            return
        geom, sig0 = ch.assist.get("geom"), ch.assist.get("sig0")
        grip = float(np.median(measured[-HELD_GRIPPER_SAMPLES:])) if measured else None
        state = grip_state(self.gripper_cmds.get(ee), measured, embodiment=srv.embodiment,
                           ee=ee)
        if self._grip_tracking() and state == "object" and grip <= HELD_JAM_READING:
            state = "jammed"
        level, message = srv.world.picked(
            name, ch.chunk_id, state=state, grip=grip, geom=geom, sig0=sig0,
            check=HeldCheck(object=name, sig0=sig0, geom=geom), ee=ee)
        if self._grip_tracking() and state == "object" and srv.world.held_by(ee) == name:
            self._grip_seen.add(ee)
        if obs is not None:
            srv.world.track_held(srv.obs.observed_tcp(ee), ee)
        srv.log(level, message)

    def _grip_tracking(self) -> bool:
        settings = getattr(self._srv, "settings", None)
        return getattr(settings, "held_tracking", "plans") == "gripper"

    def _track_grip(self, ch) -> None:
        """held_tracking gripper, at any motion chunk's end: each closed gripper's readings drop
        a hold that slipped to empty, or hold the object at the TCP of one settled on it."""
        srv, lay = self._srv, self.layout
        world = srv.world
        emb = embodiment_or_droid(srv.embodiment)
        rows = ch.sent_actions()
        for ee in lay.gripper_ees:
            if world.held_by(ee) is None:
                self._grip_seen.discard(ee)
            gripper = emb.gripper(ee)
            measured = (ch.ee_gripper_measured.get(ee) if ch.ee_gripper_measured is not None
                        else ch.gripper_measured if ee == ch.ee else None)
            cmds = [int(lay.grip_of(a, ee) >= 0.5) for a in rows]
            self._note_close(ch, ee, cmds)
            if gripper is None or not measured or self.gripper_cmds.get(ee) != 1:
                continue
            tail = []
            for c, g in zip(cmds, measured):
                if c != 1:
                    tail = []
                elif g is not None:
                    tail.append(float(g))
            if not tail:
                continue
            m = float(np.median(tail[-HELD_GRIPPER_SAMPLES:]))
            settled = [i for i in range(len(tail) - HELD_GRIPPER_SAMPLES + 1)
                       if _settled_hold(gripper, tail[i:i + HELD_GRIPPER_SAMPLES])]
            if world.held_by(ee) is not None:
                if settled:
                    self._grip_seen.add(ee)
                lost = grip_lost_step(cmds, measured, embodiment=srv.embodiment, ee=ee)
                if not gripper.holds(m) and (lost is not None or ee in self._grip_seen):
                    self._grip_dropped(ch, ee, m, lost)
            elif (world.held is None and world.held_check is None and settled
                    and settled[-1] == len(tail) - HELD_GRIPPER_SAMPLES):
                self._grip_hold(ch, ee, m)

    def _slipped(self, ch, ee) -> bool:
        """held_tracking gripper, as ee opens mid-chunk: drop its hold if the readings lost it
        while still commanded closed (a carry that slipped is not a release at the TCP)."""
        emb = embodiment_or_droid(self._srv.embodiment)
        gripper = emb.gripper(ee)
        measured = (ch.ee_gripper_measured.get(ee) if ch.ee_gripper_measured is not None
                    else ch.gripper_measured if ee == ch.ee else None)
        if gripper is None or not measured:
            return False
        cmds = [int(self.layout.grip_of(a, ee) >= 0.5) for a in ch.sent_actions()[:len(measured)]]
        closed = [float(g) for c, g in zip(cmds, measured) if c == 1 and g is not None]
        if len(closed) < HELD_GRIPPER_SAMPLES:
            return False
        m = float(np.median(closed[-HELD_GRIPPER_SAMPLES:]))
        lost = grip_lost_step(cmds, measured, embodiment=self._srv.embodiment, ee=ee)
        if gripper.holds(m) or (lost is None and ee not in self._grip_seen):
            return False
        self._grip_dropped(ch, ee, m, lost)
        return True

    def _grip_dropped(self, ch, ee, reading, lost) -> None:
        srv = self._srv
        name = srv.world.dropped(ch.chunk_id, ee)
        self._grip_seen.discard(ee)
        if name is None:
            return
        ch.dropped = name
        at = "" if lost is None else f" at step {lost}"
        srv.log("warn", f"chunk {ch.chunk_id}: dropped {name}: the gripper lost its hold{at} "
                f"(reads {reading:.3f} commanded closed)")

    def _note_close(self, ch, ee, cmds) -> None:
        """Remember ee's TCP at the step its gripper was last commanded closed: perception's box
        of an object lifted since then is still there, not at the TCP the chunk ends at."""
        prev = (self._grip_close.get(ee) or (None, 0))[1]
        tcps = (ch.ee_actual_tcp or {}).get(ee) if ch.ee_actual_tcp is not None else ch.actual_tcp
        for i, c in enumerate(cmds):
            if c == 1 and prev != 1:
                pose = tcps[i] if tcps and i < len(tcps) else None
                T = kin.make_pose(pose[:3], pose[3:]) if pose is not None else None
                self._grip_close[ee] = (T, 1)
            elif c != 1:
                self._grip_close[ee] = (None, 0)
            prev = c

    def _grip_hold(self, ch, ee, reading) -> None:
        srv = self._srv
        try:
            T = srv.obs.observed_tcp(ee)
        except KeyError:
            return
        T_close = (self._grip_close.get(ee) or (None, 0))[0]
        name = srv.world.grip_held(ch.chunk_id, T if T_close is None else T_close, ee)
        if name is None:
            return
        ch.grip_held = name
        self._grip_seen.add(ee)
        srv.world.track_held(T, ee)
        srv.log("info", f"chunk {ch.chunk_id}: holding {name} (the gripper reads {reading:.3f} "
                "closed on it at the TCP)")


def _settled_hold(gripper, window) -> bool:
    """A full window of closed readings that hold something, not jammed, and stopped moving."""
    return (len(window) == HELD_GRIPPER_SAMPLES and max(window) - min(window) <= GRIP_SETTLED
            and all(HELD_JAM_READING < r < gripper.spec.closed_empty and gripper.holds(r)
                    for r in window))
