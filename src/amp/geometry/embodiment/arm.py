#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""An arm's kinematics: dh_craig (the built-in Panda, amp.geometry.kinematics) or a chain."""

from __future__ import annotations

import math

import numpy as np

from amp.contracts.embodiment import ArmSpec, RobotDescription
from amp.geometry import kinematics as K
from amp.geometry.embodiment.description import path_joints
from amp.geometry.kinematics.ik import LIMIT_DAMPING, MAX_STEP, PUSH_COST, IKResult
from amp.geometry.kinematics.transforms import rot_error


class Arm:
    """One arm: limits, home, FK of its link frames, Jacobian and IK, in the robot-root frame."""

    def __init__(self, spec: ArmSpec, link_names: tuple[str, ...]):
        self.spec = spec
        self.name = spec.name
        self.joints = spec.joints
        self.n = len(spec.joints)
        self.link_names = link_names
        self.q_min = np.array(spec.q_min)
        self.q_max = np.array(spec.q_max)
        self.qd_max = np.array(spec.qd_max)
        self.q_home = np.array(spec.q_home)
        self.singularities = spec.singularities

    def fk_links(self, q) -> np.ndarray:
        """(L,4,4) frames of link_names at q."""
        raise NotImplementedError

    def fk_links_batch(self, qs) -> np.ndarray:
        """(B,L,4,4) fk_links for every row of qs."""
        raise NotImplementedError

    def fk_flange(self, q) -> np.ndarray:
        """The flange_link pose."""
        raise NotImplementedError

    def jacobian(self, q, T_flange_tool=None) -> np.ndarray:
        """Geometric (6,n) Jacobian [v; w] of the tool origin, robot-root axes."""
        raise NotImplementedError

    def ik(self, T_target, q_seed, T_flange_tool=None, **kw) -> IKResult:
        """Damped-least-squares IK for the tool pose (kinematics.solve_ik's keywords)."""
        raise NotImplementedError

    def ik_batch(self, T_target, q_seeds, T_flange_tool=None, **kw) -> list[IKResult]:
        """ik from every row of q_seeds."""
        T = np.broadcast_to(np.asarray(T_target, dtype=np.float64), (len(q_seeds), 4, 4))
        return [self.ik(t, s, T_flange_tool, **kw) for t, s in zip(T, q_seeds)]

    def limit_margin(self, q) -> float:
        """Distance (rad or m) from q to the nearest joint limit; negative if outside."""
        q = np.asarray(q, dtype=np.float64)
        return float(np.min(np.minimum(q - self.q_min, self.q_max - q)))

    def clip(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=np.float64), self.q_min, self.q_max)

    def singular_ok(self, q) -> bool:
        """Whether q keeps every declared singular joint at least its min_abs off 0."""
        q = np.asarray(q, dtype=np.float64)
        return all(abs(float(q[s.joint])) >= s.min_abs for s in self.singularities)


class DHArm(Arm):
    """AMP's built-in Panda (kinematics kind dh_craig): every call is amp.geometry.kinematics'."""

    def __init__(self, spec: ArmSpec):
        if spec.kinematics.kind != "dh_craig":
            raise ValueError(f"arm {spec.name}: not a dh_craig arm")
        want = (tuple(K.Q_MIN), tuple(K.Q_MAX), (K.SINGULAR_Q2, K.SINGULAR_ZONE))
        sing = [(s.min_abs, s.zone) for s in spec.singularities if s.joint == 1]
        if len(spec.joints) != 7 or (spec.q_min, spec.q_max) != want[:2] or \
                len(spec.singularities) != 1 or sing != [want[2]]:
            raise ValueError(f"arm {spec.name}: dh_craig is the built-in Panda, with its limits "
                             "and shoulder singularity; declare any other arm as a chain")
        names = (spec.base_link, *(f"panda_link{i}" for i in range(1, 8)), spec.flange_link)
        super().__init__(spec, names)

    def fk_links(self, q) -> np.ndarray:
        out = np.empty((9, 4, 4))
        out[0] = np.eye(4)
        out[1:] = K.fk_chain(q)
        return out

    def fk_links_batch(self, qs) -> np.ndarray:
        chain = K.fk_chain_batch(qs)
        out = np.empty((len(chain), 9, 4, 4))
        out[:, 0] = np.eye(4)
        out[:, 1:] = chain
        return out

    def fk_flange(self, q) -> np.ndarray:
        return K.fk_flange(q)

    def jacobian(self, q, T_flange_tool=None) -> np.ndarray:
        return K.jacobian(q, T_flange_tool)

    def ik(self, T_target, q_seed, T_flange_tool=None, **kw) -> IKResult:
        return K.solve_ik(T_target, q_seed, T_flange_tool, **kw)

    def ik_batch(self, T_target, q_seeds, T_flange_tool=None, **kw) -> list[IKResult]:
        return K.solve_ik_batch(T_target, q_seeds, T_flange_tool, **kw)


def _cross_matrix(a) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


class ChainArm(Arm):
    """An arm read from a RobotDescription: the joint path base_link -> flange_link."""

    def __init__(self, spec: ArmSpec, description: RobotDescription):
        path = path_joints(description, spec.base_link, spec.flange_link)
        movable = [j.name for j in path if j.type != "fixed"]
        if tuple(movable) != spec.joints:
            raise ValueError(f"arm {spec.name}: the movable joints {spec.base_link} -> "
                             f"{spec.flange_link} are {movable}, the spec declares "
                             f"{list(spec.joints)}")
        super().__init__(spec, (spec.base_link, *(j.child for j in path)))
        self.description = description
        T = np.eye(4)
        for j in path_joints(description, description.root, spec.base_link):
            T = T @ np.array(j.origin).reshape(4, 4)
        self.T_root_base = T
        self._origin = [np.array(j.origin).reshape(4, 4) for j in path]
        self._qi = []
        self._axis, self._K, self._KK, self._prismatic = [], [], [], []
        k = 0
        for j in path:
            a = np.array(j.axis, dtype=np.float64)
            if j.type != "fixed":
                if not np.linalg.norm(a) > 0:
                    raise ValueError(f"joint {j.name}: zero axis")
                a = a / np.linalg.norm(a)
            Km = _cross_matrix(a)
            self._axis.append(a)
            self._K.append(Km)
            self._KK.append(Km @ Km)
            self._prismatic.append(j.type == "prismatic")
            self._qi.append(-1 if j.type == "fixed" else k)
            k += j.type != "fixed"
        movable = [i for i, k in enumerate(self._qi) if k >= 0]
        self.prismatic = np.array([self._prismatic[i] for i in movable], dtype=bool)
        self._joint_axes = np.array([self._axis[i] for i in movable]).reshape(self.n, 3)

    def _frames(self, qs) -> tuple[np.ndarray, np.ndarray]:
        """(B,L,4,4) link frames and (B,n,4,4) joint frames (the parent's frame at the joint)."""
        qs = np.asarray(qs, dtype=np.float64).reshape(-1, self.n)
        B = len(qs)
        links = np.empty((B, len(self.link_names), 4, 4))
        jf = np.empty((B, self.n, 4, 4))
        T = np.broadcast_to(self.T_root_base, (B, 4, 4)).copy()
        links[:, 0] = T
        for i, origin in enumerate(self._origin):
            T = T @ origin
            k = self._qi[i]
            if k >= 0:
                jf[:, k] = T
                x = qs[:, k]
                M = np.broadcast_to(np.eye(4), (B, 4, 4)).copy()
                if self._prismatic[i]:
                    M[:, :3, 3] = x[:, None] * self._axis[i]
                else:
                    M[:, :3, :3] += (np.sin(x)[:, None, None] * self._K[i]
                                     + (1.0 - np.cos(x))[:, None, None] * self._KK[i])
                T = T @ M
            links[:, i + 1] = T
        return links, jf

    def fk_links(self, q) -> np.ndarray:
        return self._frames(q)[0][0]

    def fk_links_batch(self, qs) -> np.ndarray:
        return self._frames(qs)[0]

    def fk_flange(self, q) -> np.ndarray:
        return self._frames(q)[0][0, -1]

    def _jacobian_from(self, jf: np.ndarray, p_tool: np.ndarray) -> np.ndarray:
        """jacobian_from_chain for n revolute or prismatic joints."""
        z = np.einsum("nij,nj->ni", jf[:, :3, :3], self._joint_axes)
        r = p_tool - jf[:, :3, 3]
        J = np.zeros((6, self.n))
        J[:3] = np.where(self.prismatic[:, None], z, np.cross(z, r)).T
        J[3:] = np.where(self.prismatic[:, None], 0.0, z).T
        return J

    def jacobian(self, q, T_flange_tool=None) -> np.ndarray:
        links, jf = self._frames(q)
        T = links[0, -1] if T_flange_tool is None else links[0, -1] @ np.asarray(T_flange_tool)
        return self._jacobian_from(jf[0], T[:3, 3])

    def ik(self, T_target, q_seed, T_flange_tool=None, **kw) -> IKResult:
        return solve_ik_chain(self, T_target, q_seed, T_flange_tool, **kw)


def limit_push(q, q_min, q_max, zone: float, singularities=()) -> np.ndarray:
    """kinematics.limit_push for any limits, with a push off 0 for each declared singular joint."""
    lo = np.maximum(0.0, 1.0 - (q - q_min) / zone)
    hi = np.maximum(0.0, 1.0 - (q_max - q) / zone)
    p = lo * lo - hi * hi
    for s in singularities:
        x = q[..., s.joint]
        w = np.maximum(0.0, 1.0 - np.abs(x) / s.zone)
        p[..., s.joint] += np.where(x > 0.0, 1.0, -1.0) * w * w
    return p


def _clamp(J, e, lam2, dq, q, q_min, q_max):
    m = np.abs(dq).max()
    if m > MAX_STEP:
        dq *= MAX_STEP / m
    q_new = q + dq
    if (q_new >= q_min).all() and (q_new <= q_max).all():
        return dq
    free = np.ones(len(q), dtype=bool)
    while True:
        over = free & ((q_new < q_min) | (q_new > q_max))
        if not over.any() or over.sum() == free.sum():
            return dq
        free &= ~over
        dq[over] = np.clip(q_new[over], q_min[over], q_max[over]) - q[over]
        held = ~free
        Jf = J[:, free]
        e_f = e - J[:, held] @ dq[held]
        A = Jf.T @ Jf
        A.flat[:: A.shape[0] + 1] += lam2 + 1e-12
        dq[free] = np.linalg.solve(A, Jf.T @ e_f)
        m = np.abs(dq).max()
        if m > MAX_STEP:
            dq *= MAX_STEP / m
        q_new = q + dq


def _step(J, e, lam2, dn, v, q, q_min, q_max):
    """kinematics.ik._step for n joints: DLS, exact null-space rest and limit terms, clamped."""
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    dq = Vt.T @ ((S / (S * S + lam2)) * (U.T @ e))
    V_r = Vt[S > 1e-6 * S[0]]
    dq += dn - V_r.T @ (V_r @ dn)
    if v.any():
        idx = np.flatnonzero(v)
        N = np.eye(len(q)) - V_r.T @ V_r
        Nc = N[:, idx]
        A = Nc[idx]
        A.flat[:: idx.size + 1] += LIMIT_DAMPING**2
        dq += Nc @ np.linalg.solve(A, v[idx])
    return _clamp(J, e, lam2, dq, q, q_min, q_max)


def solve_ik_chain(
    arm: ChainArm,
    T_target,
    q_seed,
    T_flange_tool=None,
    *,
    max_iters: int = 100,
    pos_tol: float = 1e-4,
    rot_tol: float = 1e-3,
    damping: float = 0.05,
    q_rest=None,
    rest_gain: float = 0.1,
    limit_gain: float = 0.3,
    limit_zone: float = 0.3,
    singular_push: bool = True,
) -> IKResult:
    """kinematics.solve_ik on a chain arm, its limits and its declared singularities."""
    q_min, q_max = arm.q_min, arm.q_max
    T_target = np.asarray(T_target, dtype=np.float64)
    T_ft = np.eye(4) if T_flange_tool is None else np.asarray(T_flange_tool, dtype=np.float64)
    q = np.clip(np.asarray(q_seed, dtype=np.float64).copy(), q_min, q_max)
    q_rest = q.copy() if q_rest is None else np.asarray(q_rest, dtype=np.float64)
    p_t, R_t = T_target[:3, 3], T_target[:3, :3]
    push = limit_gain * limit_zone
    sing = arm.singularities if singular_push else ()
    zero = np.zeros(arm.n)

    best = (q.copy(), np.inf, np.inf)
    best_cost = np.inf
    stall = 0
    it = 0
    while True:
        links, jf = arm._frames(q)
        T = links[0, -1] @ T_ft
        e = np.concatenate([p_t - T[:3, 3], rot_error(R_t, T[:3, :3])])
        pe, re = math.sqrt(float(e[:3] @ e[:3])), math.sqrt(float(e[3:] @ e[3:]))
        cost = pe + 0.1 * re
        converged = pe <= pos_tol and re <= rot_tol
        stall = 0 if cost < best_cost * (1.0 - 1e-3) else stall + 1
        if converged or cost < best_cost:
            best, best_cost = (q.copy(), pe, re), cost
        if converged or it >= max_iters or stall >= 15:
            break
        J = arm._jacobian_from(jf[0], T[:3, 3])
        lam2 = damping * damping * min(1.0, math.sqrt(float(e @ e)) / 0.05)
        dn = rest_gain * (q_rest - q)
        v = zero
        if push > 0.0:
            v = push * min(1.0, cost / PUSH_COST) * limit_push(q, q_min, q_max, limit_zone, sing)
        q = np.clip(q + _step(J, e, lam2, dn, v, q, q_min, q_max), q_min, q_max)
        it += 1

    q_best, pe, re = best
    return IKResult(q=q_best, pos_err=pe, rot_err=re, converged=pe <= pos_tol and re <= rot_tol,
                    iterations=it, margin=arm.limit_margin(q_best))


def make_arm(spec: ArmSpec, description: RobotDescription | None) -> Arm:
    """The arm a spec declares: DHArm for dh_craig, ChainArm over the description for chain."""
    if spec.kinematics.kind == "dh_craig":
        return DHArm(spec)
    if description is None:
        raise ValueError(f"arm {spec.name}: a chain arm needs the robot description")
    return ChainArm(spec, description)
