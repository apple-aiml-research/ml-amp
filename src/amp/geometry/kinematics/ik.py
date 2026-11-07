#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Damped-least-squares IK for one target, with joint-limit and singularity pushes."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amp.geometry.kinematics.fk import fk_chain, jacobian_from_chain
from amp.geometry.kinematics.joints import Q_MAX, Q_MIN, SINGULAR_ZONE, limit_margin
from amp.geometry.kinematics.transforms import rot_error

MAX_STEP = 0.2
LIMIT_DAMPING = 0.2
PUSH_COST = 0.005
_EYE7 = np.eye(7)
_ZERO7 = np.zeros(7)
_ZERO7.flags.writeable = False


@dataclass
class IKResult:
    """The best configuration solve_ik found, its residuals and its joint-limit margin."""

    q: np.ndarray
    pos_err: float
    rot_err: float
    converged: bool
    iterations: int
    margin: float | None = None

    def __post_init__(self):
        if self.margin is None:
            self.margin = limit_margin(self.q)


def limit_push(q: np.ndarray, zone: float, singular: bool = True) -> np.ndarray:
    """Away-from-limit direction per joint: 0 outside zone, rising smoothly (C1) to 1 at it."""
    lo = np.maximum(0.0, 1.0 - (q - Q_MIN) / zone)
    hi = np.maximum(0.0, 1.0 - (Q_MAX - q) / zone)
    p = lo * lo - hi * hi
    if not singular:
        return p
    q2 = q[..., 1]
    w = np.maximum(0.0, 1.0 - np.abs(q2) / SINGULAR_ZONE)
    p[..., 1] += np.where(q2 > 0.0, 1.0, -1.0) * w * w
    return p


def _step(J, e, lam2, dn, v, q):
    """DLS step plus null-space terms; joints the step would push past a limit are held there."""
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    dq = Vt.T @ ((S / (S * S + lam2)) * (U.T @ e))
    V_r = Vt[S > 1e-6 * S[0]]
    dq += dn - V_r.T @ (V_r @ dn)
    if v.any():
        idx = np.flatnonzero(v)
        N = _EYE7 - V_r.T @ V_r
        Nc = N[:, idx]
        A = Nc[idx]
        A.flat[:: idx.size + 1] += LIMIT_DAMPING**2
        dq += Nc @ np.linalg.solve(A, v[idx])
    return _clamp(J, e, lam2, dq, q)


def _clamp(J, e, lam2, dq, q):
    m = np.abs(dq).max()
    if m > MAX_STEP:
        dq *= MAX_STEP / m
    q_new = q + dq
    if (q_new >= Q_MIN).all() and (q_new <= Q_MAX).all():
        return dq
    free = np.ones(7, dtype=bool)
    while True:
        over = free & ((q_new < Q_MIN) | (q_new > Q_MAX))
        if not over.any() or over.sum() == free.sum():
            return dq
        free &= ~over
        dq[over] = np.clip(q_new[over], Q_MIN[over], Q_MAX[over]) - q[over]
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


def solve_ik(
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
    """Damped-least-squares IK for the TOOL pose; returns the best q found, never raises."""
    T_target = np.asarray(T_target, dtype=np.float64)
    T_ft = np.eye(4) if T_flange_tool is None else np.asarray(T_flange_tool, dtype=np.float64)
    q = np.clip(np.asarray(q_seed, dtype=np.float64).copy(), Q_MIN, Q_MAX)
    q_rest = q.copy() if q_rest is None else np.asarray(q_rest, dtype=np.float64)
    p_t, R_t = T_target[:3, 3], T_target[:3, :3]
    push = limit_gain * limit_zone

    best = (q.copy(), np.inf, np.inf)
    best_cost = np.inf
    stall = 0
    it = 0
    while True:
        chain = fk_chain(q)
        T = chain[-1] @ T_ft
        e = np.concatenate([p_t - T[:3, 3], rot_error(R_t, T[:3, :3])])
        pe, re = math.sqrt(float(e[:3] @ e[:3])), math.sqrt(float(e[3:] @ e[3:]))
        cost = pe + 0.1 * re
        converged = pe <= pos_tol and re <= rot_tol
        stall = 0 if cost < best_cost * (1.0 - 1e-3) else stall + 1
        if converged or cost < best_cost:
            best, best_cost = (q.copy(), pe, re), cost
        if converged or it >= max_iters or stall >= 15:
            break
        J = jacobian_from_chain(chain, T[:3, 3])
        lam2 = damping * damping * min(1.0, math.sqrt(float(e @ e)) / 0.05)
        dn = rest_gain * (q_rest - q)
        v = _ZERO7
        if push > 0.0:
            v = push * min(1.0, cost / PUSH_COST) * limit_push(q, limit_zone, singular_push)
        q = np.clip(q + _step(J, e, lam2, dn, v, q), Q_MIN, Q_MAX)
        it += 1

    q_best, pe, re = best
    return IKResult(
        q=q_best, pos_err=pe, rot_err=re, converged=pe <= pos_tol and re <= rot_tol, iterations=it
    )
