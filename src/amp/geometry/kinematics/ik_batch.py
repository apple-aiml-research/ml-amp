#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""solve_ik for many seeds or targets at once, vectorised over the unfinished ones."""

from __future__ import annotations

import numpy as np

from amp.geometry.kinematics.fk import fk_chain_batch
from amp.geometry.kinematics.ik import (
    LIMIT_DAMPING,
    MAX_STEP,
    PUSH_COST,
    IKResult,
    limit_push,
)
from amp.geometry.kinematics.joints import Q_MAX, Q_MIN
from amp.geometry.kinematics.transforms import rot_error_batch

_EYE7 = np.eye(7)


def _step_batch(J, e, lam2, dn, v, q):
    """_step for stacks: SVD and projections vectorised, the rare clamp re-solves one by one."""
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    w = S / (S * S + lam2[:, None]) * np.einsum("bji,bj->bi", U, e)
    dq = np.einsum("bij,bi->bj", Vt, w)
    Vm = Vt * (S > 1e-6 * S[:, :1])[:, :, None]
    dq += dn - np.einsum("bij,bi->bj", Vm, np.einsum("bij,bj->bi", Vm, dn))
    pushed = np.flatnonzero(v.any(axis=1))
    if len(pushed):
        Vp = Vm[pushed]
        N = _EYE7 - np.swapaxes(Vp, 1, 2) @ Vp
        sel = v[pushed] != 0
        A = np.where(sel[:, :, None] & sel[:, None, :], N, 0.0)
        A += np.eye(7) * np.where(sel, LIMIT_DAMPING**2, 1.0)[:, None, :]
        x = np.linalg.solve(A, np.where(sel, v[pushed], 0.0)[..., None])[..., 0]
        dq[pushed] += np.einsum("bij,bj->bi", N, x)
    raw = dq.copy()
    m = np.abs(dq).max(axis=1)
    big = m > MAX_STEP
    dq[big] *= (MAX_STEP / m[big])[:, None]
    q_new = q + dq
    n_over = ((q_new < Q_MIN) | (q_new > Q_MAX)).sum(axis=1)
    hard = np.flatnonzero((n_over > 0) & (n_over < 7))
    if len(hard):
        dq[hard] = _clamp_batch(J[hard], e[hard], lam2[hard], raw[hard], q[hard])
    return dq


def _clamp_batch(J, e, lam2, dq, q):
    """_clamp for stacks; held joints get identity rows, so the free ones solve as before."""
    free = np.ones(dq.shape, dtype=bool)
    act = np.arange(len(dq))
    while len(act):
        d = dq[act]
        m = np.abs(d).max(axis=1)
        big = m > MAX_STEP
        d[big] *= (MAX_STEP / m[big])[:, None]
        q_new = q[act] + d
        fr = free[act]
        over = fr & ((q_new < Q_MIN) | (q_new > Q_MAX))
        n_over = over.sum(axis=1)
        dq[act] = d
        go = (n_over > 0) & (n_over < fr.sum(axis=1))
        act, over, fr, d, q_new = act[go], over[go], fr[go], d[go], q_new[go]
        if not len(act):
            break
        fr &= ~over
        free[act] = fr
        d = np.where(over, np.clip(q_new, Q_MIN, Q_MAX) - q[act], d)
        Ja = J[act]
        e_f = e[act] - np.einsum("bij,bj->bi", Ja, np.where(fr, 0.0, d))
        Jm = Ja * fr[:, None, :]
        A = np.swapaxes(Jm, 1, 2) @ Jm
        A += np.eye(7) * np.where(fr, (lam2[act] + 1e-12)[:, None], 1.0)[:, None, :]
        x = np.linalg.solve(A, np.einsum("bji,bj->bi", Jm, e_f)[..., None])[..., 0]
        dq[act] = np.where(fr, x, d)
    return dq


def solve_ik_batch(
    T_target,
    q_seeds,
    T_flange_tool=None,
    *,
    max_iters: int = 100,
    pos_tol: float = 1e-4,
    rot_tol: float = 1e-3,
    damping: float = 0.05,
    q_rest=None,
    rest_gain: float = 0.1,
    limit_gain=0.3,
    limit_zone: float = 0.3,
    singular_push: bool = True,
) -> list[IKResult]:
    """solve_ik from every row of q_seeds at once, iterating all unfinished seeds together."""
    seeds = np.clip(np.asarray(q_seeds, dtype=np.float64).reshape(-1, 7), Q_MIN, Q_MAX)
    B = len(seeds)
    T_target = np.broadcast_to(np.asarray(T_target, dtype=np.float64), (B, 4, 4))
    T_ft = np.eye(4) if T_flange_tool is None else np.asarray(T_flange_tool, dtype=np.float64)
    q = seeds.copy()
    q_rest = seeds.copy() if q_rest is None else np.broadcast_to(
        np.asarray(q_rest, dtype=np.float64), (B, 7))
    push = np.broadcast_to(np.asarray(limit_gain, dtype=np.float64) * limit_zone, (B,))
    p_t, R_t = T_target[:, :3, 3], T_target[:, :3, :3]
    best_q, best_pe, best_re = q.copy(), np.full(B, np.inf), np.full(B, np.inf)
    best_cost = np.full(B, np.inf)
    stall = np.zeros(B, dtype=np.int64)
    it = np.zeros(B, dtype=np.int64)
    act = np.arange(B)
    while len(act):
        qa = q[act]
        chain = fk_chain_batch(qa)
        T = chain[:, -1] @ T_ft
        e = np.concatenate([p_t[act] - T[:, :3, 3], rot_error_batch(R_t[act], T[:, :3, :3])], 1)
        pe = np.sqrt(np.einsum("ij,ij->i", e[:, :3], e[:, :3]))
        re = np.sqrt(np.einsum("ij,ij->i", e[:, 3:], e[:, 3:]))
        cost = pe + 0.1 * re
        conv = (pe <= pos_tol) & (re <= rot_tol)
        stall[act] = np.where(cost < best_cost[act] * (1.0 - 1e-3), 0, stall[act] + 1)
        upd = conv | (cost < best_cost[act])
        u = act[upd]
        best_q[u], best_pe[u], best_re[u], best_cost[u] = qa[upd], pe[upd], re[upd], cost[upd]
        keep = ~(conv | (it[act] >= max_iters) | (stall[act] >= 15))
        act, qa, e, cost, chain, T = act[keep], qa[keep], e[keep], cost[keep], chain[keep], T[keep]
        if not len(act):
            break
        z, o = chain[:, :7, :3, 2], chain[:, :7, :3, 3]
        J = np.concatenate([np.swapaxes(np.cross(z, T[:, None, :3, 3] - o), 1, 2),
                            np.swapaxes(z, 1, 2)], axis=1)
        lam2 = damping * damping * np.minimum(1.0, np.sqrt(np.einsum("ij,ij->i", e, e)) / 0.05)
        dn = rest_gain * (q_rest[act] - qa)
        v = (push[act] * np.minimum(1.0, cost / PUSH_COST))[:, None] * limit_push(
            qa, limit_zone, singular_push)
        q[act] = np.clip(qa + _step_batch(J, e, lam2, dn, v, qa), Q_MIN, Q_MAX)
        it[act] += 1
    return [
        IKResult(q=best_q[i], pos_err=float(best_pe[i]), rot_err=float(best_re[i]),
                 converged=bool(best_pe[i] <= pos_tol and best_re[i] <= rot_tol),
                 iterations=int(it[i]))
        for i in range(B)
    ]
