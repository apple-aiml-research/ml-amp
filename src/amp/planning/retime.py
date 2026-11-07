#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Dense sampling and joint-limited time parameterization of a path s in [0, 1]."""

from __future__ import annotations

import math
from collections.abc import Callable

import numpy as np

from amp.planning import clock


def march(
    solve: Callable[[float, np.ndarray], np.ndarray], q0, h_max: float, h_min: float,
    dq_step: float, deadline: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(s, q, jump) sampled from s=0 at q0, each sample's IK seeded by the previous one."""
    s_out, q_out, jump = [0.0], [np.asarray(q0, dtype=np.float64)], []
    s, q, h = 0.0, q_out[0], h_max
    while s < 1.0:
        while True:
            s_next = 1.0 if s + h >= 1.0 - 1e-9 else s + h
            q_next = solve(s_next, q)
            step = float(np.max(np.abs(q_next - q)))
            if step <= dq_step or h <= h_min:
                break
            if deadline is not None and clock.now() > deadline:
                break
            h = max(0.5 * h, h_min)
        jump.append(step > dq_step)
        s, q = s_next, q_next
        s_out.append(s)
        q_out.append(q)
        if step < 0.5 * dq_step:
            h = min(2.0 * h, h_max)
    return np.array(s_out), np.array(q_out), np.array(jump, dtype=bool)


def timing(
    s, q, cap_s, cap_v, v_max, accel: float, free=None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(g, x = (ds/dt)^2, t): fastest under cap_v, v_max and accel; `free` gets cap_v only."""
    s, q = np.asarray(s, dtype=np.float64), np.asarray(q, dtype=np.float64)
    ds = np.maximum(np.diff(s), 1e-12)
    slope = np.abs(np.diff(q, axis=0)) / ds[:, None]
    qv = np.max(slope / np.asarray(v_max, dtype=np.float64), axis=1)
    qa = slope.max(axis=1)
    kink = np.zeros(len(s))
    if len(s) > 2:
        kink[1:-1] = np.abs(np.diff(np.diff(q, axis=0) / ds[:, None], axis=0)).max(axis=1)
        kink[1:-1] /= 0.5 * (ds[:-1] + ds[1:])
    if free is not None and np.any(free):
        free = np.asarray(free, dtype=bool)
        qv[free], qa[free] = 0.0, 0.0
        kink[:-1][free] = kink[1:][free] = 0.0
    curv = np.maximum(kink[:-1], kink[1:])

    g = np.union1d(s, cap_s)
    dg = np.maximum(np.diff(g), 1e-12)
    k = np.clip(np.searchsorted(s, 0.5 * (g[:-1] + g[1:]), side="right") - 1, 0, len(ds) - 1)
    qv, qa, curv = qv[k], qa[k], curv[k]
    cap = np.interp(g, cap_s, cap_v) ** 2
    with np.errstate(divide="ignore"):
        lim = np.minimum(1.0 / np.square(qv), 0.5 * accel / curv)
    cap[:-1] = np.minimum(cap[:-1], lim)
    cap[1:] = np.minimum(cap[1:], lim)
    cap[0] = cap[-1] = 0.0

    x = cap.tolist()
    qa_l, curv_l, dg_l = qa.tolist(), curv.tolist(), dg.tolist()

    def rise(xi, i):
        return math.inf if qa_l[i] == 0.0 else xi + 2.0 * dg_l[i] * (
            max(0.0, accel - curv_l[i] * xi) / qa_l[i])

    for i in range(len(dg_l)):
        x[i + 1] = min(x[i + 1], rise(x[i], i))
    for i in range(len(dg_l) - 1, -1, -1):
        x[i] = min(x[i], rise(x[i + 1], i))
    x = np.array(x)
    v = np.sqrt(x)
    t = np.concatenate([[0.0], np.cumsum(2.0 * dg / np.maximum(v[:-1] + v[1:], 1e-12))])
    return g, x, t


def resample(g, x, t, dt: float, min_steps: int = 2) -> np.ndarray:
    """s at the end of every dt step, the timing stretched to a whole number of steps."""
    n = max(min_steps, math.ceil(t[-1] / dt - 1e-9))
    tt = (np.arange(n) + 1) * (t[-1] / n)
    k = np.clip(np.searchsorted(t, tt, side="right") - 1, 0, len(g) - 2)
    dg = np.maximum(g[k + 1] - g[k], 1e-12)
    tau = tt - t[k]
    s = g[k] + np.sqrt(x[k]) * tau + 0.25 * (x[k + 1] - x[k]) / dg * tau * tau
    s = np.clip(s, g[k], g[k + 1])
    s[-1] = 1.0
    return s
