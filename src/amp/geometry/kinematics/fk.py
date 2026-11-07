#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Forward kinematics and the geometric Jacobian of the Panda arm (modified DH, Craig)."""

from __future__ import annotations

import numpy as np

DH = np.array(
    [
        [0.0, 0.333, 0.0],
        [0.0, 0.0, -np.pi / 2],
        [0.0, 0.316, np.pi / 2],
        [0.0825, 0.0, np.pi / 2],
        [-0.0825, 0.384, -np.pi / 2],
        [0.0, 0.0, np.pi / 2],
        [0.088, 0.0, np.pi / 2],
        [0.0, 0.107, 0.0],
    ]
)
_CA, _SA = np.cos(DH[:, 2]), np.sin(DH[:, 2])


def fk_chain(q) -> np.ndarray:
    """Frames panda_link1..link7 and the flange (link8), robot-root frame, shape (8,4,4)."""
    theta = np.append(np.asarray(q, dtype=np.float64), 0.0)
    ct, st = np.cos(theta), np.sin(theta)
    A = np.zeros((8, 4, 4))
    A[:, 0, 0], A[:, 0, 1], A[:, 0, 3] = ct, -st, DH[:, 0]
    A[:, 1, 0], A[:, 1, 1], A[:, 1, 2], A[:, 1, 3] = st * _CA, ct * _CA, -_SA, -_SA * DH[:, 1]
    A[:, 2, 0], A[:, 2, 1], A[:, 2, 2], A[:, 2, 3] = st * _SA, ct * _SA, _CA, _CA * DH[:, 1]
    A[:, 3, 3] = 1.0
    out = np.empty((8, 4, 4))
    out[0] = T = A[0]
    for i in range(1, 8):
        out[i] = T = T @ A[i]
    return out


def fk_flange(q) -> np.ndarray:
    """Flange (panda_link8) pose in the robot-root frame."""
    return fk_chain(q)[-1]


def fk_chain_batch(qs) -> np.ndarray:
    """fk_chain for every row of qs: (B,8,4,4)."""
    qs = np.asarray(qs, dtype=np.float64).reshape(-1, 7)
    n = len(qs)
    ct, st = np.cos(qs), np.sin(qs)
    out = np.empty((n, 8, 4, 4))
    A = np.zeros((n, 4, 4))
    A[:, 3, 3] = 1.0
    for i in range(8):
        a, d, _ = DH[i]
        ca, sa = _CA[i], _SA[i]
        c, s = (ct[:, i], st[:, i]) if i < 7 else (1.0, 0.0)
        A[:, 0, 0], A[:, 0, 1], A[:, 0, 3] = c, -s, a
        A[:, 1, 0], A[:, 1, 1], A[:, 1, 2], A[:, 1, 3] = s * ca, c * ca, -sa, -sa * d
        A[:, 2, 0], A[:, 2, 1], A[:, 2, 2], A[:, 2, 3] = s * sa, c * sa, ca, ca * d
        out[:, i] = A if i == 0 else out[:, i - 1] @ A
    return out


def jacobian_from_chain(chain: np.ndarray, p_tool: np.ndarray) -> np.ndarray:
    z = chain[:7, :3, 2]
    r = p_tool - chain[:7, :3, 3]
    J = np.empty((6, 7))
    J[0] = z[:, 1] * r[:, 2] - z[:, 2] * r[:, 1]
    J[1] = z[:, 2] * r[:, 0] - z[:, 0] * r[:, 2]
    J[2] = z[:, 0] * r[:, 1] - z[:, 1] * r[:, 0]
    J[3:] = z.T
    return J


def jacobian(q, T_flange_tool=None) -> np.ndarray:
    """Geometric (6,7) Jacobian [v; w] of the tool origin, robot-root axes."""
    chain = fk_chain(q)
    T_tool = chain[-1] if T_flange_tool is None else chain[-1] @ np.asarray(T_flange_tool)
    return jacobian_from_chain(chain, T_tool[:3, 3])
