#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Rotations and poses: (w, x, y, z) Hamilton quaternions, rpy, 4x4 transforms, pose7."""

from __future__ import annotations

import math

import numpy as np


def quat_normalize(q) -> np.ndarray:
    """Unit quaternion; raises ValueError on a zero-norm input."""
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-12:
        raise ValueError(f"cannot normalize quaternion {q.tolist()}")
    return q / n


def quat_mul(a, b) -> np.ndarray:
    """Hamilton product a * b."""
    aw, ax, ay, az = np.asarray(a, dtype=np.float64)
    bw, bx, by, bz = np.asarray(b, dtype=np.float64)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_matrix(q) -> np.ndarray:
    """Rotation matrix of a quaternion (normalized first)."""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(R) -> np.ndarray:
    """Quaternion of a rotation matrix, canonical with w >= 0."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = np.array(
            [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 1e-300))
        q = np.array(
            [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
        )
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 1e-300))
        q = np.array(
            [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
        )
    else:
        s = 2.0 * np.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 1e-300))
        q = np.array(
            [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
        )
    q = q / math.sqrt(float(q @ q))
    return -q if q[0] < 0 else q


def quat_slerp(a, b, t) -> np.ndarray:
    """Shortest-path spherical interpolation from a (t=0) to b (t=1)."""
    a = quat_normalize(a)
    b = quat_normalize(b)
    d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        return quat_normalize(a + t * (b - a))
    theta = np.arccos(min(d, 1.0))
    s = np.sin(theta)
    return (np.sin((1 - t) * theta) * a + np.sin(t * theta) * b) / s


def _axis_quat(axis: int, angle: float) -> np.ndarray:
    q = np.zeros(4)
    q[0] = np.cos(angle / 2)
    q[1 + axis] = np.sin(angle / 2)
    return q


def rpy_to_quat(rpy) -> np.ndarray:
    """Quaternion of Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    q = quat_mul(_axis_quat(2, yaw), quat_mul(_axis_quat(1, pitch), _axis_quat(0, roll)))
    return -q if q[0] < 0 else q


def make_pose(pos, quat) -> np.ndarray:
    """4x4 homogeneous transform from a position and a quaternion."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(quat)
    T[:3, 3] = np.asarray(pos, dtype=np.float64)
    return T


def pose_parts(T) -> tuple[np.ndarray, np.ndarray]:
    """(pos (3,), quat (4,)) of a 4x4 transform."""
    T = np.asarray(T, dtype=np.float64)
    return T[:3, 3].copy(), matrix_to_quat(T[:3, :3])


def pose7(T) -> np.ndarray:
    """[x, y, z, qw, qx, qy, qz] of a 4x4 transform."""
    pos, quat = pose_parts(T)
    return np.concatenate([pos, quat])


def rot_error(R_target, R_current) -> np.ndarray:
    """Axis-angle vector of R_target @ R_current.T."""
    q = matrix_to_quat(np.asarray(R_target, dtype=np.float64) @ np.asarray(R_current).T)
    v = q[1:]
    s = math.sqrt(float(v @ v))
    if s < 1e-12:
        return 2.0 * v
    return v * (2.0 * np.arctan2(s, q[0]) / s)


def rot_error_batch(R_t, R) -> np.ndarray:
    """rot_error for stacks of rotations: (B,3)."""
    M = R_t @ np.swapaxes(R, 1, 2)
    m00, m11, m22 = M[:, 0, 0], M[:, 1, 1], M[:, 2, 2]
    tr = m00 + m11 + m22
    c1 = tr > 0
    c2 = ~c1 & (m00 > m11) & (m00 > m22)
    c3 = ~c1 & ~c2 & (m11 > m22)
    s = 2.0 * np.sqrt(np.select(
        [c1, c2, c3],
        [tr + 1.0, np.maximum(1.0 + m00 - m11 - m22, 1e-300),
         np.maximum(1.0 + m11 - m00 - m22, 1e-300)],
        np.maximum(1.0 + m22 - m00 - m11, 1e-300),
    ))
    d21, d02, d10 = M[:, 2, 1] - M[:, 1, 2], M[:, 0, 2] - M[:, 2, 0], M[:, 1, 0] - M[:, 0, 1]
    s01, s02, s12 = M[:, 0, 1] + M[:, 1, 0], M[:, 0, 2] + M[:, 2, 0], M[:, 1, 2] + M[:, 2, 1]
    q = np.select(
        [c1[:, None], c2[:, None], c3[:, None]],
        [np.stack([0.25 * s, d21 / s, d02 / s, d10 / s], 1),
         np.stack([d21 / s, 0.25 * s, s01 / s, s02 / s], 1),
         np.stack([d02 / s, s01 / s, 0.25 * s, s12 / s], 1)],
        np.stack([d10 / s, s02 / s, s12 / s, 0.25 * s], 1),
    )
    q /= np.sqrt(np.einsum("ij,ij->i", q, q))[:, None]
    q[q[:, 0] < 0] *= -1.0
    v = q[:, 1:]
    n = np.sqrt(np.einsum("ij,ij->i", v, v))
    small = n < 1e-12
    k = np.where(small, 2.0, 2.0 * np.arctan2(n, q[:, 0]) / np.where(small, 1.0, n))
    return v * k[:, None]
