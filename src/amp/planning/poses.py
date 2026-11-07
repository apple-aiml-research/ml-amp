#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Small pose and array helpers grasp, place and motion share."""

from __future__ import annotations

import math

import numpy as np

from amp.geometry.kinematics import Q_HOME, make_pose, pose7

__all__ = [
    "EZ",
    "FLIP",
    "as_pose",
    "as_q",
    "make_frame",
    "pose7_rounded",
    "rotz",
    "rounded",
    "to_tcp_frame",
    "unique_in_order",
]

FLIP = np.diag([-1.0, -1.0, 1.0, 1.0])
EZ = np.array([0.0, 0.0, 1.0])


def rounded(a, digits: int) -> list:
    """a as a list rounded to digits."""
    return np.round(np.asarray(a, dtype=np.float64), digits).tolist()


def pose7_rounded(T) -> list:
    """T's pose7, position to 5 and quaternion to 6 digits."""
    p = pose7(T)
    return np.concatenate([np.round(p[:3], 5), np.round(p[3:], 6)]).tolist()


def as_pose(T) -> np.ndarray:
    """T as a finite 4x4 float array, or ValueError."""
    T = np.asarray(T, dtype=np.float64)
    if T.shape == (7,):
        return make_pose(T[:3], T[3:])
    if T.shape != (4, 4):
        raise ValueError(f"expected a (4,4) pose or pose7, got shape {T.shape}")
    return T.copy()


def as_q(q, arm=None) -> np.ndarray:
    """q as the arm's joint vector (the built-in Panda's 7 when arm is None), home when None."""
    if arm is None:
        return Q_HOME.copy() if q is None else np.asarray(q, dtype=np.float64).reshape(7).copy()
    return arm.q_home.copy() if q is None else np.asarray(q, dtype=np.float64).reshape(arm.n).copy()


def make_frame(R, p) -> np.ndarray:
    """The 4x4 pose of rotation R at position p."""
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def rotz(a: float) -> np.ndarray:
    """The 4x4 rotation by a radians about z."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def to_tcp_frame(T, P) -> np.ndarray:
    """World points P in the frame of pose T."""
    return (P - T[:3, 3]) @ T[:3, :3]


def unique_in_order(items) -> list:
    """items without repeats, first occurrence order."""
    return list(dict.fromkeys(items))
