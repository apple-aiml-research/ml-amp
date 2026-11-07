#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Franka Panda's joint space: position and velocity limits, home, the shoulder singularity."""

from __future__ import annotations

import numpy as np

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
QD_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
Q_HOME = np.array([0.0, -np.pi / 5, 0.0, -4 * np.pi / 5, 0.0, 3 * np.pi / 5, 0.0])

SINGULAR_Q2 = 0.10
SINGULAR_ZONE = 0.3


def limit_margin(q) -> float:
    """Distance (rad) from q to the nearest joint limit; negative if outside."""
    q = np.asarray(q, dtype=np.float64)
    return float(np.min(np.minimum(q - Q_MIN, Q_MAX - q)))
