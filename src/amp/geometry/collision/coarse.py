#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Coarse sphere models from link AABBs: the built-in Panda's (fingers open), a chain arm's."""

import math

import numpy as np

from amp.geometry.collision.spheres import RobotSpheres

_ARM_AABBS = {
    "panda_link0": ("link0", (-0.154, -0.095, 0.0), (0.072, 0.095, 0.14)),
    "panda_link1": ("link1", (-0.055, -0.129, -0.192), (0.055, 0.055, 0.055)),
    "panda_link2": ("link2", (-0.055, -0.194, -0.055), (0.055, 0.055, 0.129)),
    "panda_link3": ("link3", (-0.055, -0.055, -0.121), (0.137, 0.111, 0.055)),
    "panda_link4": ("link4", (-0.138, -0.055, -0.055), (0.055, 0.124, 0.111)),
    "panda_link5": ("link5", (-0.055, -0.055, -0.259), (0.055, 0.13, 0.052)),
    "panda_link6": ("link6", (-0.048, -0.051, -0.044), (0.132, 0.082, 0.056)),
    "panda_link7": ("link7", (-0.044, -0.044, 0.052), (0.081, 0.081, 0.107)),
}
_GRIPPER_AABBS = (
    ((-0.004, -0.042, -0.038), (0.09, 0.042, 0.038)),
    ((0.045, -0.076, -0.018), (0.12, 0.076, 0.018)),
    ((0.111, 0.042, -0.011), (0.149, 0.062, 0.011)),
    ((0.111, -0.062, -0.011), (0.149, -0.042, 0.011)),
)
_ARM_RADIUS_MAX = 0.06
_INFLATE = 1.15


def _cover_box(lo, hi, r_max=None) -> np.ndarray:
    """Spheres (n,4) on a grid filling an AABB: covers its rounded core, not its corners."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    ext = hi - lo
    r = 0.5 * ext.min() if r_max is None else min(0.5 * ext.min(), r_max)
    axes = []
    for k in range(3):
        span = ext[k] - 2 * r
        n = math.ceil(span / r - 1e-9) + 1 if span > 1e-9 else 1
        axes.append(np.linspace(lo[k] + r, hi[k] - r, n) if n > 1 else [0.5 * (lo[k] + hi[k])])
    c = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    return np.hstack([c, np.full((len(c), 1), r * _INFLATE)])


def builtin_spheres() -> RobotSpheres:
    """Spheres filling each arm link's AABB, and the gripper's open-finger boxes."""
    links = {n: (f, _cover_box(lo, hi, _ARM_RADIUS_MAX)) for n, (f, lo, hi) in _ARM_AABBS.items()}
    links["gripper"] = ("ee", np.vstack([_cover_box(lo, hi) for lo, hi in _GRIPPER_AABBS]))
    return RobotSpheres(links, source="builtin")
