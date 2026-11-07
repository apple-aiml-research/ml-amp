#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Robotiq 2F-85's closing sweep in the TCP frame, tabulated over the finger angle."""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from amp.geometry.gripper.linkage import PADS, GripperLinkage
from amp.geometry.kinematics import TCP_OFFSET_DEFAULT
from amp.geometry.robot_asset import ROBOT_ASSET, RobotAsset

PAD_FACE_TOL = 5e-4
_GRID = 181
_VOXEL = 0.003
_SWEEP_STEP = math.radians(5.0)


def _voxel(P: np.ndarray, size: float) -> np.ndarray:
    _, idx = np.unique(np.floor(P / size).astype(np.int64), axis=0, return_index=True)
    return P[np.sort(idx)]


@dataclass(frozen=True)
class PadSpan:
    """Pad faces at one angle in the TCP frame: approach (z) range, inner/outer |x|, half |y|."""

    z_lo: float
    z_hi: float
    x_inner: float
    x_outer: float
    y_half: float

    @property
    def z_mid(self) -> float:
        return 0.5 * (self.z_lo + self.z_hi)


class GripperProfile:
    """gap, tip reach and pad span as functions of the finger angle theta in [0, closed_angle]."""

    def __init__(self, path: Path = ROBOT_ASSET, tcp_offset: float = TCP_OFFSET_DEFAULT):
        linkage = GripperLinkage(RobotAsset.load(path))
        self.closed_angle = linkage.closed_angle
        self.theta = np.linspace(0.0, self.closed_angle, _GRID)
        pads = linkage.pad_faces(PAD_FACE_TOL)
        gap, reach, span = [], [], []
        self._points: list[np.ndarray] = []
        self._coarse: dict[int, np.ndarray] = {}
        for th in self.theta:
            P = np.vstack(list(linkage.tcp_points(th, tcp_offset).values()))
            L, R = (linkage.tcp_points(th, tcp_offset, pads)[n] for n in PADS)
            face = np.vstack([L, R])
            gap.append(float(R[:, 0].min() - L[:, 0].max()))
            reach.append(float(P[:, 2].max()))
            span.append((face[:, 2].min(), face[:, 2].max(), np.abs(face[:, 1]).max()))
            self._points.append(P.astype(np.float32))
        self._gap = np.array(gap)
        self._reach = np.array(reach)
        self._span = np.array(span)
        open_ = linkage.tcp_points(0.0, tcp_offset)
        outer = np.concatenate([np.abs(open_[n][:, 0]) for n in PADS])
        self._pad_depth = float(np.max(outer)) - 0.5 * float(self._gap[0])

    def _clip(self, theta):
        return np.clip(theta, 0.0, self.closed_angle)

    def gap(self, theta):
        """Distance between the pad faces (m)."""
        return np.interp(self._clip(theta), self.theta, self._gap)

    def theta_for_gap(self, w):
        """Finger angle at which the pads are w apart (clamped to the stroke)."""
        return np.interp(w, self._gap[::-1], self.theta[::-1])

    def tip_reach(self, theta):
        """Furthest any finger point reaches past the TCP along the approach (m)."""
        return np.interp(self._clip(theta), self.theta, self._reach)

    def max_tip_reach(self, theta) -> float:
        """Furthest reach over the whole sweep from open to theta."""
        k = int(np.searchsorted(self.theta, self._clip(theta), side="right"))
        return float(max(self._reach[: max(k, 1)].max(), self.tip_reach(theta)))

    def pad_span(self, theta) -> PadSpan:
        """The pad faces' extent at theta."""
        th = float(self._clip(theta))
        lo, hi, yh = (float(np.interp(th, self.theta, self._span[:, i])) for i in range(3))
        g = 0.5 * float(self.gap(th))
        return PadSpan(lo, hi, g, g + self._pad_depth, yh)

    def sweep_thetas(self, theta, step=_SWEEP_STEP) -> np.ndarray:
        """Angles sampling the sweep from open to theta, both ends included."""
        th = float(self._clip(theta))
        n = max(1, math.ceil(th / step))
        return np.linspace(0.0, th, n + 1)

    def finger_points(self, theta, coarse: bool = False) -> np.ndarray:
        """(N,3) finger surface points in the TCP frame at the grid angle nearest theta."""
        k = int(np.argmin(np.abs(self.theta - self._clip(theta))))
        if not coarse:
            return self._points[k]
        if k not in self._coarse:
            self._coarse[k] = _voxel(self._points[k].astype(np.float64), _VOXEL)
        return self._coarse[k]


@functools.cache
def profile() -> GripperProfile:
    """The profile of the packaged robot asset, built once."""
    return GripperProfile()
