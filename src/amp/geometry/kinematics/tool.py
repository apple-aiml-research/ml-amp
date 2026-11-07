#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Robotiq tool on the flange: flange -> ee (base_link) -> TCP, and its calibration."""

from __future__ import annotations

import numpy as np

from amp.geometry.kinematics.fk import fk_chain, fk_flange
from amp.geometry.kinematics.transforms import make_pose, rot_error

TCP_OFFSET_DEFAULT = 0.134
EE_TO_TCP_QUAT = np.array([0.5, -0.5, 0.5, -0.5])
DEFAULT_T_FLANGE_EE = np.array(
    [[0.0, 0.0, 1.0, 0.0], [0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.018174], [0.0, 0.0, 0.0, 1.0]]
)
N_LINK_POINTS = 11


class ToolModel:
    """Flange -> Robotiq base_link ("ee") -> TCP (RoboLab's eef_frame moved to the pad centre)."""

    def __init__(self, T_flange_ee=None, tcp_offset: float = TCP_OFFSET_DEFAULT):
        src = DEFAULT_T_FLANGE_EE if T_flange_ee is None else T_flange_ee
        self.T_flange_ee = np.array(src, dtype=np.float64)
        self.tcp_offset = float(tcp_offset)
        self.finger_axis = "x"
        self.calibrated = False
        self.last_residual: tuple[float, float] | None = None

    @property
    def T_ee_tcp(self) -> np.ndarray:
        return make_pose([self.tcp_offset, 0.0, 0.0], EE_TO_TCP_QUAT)

    @property
    def T_tcp_ee(self) -> np.ndarray:
        T = self.T_ee_tcp
        R = T[:3, :3].T
        out = np.eye(4)
        out[:3, :3] = R
        out[:3, 3] = -R @ T[:3, 3]
        return out

    @property
    def T_flange_tcp(self) -> np.ndarray:
        return self.T_flange_ee @ self.T_ee_tcp

    def _fk_flange(self, q) -> np.ndarray:
        return fk_flange(q)

    def fk_ee(self, q) -> np.ndarray:
        return self._fk_flange(q) @ self.T_flange_ee

    def fk_tcp(self, q) -> np.ndarray:
        return self._fk_flange(q) @ self.T_flange_tcp

    def link_points(self, q) -> np.ndarray:
        """(11,3): base, panda_link1..7 origins, flange, ee, tcp."""
        chain = fk_chain(q)
        T_ee = chain[-1] @ self.T_flange_ee
        T_tcp = T_ee @ self.T_ee_tcp
        pts = np.empty((N_LINK_POINTS, 3))
        pts[0] = 0.0
        pts[1:9] = chain[:, :3, 3]
        pts[9] = T_ee[:3, 3]
        pts[10] = T_tcp[:3, 3]
        return pts

    def calibrate(self, q, ee_pos, ee_quat) -> tuple[float, float]:
        """Fit T_flange_ee to one observed ee pose; returns the residual of the model before it."""
        T_flange = self._fk_flange(q)
        T_obs = make_pose(ee_pos, ee_quat)
        T_model = T_flange @ self.T_flange_ee
        residual = (
            float(np.linalg.norm(T_model[:3, 3] - T_obs[:3, 3])),
            float(np.linalg.norm(rot_error(T_obs[:3, :3], T_model[:3, :3]))),
        )
        self.T_flange_ee = np.linalg.inv(T_flange) @ T_obs
        self.calibrated = True
        self.last_residual = residual
        return residual

    def tcp_from_ee(self, T_ee) -> np.ndarray:
        return np.asarray(T_ee, dtype=np.float64) @ self.T_ee_tcp
