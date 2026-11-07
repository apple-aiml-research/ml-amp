#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""End-effector tools and grippers: flange -> ee -> TCP, and the finger gap of a reading."""

from __future__ import annotations

import numpy as np

from amp.contracts.embodiment import EndEffectorSpec, GripperSpec
from amp.geometry.embodiment.arm import Arm, DHArm
from amp.geometry.gripper.profile import GripperProfile, profile
from amp.geometry.kinematics.tool import EE_TO_TCP_QUAT, ToolModel
from amp.geometry.kinematics.transforms import make_pose


def pose44(flat) -> np.ndarray:
    """A contract pose (16 floats, row-major) as a 4x4 array."""
    return np.array(flat, dtype=np.float64).reshape(4, 4)


class ArmToolModel(ToolModel):
    """ToolModel (nominal T_flange_ee, calibrate) on any arm, with the spec's fixed T_ee_tcp."""

    def __init__(self, arm: Arm, T_flange_ee, T_ee_tcp, finger_axis: str = "x"):
        T_ee_tcp = np.array(T_ee_tcp, dtype=np.float64)
        super().__init__(T_flange_ee, tcp_offset=float(np.linalg.norm(T_ee_tcp[:3, 3])))
        self.arm = arm
        self._T_ee_tcp = T_ee_tcp
        self.finger_axis = finger_axis

    @property
    def T_ee_tcp(self) -> np.ndarray:
        return self._T_ee_tcp.copy()

    def _fk_flange(self, q) -> np.ndarray:
        return self.arm.fk_flange(q)

    def link_points(self, q) -> np.ndarray:
        """(L+2,3): the arm's link origins (base .. flange), ee, tcp."""
        links = self.arm.fk_links(q)
        T_ee = links[-1] @ self.T_flange_ee
        return np.vstack([links[:, :3, 3], T_ee[:3, 3], (T_ee @ self._T_ee_tcp)[:3, 3]])


def _robotiq_offset(T_ee_tcp: np.ndarray) -> float | None:
    """The tcp_offset for which ToolModel's own T_ee_tcp is exactly this one, else None."""
    off = float(T_ee_tcp[0, 3])
    return off if np.array_equal(make_pose([off, 0.0, 0.0], EE_TO_TCP_QUAT), T_ee_tcp) else None


def make_tool(arm: Arm, ee: EndEffectorSpec) -> ToolModel:
    """A fresh tool for an end effector: kinematics.ToolModel itself on the built-in Panda."""
    T_fe, T_et = pose44(ee.T_flange_ee), pose44(ee.T_ee_tcp)
    off = _robotiq_offset(T_et)
    if isinstance(arm, DHArm) and off is not None and ee.closing_axis == "x":
        return ToolModel(T_fe, tcp_offset=off)
    return ArmToolModel(arm, T_fe, T_et, finger_axis=ee.closing_axis)


class Gripper:
    """A gripper's reading (0 open .. 1 closed) as a finger gap, and its held test."""

    def __init__(self, spec: GripperSpec):
        self.spec = spec
        self.kind, self.model = spec.kind, spec.model
        self.stroke = spec.stroke_m
        self.max_width = spec.max_width_m

    @property
    def profile(self) -> GripperProfile | None:
        """The Robotiq 2F-85 closing profile (the packaged asset's), None for other models."""
        return profile() if self.model == "robotiq_2f85_linkage" else None

    def gap(self, reading):
        """Distance between the finger pads (m) at a reading."""
        g = np.clip(reading, 0.0, 1.0)
        if self.kind == "none":
            return 0.0 * g
        if self.model == "robotiq_2f85_linkage":
            gp = profile()
            return gp.gap(g * gp.closed_angle)
        return self.stroke * (1.0 - g)

    def holds(self, reading) -> bool:
        """Whether a closed reading means something is held (robot_server.executor.grip_state)."""
        return float(reading) <= self.spec.held_below
