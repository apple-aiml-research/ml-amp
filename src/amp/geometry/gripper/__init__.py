#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Robotiq 2F-85: its finger linkage and its closing profile in the TCP frame."""

from amp.geometry.gripper.linkage import GripperLinkage, axis_rotation, ee_points_to_tcp
from amp.geometry.gripper.profile import PAD_FACE_TOL, GripperProfile, PadSpan, profile
from amp.geometry.kinematics import TCP_OFFSET_DEFAULT
from amp.geometry.robot_asset import ROBOT_ASSET

__all__ = [
    "PAD_FACE_TOL", "ROBOT_ASSET", "TCP_OFFSET_DEFAULT", "GripperLinkage", "GripperProfile",
    "PadSpan", "axis_rotation", "ee_points_to_tcp", "profile",
]
