#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The DROID robot's camera mounts (RoboLab's), as amp.link.pointcloud.camera_from_offset kwargs."""

from __future__ import annotations

# isort: off
from amp.geometry.robolab_cameras import (  # noqa: F401
    _DROID_INTRINSICS,
    WRIST_CAMERA_CFG,
    DROID_LEFT_CAMERA_CFG,
    DROID_RIGHT_CAMERA_CFG,
)
# isort: on

WRIST_CAMERA = "wrist"
CAMERA_RIGS = {
    "droid": {"ego_l": DROID_LEFT_CAMERA_CFG, "ego_r": DROID_RIGHT_CAMERA_CFG},
}
