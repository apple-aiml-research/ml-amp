# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Modified for AMP: values restated as camera_from_offset kwargs.

"""RoboLab v0.3.1's DROID camera mounts, as amp.link.pointcloud.camera_from_offset kwargs."""

from __future__ import annotations

_DROID_INTRINSICS = {"focal_length": 2.1, "horizontal_aperture": 5.376,
                     "vertical_aperture": 3.024, "width": 1280, "height": 720}
WRIST_CAMERA_CFG = {"pos": (0.011, -0.031, -0.074), "rot_wxyz": (-0.420, 0.570, 0.576, -0.409),
                    "convention": "opengl", **_DROID_INTRINSICS, "focal_length": 2.8}
DROID_LEFT_CAMERA_CFG = {"pos": (0.05, 0.57, 0.66), "rot_wxyz": (-0.393, -0.195, 0.399, 0.805),
                         "convention": "opengl", **_DROID_INTRINSICS}
DROID_RIGHT_CAMERA_CFG = {"pos": (0.05, -0.57, 0.66), "rot_wxyz": (0.805, 0.399, -0.195, -0.393),
                          "convention": "opengl", **_DROID_INTRINSICS}
