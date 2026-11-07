#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot SDK for one episode: perception, manual motion by waypoints and the gripper, and,
on a robot with one, a mobile base and joint groups (mobile.py)."""

from .client import (
    CAMERAS,
    Cloud,
    EpisodeOver,
    HarnessError,
    Objects,
    color_name,
    explain,
    issues,
)
from .geometry import (
    OBB,
    Camera,
    T_to_pose,
    View,
    approach_axis,
    backproject,
    finger_yaw,
    fit_circle,
    fit_obb,
    footprint_overlap,
    heightmap,
    make_pose,
    mat_to_quat,
    offset_pose,
    pca_axes,
    points_in,
    pose_from_axes,
    pose_from_rpy,
    pose_to_T,
    quat_to_mat,
    quat_to_rpy,
    rpy_to_quat,
    side_pose,
    top_down,
    waypoint,
    waypoint_rpy,
    waypoint_top_down,
)
from .mobile import Episode, connect  # the base and joint-group calls on top of client's
from .render import Obj, SceneData, annotate_image, crop, depth_colormap, grid, render_scene

__all__ = [
    "CAMERAS", "OBB", "Camera", "Cloud", "Episode", "EpisodeOver", "HarnessError",
    "Obj", "Objects", "SceneData", "T_to_pose", "View", "annotate_image",
    "approach_axis", "backproject", "color_name", "connect", "crop", "depth_colormap", "explain",
    "finger_yaw", "fit_circle", "fit_obb", "footprint_overlap", "grid", "heightmap", "issues",
    "make_pose", "mat_to_quat", "offset_pose", "pca_axes", "points_in", "pose_from_axes",
    "pose_from_rpy", "pose_to_T", "quat_to_mat", "quat_to_rpy", "render_scene",
    "rpy_to_quat", "side_pose", "snapshot", "top_down", "waypoint", "waypoint_rpy",
    "waypoint_top_down",
]
from .cli import snapshot
