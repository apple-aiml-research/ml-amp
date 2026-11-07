#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Built-in embodiment profiles: DROID (RoboLab's Panda + Robotiq 2F-85) from geometry's values."""

from __future__ import annotations

from amp.contracts.embodiment import (
    DROID,
    ActionSegment,
    ArmSpec,
    CameraSpec,
    DriveSpec,
    EmbodimentSpec,
    EndEffectorSpec,
    GripperSpec,
    Kinematics,
    Singularity,
    WorkspaceSpec,
)
from amp.geometry import cameras
from amp.geometry.kinematics import joints as J
from amp.geometry.kinematics import tool as T
from amp.geometry.kinematics.transforms import make_pose

DROID_ARM = "panda"
DROID_EE = "robotiq_2f85"
DROID_JOINTS = tuple(f"panda_joint{i}" for i in range(1, 8))
DROID_BASE_LINK = "panda_link0"
DROID_FLANGE_LINK = "panda_link8"
DROID_EE_LINK = "base_link"

ROBOTIQ_STROKE_M = 0.085
ROBOTIQ_MAX_WIDTH_M = 0.080
HELD_CLOSED_READING = 0.96
DRIVE_TAU_S = 0.2
DRIVE_TAU_BAND_S = (0.16, 0.23)
HOLD_OFFSET_MAX_RAD = (0.03, 0.03, 0.03, 0.03, 0.2, 0.2, 0.2)
DROID_CONTROL_DT = 1.0 / 15.0
DROID_WORKSPACE_BOX = (-0.3, 1.2, -0.8, 0.8, -0.1, 1.0)


def _flat(M) -> tuple[float, ...]:
    return tuple(float(x) for x in M.reshape(-1))


def _floats(v) -> tuple[float, ...]:
    return tuple(float(x) for x in v)


def droid() -> EmbodimentSpec:
    """The DROID embodiment: what a backend whose hello declares no embodiment is."""
    arm = ArmSpec(
        name=DROID_ARM, joints=DROID_JOINTS,
        q_min=_floats(J.Q_MIN), q_max=_floats(J.Q_MAX), qd_max=_floats(J.QD_MAX),
        q_home=_floats(J.Q_HOME), base_link=DROID_BASE_LINK, flange_link=DROID_FLANGE_LINK,
        kinematics=Kinematics("dh_craig"),
        singularities=(Singularity(1, J.SINGULAR_Q2, J.SINGULAR_ZONE),),
    )
    gripper = GripperSpec(
        kind="parallel_jaw", model="robotiq_2f85_linkage", stroke_m=ROBOTIQ_STROKE_M,
        max_width_m=ROBOTIQ_MAX_WIDTH_M, closed_empty=1.0, held_below=HELD_CLOSED_READING,
    )
    ee = EndEffectorSpec(
        name=DROID_EE, arm=DROID_ARM, ee_link=DROID_EE_LINK,
        T_flange_ee=_flat(T.DEFAULT_T_FLANGE_EE),
        T_ee_tcp=_flat(make_pose([T.TCP_OFFSET_DEFAULT, 0.0, 0.0], T.EE_TO_TCP_QUAT)),
        gripper=gripper, closing_axis="x", approach_axis="z",
    )
    cams = [CameraSpec(name, cfg["width"], cfg["height"], mount="root", depth=True, role="scene")
            for name, cfg in cameras.CAMERA_RIGS["droid"].items()]
    w = cameras.WRIST_CAMERA_CFG
    cams.append(CameraSpec(cameras.WRIST_CAMERA, w["width"], w["height"], mount=DROID_EE_LINK,
                           depth=True, role="wrist"))
    return EmbodimentSpec(
        name=DROID, arms=(arm,), end_effectors=(ee,), cameras=tuple(cams),
        action=(ActionSegment("joint_pos_abs", DROID_ARM, len(DROID_JOINTS)),
                ActionSegment("gripper01", DROID_EE, 1)),
        control_dt=DROID_CONTROL_DT,
        drive=DriveSpec(tau_s=DRIVE_TAU_S, tau_band_s=DRIVE_TAU_BAND_S,
                        hold_offset_max=HOLD_OFFSET_MAX_RAD),
        workspace=WorkspaceSpec(support="table", box=DROID_WORKSPACE_BOX),
    )
