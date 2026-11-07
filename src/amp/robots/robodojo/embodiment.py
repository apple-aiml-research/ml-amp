#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""robodojo_dual_x5: the two ARX X5A arms of RoboDojo's dual_x5 declaration, as AMP declares them."""

from __future__ import annotations

from amp.contracts.embodiment import (
    ActionSegment,
    ArmSpec,
    CameraSpec,
    DriveSpec,
    EmbodimentSpec,
    EndEffectorSpec,
    GripperSpec,
    Kinematics,
    WorkspaceSpec,
)

NAME = "robodojo_dual_x5"
TASK_SET = "robodojo"
ROOT = "dual_x5_root"
SIDES = ("left", "right")
ARMS = SIDES
HANDS = tuple(f"{s}_hand" for s in SIDES)
JOINTS = {s: tuple(f"{s}_joint{i}" for i in range(1, 7)) for s in SIDES}
BASE_LINK = {s: f"{s}_base_link" for s in SIDES}
EE_LINK = {s: f"{s}_link6" for s in SIDES}
CAMERA_LINK = {s: f"{s}_camera" for s in SIDES}
FINGER_JOINTS = {s: (f"{s}_joint7", f"{s}_joint8") for s in SIDES}
URDF_ROOT_LINK = "base_link"
BASE_POS_WORLD = {"left": (-0.3, -0.45, 0.765), "right": (0.3, -0.45, 0.765)}
BASE_ROT_WXYZ = (0.707, 0.0, 0.0, 0.707)
BASE_POS = {"left": (0.0, 0.3, 0.0), "right": (0.0, -0.3, 0.0)}
T_WORLD_ROOT = (0.0, -1.0, 0.0, 0.0,
                1.0, 0.0, 0.0, -0.45,
                0.0, 0.0, 1.0, 0.765,
                0.0, 0.0, 0.0, 1.0)
Q_MIN = (-1.57, -0.1, -0.1, -1.29, -1.48, -1.74)
Q_MAX = (1.57, 3.6, 3.0, 1.29, 1.48, 1.74)
QD_MAX = (5.0,) * 6
Q_HOME = (0.0,) * 6
EYE = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
TCP_X, TCP_Z = 0.15007, -0.00144
T_EE_TCP = (0.0, 0.0, 1.0, TCP_X,
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, TCP_Z,
            0.0, 0.0, 0.0, 1.0)
FINGER_STROKE_M = 0.044
STROKE_M = 2 * FINGER_STROKE_M
MAX_WIDTH_M = 0.085
FINGER_OPEN = (FINGER_STROKE_M, FINGER_STROKE_M)
FINGER_CLOSED = (0.0, 0.0)
TIP_REACH_M = (0.0075, 0.0075)
CLOSED_EMPTY, HELD_BELOW = 1.0, 0.999
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
HEAD_CAMERA = "cam_head"
WRIST_CAMERAS = {s: f"cam_{s}_wrist" for s in SIDES}
CONTROL_DT = 0.04
DESCRIPTION_SHA256 = "612beab4dcfe1a75cce82e5e5317fbfacacb8ceb647dc924a642cbe32c625717"
TABLE_TOP_Z = 0.0
TABLE_TOP_Z_WORLD = 0.765
WORKSPACE_BOX = (-0.25, 1.05, -0.9, 0.9, -0.065, 0.685)
TABLE_REGION = (-0.10, 1.00, 0.6, TABLE_TOP_Z - 0.05, TABLE_TOP_Z + 0.10,
                TABLE_TOP_Z - 0.02, TABLE_TOP_Z + 0.05)


def _arm(side: str, sha: str) -> tuple[ArmSpec, EndEffectorSpec]:
    arm = ArmSpec(name=side, joints=JOINTS[side], q_min=Q_MIN, q_max=Q_MAX, qd_max=QD_MAX,
                  q_home=Q_HOME, base_link=BASE_LINK[side], flange_link=EE_LINK[side],
                  kinematics=Kinematics("chain", sha))
    gripper = GripperSpec(kind="parallel_jaw", model="linear", stroke_m=STROKE_M,
                          max_width_m=MAX_WIDTH_M, closed_empty=CLOSED_EMPTY,
                          held_below=HELD_BELOW, finger_joints=FINGER_JOINTS[side],
                          finger_open=FINGER_OPEN, finger_closed=FINGER_CLOSED,
                          tip_reach_m=TIP_REACH_M)
    hand = EndEffectorSpec(name=f"{side}_hand", arm=side, ee_link=EE_LINK[side], T_flange_ee=EYE,
                           T_ee_tcp=T_EE_TCP, gripper=gripper, closing_axis="x",
                           approach_axis="z")
    return arm, hand


def robodojo_dual_x5(description_sha256: str = DESCRIPTION_SHA256) -> EmbodimentSpec:
    """RoboDojo's two X5A arms on the table, wrist cameras on link6 and a head camera."""
    left, left_hand = _arm("left", description_sha256)
    right, right_hand = _arm("right", description_sha256)
    cameras = (CameraSpec(HEAD_CAMERA, CAMERA_WIDTH, CAMERA_HEIGHT, mount="root", depth=True,
                          role="scene"),) + tuple(
        CameraSpec(WRIST_CAMERAS[s], CAMERA_WIDTH, CAMERA_HEIGHT, mount=CAMERA_LINK[s],
                   depth=True, role="wrist") for s in SIDES)
    return EmbodimentSpec(
        name=NAME, arms=(left, right), end_effectors=(left_hand, right_hand), cameras=cameras,
        action=(ActionSegment("joint_pos_abs", "left", 6),
                ActionSegment("gripper01", "left_hand", 1),
                ActionSegment("joint_pos_abs", "right", 6),
                ActionSegment("gripper01", "right_hand", 1)),
        control_dt=CONTROL_DT,
        drive=DriveSpec(tau_s=0.020, tau_band_s=(0.016, 0.025)),
        workspace=WorkspaceSpec(support="table", box=WORKSPACE_BOX, table_region=TABLE_REGION),
    )


EMBODIMENTS = {TASK_SET: robodojo_dual_x5}
