#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""capx_libero_panda: the robot of CaP-X's official LIBERO env, as AMP declares it."""

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
    Singularity,
    WorkspaceSpec,
)

NAME = "capx_libero_panda"
ARM, HAND = "arm", "hand"
ROOT = "robot0_base"
JOINTS = tuple(f"robot0_joint{i}" for i in range(1, 8))


DESCRIPTION_SHA256 = "ae3b3a1732e8d1275d793022170dc5c45c4f2fa2b3bdf60267f50bbec6333a02"

ARM_SHA256 = "a220ecd2910c01236d2566f4bac05957dc09786ea1d91862574762ba33537e5e"
MOUNT_PREFIX, PEDESTAL = "mount0_", "mount0_pedestal"
Q_MIN = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
Q_MAX = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)
Q_HOME = (0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483)


QD_MAX = (0.160, 0.160, 0.158, 0.156, 0.154, 0.150, 0.146)
CONTROL_DT = 0.05
STEP_CLAMP = (0.05,) * 7
TAU_S, TAU_BAND_S = 0.32, (0.30, 0.35)
HOLD_OFFSET_MAX = (0.00026, 0.00038, 0.00056, 0.00067, 0.00137, 0.00168, 0.00225)
EE_LINK, FLANGE_LINK = "gripper0_eef", "robot0_right_hand"
T_FLANGE_EE = (0.0, 1.0, 0.0, 0.0,
               -1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.097,
               0.0, 0.0, 0.0, 1.0)

T_EE_TCP = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, -0.0036,
            0.0, 0.0, 0.0, 1.0)
FINGER_JOINTS = ("gripper0_finger_joint1", "gripper0_finger_joint2")
FINGER_OPEN, FINGER_CLOSED = (0.04, -0.04), (0.0, 0.0)
CAMERA_WIDTH, CAMERA_HEIGHT = 800, 512
CAMERAS = (("agentview", "root", "scene"), ("robot0_eye_in_hand", FLANGE_LINK, "wrist"))
WORKSPACE_BOX = (-0.3, 1.2, -0.8, 0.8, -0.1, 1.0)


TABLE_REGION = (0.2, 0.85, 0.4, -0.05, 0.05, -0.03, 0.02)


def capx_libero_panda() -> EmbodimentSpec:
    """The official LIBERO Panda + PandaGripper under CaP-X's JOINT_POSITION controller."""
    arm = ArmSpec(name=ARM, joints=JOINTS, q_min=Q_MIN, q_max=Q_MAX, qd_max=QD_MAX,
                  q_home=Q_HOME, base_link=ROOT, flange_link=FLANGE_LINK,
                  kinematics=Kinematics("chain", DESCRIPTION_SHA256),
                  singularities=(Singularity(1, 0.10, 0.3),))
    gripper = GripperSpec(kind="parallel_jaw", model="linear", stroke_m=0.08, max_width_m=0.079,
                          closed_empty=0.985, held_below=0.95, finger_joints=FINGER_JOINTS,
                          finger_open=FINGER_OPEN, finger_closed=FINGER_CLOSED)
    hand = EndEffectorSpec(name=HAND, arm=ARM, ee_link=EE_LINK, T_flange_ee=T_FLANGE_EE,
                           T_ee_tcp=T_EE_TCP, gripper=gripper, closing_axis="x",
                           approach_axis="z")
    cameras = tuple(CameraSpec(n, CAMERA_WIDTH, CAMERA_HEIGHT, mount=mount, depth=True, role=role)
                    for n, mount, role in CAMERAS)
    return EmbodimentSpec(
        name=NAME, arms=(arm,), end_effectors=(hand,), cameras=cameras,
        action=(ActionSegment("joint_pos_abs", ARM, len(JOINTS)),
                ActionSegment("gripper01", HAND, 1)),
        control_dt=CONTROL_DT,
        drive=DriveSpec(tau_s=TAU_S, tau_band_s=TAU_BAND_S, hold_offset_max=HOLD_OFFSET_MAX,
                        step_clamp=STEP_CLAMP),
        workspace=WorkspaceSpec(support="table", box=WORKSPACE_BOX, table_region=TABLE_REGION),
    )
