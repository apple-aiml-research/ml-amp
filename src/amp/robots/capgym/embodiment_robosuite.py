#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""capx_robosuite_panda(_nut): the robot of CaP-X's single-arm robosuite envs, as AMP declares it."""

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

CUBE, NUT = "capx_robosuite_panda", "capx_robosuite_panda_nut"
CUBE_TASK_SET, NUT_TASK_SET = "capgym_robosuite_cube", "capgym_robosuite_nut"
ARM, HAND = "arm", "hand"
ROOT = "robot0_base"
JOINTS = tuple(f"robot0_joint{i}" for i in range(1, 8))


DESCRIPTION_SHA256 = "7d4d75f1ac80bc5fd2830c777b6d2f6328d1514378b42e578f830eff7cdd75d9"

ARM_SHA256 = "0da3247facfe326ce0e4422836d4f8f814a7db0b6dfd494afb31ff1d91362b43"
MOUNT_PREFIX = "fixed_mount0_"
Q_MIN = (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973)
Q_MAX = (2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973)


Q_HOME_CUBE = (0.0, 0.19634954084936207, 0.0, -2.617993877991494, 0.0, 2.941592653589793,
               0.7853981633974483 - 3.141592653589793)
Q_HOME_NUT = (0.0, -1.585, 0.0, -2.645, 0.0, 1.0, 0.785)


QD_MAX = (2.40, 2.45, 2.85, 3.00, 3.10, 1.50, 1.45)
CONTROL_DT = 0.05


TAU_S, TAU_BAND_S = 0.28, (0.26, 0.33)

HOLD_OFFSET_MAX = (0.00002, 0.00010, 0.00004, 0.00003, 0.00005, 0.00007, 0.00007)
EE_LINK, FLANGE_LINK = "gripper0_right_eef", "robot0_right_hand"
T_FLANGE_EE = (0.0, 1.0, 0.0, 0.0,
               -1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.097,
               0.0, 0.0, 0.0, 1.0)

T_EE_TCP = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, -0.0036,
            0.0, 0.0, 0.0, 1.0)
FINGER_JOINTS = ("gripper0_right_finger_joint1", "gripper0_right_finger_joint2")
FINGER_OPEN, FINGER_CLOSED = (0.04, -0.04), (0.0, 0.0)


CLOSED_EMPTY, HELD_BELOW, MAX_WIDTH_M = 0.985, 0.95, 0.079
CAMERA_WIDTH, CAMERA_HEIGHT = 512, 512
CUBE_CAMERA, NUT_CAMERA = "robot0_robotview", "birdview"
WORKSPACE_BOX = (-0.3, 1.2, -0.8, 0.8, -0.2, 1.0)


CUBE_TABLE_REGION = (0.2, 0.9, 0.35, -0.16, -0.06, -0.13, -0.09)
NUT_TABLE_REGION = (0.2, 0.9, 0.35, -0.14, -0.04, -0.11, -0.07)


def _spec(name: str, q_home: tuple, camera: str, table_region: tuple) -> EmbodimentSpec:
    arm = ArmSpec(name=ARM, joints=JOINTS, q_min=Q_MIN, q_max=Q_MAX, qd_max=QD_MAX,
                  q_home=q_home, base_link=ROOT, flange_link=FLANGE_LINK,
                  kinematics=Kinematics("chain", DESCRIPTION_SHA256),
                  singularities=(Singularity(1, 0.10, 0.3),))
    gripper = GripperSpec(kind="parallel_jaw", model="linear", stroke_m=0.08,
                          max_width_m=MAX_WIDTH_M, closed_empty=CLOSED_EMPTY,
                          held_below=HELD_BELOW, finger_joints=FINGER_JOINTS,
                          finger_open=FINGER_OPEN, finger_closed=FINGER_CLOSED)
    hand = EndEffectorSpec(name=HAND, arm=ARM, ee_link=EE_LINK, T_flange_ee=T_FLANGE_EE,
                           T_ee_tcp=T_EE_TCP, gripper=gripper, closing_axis="x",
                           approach_axis="z")
    return EmbodimentSpec(
        name=name, arms=(arm,), end_effectors=(hand,),
        cameras=(CameraSpec(camera, CAMERA_WIDTH, CAMERA_HEIGHT, mount="root", depth=True,
                            role="scene"),),
        action=(ActionSegment("joint_pos_abs", ARM, len(JOINTS)),
                ActionSegment("gripper01", HAND, 1)),
        control_dt=CONTROL_DT,
        drive=DriveSpec(tau_s=TAU_S, tau_band_s=TAU_BAND_S, hold_offset_max=HOLD_OFFSET_MAX),
        workspace=WorkspaceSpec(support="table", box=WORKSPACE_BOX, table_region=table_region),
    )


def capx_robosuite_panda() -> EmbodimentSpec:
    """The cube envs' Panda + PandaGripper; robot0_robotview rides on robot0_base."""
    return _spec(CUBE, Q_HOME_CUBE, CUBE_CAMERA, CUBE_TABLE_REGION)


def capx_robosuite_panda_nut() -> EmbodimentSpec:
    """The nut env's Panda, the same robot; birdview is a world camera, static in the root."""
    return _spec(NUT, Q_HOME_NUT, NUT_CAMERA, NUT_TABLE_REGION)


EMBODIMENTS = {CUBE_TASK_SET: capx_robosuite_panda, NUT_TASK_SET: capx_robosuite_panda_nut}
