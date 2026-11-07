#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""capx_robosuite_two_panda_*: the two Pandas of CaP-X's two-arm robosuite envs, as AMP declares them."""

from __future__ import annotations

import math

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
from amp.robots.capgym import embodiment_robosuite as ER

LIFT, HANDOVER = "capx_robosuite_two_panda_lift", "capx_robosuite_two_panda_handover"
LIFT_TASK_SET = "capgym_robosuite_two_arm_lift"
HANDOVER_TASK_SET = "capgym_robosuite_two_arm_handover"
ARMS, HANDS = ("robot0", "robot1"), ("hand0", "hand1")
ROOT = ER.ROOT
BASES = tuple(f"robot{i}_base" for i in range(2))
JOINTS = tuple(tuple(f"robot{i}_joint{k}" for k in range(1, 8)) for i in range(2))
FLANGE_LINKS = tuple(f"robot{i}_right_hand" for i in range(2))
EE_LINKS = tuple(f"gripper{i}_right_eef" for i in range(2))
FINGER_JOINTS = tuple((f"gripper{i}_right_finger_joint1", f"gripper{i}_right_finger_joint2")
                      for i in range(2))
MOUNT_PREFIXES = ("fixed_mount0_", "fixed_mount1_")


BASE1_X = {LIFT_TASK_SET: 1.12, HANDOVER_TASK_SET: 1.62}


def base1_pose(task_set: str) -> tuple[float, ...]:
    """robot1_base in robot0_base, 4x4 row-major."""
    return (-1.0, 0.0, 0.0, BASE1_X[task_set],
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0)





DESCRIPTION_SHA256 = {
    LIFT_TASK_SET: "997d35699a667b223bc21768df46fe13b33914cc02df5cf96e1a92e005153911",
    HANDOVER_TASK_SET: "ca3a3d04806992d77496a2219150f7d86097290635663b35515e223a313afec2",
}

ARM_SHA256 = {
    LIFT_TASK_SET: "05a01633351d5d6c0bc557b78dcea9994469dfea2c5fe11138669578642fa118",
    HANDOVER_TASK_SET: "599d21a02f158e39fe8108289f005302f8a99b8fd8a4a152c9c17e87601ca570",
}


Q_HOME = (0.0, math.pi / 16.0, 0.0, -math.pi / 2.0 - math.pi / 3.0, 0.0, math.pi - 0.2,
          math.pi / 4.0)


QD_MAX = (2.40, 2.45, 2.75, 2.90, 3.00, 1.50, 1.45)


TAU_S, TAU_BAND_S = ER.TAU_S, ER.TAU_BAND_S


HOLD_OFFSET_MAX = (0.00025, 0.00035, 0.00085, 0.00125, 0.00055, 0.00075, 0.00015) * 2
CAMERA = "agentview"
WORKSPACE_BOX = {LIFT_TASK_SET: (-0.3, 1.45, -0.8, 0.8, -0.2, 1.0),
                 HANDOVER_TASK_SET: (-0.3, 1.95, -0.8, 0.8, -0.2, 1.0)}


TABLE_REGION = {LIFT_TASK_SET: (0.2, 0.92, 0.35, -0.16, -0.06, -0.13, -0.09),
                HANDOVER_TASK_SET: (0.15, 0.92, 0.35, -0.16, -0.06, -0.13, -0.09)}


def _arm(i: int, sha: str) -> tuple[ArmSpec, EndEffectorSpec]:
    arm = ArmSpec(name=ARMS[i], joints=JOINTS[i], q_min=ER.Q_MIN, q_max=ER.Q_MAX,
                  qd_max=QD_MAX, q_home=Q_HOME, base_link=BASES[i],
                  flange_link=FLANGE_LINKS[i], kinematics=Kinematics("chain", sha),
                  singularities=(Singularity(1, 0.10, 0.3),))
    gripper = GripperSpec(kind="parallel_jaw", model="linear", stroke_m=0.08,
                          max_width_m=ER.MAX_WIDTH_M, closed_empty=ER.CLOSED_EMPTY,
                          held_below=ER.HELD_BELOW, finger_joints=FINGER_JOINTS[i],
                          finger_open=ER.FINGER_OPEN, finger_closed=ER.FINGER_CLOSED)
    hand = EndEffectorSpec(name=HANDS[i], arm=ARMS[i], ee_link=EE_LINKS[i],
                           T_flange_ee=ER.T_FLANGE_EE, T_ee_tcp=ER.T_EE_TCP, gripper=gripper,
                           closing_axis="x", approach_axis="z")
    return arm, hand


def _spec(name: str, task_set: str) -> EmbodimentSpec:
    sha = DESCRIPTION_SHA256[task_set]
    (a0, h0), (a1, h1) = _arm(0, sha), _arm(1, sha)
    return EmbodimentSpec(
        name=name, arms=(a0, a1), end_effectors=(h0, h1),
        cameras=(CameraSpec(CAMERA, ER.CAMERA_WIDTH, ER.CAMERA_HEIGHT, mount="root", depth=True,
                            role="scene"),),

        action=(ActionSegment("joint_pos_abs", ARMS[0], 7), ActionSegment("gripper01", HANDS[0], 1),
                ActionSegment("joint_pos_abs", ARMS[1], 7), ActionSegment("gripper01", HANDS[1], 1)),
        control_dt=ER.CONTROL_DT,
        drive=DriveSpec(tau_s=TAU_S, tau_band_s=TAU_BAND_S, hold_offset_max=HOLD_OFFSET_MAX),
        workspace=WorkspaceSpec(support="table", box=WORKSPACE_BOX[task_set],
                                table_region=TABLE_REGION[task_set]),
    )


def capx_robosuite_two_panda_lift() -> EmbodimentSpec:
    """two_arm_lift's Pandas, robot1 1.12 m ahead of robot0 facing it; agentview in the world."""
    return _spec(LIFT, LIFT_TASK_SET)


def capx_robosuite_two_panda_handover() -> EmbodimentSpec:
    """two_arm_handover's Pandas, robot1 1.62 m ahead of robot0 facing it; agentview in the world."""
    return _spec(HANDOVER, HANDOVER_TASK_SET)


EMBODIMENTS = {LIFT_TASK_SET: capx_robosuite_two_panda_lift,
               HANDOVER_TASK_SET: capx_robosuite_two_panda_handover}
