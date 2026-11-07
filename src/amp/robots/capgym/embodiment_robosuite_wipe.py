#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""capx_robosuite_panda_wipe: the Panda of CaP-X's spill_wipe env, a fingerless wiping tool as its end effector."""

from __future__ import annotations

from amp.contracts.embodiment import (
    ActionSegment,
    ArmSpec,
    CameraSpec,
    DriveSpec,
    EmbodimentSpec,
    EndEffectorSpec,
    Kinematics,
    Singularity,
    WorkspaceSpec,
)
from amp.robots.capgym import embodiment_robosuite as ER

NAME, TASK_SET = "capx_robosuite_panda_wipe", "capgym_robosuite_wipe"
ARM, TOOL = ER.ARM, "wiper"
ROOT, JOINTS, MOUNT_PREFIX = ER.ROOT, ER.JOINTS, ER.MOUNT_PREFIX
CAMERA = ER.CUBE_CAMERA



DESCRIPTION_SHA256 = "708a7ba78a18b305f4c676002ed0e63ebe9b5791afeb1c0ce38b5ade5ec8ea27"
ARM_SHA256 = "2d2db4450b3d71085a72c29613b2afc0ff6d6494a04b5abbe00899a0c9b798d5"




Q_HOME_WIPE = (0.0, 0.2176, 0.0, -2.6296, 0.0, 2.9411, ER.Q_HOME_CUBE[6])
EE_LINK, FLANGE_LINK = ER.EE_LINK, ER.FLANGE_LINK
T_FLANGE_EE = (0.0, 1.0, 0.0, 0.0,
               -1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.03,
               0.0, 0.0, 0.0, 1.0)


T_EE_TCP = (1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0)
WORKSPACE_BOX = ER.WORKSPACE_BOX


TABLE_REGION = (0.2, 0.72, 0.35, -0.06, 0.04, -0.03, 0.01)


def capx_robosuite_panda_wipe() -> EmbodimentSpec:
    """The Wipe env's Panda; its tool has no fingers, so the action is the 7 joint targets alone."""
    arm = ArmSpec(name=ARM, joints=JOINTS, q_min=ER.Q_MIN, q_max=ER.Q_MAX, qd_max=ER.QD_MAX,
                  q_home=Q_HOME_WIPE, base_link=ROOT, flange_link=FLANGE_LINK,
                  kinematics=Kinematics("chain", DESCRIPTION_SHA256),
                  singularities=(Singularity(1, 0.10, 0.3),))
    tool = EndEffectorSpec(name=TOOL, arm=ARM, ee_link=EE_LINK, T_flange_ee=T_FLANGE_EE,
                           T_ee_tcp=T_EE_TCP, gripper=None, closing_axis="x", approach_axis="z")
    return EmbodimentSpec(
        name=NAME, arms=(arm,), end_effectors=(tool,),
        cameras=(CameraSpec(CAMERA, ER.CAMERA_WIDTH, ER.CAMERA_HEIGHT, mount="root", depth=True,
                            role="scene"),),
        action=(ActionSegment("joint_pos_abs", ARM, len(JOINTS)),),
        control_dt=ER.CONTROL_DT,
        drive=DriveSpec(tau_s=ER.TAU_S, tau_band_s=ER.TAU_BAND_S,
                        hold_offset_max=ER.HOLD_OFFSET_MAX),
        workspace=WorkspaceSpec(support="table", box=WORKSPACE_BOX, table_region=TABLE_REGION),
    )


EMBODIMENTS = {TASK_SET: capx_robosuite_panda_wipe}
