# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Modified for AMP: depth on the over-shoulder cameras and the wrist camera.

"""RoboLab's DROID registration with depth on all three DROID cameras, as AMP's harnesses read."""


WRIST_DEPTH_TERM = "wrist_cam_depth"


def register_amp_envs(task_dirs=None, task=None) -> None:
    """Register the DROID envs (left, right and wrist cameras, rgb and depth); call after Kit."""
    import robolab.constants
    from isaaclab.managers import ObservationTermCfg as ObsTerm
    from isaaclab.managers import SceneEntityCfg
    from robolab.constants import DEFAULT_TASK_SUBFOLDERS, TASK_DIR
    from robolab.core.environments.factory import auto_discover_and_create_cfgs
    from robolab.core.observations.observation_utils import (
        generate_image_obs_from_cameras,
        generate_obs_cfg,
        image_safe,
    )
    from robolab.robots.droid import (
        DroidCfg,
        DroidJointPositionActionCfg,
        ProprioceptionObservationCfg,
        WristCameraCfg,
        contact_gripper,
    )
    from robolab.variations.backgrounds import HomeOfficeBackgroundCfg
    from robolab.variations.camera import (
        EgocentricMirroredCameraCfg,
        OverShoulderLeftCameraCfg,
        OverShoulderRightCameraCfg,
        with_depth,
    )
    from robolab.variations.lighting import SphereLightCfg

    cameras = [with_depth(OverShoulderLeftCameraCfg), with_depth(OverShoulderRightCameraCfg),
               WristCameraCfg]
    viewport_camera = EgocentricMirroredCameraCfg

    ImageObsCfg = generate_image_obs_from_cameras(cameras)
    setattr(ImageObsCfg, WRIST_DEPTH_TERM, ObsTerm(func=image_safe, params={
        "sensor_cfg": SceneEntityCfg("wrist_cam"), "data_type": "depth", "normalize": False}))
    ViewportCameraCfg = generate_image_obs_from_cameras([viewport_camera])

    ObservationCfg = generate_obs_cfg({
        "image_obs": ImageObsCfg(),
        "proprio_obs": ProprioceptionObservationCfg(),
        "viewport_cam": ViewportCameraCfg()})

    robot_cfg = with_depth(DroidCfg)
    robot_cfg.ee_recorder_bodies = DroidCfg.ee_recorder_bodies
    scene_cameras = [c for c in cameras if c is not WristCameraCfg]

    auto_discover_and_create_cfgs(
        task_dir=TASK_DIR,
        task_subdirs=DEFAULT_TASK_SUBFOLDERS if task_dirs is None else task_dirs,
        tasks=task,
        pattern="*.py",
        env_prefix="",
        env_postfix="",
        observations_cfg=ObservationCfg(),
        actions_cfg=DroidJointPositionActionCfg(),
        robot_cfg=robot_cfg,
        camera_cfg=[*scene_cameras, viewport_camera],
        lighting_cfg=SphereLightCfg,
        background_cfg=HomeOfficeBackgroundCfg,
        contact_gripper=contact_gripper,
        lazy_sensor_update=True,
        object_state_obs=False,
        dt=1 / (60 * 2),
        render_interval=8,
        decimation=8,
        seed=1,
    )

    if robolab.constants.VERBOSE:
        from robolab.core.environments.factory import print_env_table
        print_env_table()
