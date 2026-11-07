# SPDX-FileCopyrightText: Copyright (c) 2026 Max Fu
# SPDX-License-Identifier: MIT
# From CaP-X 53e9966d7a8e: run_libero_batch.py:150-159, robosuite_base.py:249-256 and :267-273,
# robosuite_two_arm_lift.py:326-332 and robosuite_handover.py:343-360 (loop body), r1pro_b1k.py:766.
# Modified for AMP: restated as functions, renamed, max_steps an argument; loop and checks left out.

"""The lines AMP's official capgym adapters take from CaP-X, so its envs are built and stepped as CaP-X does."""

from __future__ import annotations

import numpy as np

from amp.robots.capgym.adapter import env_call

FRANKA_LIBERO_ENV = "capx.envs.simulators.libero.FrankaLiberoEnv"


def batch_low_level(suite: str, task_id: int, privileged: bool, max_steps: int = 8000) -> dict:
    """The low_level of one LIBERO-PRO task: run_libero_batch.py:150-159's, with max_steps."""
    return {"_target_": FRANKA_LIBERO_ENV, "suite_name": suite, "task_id": task_id,
            "privileged": privileged, "max_steps": max_steps, "seed": None, "enable_render": True,
            "viser_debug": False}


def intrinsics(env, camera: str) -> np.ndarray:
    """K as CaP-X computes it: f = 0.5 H / tan(fovy / 2), c = (W / 2, H / 2)."""
    model = env.robosuite_env.sim.model
    fovy = float(model.cam_fovy[model.camera_name2id(camera)])
    w, h = env._render_width, env._render_height
    f = 0.5 * h / np.tan(fovy * np.pi / 360.0)
    return np.array([[f, 0.0, 0.5 * w], [0.0, f, 0.5 * h], [0.0, 0.0, 1.0]])


def opencv_from_mujoco() -> np.ndarray:
    """A new 4x4 that turns a MuJoCo camera frame into CaP-X's pose_mat (OpenCV) frame."""

    return np.diag([1.0, -1.0, -1.0, 1.0])


def step_both(env, a: np.ndarray, step_kw: dict) -> None:
    """One step of move_to_joints_blocking_both's loop: set both grippers, step, count the step."""

    env_call(env._set_gripper, 1.0 - a[7])
    env_call(env._set_gripper_arm1, 1.0 - a[15])
    robot0_action = np.concatenate([a[0:7], [1.0 - env._gripper_fraction_0 * 2.0]])
    robot1_action = np.concatenate([a[8:15], [1.0 - env._gripper_fraction_1 * 2.0]])
    env_call(env.robosuite_env.step, np.concatenate([robot0_action, robot1_action]), **step_kw)
    env._sim_step_count += 1


def world_frame_action(robot, q):
    """robot.q_to_action of q, its base made world-frame first (r1pro_b1k.py:766)."""
    from omnigibson.action_primitives.curobo import holonomic_base_command_in_world_frame

    return robot.q_to_action(holonomic_base_command_in_world_frame(robot, q))
