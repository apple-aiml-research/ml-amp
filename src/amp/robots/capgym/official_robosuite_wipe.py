#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The official robosuite wipe adapter: CaP-X's spill_wipe env, a fingerless sponge tool on the Panda."""

from __future__ import annotations

import os
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from amp.benchmarks import capgym
from amp.robots.capgym import embodiment_robosuite_wipe as EW
from amp.robots.capgym import official_robosuite as R
from amp.robots.capgym.adapter import CameraImage, Frame, env_call
from amp.robots.capgym.official import _git_head, capx_root

END_ON: tuple[str, ...] = ()
CAUSES = ("success", "collision", "joint_limit")





def frame_of(env, camera: str, ee: Callable[[Any], np.ndarray]) -> Frame:
    """The state now: q from qpos, ee(env) for gripper0_right_eef, a fresh render; no gripper."""
    rs = env.robosuite_env

    q = np.array(rs.sim.data.qpos[rs.robots[0]._ref_joint_pos_indexes], dtype=np.float64)
    T = ee(env)
    rgb, depth = R.render_camera(env, camera)
    cam = CameraImage(rgb=rgb, depth=depth, K=R.intrinsics(env, camera),
                      T_root_cam=R.camera_in_root(env, camera))
    return Frame(q=q, grippers={}, ee={EW.TOOL: (T[:3, 3].copy(), R._mat_quat(T[:3, :3]))},
                 cameras={camera: cam})


def apply_action(env, action: Sequence[float]) -> None:
    """One control step: CaP-X's non-blocking joint move, which slices the 7 targets itself."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape != (7,) or not np.all(np.isfinite(a)):
        raise ValueError(f"action must be 7 finite joint targets, got {a}")

    env_call(env.move_to_joints_non_blocking, a)


def causes_of(env) -> list[str]:
    """Which of robosuite's three early-termination conditions hold now (wipe.py:_check_terminated)."""
    rs = env.robosuite_env
    out = ["success"] if rs._check_success() else []
    if rs.check_contact(rs.robots[0].robot_model):
        out.append("collision")
    if rs.robots[0].check_q_limits():
        out.append("joint_limit")
    return out


class WipeAdapter(R.RobosuiteAdapter):
    """A SimAdapter over CaP-X's spill_wipe env; score is the fraction of the 100 markers wiped."""

    def __init__(self, *, task_set: str = EW.TASK_SET, tasks: dict | None = None,
                 build: Callable[[Any], Any] = R.build_env,
                 describe: Callable[[Any], tuple] = R.describe_env,
                 log: Callable[[str], None] = R._log):
        if task_set not in EW.EMBODIMENTS:
            raise ValueError(f"the wipe adapter serves {', '.join(EW.EMBODIMENTS)}, not "
                             f"{task_set!r}")
        self.task_set = task_set
        self.spec = EW.EMBODIMENTS[task_set]()
        every = dict(capgym.robosuite_tasks() if tasks is None else tasks)
        self._tasks = {n: t for n, t in every.items() if t.task_set == task_set}
        self._build, self._describe, self.log = build, describe, log
        self._camera = self.spec.cameras[0].name
        self._robot: tuple[Any, dict] | None = None
        self._scratch: dict = {}
        self.controller_cfg: str | None = None
        self.env: Any = None
        self.task: str | None = None
        self.unit: dict | None = None
        self._fingerprint: str | None = None
        self._ended = False

    def load(self, task: str) -> None:
        if self.env is not None and self.task == task:
            return
        if task not in self._tasks:
            raise ValueError(f"no capgym task {task!r} in {self.task_set}")
        t = self._tasks[task]
        if t.viser_debug:
            raise ValueError(f"{task}: viser_debug configs are not served by this adapter")
        self.close()
        env = self._build(t)
        try:
            self._check(task, t, env)
            self._check_wipe(task, env)
            robot = self._describe(env)
            from amp.link.robot import description_sha256

            arm = description_sha256(*R.without_mount(*robot))
            if arm != EW.ARM_SHA256:
                raise ValueError(f"{task}: the env's robot description (mount left out) is "
                                 f"{arm[:12]}, the embodiment's arm {EW.ARM_SHA256[:12]}")
            sha = description_sha256(*robot)
            if sha != EW.DESCRIPTION_SHA256:
                raise ValueError(f"{task}: the env's robot description is {sha[:12]}, the "
                                 f"embodiment names {EW.DESCRIPTION_SHA256[:12]}")
        except BaseException:
            R._close(env)
            raise
        self.env, self.task, self._robot = env, task, robot
        self.controller_cfg = getattr(env, "controller_cfg", None)

    @staticmethod
    def _check_wipe(task: str, env) -> None:
        rs = env.robosuite_env
        gripper = type(rs.robots[0].gripper["right"]).__name__
        if gripper != "WipingGripper" or int(rs.action_dim) != 7:
            raise ValueError(f"{task}: the env's tool is {gripper} with a {rs.action_dim}-dim "
                             "action, not the 7-dim WipingGripper")
        if int(rs.num_markers) != 100 or not rs.early_terminations:
            raise ValueError(f"{task}: num_markers {rs.num_markers}, early_terminations "
                             f"{rs.early_terminations}; the official env has 100 and True")

    def reset(self, episode: int) -> None:
        self._ended = False
        super().reset(episode)

    def observe(self) -> Frame:
        return frame_of(self.env, self._camera, lambda env: R.ee_in_root(env, self._scratch))

    def apply(self, action: Sequence[float]) -> None:
        apply_action(self.env, action)
        causes = causes_of(self.env)
        for c in causes:
            self.unit.setdefault("early_termination", {}).setdefault(c, self.sim_steps())
        self._ended = any(c in END_ON for c in causes)

    def ended(self) -> bool:
        """True once a cause in END_ON holds; END_ON is empty, as CaP-X ends no wipe trial early."""
        return self._ended

    def success(self) -> tuple[bool, float]:
        """(task_completed(), wiped / num_markers), the wiped set read after compute_reward()."""
        rs = self.env.robosuite_env

        self.env.get_observation()
        self.env.compute_reward()
        return bool(self.env.task_completed()), len(rs.wiped_markers) / rs.num_markers

    def provenance(self) -> dict:
        root = None
        try:
            root = capx_root()
        except RuntimeError:
            pass
        rs_mod = sys.modules.get("robosuite")
        rs_dir = Path(rs_mod.__file__).resolve().parents[1] if rs_mod and rs_mod.__file__ else None
        out = {"capx_commit": _git_head(root), "robosuite_commit": _git_head(rs_dir),
               "mujoco_gl": os.environ.get("MUJOCO_GL"),
               "egl_device": os.environ.get("MUJOCO_EGL_DEVICE_ID"),
               "embodiment": self.spec.name, "description_sha256": EW.DESCRIPTION_SHA256,
               "controller_cfg": self._controller_path(),
               "end_on": list(END_ON)}
        for mod in ("robosuite", "mujoco"):
            out[f"{mod}_version"] = getattr(sys.modules.get(mod), "__version__", None)
        return out

    def close(self) -> None:
        self._ended = False
        super().close()


EMBODIMENTS = EW.EMBODIMENTS


def make_adapter(*, task_set: str, cameras: str, seed: int | None) -> WipeAdapter:
    """The AdapterFactory: the wipe task set; seed is ignored, the trial is the unit's."""
    return WipeAdapter(task_set=task_set)
