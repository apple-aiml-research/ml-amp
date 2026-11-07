#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The official robosuite adapter: CaP-X's single-arm robosuite envs, built and driven as CaP-X does."""

from __future__ import annotations

import importlib
import inspect
import os
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from amp.benchmarks import capgym
from amp.contracts.embodiment import RobotDescription
from amp.robots.capgym import embodiment_robosuite as ER
from amp.robots.capgym.adapter import CameraImage, Frame, Mesh, env_call
from amp.robots.capgym.capx_derived import intrinsics, opencv_from_mujoco
from amp.robots.capgym.official import _git_head, capx_root, env_fingerprint

EMBODIMENTS = ER.EMBODIMENTS


def _log(message: str) -> None:
    print(f"[amp.robots.capgym.official_robosuite] {message}", file=sys.stderr, flush=True)


def env_class(dotted: str) -> type:
    """The class a manifest's env_class names."""
    module, _, name = dotted.rpartition(".")
    return getattr(importlib.import_module(module), name)


def controller_cfg(cls: type, root: Path | None = None) -> str:
    """The class's own default controller JSON, resolved in the cap-x checkout CaP-X runs from."""
    for c in cls.__mro__:
        p = inspect.signature(c.__init__).parameters.get("controller_cfg")
        if p is not None and isinstance(p.default, str):
            path = Path(p.default)
            path = path if path.is_absolute() else (root or capx_root()) / path
            if not path.is_file():
                raise FileNotFoundError(f"{cls.__name__}'s controller config {path} is missing")
            return str(path)
    raise ValueError(f"{cls.__name__} has no controller_cfg default")


def build_env(task) -> Any:
    """The task's class with exactly the YAML's flags, never through capx's lru_cached get_env."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    cls = env_class(task.env_class)

    return cls(privileged=task.privileged, enable_render=task.enable_render,
               viser_debug=task.viser_debug, controller_cfg=controller_cfg(cls))


def describe_env(env) -> tuple:
    """(description, meshes) of the env's robot from its MjModel, read only."""
    from amp.robots.capgym import official_model

    return official_model.export(env.robosuite_env.sim.model._model,
                                 mount_prefix=ER.MOUNT_PREFIX)


def without_mount(description: RobotDescription, meshes: dict) -> tuple[RobotDescription, dict]:
    """The description and meshes with the RethinkMount's links and joints left out."""
    links = tuple(lk for lk in description.links if not lk.name.startswith(ER.MOUNT_PREFIX))
    joints = tuple(j for j in description.joints if not j.child.startswith(ER.MOUNT_PREFIX))
    keep = {m for lk in links for m in lk.meshes}
    return (RobotDescription(root=description.root, links=links, joints=joints),
            {k: v for k, v in meshes.items() if k in keep})


def _body(sim, name: str) -> np.ndarray:
    b = sim.model.body_name2id(name)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.reshape(sim.data.xmat[b], (3, 3)), sim.data.xpos[b]
    return T


def _mat_quat(R) -> np.ndarray:
    """wxyz, w >= 0, of a rotation matrix."""
    R = np.asarray(R, dtype=np.float64)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i], q[1 + j], q[1 + k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q) * (1.0 if q[0] >= 0 else -1.0)


def real_depth(sim, depth: np.ndarray) -> np.ndarray:
    """robosuite's get_real_depth_map, which CaP-X's get_observation applies."""
    from robosuite.utils.camera_utils import get_real_depth_map

    return get_real_depth_map(sim, depth)


def render_camera(env, camera: str) -> tuple[np.ndarray, np.ndarray]:
    """(rgb, metric depth) of robosuite's own camera sensor now, flipped as get_observation does."""
    rs = env.robosuite_env
    cache: dict = {}

    rgb = rs._observables[f"{camera}_image"]._sensor(cache)
    depth = np.asarray(real_depth(rs.sim, cache[f"{camera}_depth"][::-1]), dtype=np.float32)
    return (np.ascontiguousarray(rgb[::-1], dtype=np.uint8),
            np.ascontiguousarray(depth.reshape(depth.shape[:2])))


def camera_in_root(env, camera: str) -> np.ndarray:
    """The camera's OpenCV frame in robot0_base: CaP-X's pose_mat, re-expressed from its mount."""
    sim = env.robosuite_env.sim
    T = np.eye(4)
    T[:3, :3] = np.reshape(sim.data.get_camera_xmat(camera), (3, 3))
    T[:3, 3] = sim.data.get_camera_xpos(camera)
    return np.linalg.inv(_body(sim, ER.ROOT)) @ T @ opencv_from_mujoco()


def ee_in_root(env, scratch: dict) -> np.ndarray:
    """gripper0_right_eef in robot0_base at the current qpos, from kinematics on a scratch MjData."""
    import mujoco

    sim = env.robosuite_env.sim
    m = sim.model._model

    if scratch.get("model") is not m:
        scratch.update(model=m, data=mujoco.MjData(m))
    d = scratch["data"]
    d.qpos[:] = sim.data._data.qpos
    mujoco.mj_kinematics(m, d)
    out = []
    for name in (ER.ROOT, ER.EE_LINK):
        b = sim.model.body_name2id(name)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = d.xmat[b].reshape(3, 3), d.xpos[b]
        out.append(T)
    return np.linalg.inv(out[0]) @ out[1]


def frame_of(env, camera: str, ee: Callable[[Any], np.ndarray]) -> Frame:
    """The state now: q and the gripper from qpos, ee(env) for gripper0_right_eef, a fresh render."""
    sim = env.robosuite_env.sim
    robot = env.robosuite_env.robots[0]


    q = np.array(sim.data.qpos[robot._ref_joint_pos_indexes], dtype=np.float64)
    finger = robot._ref_gripper_joint_pos_indexes["right"][0]
    reading = float(sim.data.qpos[finger]) / float(env.gripper_metric_length)
    T = ee(env)
    rgb, depth = render_camera(env, camera)
    cam = CameraImage(rgb=rgb, depth=depth, K=intrinsics(env, camera),
                      T_root_cam=camera_in_root(env, camera))
    return Frame(q=q, grippers={ER.HAND: float(np.clip(1.0 - reading, 0.0, 1.0))},
                 ee={ER.HAND: (T[:3, 3].copy(), _mat_quat(T[:3, :3]))}, cameras={camera: cam})


def apply_action(env, action: Sequence[float]) -> None:
    """One control step: CaP-X's non-blocking joint move, which always steps exactly once."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape != (8,) or not np.all(np.isfinite(a)):
        raise ValueError(f"action must be 8 finite numbers (7 joint targets, gripper), got {a}")
    env_call(env._set_gripper, 1.0 - a[7])

    env_call(env.move_to_joints_non_blocking, a[:7])


def fingerprint_of(env) -> str | None:
    """official.env_fingerprint of the robosuite sim: qpos, body_xpos and body_xquat."""
    return env_fingerprint(SimpleNamespace(handle=SimpleNamespace(env=env.robosuite_env)))


class RobosuiteAdapter:
    """A SimAdapter over CaP-X's single-arm robosuite envs of one task set."""

    features = ("grade",)

    def __init__(self, *, task_set: str, tasks: dict | None = None,
                 build: Callable[[Any], Any] = build_env,
                 describe: Callable[[Any], tuple] = describe_env,
                 log: Callable[[str], None] = _log):
        if task_set not in ER.EMBODIMENTS:
            raise ValueError(f"the robosuite adapter serves {', '.join(ER.EMBODIMENTS)}, not "
                             f"{task_set!r}")
        self.task_set = task_set
        self.spec = ER.EMBODIMENTS[task_set]()
        every = dict(capgym.robosuite_tasks() if tasks is None else tasks)
        self._tasks = {n: t for n, t in every.items() if t.task_set == task_set}
        self._build, self._describe, self.log = build, describe, log
        self._camera = self.spec.cameras[0].name
        self._robot: tuple[Any, dict[str, Mesh]] | None = None
        self._scratch: dict = {}
        self.controller_cfg: str | None = None
        self.env: Any = None
        self.task: str | None = None
        self.unit: dict | None = None
        self._fingerprint: str | None = None

    def embodiment(self):
        return self.spec

    def robot_description(self) -> tuple:
        """The declared description, exported from the first task's scene."""
        if self._robot is None:
            if not self._tasks:
                raise ValueError(f"no {self.task_set} task to read the robot from")
            self.load(next(iter(self._tasks)))
        return self._robot

    def success_checks(self) -> str:
        return "end"

    def tasks(self) -> list[dict]:
        return [{"name": n, "instruction": t.instruction, "attributes": [t.task_set, t.source],
                 "episode_length_s": t.max_steps / t.control_freq}
                for n, t in self._tasks.items()]

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
            robot = self._describe(env)
            from amp.link.robot import description_sha256

            arm = description_sha256(*without_mount(*robot))
            if arm != ER.ARM_SHA256:
                raise ValueError(f"{task}: the env's robot description (mount left out) is "
                                 f"{arm[:12]}, the embodiment's arm {ER.ARM_SHA256[:12]}")
            sha = description_sha256(*robot)
            if sha != ER.DESCRIPTION_SHA256:
                raise ValueError(f"{task}: the env's robot description is {sha[:12]}, the "
                                 f"embodiment names {ER.DESCRIPTION_SHA256[:12]}")
        except BaseException:
            _close(env)
            raise
        self.env, self.task, self._robot = env, task, robot
        self.controller_cfg = getattr(env, "controller_cfg", None)

    def _check(self, task: str, t, env) -> None:
        rs = env.robosuite_env
        if int(env.max_steps) != t.max_steps or int(rs.horizon) != t.max_steps:
            raise ValueError(f"{task}: the env's max_steps {env.max_steps} (robosuite horizon "
                             f"{rs.horizon}) is not the manifest's {t.max_steps}")
        if int(rs.control_freq) != t.control_freq or \
                abs(1.0 / float(rs.control_freq) - ER.CONTROL_DT) > 1e-12:
            raise ValueError(f"{task}: control_freq {rs.control_freq}, the manifest says "
                             f"{t.control_freq} and the embodiment control_dt {ER.CONTROL_DT}")
        cam = self.spec.cameras[0]
        if list(rs.camera_names) != [cam.name] or \
                (rs.camera_widths[0], rs.camera_heights[0]) != (cam.width, cam.height) or \
                not rs.camera_depths[0]:
            raise ValueError(f"{task}: the env renders {rs.camera_names} at "
                             f"{rs.camera_widths}x{rs.camera_heights}, the embodiment declares "
                             f"{cam.name} {cam.width}x{cam.height} with depth")

    def _trial(self, episode: int) -> int:
        t = self._tasks[self.task]
        if isinstance(episode, bool) or not isinstance(episode, int) or \
                not 0 <= episode < t.trials:
            raise ValueError(f"{self.task}: episode {episode!r} is out of range, the task has "
                             f"{t.trials} trials")
        return episode + 1

    def reset(self, episode: int) -> None:
        if self.env is None or self.task is None:
            raise RuntimeError("reset before load")
        trial = self._trial(episode)
        t = self._tasks[self.task]
        self.unit = self._fingerprint = None

        self.env.reset(options={"trial": trial}, seed=trial)
        self.env.get_observation()
        self._fingerprint = fingerprint_of(self.env)
        self.unit = {"trial": trial, "seed": trial, "max_steps": t.max_steps, "source": t.source,
                     "instruction_source": t.instruction_source}

    def horizon(self) -> int:

        rs = self.env.robosuite_env
        return max(0, int(rs.horizon) - int(rs.timestep))

    def control_dt(self) -> float:
        if self.env is None:
            return ER.CONTROL_DT
        return 1.0 / float(self.env.robosuite_env.control_freq)

    def observe(self) -> Frame:
        return frame_of(self.env, self._camera, lambda env: ee_in_root(env, self._scratch))

    def apply(self, action: Sequence[float]) -> None:
        apply_action(self.env, action)

    def success(self) -> tuple[bool, float]:

        self.env.get_observation()
        self.env.compute_reward()
        ok = bool(self.env.task_completed())
        return ok, 1.0 if ok else 0.0

    def sim_steps(self) -> int:
        """robosuite's step count of the episode, the one its horizon ends (not nut's doubled one)."""
        return int(self.env.robosuite_env.timestep)

    def fingerprint(self) -> str | None:
        """fingerprint_of the scene the latest reset made, before any step."""
        return self._fingerprint

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
               "embodiment": self.spec.name, "description_sha256": ER.DESCRIPTION_SHA256,
               "controller_cfg": self._controller_path()}
        for mod in ("robosuite", "mujoco"):
            out[f"{mod}_version"] = getattr(sys.modules.get(mod), "__version__", None)
        return out

    def _controller_path(self) -> str | None:
        """The resolved controller JSON the envs are built with (one file for all four)."""
        if self.controller_cfg is None and self._tasks:
            try:
                self.controller_cfg = controller_cfg(env_class(next(iter(self._tasks.values()))
                                                               .env_class))
            except Exception as exc:  # noqa: BLE001
                self.log(f"controller path unresolved: {exc!r}")
        return self.controller_cfg

    def close(self) -> None:
        env, self.env, self.task, self.unit = self.env, None, None, None
        self._fingerprint = None
        self._scratch.clear()
        if env is not None:
            _close(env)


def _close(env) -> None:
    try:
        env.robosuite_env.close()
    except Exception as exc:  # noqa: BLE001
        _log(f"closing the env failed: {exc!r}")


def make_adapter(*, task_set: str, cameras: str, seed: int | None) -> RobosuiteAdapter:
    """The AdapterFactory: every task of the task set; seed is ignored, the trial is the unit's."""
    return RobosuiteAdapter(task_set=task_set)
