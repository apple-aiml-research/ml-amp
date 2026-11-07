#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The official adapter: CaP-X's own FrankaLiberoEnv, built and driven exactly as CaP-X does."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import os
import subprocess
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from amp.benchmarks import CorpusError, capgym
from amp.contracts.embodiment import RobotDescription
from amp.robots.capgym.adapter import CameraImage, Frame, Mesh, env_call
from amp.robots.capgym.capx_derived import FRANKA_LIBERO_ENV, batch_low_level
from amp.robots.capgym.embodiment import (
    ARM_SHA256,
    CONTROL_DT,
    DESCRIPTION_SHA256,
    HAND,
    MOUNT_PREFIX,
    PEDESTAL,
    capx_libero_panda,
)

EMBODIMENTS = {capgym.TASK_SET: capx_libero_panda}
CAPX_ROOT_ENV = "AMP_CAPX_ROOT"

NOMINAL_WORKERS = 10


ARTICULATED_INIT = ("open", "close", "turnon", "turnoff")


def _log(message: str) -> None:
    print(f"[amp.robots.capgym.official] {message}", file=sys.stderr, flush=True)


def capx_root() -> Path:
    """The cap-x checkout: $AMP_CAPX_ROOT, else the one the importable capx package lives in."""
    if os.environ.get(CAPX_ROOT_ENV):
        return Path(os.environ[CAPX_ROOT_ENV])
    spec = importlib.util.find_spec("capx")
    if spec is None or not spec.origin:
        raise RuntimeError(f"no capx package in this Python ({sys.executable}); set {CAPX_ROOT_ENV}")
    return Path(spec.origin).resolve().parents[1]


def load_yaml_config(path: Path) -> dict:
    """A config read the way launch.py reads it (capx.envs.configs.loader.DictLoader)."""
    from capx.envs.configs.loader import DictLoader

    return DictLoader.load([str(path)])


def batch_privileged(base_config: Path) -> bool:
    """run_libero_batch.py:146: the base config's env.cfg.privileged, read with yaml.safe_load."""
    import yaml

    with open(base_config) as f:
        base = yaml.safe_load(f)
    return bool(base.get("env", {}).get("cfg", {}).get("privileged", False))


def low_level_config(entry: capgym.Entry, root: Path | None = None, *,
                     load_yaml: Callable[[Path], dict] = load_yaml_config,
                     privileged_of: Callable[[Path], bool] = batch_privileged) -> dict:
    """The low_level dict CaP-X instantiates for the entry's protocol, never a hand-copied one."""
    root = root or capx_root()
    s = entry.setting
    if s.base_config is not None:
        return batch_low_level(entry.task.suite, entry.task.task_id,
                               privileged_of(root / s.base_config), s.max_steps)
    low = load_yaml(root / s.source)["env"]["cfg"]["low_level"]
    if not isinstance(low, dict) or low.get("_target_") != FRANKA_LIBERO_ENV:
        raise ValueError(f"{s.source}: low_level is not a {FRANKA_LIBERO_ENV} block: {low!r}")
    return copy.deepcopy(low)


def build_env(low_level: dict):
    """capx.envs.configs.instantiate.instantiate: what tasks/base.py builds the env with."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    from capx.envs.configs.instantiate import instantiate

    return instantiate(copy.deepcopy(low_level))


def reset_history(env) -> bool:
    """True when LIBERO-PRO adds a sampler per hard reset for this env."""
    domain = getattr(getattr(getattr(env, "handle", None), "env", None), "env", None)
    problem = getattr(domain, "parsed_problem", None)
    states = getattr(domain, "object_states_dict", None)
    if not isinstance(problem, dict) or not isinstance(states, dict):

        raise RuntimeError(  # noqa: TRY004
            "the env has no handle.env.env.parsed_problem and object_states_dict; "
            "LIBERO-PRO's layout changed, so the reset-count rule cannot be derived")
    return any(isinstance(s, (list, tuple)) and len(s) > 1 and s[0] in ARTICULATED_INIT
               and s[1] in states and hasattr(states[s[1]], "set_joint")
               for s in problem.get("initial_state", ()))


def warmup_seeds(trial: int, attempt: int = 1) -> list[int]:
    """The seeds a nominal CaP-X worker's env was reset with before this attempt of trial, in order."""
    if trial < 1 or attempt < 1:
        raise ValueError(f"trial {trial} and attempt {attempt} must be >= 1")
    j = (trial - 1) // NOMINAL_WORKERS
    return [trial - NOMINAL_WORKERS * i for i in range(j, 0, -1)] + [trial] * (attempt - 1)


def env_fingerprint(env) -> str | None:
    """sha256 of the sim's qpos, body_xpos and body_xquat as float64; None without a MuJoCo sim."""
    data = getattr(getattr(getattr(getattr(env, "handle", None), "env", None), "sim", None),
                   "data", None)
    if data is None:
        return None
    h = hashlib.sha256()
    for name in ("qpos", "body_xpos", "body_xquat"):
        a = np.ascontiguousarray(getattr(data, name), dtype=np.float64)
        h.update(f"{name}{a.shape};".encode())
        h.update(a.tobytes())
    return h.hexdigest()


def without_mount(description: RobotDescription, meshes: dict) -> tuple[RobotDescription, dict]:
    """The description and meshes with the mount's links and joints left out."""
    links = tuple(lk for lk in description.links if not lk.name.startswith(MOUNT_PREFIX))
    joints = tuple(j for j in description.joints if not j.child.startswith(MOUNT_PREFIX))
    keep = {m for lk in links for m in lk.meshes}
    return (RobotDescription(root=description.root, links=links, joints=joints),
            {k: v for k, v in meshes.items() if k in keep})


def has_pedestal(description: RobotDescription) -> bool:
    """True for MountedPanda, whose pedestal link carries meshes."""
    return any(lk.name == PEDESTAL and lk.meshes for lk in description.links)


def describe_env(env) -> tuple:
    """(description, meshes) of the env's robot from its MjModel, read only."""
    from amp.robots.capgym import official_model

    return official_model.export(env.handle.env.sim.model._model)


def _quat_mat(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    n = w * w + x * x + y * y + z * z
    s = 2.0 / n if n else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def _mat_quat(R) -> np.ndarray:
    R = np.asarray(R, dtype=np.float64)
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        q = [0.0, 0.0, 0.0, 0.0]
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i], q[1 + j], q[1 + k] = 0.25 * s, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q) * (1.0 if q[0] >= 0 else -1.0)


def _pose(wxyz_xyz) -> np.ndarray:
    v = np.asarray(wxyz_xyz, dtype=np.float64).reshape(7)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = _quat_mat(v[:4]), v[4:]
    return T


def frame_of(env, cameras: Sequence[str]) -> Frame:
    """env.get_observation() as a Frame, as is: rgb, metric depth, K, pose_mat (OpenCV axes in
    robot0_base), q, the reading on AMP's wire, and gripper0_eef in robot0_base."""
    obs = env.get_observation()
    joints = np.asarray(obs["robot_joint_pos"], dtype=np.float64).reshape(-1)
    reading = float(joints[7])

    T = np.linalg.inv(_pose(env.base_link_wxyz_xyz)) @ _pose(env.gripper_link_wxyz_xyz)
    cams = {}
    for name in cameras:
        c = obs[name]
        depth = np.asarray(c["images"]["depth"], dtype=np.float32)
        cams[name] = CameraImage(rgb=np.ascontiguousarray(c["images"]["rgb"], dtype=np.uint8),
                                 depth=np.ascontiguousarray(depth.reshape(depth.shape[:2])),
                                 K=np.asarray(c["intrinsics"], dtype=np.float64),
                                 T_root_cam=np.asarray(c["pose_mat"], dtype=np.float64))
    return Frame(q=joints[:7].copy(), grippers={HAND: float(np.clip(1.0 - reading, 0.0, 1.0))},
                 ee={HAND: (T[:3, 3].copy(), _mat_quat(T[:3, :3]))}, cameras=cams)


def apply_action(env, action: Sequence[float]) -> None:
    """One control step, the one move_to_joints_blocking sends: [(target - q) * freq, 1 - 2 f]."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape != (8,) or not np.all(np.isfinite(a)):
        raise ValueError(f"action must be 8 finite numbers (7 joint targets, gripper), got {a}")
    env_call(env._set_gripper, 1.0 - a[7])
    env_call(env.move_to_joints_blocking, a[:7], max_steps=1)


class OfficialAdapter:
    """A SimAdapter over CaP-X's FrankaLiberoEnv; tasks and trials from the capgym manifest."""

    features = ("grade",)

    def __init__(self, *, entries: dict[str, capgym.Entry] | None = None,
                 low_level: Callable[[capgym.Entry], dict] = low_level_config,
                 build: Callable[[dict], Any] = build_env,
                 describe: Callable[[Any], tuple] = describe_env,
                 log: Callable[[str], None] = _log):
        self.spec = capx_libero_panda()
        self._entries = dict(capgym.load().entries() if entries is None else entries)
        self._low_level, self._build, self._describe, self.log = low_level, build, describe, log
        self._cameras = tuple(c.name for c in self.spec.cameras)
        self._robot: tuple[Any, dict[str, Mesh]] | None = None
        self.env: Any = None
        self.task: str | None = None
        self.config: dict | None = None
        self.unit: dict | None = None
        self._fingerprint: str | None = None

        self.attempt = 1
        self._history = False
        self._resets = 0

    def embodiment(self):
        return self.spec

    def robot_description(self) -> tuple:
        """The declared description, read from the first MountedPanda scene the manifest has."""
        floor: set[str] = set()
        for name, e in self._entries.items():
            if self._robot is not None:
                break
            if e.task.suite in floor:
                continue
            self.load(name)
            if self._robot is None:
                floor.add(e.task.suite)
        if self._robot is None:
            raise ValueError("no scene of the manifest has the MountedPanda pedestal")
        return self._robot

    def success_checks(self) -> str:
        return "end"

    def tasks(self) -> list[dict]:
        return [{"name": name, "instruction": e.task.instruction,
                 "attributes": [e.task.suite, e.setting.source],
                 "episode_length_s": e.setting.episode_length_s}
                for name, e in self._entries.items()]

    def load(self, task: str) -> None:
        if self.env is not None and self.task == task:
            return
        if task not in self._entries:
            raise ValueError(f"no capgym task {task!r} in the manifest")
        self.close()
        entry = self._entries[task]
        config = self._low_level(entry)
        env = self._build(config)
        try:

            prompt = getattr(getattr(env, "handle", None), "task_language", None)
            if prompt != entry.task.instruction:
                raise ValueError(f"{task}: the env's instruction {prompt!r} is not the manifest's "
                                 f"{entry.task.instruction!r}; {capgym.REGENERATE}")
            if abs(1.0 / float(env._control_freq) - CONTROL_DT) > 1e-12:
                raise ValueError(f"{task}: control_freq {env._control_freq}, the embodiment "
                                 f"declares control_dt {CONTROL_DT}")
            history = reset_history(env)
            robot = self._describe(env)
            from amp.link.robot import description_sha256

            arm = description_sha256(*without_mount(*robot))
            if arm != ARM_SHA256:
                raise ValueError(f"{task}: the env's robot description (mount left out) is "
                                 f"{arm[:12]}, the embodiment's arm {ARM_SHA256[:12]}")
            mounted = has_pedestal(robot[0])
            sha = description_sha256(*robot) if mounted else None
            if mounted and sha != DESCRIPTION_SHA256:
                raise ValueError(f"{task}: the env's robot description is {sha[:12]}, the "
                                 f"embodiment names {DESCRIPTION_SHA256[:12]}")
        except BaseException:
            _close(env)
            raise
        self.env, self.task, self.config = env, task, config
        self._history, self._resets = history, 0
        if mounted:
            self._robot = robot

    def reset(self, episode: int) -> None:
        if self.env is None or self.task is None:
            raise RuntimeError("reset before load")
        try:
            unit = capgym.unit_spec(self.task, episode)
        except CorpusError as exc:
            raise ValueError(str(exc)) from exc
        if int(self.env.max_steps) != unit["max_steps"]:
            raise ValueError(f"{self.task}: the env's max_steps {self.env.max_steps} is not the "
                             f"manifest's {unit['max_steps']}")
        self.unit = self._fingerprint = None
        if self._history:

            seeds = warmup_seeds(unit["trial"], self.attempt)
            if self._resets:
                task = self.task
                self.close()
                self.load(task)
            for seed in seeds:
                self._reset_env(seed)
            unit = {**unit, "reset_index": len(seeds),
                    "reset_schedule": {"workers": NOMINAL_WORKERS,
                                       "j": (unit["trial"] - 1) // NOMINAL_WORKERS,
                                       "attempt": self.attempt, "warmup_seeds": seeds}}
        self._reset_env(unit["seed"])
        self.unit, self._fingerprint = unit, env_fingerprint(self.env)

    def _reset_env(self, seed: int) -> None:
        self._resets += 1
        self.env.reset(seed=seed)
        self.env.get_observation()

    def fingerprint(self) -> str | None:
        """env_fingerprint of the scene the latest reset made, before any step."""
        return self._fingerprint

    def horizon(self) -> int:

        return max(0, int(self.env.max_steps) - int(self.env._sim_step_count))

    def control_dt(self) -> float:
        return CONTROL_DT if self.env is None else 1.0 / float(self.env._control_freq)

    def observe(self) -> Frame:
        return frame_of(self.env, self._cameras)

    def apply(self, action: Sequence[float]) -> None:
        apply_action(self.env, action)

    def success(self) -> tuple[bool, float]:

        self.env.get_observation()
        self.env.compute_reward()
        ok = bool(self.env.task_completed())
        return ok, 1.0 if ok else 0.0

    def sim_steps(self) -> int:
        """CaP-X's own step count of the episode (its truncation counter)."""
        return int(self.env._sim_step_count)

    def provenance(self) -> dict:
        root = None
        try:
            root = capx_root()
        except RuntimeError:
            pass
        out = {"capx_commit": _git_head(root), "manifest_capx_commit": None,
               "libero_pro_commit": _git_head(root / "capx/third_party/LIBERO-PRO") if root else None,
               "mujoco_gl": os.environ.get("MUJOCO_GL"),
               "egl_device": os.environ.get("MUJOCO_EGL_DEVICE_ID"),
               "embodiment": self.spec.name, "description_sha256": DESCRIPTION_SHA256}
        try:
            m = capgym.load()
            out.update(manifest_capx_commit=m.capx_commit,
                       libero_pro_dataset=m.libero_pro_dataset)
        except CorpusError:
            pass
        for mod in ("robosuite", "mujoco"):
            out[f"{mod}_version"] = getattr(sys.modules.get(mod), "__version__", None)
        return out

    def close(self) -> None:
        env, self.env, self.task, self.unit = self.env, None, None, None
        self._fingerprint, self._history, self._resets = None, False, 0
        if env is not None:
            _close(env)


def _close(env) -> None:
    try:
        env.handle.env.close()
    except Exception as exc:  # noqa: BLE001
        _log(f"closing the env failed: {exc!r}")


def _git_head(path: Path | None) -> str | None:
    if path is None:
        return None

    cmd = ["git", "-c", f"safe.directory={path}", "-C", str(path), "rev-parse", "HEAD"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=10, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None


def make_adapter(*, task_set: str, cameras: str, seed: int | None) -> OfficialAdapter:
    """The AdapterFactory: one adapter over every task of the manifest; seed is CaP-X's trial."""
    if task_set != capgym.TASK_SET:
        raise ValueError(f"the official adapter serves {capgym.TASK_SET}, not {task_set!r}")
    return OfficialAdapter()

