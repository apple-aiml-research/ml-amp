#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The official BEHAVIOR adapter: CaP-X's R1ProBehaviourLowLevel, built and driven as CaP-X does."""

from __future__ import annotations

import hashlib
import importlib
import json
import os
import random
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from amp.benchmarks import capgym
from amp.fs import faults
from amp.robots.capgym import body_contacts as BC
from amp.robots.capgym import embodiment_behavior as EB
from amp.robots.capgym import object_poses
from amp.robots.capgym.adapter import CameraImage, EnvError, Frame, env_call
from amp.robots.capgym.capx_derived import world_frame_action
from amp.robots.capgym.official import _git_head, capx_root

EMBODIMENTS = EB.EMBODIMENTS
DATA_ENV = "OMNIGIBSON_DATA_PATH"
DATA_PARTS = ("omnigibson-robot-assets", "behavior-1k-assets", "2025-challenge-task-instances",
              "omnigibson.key")
LIMIT_TOL = 1e-3
OPENCV_FROM_USD = np.diag([1.0, -1.0, -1.0, 1.0])
PER_STEP = ("env.step(robot.q_to_action(holonomic_base_command_in_world_frame(robot, q28))), "
            "q28 = robot.get_joint_positions() with the action's targets in")
PER_STEP_MIRRORS = "capx/envs/simulators/r1pro_b1k.py:742-779 (_move_to_joint_positions)"



SCORE_STEPS = ("get_observation(); compute_reward() (BDDL: one zero-action env.step inside); "
               "task_completed() (BDDL: one more)")


def _log(message: str) -> None:
    print(f"[amp.robots.capgym.official_behavior] {message}", file=sys.stderr, flush=True)


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def _like(values: np.ndarray, ref):
    """values as ref's array type: a torch tensor of its dtype and device, else numpy."""
    if hasattr(ref, "detach"):
        import torch

        return torch.as_tensor(values, dtype=ref.dtype, device=ref.device)
    return np.asarray(values, dtype=np.asarray(ref).dtype)


def _quat_mat(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in _np(xyzw).reshape(4))
    n = w * w + x * x + y * y + z * z
    s = 2.0 / n if n else 0.0
    return np.array([[1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
                     [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
                     [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def pose(pos_quat) -> np.ndarray:
    """OmniGibson's (position, quaternion xyzw) as a 4x4."""
    pos, quat = pos_quat
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = _quat_mat(quat), _np(pos).reshape(3)
    return T


def _mat_quat(R) -> np.ndarray:
    from amp.robots.capgym.official import _mat_quat as wxyz

    return wxyz(R)


def env_class(dotted: str) -> type:
    module, _, name = dotted.rpartition(".")
    return getattr(importlib.import_module(module), name)


def skip_curobo(cls: type) -> None:
    """Make the env class's primitives skip cuRobo's motion generator."""
    mod = sys.modules[cls.__module__]
    base = getattr(mod, "StarterSemanticActionPrimitives", None)
    if base is None:
        raise RuntimeError(f"{cls.__module__} has no StarterSemanticActionPrimitives to build "
                           "without cuRobo: CaP-X's env class changed")
    if getattr(base, "amp_skips_curobo", False):
        return

    class WithoutCuRobo(base):
        amp_skips_curobo = True

        def __init__(self, *args, **kwargs):
            super().__init__(*args, skip_curobo_initilization=True, **kwargs)


            random.seed(2)
            np.random.seed(2)

    mod.StarterSemanticActionPrimitives = WithoutCuRobo


def build_env(task: capgym.BehaviorTask) -> Any:
    """The class with exactly the YAML's kwargs, as CaP-X's config loader or get_env builds it;
    its primitives are built without cuRobo."""
    data = os.environ.get(DATA_ENV)

    if not data or not all(os.path.exists(os.path.join(data, p)) for p in DATA_PARTS):
        raise ValueError(f"{DATA_ENV}={data!r} is no staged BEHAVIOR dataset (needs "
                         f"{', '.join(DATA_PARTS)})")
    cls = env_class(task.env_class)
    skip_curobo(cls)
    return cls(**task.low_level_kwargs)


def robot_urdf() -> Path:
    """The robot's own URDF in the staged robot assets (robot_base.py:630-636), no env needed."""
    data = os.environ.get(DATA_ENV)
    if not data:
        raise ValueError(f"{DATA_ENV} is not set: no staged BEHAVIOR dataset")
    return Path(data) / "omnigibson-robot-assets" / "models" / "r1pro" / "urdf" / EB.URDF


def describe_env() -> tuple:
    """(description, visual meshes) of the robot's r1pro.urdf at the pinned sha; read only."""
    path = robot_urdf()
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    if sha != EB.URDF_SHA256:
        raise ValueError(f"{path} is {sha[:12]}, the embodiment's r1pro.urdf {EB.URDF_SHA256[:12]}")
    return EB.describe_urdf(path)


class BaseTarget:
    """The base pose AMP has commanded, held across steps and chased."""

    def __init__(self, step_max: Sequence[float]):
        self.step_max = np.asarray(step_max, dtype=np.float64)
        self.target: np.ndarray | None = None
        self._last: np.ndarray | None = None
        self._stalled = 0

    def reset(self) -> None:
        """Forget the target: the next command anchors at the measured pose, so zero holds there."""
        self.target = self._last = None
        self._stalled = 0

    def command(self, cur: Sequence[float], delta: Sequence[float]) -> np.ndarray:
        """The (x, y, yaw) to command this step, from the measured (x, y, yaw) and a delta in the
        target's own frame."""
        cur = np.asarray(cur, dtype=np.float64)
        d = np.asarray(delta, dtype=np.float64)
        if self.target is None:
            self.target = cur.copy()
        t = self.target
        c, s = np.cos(t[2]), np.sin(t[2])
        t += np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])
        cc, ss = np.cos(cur[2]), np.sin(cur[2])
        err = t - cur
        if np.hypot(err[0], err[1]) > BASE_LEAD_M:
            err[:2] *= BASE_LEAD_M / np.hypot(err[0], err[1])
            t[:2] = cur[:2] + err[:2]
        if self._stalled_at(cur, err):
            t[:] = cur
            err = np.zeros(3)
        e = np.array([cc * err[0] + ss * err[1], -ss * err[0] + cc * err[1], err[2]])
        peak = float(np.max(np.abs(e) / self.step_max))
        step = e if peak <= 1.0 else e / peak
        return cur + np.array([cc * step[0] - ss * step[1], ss * step[0] + cc * step[1], step[2]])

    def _stalled_at(self, cur: np.ndarray, err: np.ndarray) -> bool:
        """Whether the base has gone BASE_STALL_STEPS steps without moving, far from its target."""
        last, self._last = self._last, cur.copy()
        leading = np.hypot(err[0], err[1]) > BASE_STALL_LEAD_M or abs(err[2]) > BASE_STALL_LEAD_RAD
        moved = last is None or np.hypot(*(cur[:2] - last[:2])) > BASE_STALL_STEP_M \
            or abs(cur[2] - last[2]) > BASE_STALL_STEP_RAD
        self._stalled = self._stalled + 1 if leading and not moved else 0
        if self._stalled >= BASE_STALL_STEPS:
            self._stalled = 0
            return True
        return False


def joint_target(robot, action: Sequence[float], base: BaseTarget) -> np.ndarray:
    """The 28-q target: the current q with the action's base, torso, arm and finger targets in."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape != (23,) or not np.all(np.isfinite(a)):
        raise ValueError(f"action must be 23 finite numbers ([base dx dy dyaw, torso(4), left(7), "
                         f"left gripper, right(7), right gripper]), got {a}")
    q = _np(robot.get_joint_positions()).copy()
    idx = {n: i for i, n in enumerate(EB.OG_JOINTS)}

    def put(names, values):
        q[[idx[n] for n in names]] = values


    put(EB.BASE_JOINTS, base.command(q[[idx[n] for n in EB.BASE_JOINTS]], a[0:3]))
    put(EB.TORSO, a[3:7])
    put(EB.ARM_JOINTS["left"], a[7:14])
    put(EB.ARM_JOINTS["right"], a[15:22])
    for arm, g in (("left", a[14]), ("right", a[22])):

        g = float(np.clip(g, 0.0, 1.0))
        put(EB.FINGER_JOINTS[arm], [EB.FINGER_OPEN + g * (EB.FINGER_CLOSED - EB.FINGER_OPEN)] * 2)
    return q


def og_action(robot, target: np.ndarray):
    """robot.q_to_action of the target, its base made world-frame first (r1pro_b1k.py:766)."""
    q = _like(target, robot.get_joint_positions())
    return world_frame_action(robot, q)


def frame_of(env, obs: dict, spec) -> Frame:
    """q, grippers and hands in base_link, and each declared camera's rgb and depth_linear only."""
    robot = env.robot
    q28 = _np(robot.get_joint_positions())
    idx = {n: i for i, n in enumerate(EB.OG_JOINTS)}
    T_wr = pose(robot.get_position_orientation())
    inv = np.linalg.inv(T_wr)
    grippers, ee = {}, {}
    for arm, hand in zip(EB.ARMS, EB.HANDS):
        fingers = q28[[idx[n] for n in EB.FINGER_JOINTS[arm]]]
        grippers[hand] = float(np.clip(1.0 - float(np.mean(fingers)) / EB.FINGER_OPEN, 0.0, 1.0))
        T = inv @ pose(robot.links[EB.EE_LINKS[arm]].get_position_orientation())
        ee[hand] = (T[:3, 3].copy(), _mat_quat(T[:3, :3]))
    cams = {}
    mine = obs[robot.name]
    for c in spec.cameras:
        key = EB.og_sensor(robot.name, c.name)
        data = mine[key]

        rgb = data["rgb"]
        rgb = rgb.detach().cpu().numpy() if hasattr(rgb, "detach") else np.asarray(rgb)
        depth = _np(data["depth_linear"]).astype(np.float32)
        if rgb.shape[:2] != (c.height, c.width) or depth.shape[:2] != (c.height, c.width):
            raise ValueError(f"camera {c.name}: rendered {rgb.shape[:2]}, declared "
                             f"{(c.height, c.width)}")
        sensor = robot.sensors[key]
        T_wc = pose(sensor.get_position_orientation()) @ OPENCV_FROM_USD
        cams[c.name] = CameraImage(rgb=np.ascontiguousarray(rgb[..., :3], dtype=np.uint8),
                                   depth=np.ascontiguousarray(depth.reshape(depth.shape[:2])),
                                   K=_np(sensor.intrinsic_matrix), T_root_cam=inv @ T_wc)
    q = q28[[idx[n] for n in EB.Q_JOINTS]].copy()
    return Frame(q=q, grippers=grippers, ee=ee, cameras=cams, T_world_base=T_wr)


ARM_CMD_EPS = 2e-3
GRIPPER_CMD_EPS = 0.02
BASE_LEAD_M = 3.0


BASE_STALL_LEAD_M, BASE_STALL_LEAD_RAD = 0.02, 0.05
BASE_STALL_STEP_M, BASE_STALL_STEP_RAD = 0.0005, 0.0005
BASE_STALL_STEPS = 10

BASE_QUIET_STEPS, BASE_QUIET_M, BASE_QUIET_RAD = 2, 0.0001, 0.00017
INSTANCES_DIR = "2025-challenge-task-instances"
CONTACTS_EVENT = "node.b1k.contacts_unavailable"


READ_FAILURE_EPISODES = 2


def instance_file(task: capgym.BehaviorTask, trial: int) -> Path:
    """The challenge instance `trial` of the task, as the staged dataset holds it (the saved
    task-relevant object state and robot poses that load_task_instance puts into the scene)."""
    data = os.environ.get(DATA_ENV)
    if not data:
        raise ValueError(f"{DATA_ENV} is not set: no staged BEHAVIOR dataset")
    scene, act = task.scene_model, task.activity_name
    return (Path(data) / INSTANCES_DIR / "scenes" / scene / "json" / f"{scene}_task_{act}_instances"
            / f"{scene}_task_{act}_0_{trial}_template-tro_state.json")


def read_instance(task: capgym.BehaviorTask, trial: int) -> dict:
    with open(instance_file(task, trial)) as f:
        return json.load(f)


def fingerprint_of(task: capgym.BehaviorTask, trial: int, instance: dict) -> str:
    """sha256 of what instance `trial` LOADS: its identity and the saved poses of the
    task-relevant objects and the robot. Settled physics (robot q varies 5e-5..4.4e-4 rad and
    objects 2e-6..3e-5 m between resets of one trial) is deliberately not in it, so
    resets of one trial agree exactly and different trials differ."""
    h = hashlib.sha256()
    ident = f"{task.scene_model}/{task.activity_name}/{trial}"
    h.update(ident.encode() + b"\0" + json.dumps(instance, sort_keys=True,
                                                  separators=(",", ":")).encode())
    return h.hexdigest()


class FreshProcessRequired(ValueError):
    """This process cannot run the job: a second config (og.clear() cannot make room), a
    poisoned stage, or a trial that started with an object held."""


class HeldAtTrialStart(FreshProcessRequired):
    """An object was held right after env.reset(): this process is not a fresh stage."""


class BehaviorAdapter:
    """A SimAdapter over CaP-X's BEHAVIOR-1K base configs (task set capgym_b1k)."""



    features = ("grade",)

    def __init__(self, *, tasks: dict | None = None,
                 build: Callable[[Any], Any] = build_env,
                 describe: Callable[[], tuple] = describe_env,
                 to_action: Callable[[Any, np.ndarray], Any] = og_action,
                 instance: Callable[[Any, int], dict] = read_instance,
                 log: Callable[[str], None] = _log,
                 fault: Callable[[dict], Any] = faults.report):
        self.spec = EB.capx_b1k_r1pro()
        self._tasks = dict(capgym.behavior_tasks() if tasks is None else tasks)
        self._build, self._describe, self._to_action = build, describe, to_action
        self._instance, self.log, self._fault = instance, log, fault


        self.disabled: str | None = None
        self._read_failures = 0
        self._read_failed = False


        self._config: str | None = None
        self._poisoned = False
        self._held_at_start = False
        self._env_raised = False
        self._process_trials = 0
        self._arm_anchor: dict | None = None
        self._overwritten: int | None = None
        self._hold: dict | None = None
        self._named: tuple | None = None
        self._base = BaseTarget(np.asarray(EB.BASE_QD) * EB.CONTROL_DT)

        self._shots = object_poses.Snapshots(lambda: object_poses.snapshot(self.env))
        self._contacts: BC.BodyContacts | None = None
        self._motion: str | None = None
        self._quiet = 0
        self._last_base: np.ndarray | None = None
        self._prev_arm: np.ndarray | None = None
        self._arm_touching: set[str] = set()
        self._arm_contacts: list[dict] = []
        self._base_contacts: list[dict] = []
        self._base_seen: set[str] = set()
        self._idle_contacts: set[str] = set()
        self._start_pairs: set[str] = set()
        self._robot: tuple | None = None
        self.env: Any = None
        self.task: str | None = None
        self.unit: dict | None = None
        self.verdict: dict | None = None
        self._obs: dict | None = None
        self._steps = 0
        self._fingerprint: str | None = None

    def embodiment(self):
        return self.spec

    def robot_description(self) -> tuple:
        """The declared description, from the staged robot assets; builds no env."""
        if self._robot is None:
            robot = self._describe()
            from amp.link.robot import description_sha256

            sha = description_sha256(*robot)
            if sha != EB.DESCRIPTION_SHA256:
                raise ValueError(f"the robot description is {sha[:12]}, the embodiment names "
                                 f"{EB.DESCRIPTION_SHA256[:12]}")
            self._robot = robot
        return self._robot

    def success_checks(self) -> str:
        return "end"

    def tasks(self) -> list[dict]:
        return [{"name": n, "instruction": t.instruction,
                 "attributes": [t.task_set, t.source, t.activity_name],
                 "episode_length_s": t.episode_length_s} for n, t in self._tasks.items()]

    @property
    def loaded_config(self) -> str | None:
        """The config this process has built and can still run; None when none or unusable."""
        return None if self._tainted() else self._config

    def needs_fresh_process(self, task: str) -> bool:
        """True when loading `task` here would need a stage this process can no longer make."""
        return self._tainted() or (self._config is not None and task != self._config)

    def _tainted(self) -> bool:
        return self._poisoned or self._held_at_start or self._env_raised

    def _taint(self) -> str:
        return ("failed to build a config and its stage is unusable" if self._poisoned
                else "found an object held at a trial's start, so its stage is not fresh"
                if self._held_at_start
                else "had an env step raise in a trial, so its stage is not fresh"
                if self._env_raised
                else f"already ran {self._config}, and og.clear() cannot make room")

    def load(self, task: str) -> None:
        if self.env is not None and self.task == task and not self.needs_fresh_process(task):
            return
        if task not in self._tasks:
            raise ValueError(f"no capgym task {task!r} in {EB.TASK_SET}")
        t = self._tasks[task]
        if self.needs_fresh_process(task):
            raise FreshProcessRequired(f"{task} needs a fresh process: this one " + self._taint())
        if self.env is not None and self._config == task:
            self.task = task
            return
        self._poisoned = True
        env = self._build(t)
        self._check(task, t, env)
        self._poisoned = False
        self.env, self.task, self._config = env, task, task

    def _check(self, task: str, t, env) -> None:
        robot = env.robot
        names = tuple(robot.joints)
        if names != EB.OG_JOINTS:
            raise ValueError(f"{task}: the robot's joints are {names}, the embodiment's "
                             f"{EB.OG_JOINTS}")
        if int(robot.action_dim) != EB.OG_ACTION_DIM:
            raise ValueError(f"{task}: action_dim {robot.action_dim}, CaP-X's position "
                             f"controllers give {EB.OG_ACTION_DIM}")
        if env.task_name != t.activity_name:
            raise ValueError(f"{task}: the env runs {env.task_name!r}, the manifest "
                             f"{t.activity_name!r}")
        freq = float(env.controller_cfg["env"]["action_frequency"])
        if abs(1.0 / freq - EB.CONTROL_DT) > 1e-12:
            raise ValueError(f"{task}: action_frequency {freq}, the embodiment's control_dt "
                             f"{EB.CONTROL_DT}")
        lo, hi = _np(robot.joint_lower_limits), _np(robot.joint_upper_limits)
        idx = {n: i for i, n in enumerate(EB.OG_JOINTS)}
        want = [(g.joints, g.q_min, g.q_max) for g in self.spec.joint_groups] + \
            [(a.joints, a.q_min, a.q_max) for a in self.spec.arms] + \
            [(EB.FINGER_JOINTS[a], (EB.FINGER_CLOSED,) * 2, (EB.FINGER_OPEN,) * 2)
             for a in EB.ARMS]
        for joints, qmin, qmax in want:
            got = [(lo[idx[j]], hi[idx[j]]) for j in joints]
            if np.abs(np.array(got) - np.array(list(zip(qmin, qmax)))).max() > LIMIT_TOL:
                raise ValueError(f"{task}: limits of {joints} are {np.round(got, 4).tolist()}, "
                                 f"the embodiment declares {list(zip(qmin, qmax))}")
        missing = [EB.og_sensor(robot.name, c.name) for c in self.spec.cameras
                   if EB.og_sensor(robot.name, c.name) not in robot.sensors]
        if missing:
            raise ValueError(f"{task}: the robot has no {missing}")

    def _trial(self, episode: int) -> int:
        t = self._tasks[self.task]
        if isinstance(episode, bool) or not isinstance(episode, int) or \
                not 0 <= episode < t.trials:
            raise ValueError(f"{self.task}: episode {episode!r} is out of range, the task has "
                             f"{t.trials} trials")
        return episode + capgym.FIRST_TRIAL

    def reset(self, episode: int) -> None:
        if self.env is None or self.task is None:
            raise RuntimeError("reset before load")
        if self.needs_fresh_process(self.task):
            raise FreshProcessRequired(f"{self.task} needs a fresh process: this one "
                                       + self._taint())
        trial = self._trial(episode)
        t = self._tasks[self.task]
        self.unit = self._fingerprint = self.verdict = self._obs = None
        self._steps = 0
        self._process_trial_index, self._process_trials = self._process_trials, \
            self._process_trials + 1
        self._read_failures = self._read_failures + 1 if self._read_failed else 0
        self._read_failed = False
        self._base.reset()
        self._shots.reset()

        self.env.reset(options={"trial": trial}, seed=trial)
        self._obs = self.env.get_observation()
        state = self._instance(t, trial)
        self._fingerprint = fingerprint_of(t, trial, state)
        self.unit = {"trial": trial, "seed": trial, "source": t.source,
                     "instruction_source": t.instruction_source,
                     "process_trial_index": self._process_trial_index}


        held = self._held_arms()
        self.unit["reset_hold_check"] = {"held_arms": held}
        if held:
            self._held_at_start = True
            raise HeldAtTrialStart(f"{self.task} trial {trial}: {' and '.join(held)} held an "
                                   "object at the start; this process restarts")
        self._shots.add("initial")
        self._motion, self._quiet, self._last_base = None, 0, None
        self._prev_arm, self._arm_touching, self._arm_contacts = None, set(), []
        self._base_contacts, self._base_seen, self._idle_contacts = [], set(), set()
        self._arm_anchor = self._initial_arm_anchor()
        self._hold = self._named = None
        try:
            self._contacts = BC.BodyContacts(self.env.robot,
                                             lambda: object_poses.scene_of(self.env).objects)
            self._contacts.rest()
        except Exception as exc:  # noqa: BLE001
            self._contacts_lost("construct", exc)

    def _contacts_lost(self, phase: str, exc: BaseException) -> None:
        """Contact sensing is gone: base moves are refused and the episode says so."""
        self._contacts = None
        if isinstance(self.unit, dict):
            self.unit["body_contacts"] = {"state": "unavailable", "phase": phase,
                                          "reason": f"{type(exc).__name__}: {exc}"[:300]}


            self.unit.setdefault("contact_lost", {
                "at_step": self._steps, "reason": f"{phase}: {type(exc).__name__}: {exc}"[:200]})
        if phase == "read":
            self._read_failed = True
        systematic = phase == "construct" or self._read_failures + 1 >= READ_FAILURE_EPISODES
        if systematic and self.disabled is None:
            self.disabled = f"contact sensing unavailable: {type(exc).__name__}: {exc}"[:500]
        self.log(f"ERROR contact sensing unavailable ({phase}), base moves are refused for the "
                 f"rest of the episode{'; BEHAVIOR is disabled on this host' if systematic else ''}"
                 f": {exc!r}")
        try:
            self._fault({"event": CONTACTS_EVENT, "phase": phase, "error": repr(exc),
                         "task": self.task, "trial": (self.unit or {}).get("trial"),
                         "process_trial_index": self._process_trial_index,
                         "disable": self.disabled if systematic else None})
        except Exception as e:  # noqa: BLE001
            self.log(f"ERROR the contact fault was not reported to the slot runner: {e!r}")

    def horizon(self) -> None:

        return None

    def idle(self) -> None:
        """The server has nothing to dispense: no base move is in flight, so none is chased."""
        self._base.reset()
        self._motion, self._quiet = None, 0
        self._arm_touching = set()
        self._shots.after_base_move()

    def _note_base_contacts(self, names) -> None:
        """Record each (link, object) pair a base move meets, once per move, for run_<k>_state.json."""
        if self._motion != "base":
            self._idle_contacts = set(names)
            return
        k = self._shots.moves + 1
        for e in sorted(set(names) - self._base_seen):
            link, _, obj = e.partition(" touched ")
            self._base_contacts.append({"move": k, "step": self._steps, "link": link,
                                        "object": obj, "start": e in self._start_pairs})
        self._base_seen |= set(names)

    def _track_motion(self, a: np.ndarray) -> None:
        """Which move is in flight (a base delta, or a changed arm or torso target) and whether it is over."""
        base_q = _np(self.env.robot.get_joint_positions())[
            [EB.OG_JOINTS.index(j) for j in EB.BASE_JOINTS]]
        base_moved = self._last_base is not None and (
            np.hypot(*(base_q[:2] - self._last_base[:2])) > BASE_QUIET_M
            or abs(base_q[2] - self._last_base[2]) > BASE_QUIET_RAD)
        self._last_base = base_q.copy()
        arm_changed = self._prev_arm is not None and bool(np.any(np.abs(a[3:] - self._prev_arm) > 1e-9))
        self._prev_arm = a[3:].copy()
        if np.any(a[:3]):
            self._shots.base_moved = True
            if self._motion != "base":
                self._start_pairs, self._base_seen = set(self._idle_contacts), set()
            self._motion, self._quiet = "base", 0
        elif arm_changed:
            self._motion, self._quiet = "arm", 0
        elif self._motion == "base" and base_moved:
            self._quiet = 0
        elif self._motion is not None:
            self._quiet += 1
            if self._quiet >= BASE_QUIET_STEPS:
                self._motion, self._quiet = None, 0

    def _record_arm_contacts(self) -> None:
        """While an arm or torso moves: note each new contact in the trace. Nothing acts on it."""
        if self._contacts is None:
            return
        try:
            now = set(self._contacts.read())
        except Exception as exc:  # noqa: BLE001
            self._contacts_lost("read", exc)
            return
        self._arm_contacts += [{"step": self._steps, "contact": c}
                               for c in sorted(now - self._arm_touching)]
        self._arm_touching = now

    def state_record(self) -> dict:
        """The episode's object-pose snapshots and arm-move contacts (run_<k>_state.json); takes
        the final snapshot now."""
        rec = self._shots.record()
        rec["process_trial_index"] = self._process_trial_index
        rec["arm_contacts"] = list(self._arm_contacts)
        rec["base_contacts"] = list(self._base_contacts)
        return rec

    def control_dt(self) -> float:
        return EB.CONTROL_DT

    def observe(self) -> Frame:
        if self._obs is None:
            self._obs = self.env.get_observation()
        frame = frame_of(self.env, self._obs, self.spec)
        if self._motion != "arm" and self._contacts is not None:
            try:
                frame.body_contacts = self._contacts.read() or None
                self._note_base_contacts(frame.body_contacts or ())
            except Exception as exc:  # noqa: BLE001
                self._contacts_lost("read", exc)
        frame.contacts_unavailable = self._contacts is None
        return frame

    def hold_action(self, frame: Frame | None = None) -> np.ndarray:
        """The 23-d action that holds every arm, the torso and both grippers where `frame` (default:
        a new observation) has them and the base still: what a base-only move sends beside its delta."""
        f = frame or self.observe()
        q = np.asarray(f.q, dtype=np.float64)
        n = len(EB.ARMS) * len(EB.ARM_JOINTS[EB.ARMS[0]])
        arms = {a: q[i * 7:(i + 1) * 7] for i, a in enumerate(EB.ARMS)}
        left, right = EB.ARMS
        self._hold = {arm: (arms[arm].copy(), float(f.grippers[f"{arm}_hand"])) for arm in EB.ARMS}
        return np.r_[0.0, 0.0, 0.0, q[n:n + len(EB.TORSO)], arms[left], f.grippers[f"{left}_hand"],
                     arms[right], f.grippers[f"{right}_hand"]]

    def _held_arms(self) -> list[str]:
        """The arms whose gripper holds an object, read through controller.arm and restored after."""
        ctrl = self.env.controller
        names = list(getattr(self.env.robot, "arm_names", EB.ARMS))
        before = ctrl.arm
        try:
            held = []
            for idx, arm in enumerate(names):
                ctrl.overwrite_arm(idx)
                if ctrl._get_obj_in_hand() is not None:
                    held.append(arm)
            return held
        finally:
            ctrl.overwrite_arm(names.index(before))
            self._overwritten = names.index(before)

    def _initial_arm_anchor(self) -> dict:
        """What each arm's command is measured against at reset: where it stands, gripper open."""
        q = _np(self.env.robot.get_joint_positions())
        return {arm: (q[[EB.OG_JOINTS.index(j) for j in EB.ARM_JOINTS[arm]]].copy(), 0.0)
                for arm in EB.ARMS}

    def follow_select(self, arm: str | None) -> None:
        """The server's call kind for the next action: the arm a selecting call named, or None."""
        self._named = (arm,)

    def _select(self, arm: str) -> None:
        names = list(getattr(self.env.robot, "arm_names", EB.ARMS))
        idx = names.index(arm)
        if idx != self._overwritten:
            self.env.controller.overwrite_arm(idx)
            self._overwritten = idx

    def _follow_arm(self, a: np.ndarray) -> None:
        """Point controller.arm, which the lift scoring reads, at the arm official's call would."""
        segments = {"left": (a[7:14], a[14]), "right": (a[15:22], a[22])}
        named, self._named = self._named, None
        if named is not None:


            for arm in EB.ARMS:
                self._arm_anchor[arm] = (segments[arm][0].copy(), float(segments[arm][1]))
            if named[0] is not None:
                self._select(named[0])
            return


        for arm in reversed(EB.ARMS):
            joints, grip = segments[arm]
            if self._hold is not None and np.allclose(joints, self._hold[arm][0], rtol=0, atol=1e-6) \
                    and abs(grip - self._hold[arm][1]) <= 1e-6:
                continue
            anchor_q, anchor_g = self._arm_anchor[arm]
            moved = np.max(np.abs(joints - anchor_q)) > ARM_CMD_EPS
            gripped = abs(grip - anchor_g) > GRIPPER_CMD_EPS
            if not (moved or gripped):
                continue
            self._arm_anchor[arm] = (joints.copy(), float(grip))
            names = list(getattr(self.env.robot, "arm_names", EB.ARMS))
            if names.index(arm) == self._overwritten:
                continue
            if not gripped and self.env.controller._get_obj_in_hand() is not None:
                continue
            self._select(arm)

    def apply(self, action: Sequence[float]) -> None:
        act = np.asarray(action, dtype=np.float64).reshape(-1)
        if self._contacts is None and act.shape == (23,):

            act = act.copy()
            act[:3] = 0.0
            self._base.reset()
        target = joint_target(self.env.robot, act, self._base)
        self._follow_arm(act)
        a = self._to_action(self.env.robot, target)
        if int(np.asarray(_np(a)).size) != EB.OG_ACTION_DIM:
            raise ValueError(f"q_to_action gave {np.asarray(_np(a)).size} values, not "
                             f"{EB.OG_ACTION_DIM}")

        self._obs = None

        self._steps += 1
        try:

            env_call(self.env.step, a)
        except EnvError:
            self._env_raised = True
            raise
        self._track_motion(act)
        if self._motion == "arm":
            self._record_arm_contacts()

        self._obs = self.env.cur_observation

    def success(self) -> tuple[bool, float]:
        self.env.get_observation()
        reward = float(_np(self.env.compute_reward()))
        ok = bool(self.env.task_completed())
        self.verdict = {"reward": reward, "task_completed": ok,
                        "scoring": self._tasks[self.task].scoring}
        return ok, 1.0 if ok else float(np.clip(reward, 0.0, 1.0))

    def grade_refusal(self) -> str | None:
        """Why the loaded task's block cannot be graded now, or None: only the paper's lift tasks."""
        t = self._tasks.get(self.task) if self.task else None
        if t is None or self.env is None:
            return "no BEHAVIOR task is loaded"
        if self.task not in capgym.BEHAVIOR_PAPER_TRIALS:
            return (f"{self.task} is not one of the CaP-X paper's BEHAVIOR tasks, whose grading "
                    "steps no physics")
        return None

    def sim_steps(self) -> int:
        """AMP's control steps of the episode (the scoring's zero-action steps are not counted)."""
        return self._steps

    def fingerprint(self) -> str | None:
        """fingerprint_of the instance the latest reset loaded."""
        return self._fingerprint

    def provenance(self) -> dict:
        root = None
        try:
            root = capx_root()
        except RuntimeError:
            pass
        og_mod = sys.modules.get("omnigibson")
        og_dir = Path(og_mod.__file__).resolve().parents[2] if og_mod and og_mod.__file__ else None
        out = {"capx_commit": _git_head(root), "b1k_commit": _git_head(og_dir),
               "omnigibson_version": getattr(og_mod, "__version__", None),
               "data_path": os.environ.get(DATA_ENV),
               "gpu_id": os.environ.get("OMNIGIBSON_GPU_ID"),
               "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
               "embodiment": self.spec.name, "embodiment_format": self.spec.format,
               "description_sha256": EB.DESCRIPTION_SHA256, "curobo": "skipped",
               "per_step": PER_STEP,
               "per_step_mirrors": PER_STEP_MIRRORS, "score_steps": SCORE_STEPS,
               "horizon": None,

               "execution": {"torchdynamo_disabled": os.environ.get("TORCHDYNAMO_DISABLE") == "1",
                             "torch_compile": "disabled"
                             if os.environ.get("TORCHDYNAMO_DISABLE") == "1" else "enabled"}}
        try:
            out["manifest_b1k_commit"] = capgym.load().b1k_commit
        except capgym.CorpusError:
            pass
        return out

    def close(self) -> None:
        """Release the task; the built env stays for this config's next episodes. The process
        ending (or being replaced by a fresh one) tears Kit down, as og.clear() cannot."""
        self.task, self.unit = None, None
        self._fingerprint = self._obs = None


def make_adapter(*, task_set: str, cameras: str, seed: int | None) -> BehaviorAdapter:
    """The AdapterFactory: every capgym_b1k task; seed is ignored, the trial is the unit's."""
    if task_set != EB.TASK_SET:
        raise ValueError(f"the BEHAVIOR adapter serves {EB.TASK_SET}, not {task_set!r}")
    return BehaviorAdapter()
