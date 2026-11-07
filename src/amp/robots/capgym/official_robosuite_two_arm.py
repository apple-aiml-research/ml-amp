#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The official two-arm robosuite adapter: CaP-X's two_arm_lift and two_arm_handover, driven as CaP-X does."""

from __future__ import annotations

import os
import re
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from amp.benchmarks import capgym
from amp.contracts.embodiment import JointSpec, LinkSpec, RobotDescription
from amp.robots.capgym import embodiment_robosuite_two_arm as E2
from amp.robots.capgym import official_robosuite as R
from amp.robots.capgym.adapter import (  # noqa: F401
    CameraImage,
    Frame,
    Mesh,
    env_call,
)
from amp.robots.capgym.capx_derived import step_both
from amp.robots.capgym.official import _git_head, capx_root

EMBODIMENTS = E2.EMBODIMENTS
MOUNT_JOINT = "robot1_base_mount"
BASE_TOL_M = 1e-9


PER_STEP = ("env._set_gripper(1 - a[7]); env._set_gripper_arm1(1 - a[15]); "
            "env.robosuite_env.step(concat(a[0:7], [1 - 2 env._gripper_fraction_0], a[8:15], "
            "[1 - 2 env._gripper_fraction_1]){kw}); env._sim_step_count += 1")
MIRRORS = {
    E2.LIFT_TASK_SET: ("capx/envs/simulators/robosuite_two_arm_lift.py:326-332", {}),

    E2.HANDOVER_TASK_SET: ("capx/envs/simulators/robosuite_handover.py:343-360",
                           {"skip_render_images": True}),
}
_PREFIX = re.compile(r"^(robot|gripper|fixed_mount)0_")


def _log(message: str) -> None:
    print(f"[amp.robots.capgym.official_robosuite_two_arm] {message}", file=sys.stderr, flush=True)


def export_subtree(m, root: str) -> tuple[RobotDescription, dict[str, Mesh]]:
    """official_model.export of the body `root` and its subtree, mount included, read only."""
    import mujoco

    from amp.robots.capgym import official_model as OM

    r = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, root)
    if r < 0:
        raise ValueError(f"the model has no {root} body")
    bodies = [r]
    for b in range(r + 1, m.nbody):
        p = b
        while p not in (0, r):
            p = m.body_parentid[p]
        if p == r:
            bodies.append(b)
    keep = set(bodies)
    links, joints, meshes = [], [], {}
    for b in bodies:
        name = m.body(b).name
        names = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == b and m.geom_group[g] == OM.VISUAL_GROUP:
                key = f"{name}.{m.geom(g).name or g}"
                meshes[key] = OM._visual_mesh(m, g)
                names.append(key)
        links.append(LinkSpec(name=name, meshes=tuple(names)))
        if b == r:
            continue
        parent = int(m.body_parentid[b])
        if parent not in keep:
            raise ValueError(f"body {name}: parent {m.body(parent).name} is not described")
        origin = tuple(OM.pose(m.body_pos[b], m.body_quat[b]).reshape(-1))
        js = [j for j in range(m.njnt) if m.jnt_bodyid[j] == b]
        if not js:
            joints.append(JointSpec(name=f"{name}_fixed", type="fixed",
                                    parent=m.body(parent).name, child=name, origin=origin))
            continue
        j = js[0]
        if len(js) > 1 or m.jnt_type[j] not in OM._TYPES or np.any(m.jnt_pos[j] != 0):
            raise ValueError(f"body {name}: only one hinge or slide joint anchored at the body "
                             "origin maps to a description joint")
        limits = tuple(map(float, m.jnt_range[j])) if m.jnt_limited[j] else (0.0, 0.0)
        joints.append(JointSpec(name=m.joint(j).name, type=OM._TYPES[m.jnt_type[j]],
                                parent=m.body(parent).name, child=name, origin=origin,
                                axis=tuple(map(float, m.jnt_axis[j])), limits=limits))
    return RobotDescription(root=root, links=tuple(links), joints=tuple(joints)), meshes


def combine(arm0: tuple, arm1: tuple, task_set: str) -> tuple[RobotDescription, dict[str, Mesh]]:
    """One tree at robot0_base: robot1's subtree hangs off it at base1_pose by a fixed joint."""
    (d0, m0), (d1, m1) = arm0, arm1
    if d0.root != E2.BASES[0] or d1.root != E2.BASES[1]:
        raise ValueError(f"subtrees rooted at {d0.root}, {d1.root}, not {E2.BASES}")
    mount = JointSpec(name=MOUNT_JOINT, type="fixed", parent=E2.BASES[0], child=E2.BASES[1],
                      origin=E2.base1_pose(task_set))
    return (RobotDescription(root=E2.ROOT, links=d0.links + d1.links,
                             joints=d0.joints + (mount,) + d1.joints), {**m0, **m1})


def _to_robot1(name: str) -> str:
    return ".".join(_PREFIX.sub(r"\g<1>1_", part) for part in name.split("."))


def as_robot1(description: RobotDescription, meshes: dict) -> tuple[RobotDescription, dict]:
    """A robot0 export renamed as robosuite names the same Panda's robot1 bodies and geoms."""
    links = tuple(LinkSpec(_to_robot1(lk.name), tuple(_to_robot1(m) for m in lk.meshes))
                  for lk in description.links)
    joints = tuple(JointSpec(_to_robot1(j.name), j.type, _to_robot1(j.parent),
                             _to_robot1(j.child), j.origin, j.axis, j.limits)
                   for j in description.joints)
    return (RobotDescription(_to_robot1(description.root), links, joints),
            {_to_robot1(k): v for k, v in meshes.items()})


def compose(single: tuple, task_set: str) -> tuple[RobotDescription, dict[str, Mesh]]:
    """The two-arm description from the single-arm Panda's export, which both arms reproduce."""
    return combine(single, as_robot1(*single), task_set)


def without_mounts(description: RobotDescription, meshes: dict) -> tuple[RobotDescription, dict]:
    """The description and meshes with both RethinkMounts' links and joints left out."""
    links = tuple(lk for lk in description.links if not lk.name.startswith(E2.MOUNT_PREFIXES))
    joints = tuple(j for j in description.joints if not j.child.startswith(E2.MOUNT_PREFIXES))
    keep = {m for lk in links for m in lk.meshes}
    return (RobotDescription(root=description.root, links=links, joints=joints),
            {k: v for k, v in meshes.items() if k in keep})


def describe_env(env, task_set: str) -> tuple:
    """(description, meshes) of both arms from the env's MjModel, read only."""
    m = env.robosuite_env.sim.model._model
    return combine(export_subtree(m, E2.BASES[0]), export_subtree(m, E2.BASES[1]), task_set)


def sync_cameras(env) -> None:
    """mj_camlight: the live camera poses follow CaP-X's re-applied override; nothing else moves."""
    import mujoco

    sim = env.robosuite_env.sim
    mujoco.mj_camlight(sim.model._model, sim.data._data)


def camera_in_root(env, camera: str) -> np.ndarray:
    """official_robosuite.camera_in_root with the rotation the render uses, made orthonormal."""
    T = R.camera_in_root(env, camera)


    U, _, Vt = np.linalg.svd(T[:3, :3])
    T[:3, :3] = U @ Vt
    return T


def ee_in_root(env, scratch: dict) -> dict[str, np.ndarray]:
    """Both gripperN_right_eef in robot0_base at the current qpos, on a scratch MjData."""
    import mujoco

    sim = env.robosuite_env.sim
    m = sim.model._model

    if scratch.get("model") is not m:
        scratch.update(model=m, data=mujoco.MjData(m))
    d = scratch["data"]
    d.qpos[:] = sim.data._data.qpos
    mujoco.mj_kinematics(m, d)

    def pose(name):
        b = sim.model.body_name2id(name)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = d.xmat[b].reshape(3, 3), d.xpos[b]
        return T

    inv = np.linalg.inv(pose(E2.ROOT))
    return {h: inv @ pose(link) for h, link in zip(E2.HANDS, E2.EE_LINKS)}


def frame_of(env, camera: str, ee: Callable[[Any], dict]) -> Frame:
    """q of both arms and the grippers from qpos, ee(env) per hand, a fresh render."""
    rs = env.robosuite_env
    qpos = rs.sim.data.qpos
    q = np.concatenate([np.asarray(qpos[r._ref_joint_pos_indexes], dtype=np.float64)
                        for r in rs.robots])
    grippers = {}
    for hand, robot in zip(E2.HANDS, rs.robots):
        finger = robot._ref_gripper_joint_pos_indexes["right"][0]
        reading = float(qpos[finger]) / float(env.gripper_metric_length)
        grippers[hand] = float(np.clip(1.0 - reading, 0.0, 1.0))
    poses = ee(env)
    sync_cameras(env)
    rgb, depth = R.render_camera(env, camera)
    cam = CameraImage(rgb=rgb, depth=depth, K=R.intrinsics(env, camera),
                      T_root_cam=camera_in_root(env, camera))
    return Frame(q=q, grippers=grippers,
                 ee={h: (T[:3, 3].copy(), R._mat_quat(T[:3, :3])) for h, T in poses.items()},
                 cameras={camera: cam})


def apply_action(env, action: Sequence[float], step_kw: dict) -> None:
    """One control step, built as the class's move_to_joints_blocking_both builds each one."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.shape != (16,) or not np.all(np.isfinite(a)):
        raise ValueError(f"action must be 16 finite numbers ([arm0 q(7), gripper0, arm1 q(7), "
                         f"gripper1]), got {a}")
    step_both(env, a, step_kw)


class TwoArmAdapter:
    """A SimAdapter over CaP-X's two-arm robosuite envs of one task set."""

    features = ("grade",)

    def __init__(self, *, task_set: str, tasks: dict | None = None,
                 build: Callable[[Any], Any] = R.build_env,
                 describe: Callable[[Any, str], tuple] = describe_env,
                 log: Callable[[str], None] = _log):
        if task_set not in EMBODIMENTS:
            raise ValueError(f"the two-arm robosuite adapter serves {', '.join(EMBODIMENTS)}, "
                             f"not {task_set!r}")
        self.task_set = task_set
        self.spec = EMBODIMENTS[task_set]()
        every = dict(capgym.robosuite_tasks() if tasks is None else tasks)
        self._tasks = {n: t for n, t in every.items() if t.task_set == task_set}
        self._build, self._describe, self.log = build, describe, log
        self._camera = self.spec.cameras[0].name
        self._mirrors, self._step_kw = MIRRORS[task_set]
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
        """The declared description, exported from the task's scene."""
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
            robot = self._describe(env, self.task_set)
            from amp.link.robot import description_sha256

            arm = description_sha256(*without_mounts(*robot))
            want = E2.ARM_SHA256[self.task_set]
            if arm != want:
                raise ValueError(f"{task}: the env's two arms (mounts left out) are {arm[:12]}, "
                                 f"the embodiment's {want[:12]}")
            sha, want = description_sha256(*robot), E2.DESCRIPTION_SHA256[self.task_set]
            if sha != want:
                raise ValueError(f"{task}: the env's robot description is {sha[:12]}, the "
                                 f"embodiment names {want[:12]}")
        except BaseException:
            R._close(env)
            raise
        self.env, self.task, self._robot = env, task, robot
        self.controller_cfg = getattr(env, "controller_cfg", None)

    def _check(self, task: str, t, env) -> None:
        rs = env.robosuite_env
        if int(env.max_steps) != t.max_steps or int(rs.horizon) != t.max_steps:
            raise ValueError(f"{task}: the env's max_steps {env.max_steps} (robosuite horizon "
                             f"{rs.horizon}) is not the manifest's {t.max_steps}")
        if int(rs.control_freq) != t.control_freq or \
                abs(1.0 / float(rs.control_freq) - self.spec.control_dt) > 1e-12:
            raise ValueError(f"{task}: control_freq {rs.control_freq}, the manifest says "
                             f"{t.control_freq} and the embodiment control_dt "
                             f"{self.spec.control_dt}")
        cam = self.spec.cameras[0]
        if list(rs.camera_names) != [cam.name] or \
                (rs.camera_widths[0], rs.camera_heights[0]) != (cam.width, cam.height) or \
                not rs.camera_depths[0]:
            raise ValueError(f"{task}: the env renders {rs.camera_names} at "
                             f"{rs.camera_widths}x{rs.camera_heights}, the embodiment declares "
                             f"{cam.name} {cam.width}x{cam.height} with depth")
        if len(rs.robots) != 2 or int(rs.action_dim) != self.spec.action_dim:
            raise ValueError(f"{task}: {len(rs.robots)} robots and a {rs.action_dim}-dim action, "
                             f"the embodiment declares 2 arms and {self.spec.action_dim}")
        T = np.linalg.inv(R._body(rs.sim, E2.BASES[0])) @ R._body(rs.sim, E2.BASES[1])
        err = np.abs(T - np.reshape(E2.base1_pose(self.task_set), (4, 4))).max()
        if err > BASE_TOL_M:
            raise ValueError(f"{task}: robot1_base sits at {np.round(T[:3, 3], 6).tolist()} in "
                             f"robot0_base, {err:.2g} off the declared base1_pose")

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
        self._fingerprint = R.fingerprint_of(self.env)
        self.unit = {"trial": trial, "seed": trial, "max_steps": t.max_steps, "source": t.source,
                     "instruction_source": t.instruction_source}

    def horizon(self) -> int:
        rs = self.env.robosuite_env
        return max(0, int(rs.horizon) - int(rs.timestep))

    def control_dt(self) -> float:
        if self.env is None:
            return self.spec.control_dt
        return 1.0 / float(self.env.robosuite_env.control_freq)

    def observe(self) -> Frame:
        return frame_of(self.env, self._camera, lambda env: ee_in_root(env, self._scratch))

    def apply(self, action: Sequence[float]) -> None:
        apply_action(self.env, action, self._step_kw)

    def success(self) -> tuple[bool, float]:


        self.env.get_observation()
        self.env.compute_reward()
        ok = bool(self.env.task_completed())
        return ok, 1.0 if ok else 0.0

    def sim_steps(self) -> int:
        """robosuite's step count of the episode, the one its horizon ends."""
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
        kw = "".join(f", {k}={v}" for k, v in self._step_kw.items())
        out = {"capx_commit": _git_head(root), "robosuite_commit": _git_head(rs_dir),
               "mujoco_gl": os.environ.get("MUJOCO_GL"),
               "egl_device": os.environ.get("MUJOCO_EGL_DEVICE_ID"),
               "embodiment": self.spec.name,
               "description_sha256": E2.DESCRIPTION_SHA256[self.task_set],
               "controller_cfg": self._controller_path(),
               "per_step": PER_STEP.format(kw=kw), "per_step_mirrors": self._mirrors}
        for mod in ("robosuite", "mujoco"):
            out[f"{mod}_version"] = getattr(sys.modules.get(mod), "__version__", None)
        return out

    def _controller_path(self) -> str | None:
        """The resolved controller JSON of the task set's class (ctrl2 for lift, ctrl handover)."""
        if self.controller_cfg is None and self._tasks:
            try:
                self.controller_cfg = R.controller_cfg(R.env_class(
                    next(iter(self._tasks.values())).env_class))
            except Exception as exc:  # noqa: BLE001
                self.log(f"controller path unresolved: {exc!r}")
        return self.controller_cfg

    def close(self) -> None:
        env, self.env, self.task, self.unit = self.env, None, None, None
        self._fingerprint = None
        self._scratch.clear()
        if env is not None:
            R._close(env)


def make_adapter(*, task_set: str, cameras: str, seed: int | None) -> TwoArmAdapter:
    """The AdapterFactory: every task of the task set; seed is ignored, the trial is the unit's."""
    return TwoArmAdapter(task_set=task_set)
