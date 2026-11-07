#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Isaac-side link client: RoboLab observations to /sim/step frames, the actions back."""


from __future__ import annotations

import dataclasses
import functools
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from amp.contracts.harness import CAMERA_SETS, INFO_MODES
from amp.contracts.robot import ACTION_DIM
from amp.geometry.cameras import (
    DROID_LEFT_CAMERA_CFG,
    DROID_RIGHT_CAMERA_CFG,
    WRIST_CAMERA,
    WRIST_CAMERA_CFG,
)
from amp.geometry.kinematics import make_pose
from amp.link.client import EpisodeAborted, LinkError
from amp.link.obs import pack_observation
from amp.link.pointcloud import camera_from_offset

EGO_L_OBS_KEY = "over_shoulder_left_camera"
EGO_R_OBS_KEY = "over_shoulder_right_camera"
WRIST_OBS_KEY = "wrist_cam"
CAMERA_KEYS = {
    "droid": {"ego_l": EGO_L_OBS_KEY, "ego_r": EGO_R_OBS_KEY, WRIST_CAMERA: WRIST_OBS_KEY},
}
_CFG_SOURCES = {
    EGO_L_OBS_KEY: ("robolab.variations.camera", "OverShoulderLeftCameraCfg"),
    EGO_R_OBS_KEY: ("robolab.variations.camera", "OverShoulderRightCameraCfg"),
    WRIST_OBS_KEY: ("robolab.robots.droid", "WristCameraCfg"),
}
CFG_FALLBACKS = {
    EGO_L_OBS_KEY: DROID_LEFT_CAMERA_CFG,
    EGO_R_OBS_KEY: DROID_RIGHT_CAMERA_CFG,
    WRIST_OBS_KEY: WRIST_CAMERA_CFG,
}
DEFAULT_CONTROL_DT = 1.0 / 15.0
END_TIMEOUT_S = 5.0
PROPRIO_KEYS = ("q", "gripper", "ee_pos", "ee_quat")
PROPRIO_GROUPS = (("q",), ("gripper",), ("ee_pos", "ee_quat"))
BLOWUP_EVENT = "robot.robolab.sim_blowup"


def _usable(key: str, value) -> bool:
    v = np.asarray(value, dtype=np.float64)
    if not np.isfinite(v).all():
        return False
    if key != "ee_quat":
        return True
    n = float(np.linalg.norm(v))
    return bool(np.isfinite(n) and n >= 1e-12)


def _fallback(key: str, value):
    """A fixed value for a key with no finite value yet: the identity pose, zero q and gripper."""
    if key == "ee_quat":
        return np.array([1.0, 0.0, 0.0, 0.0])
    if key == "gripper":
        return 0.0
    return np.zeros(np.shape(value) or 3)


def _log(message: str) -> None:
    print(f"[amp.robots.robolab] {message}", flush=True)


@dataclass(frozen=True)
class ClientConfig:
    """What the client sends. standard: cameras, proprio and the task text only."""

    info: str = "standard"
    cameras: str = "droid"
    depth_stride: int = 4
    jpeg_quality: int = 85
    timeout_s: float = 30.0
    takeover: bool = False

    def __post_init__(self):
        if self.info not in INFO_MODES:
            raise ValueError(f"info {self.info!r}: expected one of {', '.join(INFO_MODES)}")
        if self.cameras not in CAMERA_SETS:
            raise ValueError(f"cameras {self.cameras!r}: expected one of {', '.join(CAMERA_SETS)}")
        if isinstance(self.depth_stride, bool) or not isinstance(self.depth_stride, int) \
                or self.depth_stride < 1:
            raise ValueError(f"depth_stride {self.depth_stride!r}: expected an int >= 1")

    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, text: str | None) -> ClientConfig:
        """A config from run.py's --amp-config; unknown keys are an error."""
        data = json.loads(text) if text else {}
        if not isinstance(data, dict):
            raise TypeError(f"--amp-config must be a JSON object, got {data!r}")
        return cls(**data)


def to_numpy(value: Any, env_id: int | None = None) -> np.ndarray:
    """A torch tensor or array-like as numpy, optionally selecting one env's row first."""
    if env_id is not None:
        value = value[env_id]
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _vec(value, n: int) -> list[float]:
    return [float(x) for x in to_numpy(value).reshape(-1)[:n]]


def _cfg_dict(cam) -> dict:
    return {
        "pos": tuple(cam.offset.pos),
        "rot_wxyz": tuple(cam.offset.rot),
        "convention": cam.offset.convention,
        "focal_length": cam.spawn.focal_length,
        "horizontal_aperture": cam.spawn.horizontal_aperture,
        "vertical_aperture": cam.spawn.vertical_aperture,
        "width": cam.width,
        "height": cam.height,
    }


class LiveRoboLab:
    """The running RoboLab as the client reads it; tests pass a fake with the same methods."""

    def world(self):
        """The env-bound WorldState, or None; never calls get_world(), which replaces it."""
        try:
            from robolab.core.world import world_state
        except Exception:  # noqa: BLE001
            return None
        world = getattr(world_state, "_global_world", None)
        if world is None or getattr(world, "env", None) is None:
            return None
        return world

    def env(self):
        world = self.world()
        if world is None:
            return None
        return getattr(world.env, "unwrapped", world.env)

    def task_dir(self) -> str:
        """RoboLab's per-task output dir, else ""; never get_output_dir(), which assigns one."""
        try:
            import robolab.constants

            return str(getattr(robolab.constants, "_output_dir", None) or "")
        except Exception:  # noqa: BLE001
            return ""

    def task_name(self) -> str:
        """The task being evaluated: the basename of RoboLab's per-task output dir, else ""."""
        out = self.task_dir()
        return Path(out).name if out else ""

    def instruction(self) -> str:
        """The resolved instruction of the env being run (create_env resolves the wording)."""
        env = self.env()
        value = getattr(getattr(env, "cfg", None), "instruction", None)
        return value if isinstance(value, str) else ""

    def camera_cfgs(self, camera_set: str) -> tuple[dict, str]:
        """({link name: cfg}, source): RoboLab's cfg classes when importable, else the mounts."""
        keys = CAMERA_KEYS[camera_set]
        try:
            import importlib

            cfgs = {}
            for name, key in keys.items():
                module, cls = _CFG_SOURCES[key]
                cfgs[name] = _cfg_dict(getattr(getattr(importlib.import_module(module), cls)(),
                                               key))
            return cfgs, "robolab cfg"
        except Exception as exc:  # noqa: BLE001
            cfgs = {name: dict(CFG_FALLBACKS[key]) for name, key in keys.items()}
            return cfgs, f"built-in constants ({type(exc).__name__}: {exc})"

class LinkPolicy:
    """Hands each observation to the robot server and returns the action it sends back."""

    open_loop_horizon = 1

    def __init__(
        self,
        session: Any,
        config: ClientConfig | None = None,
        *,
        robolab: Any = None,
        log=_log,
    ) -> None:
        super().__init__()
        self.open_loop_horizon = 1
        self.config = config or ClientConfig()
        self.robolab = robolab or LiveRoboLab()
        self.log = log
        self.camera_keys = dict(CAMERA_KEYS[self.config.cameras])
        self.session = session
        self.link = session.link
        self.url = self.link.base_url
        cfgs, source = self.robolab.camera_cfgs(self.config.cameras)
        self._wrist_cfg = cfgs.pop(WRIST_CAMERA)
        self._fixed_models = {name: camera_from_offset(name, **cfg) for name, cfg in cfgs.items()}
        self._fixed_checked = False
        self._episode = 0
        self._step = 0
        self._begin_pending = True
        self._episode_open = False
        self._warned: set[str] = set()
        self._aborted = False
        self._last_finite: dict = {}
        self._blowup: dict | None = None
        self._published: str | None = None
        self.job_id: int | None = None
        self._job_task = ""
        self._job_dir = ""
        c = self.config
        log(f"link {self.url}; cameras from {source}")
        log(f"cameras {c.cameras}: " + ", ".join(f"{n}={k}" for n, k in self.camera_keys.items()))
        log(f"info mode {c.info}: gt state never sent, depth stride {c.depth_stride}")

    def _call(self, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except EpisodeAborted:
            self._aborted = True
            raise
        except LinkError as exc:
            raise RuntimeError(f"robot server {self.url} failed: {exc}") from exc

    def _new_episode(self, episode: int) -> None:
        self._report_blowup()
        self._last_finite = {}
        self._published = None
        self._episode = int(episode)
        self._step = 0
        self._begin_pending = True
        self._aborted = False

    def begin_episode(self, episode_idx: int) -> None:
        """Save the previous episode, then claim and take this one's job; raises on any fault."""
        base = getattr(super(), "begin_episode", None)
        if base is not None:
            base(episode_idx)
        self._new_episode(episode_idx)
        task = self.robolab.task_name()
        if not task:
            raise RuntimeError("RoboLab's per-task output dir is unset: no task to claim")
        instruction = self.robolab.instruction() or self.session.instructions.get(task, "")
        job = self.session.begin(task, int(episode_idx), instruction)
        self.job_id = job.get("job_id")
        self._job_task, self._job_dir = task, self.robolab.task_dir()

    def _once(self, key: str, message: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.log(message)

    def _env_value(self, name: str, read, fallback):
        why = "RoboLab world not bound"
        try:
            env = self.robolab.env()
            if env is not None:
                return read(env)
        except Exception as exc:  # noqa: BLE001
            why = f"{type(exc).__name__}: {exc}"
        self._once(name, f"{name} falls back to {fallback} ({why})")
        return fallback

    def _control_dt(self) -> float:
        return self._env_value("control_dt", lambda env: float(env.step_dt), DEFAULT_CONTROL_DT)

    def _max_steps(self) -> int | None:
        return self._env_value("max_steps", lambda env: int(env.max_episode_length), None)

    def _check_fixed_cameras_once(self, env_id: int) -> None:
        """Log how far each cfg-derived fixed camera is from its live sensor, once per process."""
        if self._fixed_checked:
            return
        env = self.robolab.env()
        if env is None:
            return
        self._fixed_checked = True
        for name, model in self._fixed_models.items():
            try:
                data = env.scene.sensors[self.camera_keys[name]].data
                pos = to_numpy(data.pos_w, env_id) - to_numpy(env.scene.env_origins, env_id)
                T_live = make_pose(pos, to_numpy(data.quat_w_ros, env_id))
                T_cfg = model.T_world_cam
                d_mm = 1000.0 * float(np.linalg.norm(T_live[:3, 3] - T_cfg[:3, 3]))
                cos = (np.trace(T_live[:3, :3].T @ T_cfg[:3, :3]) - 1.0) / 2.0
                d_deg = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
                self.log(f"{name} camera (fixed; wrist is not checked): cfg model vs live sensor "
                         f"differ by {d_mm:.1f} mm, {d_deg:.2f} deg")
            except Exception as exc:  # noqa: BLE001
                self.log(f"{name} camera check skipped ({type(exc).__name__}: {exc})")

    def _extract_observation(self, raw_obs: Any, *, env_id: int = 0) -> dict:
        images = raw_obs["image_obs"]
        missing = [key for key in self.camera_keys.values() if key not in images]
        if missing:
            raise KeyError(
                f"image_obs has no {', '.join(missing)} (it has {', '.join(sorted(images))}): "
                f"the sim registered other cameras than the {self.config.cameras} set reads"
            )
        cams = {}
        for name, key in self.camera_keys.items():
            rgb = to_numpy(images[key], env_id)
            if rgb.ndim == 3 and rgb.shape[2] == 4:
                rgb = rgb[:, :, :3]
            depth = images.get(f"{key}_depth")
            if depth is not None:
                depth = to_numpy(depth, env_id).astype(np.float32)
                if depth.ndim == 3:
                    depth = depth[:, :, 0]
            cams[name] = (np.ascontiguousarray(rgb, dtype=np.uint8), depth)
        proprio = raw_obs["proprio_obs"]
        return {
            "env_id": env_id,
            "cameras": cams,
            "q": to_numpy(proprio["arm_joint_pos"], env_id).astype(np.float64).reshape(-1),
            "gripper": float(to_numpy(proprio["gripper_pos"], env_id).reshape(-1)[0]),
            "ee_pos": to_numpy(proprio["ee_pos"], env_id).astype(np.float64).reshape(-1),
            "ee_quat": to_numpy(proprio["ee_quat"], env_id).astype(np.float64).reshape(-1),
        }

    def _pack_request(self, extracted_obs: dict, instruction: str) -> Any:
        control_dt, max_steps = self._control_dt(), self._max_steps()
        task = self.robolab.task_name()
        if self._begin_pending:
            self._call(self.link.begin_episode, self._episode, task=task, instruction=instruction,
                       max_steps=max_steps, control_dt=control_dt, job_id=self.job_id)
            self._begin_pending = False
            self._episode_open = True
        self._check_fixed_cameras_once(extracted_obs["env_id"])
        extracted_obs = self._finite_proprio(extracted_obs)
        ee_pos, ee_quat = extracted_obs["ee_pos"], extracted_obs["ee_quat"]
        wrist = camera_from_offset(WRIST_CAMERA, **self._wrist_cfg,
                                   T_world_parent=make_pose(ee_pos, ee_quat))
        models = {**self._fixed_models, WRIST_CAMERA: wrist}
        cameras = {name: (models[name], *extracted_obs["cameras"][name]) for name in models}
        return pack_observation(
            episode=self._episode, step=self._step, control_dt=control_dt, max_steps=max_steps,
            instruction=instruction, task=task, q=extracted_obs["q"],
            gripper=extracted_obs["gripper"], ee_pos=ee_pos, ee_quat=ee_quat, cameras=cameras,
            depth_stride=self.config.depth_stride, jpeg_quality=self.config.jpeg_quality,
        )

    def _finite_proprio(self, extracted_obs: dict) -> dict:
        """extracted_obs, with any non-finite proprio replaced by its last finite value."""
        bad = [key for key in PROPRIO_KEYS if not _usable(key, extracted_obs[key])]
        if not bad:
            self._last_finite.update({key: extracted_obs[key] for key in PROPRIO_KEYS})
            return extracted_obs
        held = {g for key in bad for g in PROPRIO_GROUPS if key in g}
        for group in PROPRIO_GROUPS:
            if group not in held:
                self._last_finite.update({key: extracted_obs[key] for key in group})
        subst = [key for group in held for key in group]
        fallback = [key for key in subst if key not in self._last_finite]
        if self._blowup is None:
            self._blowup = {"episode": self._episode, "step": self._step, "count": 0,
                            "fields": [], "fallback": []}
        b = self._blowup
        b["count"] += 1
        b["fields"] += [key for key in PROPRIO_KEYS if key in bad and key not in b["fields"]]
        b["fallback"] += [key for key in PROPRIO_KEYS
                          if key in fallback and key not in b["fallback"]]
        return {**extracted_obs, **{key: self._last_finite[key] if key in self._last_finite
                                    else _fallback(key, extracted_obs[key]) for key in subst}}

    def _report_blowup(self) -> None:
        """The episode's one blow-up record, when it had a non-finite frame; never raises."""
        b, self._blowup = self._blowup, None
        if b is None:
            return
        what = "EE pose" if {"ee_pos", "ee_quat"} & set(b["fields"]) else "proprio"
        held = f"{', '.join(b['fields'])} held at the last finite value"
        if b["fallback"]:
            held += f", {', '.join(b['fallback'])} at the identity/zero fallback until one"
        self.log(f"sim blow-up: non-finite {what} at step {b['step']} of episode {b['episode']} "
                 f"({b['count']} frame(s); {held}); RoboLab ran the episode on to grade it")
        try:
            self.link.tracer.event(BLOWUP_EVENT, dict(b), lvl="warn")
        except Exception as exc:  # noqa: BLE001
            self.log(f"{BLOWUP_EVENT} not traced ({type(exc).__name__}: {exc})")

    def _query_server(self, request: Any) -> Any:
        header, blobs, encodings = request
        action = self._call(self.link.step, header, blobs, encodings)
        self._step += 1
        if self.link.finish is not None and self._published is None:
            self._published = str(self.link.finish.get("reason") or "published")
            self.log(f"published at step {self._step}: the robot holds until RoboLab ends the "
                     "episode")
        return action


    def _unpack_response(self, response: Any) -> np.ndarray:
        return np.asarray(response, dtype=np.float32).reshape(1, ACTION_DIM)

    def _result(self, env_id: int = 0) -> dict:
        """RoboLab's own verdict, plus the last observed progress under "last_observed"."""
        result: dict = {"success": None, "step": None}
        try:
            env = self.robolab.env()
            verdict = None if env is None else env.get_env_results()[env_id]
            if verdict is not None:
                s, step = verdict.get("success"), verdict.get("step")
                result["success"] = None if s is None else bool(s)
                result["step"] = None if step is None else int(step)
        except Exception as exc:  # noqa: BLE001
            self.log(f"harness result unavailable ({type(exc).__name__}: {exc})")
        return result

    def _end(self, reason: str | None, env_id: int = 0) -> None:
        blowup = self._blowup
        self._report_blowup()
        if not self._episode_open:
            return
        self._episode_open = False
        if self._aborted:
            reason, result = "aborted", None
        else:
            result = self._result(env_id)
            if blowup is not None:
                result = {**result, "sim_blowup": dict(blowup)}
        if reason is None and self._published is not None:
            reason = "published"
        if reason is None:
            reason = "terminated" if result["success"] is not None else "reset"
        try:
            self.link.end_episode(self._episode, reason=reason, result=result,
                                  timeout_s=END_TIMEOUT_S, max_retries=0)
        except Exception as exc:  # noqa: BLE001
            self.log(f"/sim/end for episode {self._episode} failed, continuing: {exc}")

    def reset(self, *, env_id: int | None = None) -> None:
        aborted = self._aborted
        self._end(None, 0 if env_id is None else int(env_id))
        if self.job_id is not None and not aborted:
            self.session.ended(self.job_id, self._job_task, self._episode, self._job_dir)
        self.job_id = None
        self._new_episode(self._episode + 1)
        super().reset(env_id=env_id)

    def close(self) -> None:
        self._end("closed")
        super().close()


@functools.cache
def client_class(base: type | None = None) -> type:
    """LinkPolicy on RoboLab's InferenceClient (imported here, inside Isaac), or on `base`."""
    if base is None:
        from robolab.eval.base_client import InferenceClient as base
    return type("RoboLabLinkClient", (LinkPolicy, base), {})


def make_client(session, config: ClientConfig, **kw):
    """A RoboLab InferenceClient on the process's shared link (a bridge.Session)."""
    return client_class()(session, config, **kw)
