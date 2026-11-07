#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RoboDojo's observation and AMP's action, converted; pure numpy, no RoboDojo or Isaac imports."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from amp.robots.capgym.adapter import CameraImage, Frame
from amp.robots.robodojo import embodiment as E

ARM_DOF = 6
ACTION_DIM = 2 * (ARM_DOF + 1)
GL_TO_CV = np.diag([1.0, -1.0, -1.0, 1.0])
T_WORLD_ROOT = np.asarray(E.T_WORLD_ROOT, dtype=np.float64).reshape(4, 4)
T_ROOT_WORLD = np.eye(4)
T_ROOT_WORLD[:3, :3] = T_WORLD_ROOT[:3, :3].T
T_ROOT_WORLD[:3, 3] = -T_WORLD_ROOT[:3, :3].T @ T_WORLD_ROOT[:3, 3]
Q_ROOT_WORLD = np.array([math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)])
ARM_KEY = {s: f"{s}_arm_joint_state" for s in E.SIDES}
EE_POSE_KEY = {s: f"{s}_ee_pose" for s in E.SIDES}
GRIPPER_KEY = {s: f"{s}_ee_joint_state" for s in E.SIDES}
CAMERA_KEYS = ("color", "intrinsic_matrix", "extrinsic_matrix")


def _quat_mul(a, b) -> np.ndarray:
    """The Hamilton product a * b of two wxyz quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw])


def root_pose(pose) -> tuple[np.ndarray, np.ndarray]:
    """A world pose [x, y, z, qw, qx, qy, qz] from RoboDojo as (pos, quat wxyz) in the root."""
    p = np.asarray(pose, dtype=np.float64).reshape(-1)
    if p.size != 7:
        raise ValueError(f"pose: [x, y, z, qw, qx, qy, qz], got {p.size} values")
    quat = p[3:] / np.linalg.norm(p[3:])
    return T_ROOT_WORLD[:3, :3] @ p[:3] + T_ROOT_WORLD[:3, 3], _quat_mul(Q_ROOT_WORLD, quat)


def _state(obs) -> dict:
    state = obs.get("state") if isinstance(obs, dict) else None
    if not isinstance(state, dict):
        raise ValueError('obs: no "state" dict (RoboDojo\'s per-env obs from '  # noqa: TRY004
                         "ObsManager.get_obs)")
    return state


def _vector(state: dict, key: str, n: int) -> np.ndarray:
    if state.get(key) is None:
        raise ValueError(f'obs["state"]: no {key!r}')
    v = np.asarray(state[key], dtype=np.float64).reshape(-1)
    if v.size != n or not np.isfinite(v).all():
        raise ValueError(f'obs["state"][{key!r}]: {n} finite floats, got {v.size}')
    return v


def gripper_reading(state: dict, side: str) -> float:
    """AMP's reading (0 open .. 1 closed): 1 - the obs's <side>_ee_joint_state, RoboDojo's own."""
    return float(np.clip(1.0 - _vector(state, GRIPPER_KEY[side], 1)[0], 0.0, 1.0))


def _camera(name: str, cam) -> CameraImage:
    if not isinstance(cam, dict) or any(cam.get(k) is None for k in CAMERA_KEYS):
        raise ValueError(f"camera {name!r}: the obs needs {', '.join(CAMERA_KEYS)}")
    rgb = np.ascontiguousarray(cam["color"])
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError(f"camera {name!r}: color must be (H, W, 3) uint8, got "
                         f"{rgb.shape} {rgb.dtype}")
    depth = cam.get("depth")
    if depth is not None:
        depth = np.ascontiguousarray(np.asarray(depth, dtype=np.float32))
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[:, :, 0]
        if depth.shape != rgb.shape[:2]:
            raise ValueError(f"camera {name!r}: depth {depth.shape} is not color's "
                             f"{rgb.shape[:2]}")
    K = np.asarray(cam["intrinsic_matrix"], dtype=np.float64)
    extrinsic = np.asarray(cam["extrinsic_matrix"], dtype=np.float64)
    if K.shape != (3, 3) or not np.isfinite(K).all():
        raise ValueError(f"camera {name!r}: intrinsic_matrix must be a finite 3x3, got {K.shape}")
    if extrinsic.shape != (4, 4) or not np.isfinite(extrinsic).all():
        raise ValueError(f"camera {name!r}: extrinsic_matrix must be a finite 4x4, got "
                         f"{extrinsic.shape}")
    return CameraImage(rgb=rgb, depth=depth, K=K, T_root_cam=T_ROOT_WORLD @ extrinsic @ GL_TO_CV)


def frame_from_obs(obs: dict) -> Frame:
    """A capgym Frame from one env's RoboDojo obs, and nothing else."""
    state = _state(obs)
    vision = obs.get("vision")
    if not isinstance(vision, dict) or not vision:
        raise ValueError('obs: no non-empty "vision" dict')
    q = np.concatenate([_vector(state, ARM_KEY[s], ARM_DOF) for s in E.SIDES])
    grippers, ee = {}, {}
    for side, hand in zip(E.SIDES, E.HANDS):
        grippers[hand] = gripper_reading(state, side)
        pose = _vector(state, EE_POSE_KEY[side], 7)
        if np.linalg.norm(pose[3:]) < 1e-6:
            raise ValueError(f'obs["state"][{EE_POSE_KEY[side]!r}]: the wxyz quaternion is zero')
        ee[hand] = root_pose(pose)
    return Frame(q=q, grippers=grippers, ee=ee,
                 cameras={name: _camera(name, cam) for name, cam in vision.items()})


def _robodojo_gripper(reading: float) -> float:
    """RoboDojo's gripper value (1 = open) from AMP's reading (0 open .. 1 closed)."""
    return float(1.0 - np.clip(float(reading), 0.0, 1.0))


def _command(joints: dict[str, np.ndarray], values: dict[str, float]) -> dict[str, list[float]]:
    out: dict[str, list[float]] = {}
    for side in E.SIDES:
        out[ARM_KEY[side]] = [float(x) for x in joints[side]]
        out[GRIPPER_KEY[side]] = [values[side]]
    return out


def action_to_robodojo(action: Sequence[float]) -> dict[str, list[float]]:
    """AMP's 14-dim action as RoboDojo's command dict; RoboDojo scales the gripper value itself."""
    a = np.asarray(action, dtype=np.float64).reshape(-1)
    if a.size != ACTION_DIM or not np.isfinite(a).all():
        raise ValueError(f"action: {ACTION_DIM} finite floats, got {a.size}")
    joints, values = {}, {}
    for i, side in enumerate(E.SIDES):
        seg = a[i * (ARM_DOF + 1):(i + 1) * (ARM_DOF + 1)]
        joints[side], values[side] = seg[:ARM_DOF], _robodojo_gripper(seg[ARM_DOF])
    return _command(joints, values)


def hold_action(frame_or_obs) -> dict[str, list[float]]:
    """The command that holds a Frame's or an obs's joints and gripper where they are."""
    if isinstance(frame_or_obs, Frame):
        q = np.asarray(frame_or_obs.q, dtype=np.float64).reshape(-1)
        if q.size != len(E.SIDES) * ARM_DOF or not np.isfinite(q).all():
            raise ValueError(f"frame.q: {len(E.SIDES) * ARM_DOF} finite joints, got {q.size}")
        joints = {s: q[i * ARM_DOF:(i + 1) * ARM_DOF] for i, s in enumerate(E.SIDES)}
        values = {}
        for side, hand in zip(E.SIDES, E.HANDS):
            if hand not in frame_or_obs.grippers:
                raise ValueError(f"frame.grippers: no {hand!r}")
            values[side] = _robodojo_gripper(frame_or_obs.grippers[hand])
        return _command(joints, values)
    state = _state(frame_or_obs)
    return _command({s: _vector(state, ARM_KEY[s], ARM_DOF) for s in E.SIDES},
                    {s: float(np.clip(_vector(state, GRIPPER_KEY[s], 1)[0], 0.0, 1.0))
                     for s in E.SIDES})
