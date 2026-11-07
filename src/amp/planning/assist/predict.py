#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What a plan predicts: the robot and object state before and after each step."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from amp.planning.assist.steps import KINDS, StepSpec, as_pose, check_place_args, pose7_json
from amp.planning.planner import INFER, PlanContext
from amp.planning.robot import arm_of

HELD_RADIUS_M = 0.03


class Infeasible(Exception):
    """A step that cannot be planned in its predicted state; the message says why."""


@dataclass
class PredictedState:
    """The arm, the gripper, what it holds (and how), and every object's pose."""

    q: np.ndarray
    gripper: float
    held: str | None
    T_tcp_obj: np.ndarray | None
    poses: dict[str, np.ndarray]

    def scene(self, base):
        """A copy of base with these object poses."""
        s = base.copy()
        for name, T in self.poses.items():
            if name in s.objects:
                s.objects[name].pose = np.array(T, dtype=np.float64)
        return s

    def ctx(self, base: PlanContext) -> PlanContext:
        """base with q and gripper replaced by this state's."""
        return base.replace(q=np.array(self.q, dtype=np.float64),
                            gripper=1 if self.gripper > 0.5 else 0)

    def to_json(self) -> dict:
        objects = {}
        for name, T in self.poses.items():
            p = pose7_json(T)
            objects[name] = {"pos": p[:3], "quat": p[3:]}
        return {"q": [round(float(v), 6) for v in self.q],
                "gripper": 1 if self.gripper > 0.5 else 0, "held": self.held, "objects": objects}

    def fingerprint(self) -> tuple:
        return (
            np.asarray(self.q, dtype=np.float64).tobytes(), float(self.gripper), self.held,
            None if self.T_tcp_obj is None else np.asarray(self.T_tcp_obj).tobytes(),
            tuple(sorted((n, np.asarray(T).tobytes()) for n, T in self.poses.items())),
        )


def held_name(held) -> str | None:
    """The object name of a held value: a name, {"object": name}, or nothing."""
    if isinstance(held, dict):
        held = held.get("object")
    return held if isinstance(held, str) and held else None


def infer_held(scene, T_tcp, extra=()) -> str | None:
    """The object whose bounding box holds the TCP (the smallest: not a bin around it), else the
    one nearest the TCP within HELD_RADIUS_M; extra: more (name, SceneObject) boxes to weigh."""
    inside, best, best_d = None, None, HELD_RADIUS_M
    for name, o in [*scene.objects.items(), *extra]:
        if o.kind != "object" or not len(o.verts):
            continue
        p = np.linalg.inv(o.pose) @ np.append(T_tcp[:3, 3], 1.0)
        lo, hi = o.verts.min(axis=0), o.verts.max(axis=0)
        d = float(np.linalg.norm(p[:3] - np.clip(p[:3], lo, hi)))
        if d == 0.0:
            vol = float(np.prod(hi - lo))
            if inside is None or vol < inside[1]:
                inside = (name, vol)
        elif d < best_d:
            best, best_d = name, d
    return inside[0] if inside is not None else best


def observed_state(scene, ctx: PlanContext) -> PredictedState:
    """The state a plan starts from: the observed robot, and the held object given or inferred."""
    q = np.array(ctx.q, dtype=np.float64).reshape(arm_of(ctx.tool).n)
    gripper = 1.0 if float(ctx.gripper or 0) > 0.5 else 0.0
    poses = {n: np.array(o.pose, dtype=np.float64) for n, o in scene.objects.items()}
    T_tcp = ctx.T_world_tcp
    T_tcp = ctx.tool.fk_tcp(q) if T_tcp is None else as_pose(T_tcp, "T_world_tcp")
    if ctx.held is not INFER:
        held = held_name(ctx.held)
    else:
        held = infer_held(scene, T_tcp) if gripper > 0.5 else None
    if held not in poses:
        held = None
    T_tcp_obj = None if held is None else np.linalg.inv(T_tcp) @ poses[held]
    return PredictedState(q, gripper, held, T_tcp_obj, poses)


def held_message(state: PredictedState, name) -> str:
    if state.held is None:
        return f"the gripper holds nothing here; pick {name} before placing it"
    return f"the gripper holds {state.held} here, not {name}"


def anchor_for(kind, name, T_world_tcp, state: PredictedState, *, target=None,
               point=None) -> np.ndarray:
    """The anchor of a step whose world TCP T_world_tcp was evaluated in state."""
    if kind not in KINDS:
        raise ValueError(f"kind must be one of {list(KINDS)}, got {kind!r}")
    check_place_args(kind, name, target, point)
    T = as_pose(T_world_tcp, "tcp")
    if name not in state.poses:
        raise ValueError(f"unknown object {name!r}")
    if kind == "pick":
        return np.linalg.inv(state.poses[name]) @ T
    if state.held != name:
        raise ValueError(held_message(state, name))
    T_obj = T @ state.T_tcp_obj
    if target is None:
        return T_obj
    if target not in state.poses:
        raise ValueError(f"unknown target {target!r}")
    return np.linalg.inv(state.poses[target]) @ T_obj


def world_tcp(spec: StepSpec, state: PredictedState) -> np.ndarray:
    """The world TCP pose spec's anchor means in state; Infeasible when it means nothing there."""
    if spec.anchor is None:
        what = "grasp" if spec.kind == "pick" else "release"
        raise Infeasible(f"the step has no {what} pose")
    if spec.object not in state.poses:
        raise Infeasible(f"unknown object {spec.object!r}")
    if spec.kind == "pick":
        return state.poses[spec.object] @ spec.anchor
    if state.held != spec.object or state.T_tcp_obj is None:
        raise Infeasible(held_message(state, spec.object))
    if spec.target is not None and spec.target not in state.poses:
        raise Infeasible(f"unknown target {spec.target!r}")
    T_ref = np.eye(4) if spec.target is None else state.poses[spec.target]
    return (T_ref @ spec.anchor) @ np.linalg.inv(state.T_tcp_obj)


def anchor_of_tcp(spec: StepSpec, state: PredictedState, T_tcp) -> np.ndarray:
    """The anchor that makes world_tcp(spec, state) equal T_tcp."""
    if spec.kind == "pick":
        return np.linalg.inv(state.poses[spec.object]) @ T_tcp
    T_obj = T_tcp @ state.T_tcp_obj
    return T_obj if spec.target is None else np.linalg.inv(state.poses[spec.target]) @ T_obj


def after_pick(state: PredictedState, name, T_tcp, q_end, tool) -> PredictedState:
    """Holding name by the grasp T_tcp, the arm at q_end."""
    poses = dict(state.poses)
    T_tcp_obj = np.linalg.inv(T_tcp) @ state.poses[name]
    q_end = np.array(q_end, dtype=np.float64)
    poses[name] = tool.fk_tcp(q_end) @ T_tcp_obj
    return PredictedState(q_end, 1.0, name, T_tcp_obj, poses)


def after_place(state: PredictedState, name, T_tcp, q_end, release_height) -> PredictedState:
    """name released at T_tcp and dropped release_height onto its support, the arm at q_end."""
    poses = dict(state.poses)
    T_rest = T_tcp @ state.T_tcp_obj
    T_rest[2, 3] -= release_height
    poses[name] = T_rest
    return PredictedState(np.array(q_end, dtype=np.float64), 0.0, None, None, poses)


def held_transform(name, scene, T_world_tcp, pred: PredictedState | None) -> np.ndarray:
    """T_tcp_obj of the held object: the predicted grasp, else its pose now relative to the TCP."""
    if pred is not None and pred.held == name and pred.T_tcp_obj is not None:
        return np.asarray(pred.T_tcp_obj, dtype=np.float64)
    return np.linalg.inv(T_world_tcp) @ scene.objects[name].pose
