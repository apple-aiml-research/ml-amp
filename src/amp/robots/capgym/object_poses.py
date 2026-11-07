#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Scene-object root poses of a BEHAVIOR episode, for the results folder only (never the agent)."""

from __future__ import annotations

import time
from collections.abc import Callable

import numpy as np

STATE_FILE = "run_{k}_state.json"
ROOT_RADIUS_M = 2.0
MAX_OBJECTS = 200


def _np(x) -> np.ndarray:
    if hasattr(x, "detach"):
        x = x.detach().cpu().numpy()
    return np.asarray(x, dtype=np.float64)


def scene_of(env):
    """The OmniGibson scene of CaP-X's env: its og.Environment's (env.env.scene), else its robot's."""

    tried = []
    for path in ("env.scene", "robot.scene"):
        obj = env
        try:
            for part in path.split("."):
                obj = getattr(obj, part)
        except Exception as exc:  # noqa: BLE001
            tried.append(f"{type(env).__name__}.{path}: {exc!r}")
            continue
        if obj is not None:
            return obj
        tried.append(f"{type(env).__name__}.{path} is None")
    raise AttributeError(f"no OmniGibson scene on the env ({'; '.join(tried)})")


def root_pose(obj) -> list[float]:
    """[x, y, z, qw, qx, qy, qz] of obj's root in the world: RoboLab's root_pose order (OmniGibson's
    quaternion is xyzw)."""
    pos, quat = obj.get_position_orientation()
    x, y, z, w = (float(v) for v in _np(quat).reshape(4))
    return [*(float(v) for v in _np(pos).reshape(3)), w, x, y, z]


def snapshot(env, *, radius_m: float = ROOT_RADIUS_M, cap: int = MAX_OBJECTS) -> dict:
    """Root poses of the objects near the base (nearest first, at most cap) and any held."""
    t0 = time.perf_counter()
    robot = env.robot
    base_row = root_pose(robot)
    base = np.asarray(base_row[:3])
    held = {}
    for obj in (getattr(robot, "_ag_obj_in_hand", None) or {}).values():
        if obj is not None:
            held[obj.name] = obj
    near = []
    cats = {}
    for obj in scene_of(env).objects:
        if obj is robot or obj.name == robot.name:
            continue
        row = root_pose(obj)
        d = float(np.linalg.norm(np.asarray(row[:3]) - base))
        if d <= radius_m:
            near.append((d, obj.name, row))
            cats[obj.name] = getattr(obj, "category", None)
    near.sort(key=lambda t: (t[0], t[1]))
    poses = {f"object/{n}/root_pose": row for _, n, row in near[:cap]}
    for name, obj in sorted(held.items()):
        poses.setdefault(f"object/{name}/root_pose", root_pose(obj))
        cats.setdefault(name, getattr(obj, "category", None))
    categories = {n: str(c) for n, c in cats.items() if c and f"object/{n}/root_pose" in poses}
    return {"poses": poses, "categories": categories, "base": base_row,
            "truncated": max(0, len(near) - cap), "grasped": sorted(held),
            "read_s": time.perf_counter() - t0}


class Snapshots:
    """The labelled snapshots of one episode: initial, base_move_<k> and final."""

    def __init__(self, take: Callable[[], dict]):
        self._take = take
        self.reset()

    def reset(self) -> None:
        self.shots: dict[str, dict] = {}
        self.moves = 0
        self.base_moved = False

    def add(self, label: str) -> None:
        """A snapshot never stops an episode: a failed read is recorded as the error."""
        try:
            self.shots[label] = self._take()
        except Exception as exc:  # noqa: BLE001
            self.shots[label] = {"error": f"{type(exc).__name__}: {exc}"}

    def after_base_move(self) -> None:
        if self.base_moved:
            self.base_moved = False
            self.moves += 1
            self.add(f"base_move_{self.moves}")

    def record(self) -> dict:
        """RoboLab's h5state shape, {label: {"object/<name>/root_pose": row}}: `initial` a list of
        rows as its initial_state is, the others one row as its first observation is."""
        self.after_base_move()
        self.add("final")
        out: dict = {"schema": 1, "quat": "wxyz", "radius_m": ROOT_RADIUS_M, "cap": MAX_OBJECTS,
                     "base": {}, "categories": {}, "truncated": {}, "grasped": {}, "read_s": {}}
        for label, shot in self.shots.items():
            if "error" in shot:
                out.setdefault("errors", {})[label] = shot["error"]
                out[label] = {}
                continue
            out[label] = {k: [row] if label == "initial" else row
                          for k, row in shot["poses"].items()}
            out["base"][label] = shot["base"]
            out["categories"].update(shot["categories"])
            out["truncated"][label] = shot["truncated"]
            out["grasped"][label] = shot["grasped"]
            out["read_s"][label] = round(shot["read_s"], 6)
        return out
