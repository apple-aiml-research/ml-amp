#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The world model: scene objects and their poses, and the decimated scene frame."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from amp.geometry import kinematics as kin
from amp.link import wire

DISPLAY_MAX_FACES = 8000


class SceneError(ValueError):
    """A scene, object or pose that fails validation; the message names the first bad field."""


@dataclass(frozen=True)
class Physics:
    """Rigid-body properties from the sim: mass, centre of mass in the object frame, friction."""

    mass_kg: float
    com: tuple[float, float, float] = (0.0, 0.0, 0.0)
    static_friction: float | None = None
    dynamic_friction: float | None = None

    def to_json(self) -> dict:
        return {"mass_kg": self.mass_kg, "com": list(self.com),
                "static_friction": self.static_friction,
                "dynamic_friction": self.dynamic_friction}


@dataclass
class SceneObject:
    """One rigid object or fixture: its mesh in the object frame and its world pose (4x4)."""

    name: str
    kind: str
    verts: np.ndarray
    faces: np.ndarray
    pose: np.ndarray
    color: tuple | None = None
    physics: Physics | None = None

    def world_verts(self) -> np.ndarray:
        """(V, 3) vertices in the world frame at the current pose."""
        return self.verts @ self.pose[:3, :3].T + self.pose[:3, 3]


@dataclass
class Scene:
    """An episode's objects over a table plane at table_z; version counts accepted scenes."""

    episode: int
    objects: dict[str, SceneObject] = field(default_factory=dict)
    table_z: float = 0.0
    version: int = 0

    def copy(self) -> Scene:
        """Snapshot for planning: poses copied, verts/faces shared (never mutated)."""
        objects = {
            n: SceneObject(o.name, o.kind, o.verts, o.faces, o.pose.copy(), o.color, o.physics)
            for n, o in self.objects.items()
        }
        return Scene(self.episode, objects, self.table_z, self.version)


def pose_from7(p, name="pose") -> np.ndarray:
    """4x4 from a pose7 list; SceneError unless 7 finite numbers with a non-zero quaternion."""
    if not isinstance(p, (list, tuple, np.ndarray)) or len(p) != 7:
        raise SceneError(f"{name} must be 7 numbers [x, y, z, qw, qx, qy, qz], got {p!r}")
    vals = []
    for v in p:
        if isinstance(v, bool) or not isinstance(v, (int, float, np.integer, np.floating)):
            raise SceneError(f"{name} must be 7 numbers, got {p!r}")
        if not math.isfinite(float(v)):
            raise SceneError(f"{name} must be finite, got {p!r}")
        vals.append(float(v))
    q = np.array(vals[3:])
    if np.linalg.norm(q) < 1e-9:
        raise SceneError(f"{name} has a zero quaternion")
    return kin.make_pose(vals[:3], q / np.linalg.norm(q))


def _cluster(verts, faces, cell, lo):
    keys = np.floor((verts - lo) / cell).astype(np.int64)
    dims = keys.max(axis=0) + 1
    flat = keys[:, 0] + dims[0] * (keys[:, 1] + dims[1] * keys[:, 2])
    _, inverse, counts = np.unique(flat, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inverse, verts)
    reps = sums / counts[:, None]
    f = inverse[faces]
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[keep]
    if len(f):
        area2 = np.linalg.norm(
            np.cross(reps[f[:, 1]] - reps[f[:, 0]], reps[f[:, 2]] - reps[f[:, 0]]), axis=1
        )
        f = f[area2 > 1e-12]
        _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[np.sort(first)]
    used, f = np.unique(f, return_inverse=True)
    return reps[used], f.reshape(-1, 3)


def decimate(verts, faces, max_faces=DISPLAY_MAX_FACES):
    """Vertex-clustering decimation to <= max_faces triangles; (float32 (V,3), uint32 (F,3))."""
    verts = np.asarray(verts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    if len(faces) <= max_faces:
        return verts.astype(np.float32), faces.astype(np.uint32)
    lo = verts.min(axis=0)
    extent = float(np.max(verts.max(axis=0) - lo)) or 1.0
    best = None
    lo_res, hi_res = 2, max(4, int(math.sqrt(len(faces))) * 2)
    while lo_res <= hi_res:
        res = (lo_res + hi_res) // 2
        v, f = _cluster(verts, faces, extent / res, lo)
        if len(f) <= max_faces:
            best = (v, f)
            lo_res = res + 1
        else:
            hi_res = res - 1
    if best is None or not len(best[1]):
        best = _cluster(verts, faces, extent / 2, lo)
    v, f = best
    return v.astype(np.float32), f.astype(np.uint32)


def display_meshes(scene: Scene, max_faces=DISPLAY_MAX_FACES) -> dict:
    """{name: (verts float32, faces uint32)} decimated for the UI."""
    return {n: decimate(o.verts, o.faces, max_faces) for n, o in scene.objects.items()}


def display_frame(scene: Scene, meshes: dict, meta: dict) -> bytes:
    """The GET /api/scene TLF1 frame: current poses, decimated meshes."""
    objects, blobs = {}, {}
    for name, obj in scene.objects.items():
        v, f = meshes[name]
        objects[name] = {
            "kind": obj.kind,
            "pose": kin.pose7(obj.pose).tolist(),
            "aabb": meta.get(name, {}).get("aabb"),
            "color": None if obj.color is None else list(obj.color),
            "n_verts": len(v),
            "n_faces": len(f),
            "n_verts_full": len(obj.verts),
            "n_faces_full": len(obj.faces),
            "source": meta.get(name, {}).get("source", ""),
        }
        if obj.physics is not None:
            objects[name]["physics"] = obj.physics.to_json()
        blobs[f"verts/{name}"] = np.ascontiguousarray(v, dtype=np.float32)
        blobs[f"faces/{name}"] = np.ascontiguousarray(f, dtype=np.uint32)
    header = {"episode": scene.episode, "version": scene.version, "objects": objects}
    return wire.pack_frame(header, blobs)
