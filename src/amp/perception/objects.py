#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Perceived objects: the dataclasses, labels, boxes, lookup by point, and their JSON and scene."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from amp.geometry.collision import mesh_field
from amp.geometry.world import Scene, SceneObject
from amp.perception.color import color_name
from amp.perception.params import PerceptionParams

KINDS = ("object", "fixture")
SOURCES = ("perception", "agent")
MassFn = Callable[[np.ndarray, np.ndarray, str], "float | None"]


@dataclass
class PerceivedObject:
    id: str
    label: str
    color: tuple[int, int, int]
    points: np.ndarray
    verts: np.ndarray
    faces: np.ndarray
    centroid: np.ndarray
    aabb: np.ndarray
    kind: str = "object"
    rgb: np.ndarray | None = None
    source: str = "perception"


@dataclass
class Perception:
    """One perception pass: its objects, the lost-object memory and the next free id."""

    table_z: float
    objects: dict[str, PerceivedObject]
    time_s: float
    notes: list[str]
    next_id: int = 1
    lost: dict = field(default_factory=dict)
    stamp: float = 0.0

    def copy(self) -> Perception:
        """A copy whose object dict, notes and lost memory can change without touching this."""
        return Perception(self.table_z, dict(self.objects), self.time_s, list(self.notes),
                          self.next_id, dict(self.lost), self.stamp)


def label_kind(aabb: np.ndarray, color, table_z: float, p: PerceptionParams):
    """("<colour> · AxBxH cm", kind) of a mesh's bounds; every perceived mesh is an object."""
    ext = aabb[1] - aabb[0]
    h = aabb[1, 2] - table_z if p.floor_at_support else ext[2]
    a, b = sorted(ext[:2], reverse=True)
    dims = "x".join(str(max(1, round(float(100 * d)))) for d in (a, b, h))
    return f"{color_name(color)} · {dims} cm", "object"


def to_scene(p: Perception, episode: int, version: int) -> Scene:
    """A Scene with one SceneObject per object: world verts at an identity pose."""
    objects = {
        oid: SceneObject(oid, o.kind, np.asarray(o.verts, np.float64),
                         np.asarray(o.faces, np.int64), np.eye(4), tuple(o.color))
        for oid, o in p.objects.items()
    }
    return Scene(int(episode), objects, float(p.table_z), int(version))


def object_at(p: Perception, point, max_dist: float = 0.03) -> str | None:
    """The object whose points come within max_dist of point, else the smallest mesh holding it."""
    x = np.asarray(point, np.float64).reshape(3)
    best, best_d = None, float(max_dist)
    for oid, o in p.objects.items():
        if not len(o.points):
            continue
        lo, hi = o.aabb[0] - best_d, o.aabb[1] + best_d
        if np.any(x < lo) or np.any(x > hi):
            continue
        d = float(np.sqrt(np.min(np.sum((o.points.astype(np.float64) - x) ** 2, axis=1))))
        if d <= best_d:
            best, best_d = oid, d
    if best is None:
        best = _mesh_at(p, x)
    return best


def _mesh_at(p: Perception, x: np.ndarray) -> str | None:
    """The smallest object whose mesh holds x, base included: a point no camera saw."""
    best, best_v = None, np.inf
    for oid, o in p.objects.items():
        lo, hi = o.aabb
        if np.any(x[:2] < lo[:2]) or np.any(x[:2] > hi[:2]) or x[2] > hi[2] or x[2] < lo[2] - 0.01:
            continue
        q = np.array([x[0], x[1], max(x[2], lo[2] + 0.003)])
        vol = float(np.prod(hi - lo))
        fld = mesh_field(o.verts, o.faces)
        if vol < best_v and fld is not None and fld.inside(q[None])[0]:
            best, best_v = oid, vol
    return best


def obb(points) -> dict:
    """z-up box {"center", "size", "yaw_deg"}: size[0] along the PCA long axis, yaw in (-90, 90]."""
    pts = np.asarray(points, np.float64).reshape(-1, 3)
    if not len(pts):
        return {"center": [0.0, 0.0, 0.0], "size": [0.0, 0.0, 0.0], "yaw_deg": 0.0}
    xy = pts[:, :2] - pts[:, :2].mean(axis=0)
    yaw = 0.0
    if len(pts) >= 3:
        w, v = np.linalg.eigh(xy.T @ xy)
        if w[1] > 1.05 * w[0]:
            yaw = float(np.arctan2(v[1, 1], v[0, 1]))
    yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2
    if yaw <= -np.pi / 2 + 1e-9:
        yaw += np.pi
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s], [-s, c]])
    loc = pts[:, :2] @ R.T
    lo, hi = loc.min(axis=0), loc.max(axis=0)
    mid = 0.5 * (lo + hi)
    centre_xy = R.T @ mid
    z0, z1 = float(pts[:, 2].min()), float(pts[:, 2].max())
    return {"center": [float(centre_xy[0]), float(centre_xy[1]), 0.5 * (z0 + z1)],
            "size": [float(hi[0] - lo[0]), float(hi[1] - lo[1]), z1 - z0],
            "yaw_deg": float(np.degrees(yaw))}


def in_obb(pts, box: dict, margin: float = 0.0) -> np.ndarray:
    """Mask of pts inside a z-up box {"center", "size", "yaw_deg"} grown by margin."""
    pts = np.asarray(pts, np.float64).reshape(-1, 3)
    c = np.asarray(box["center"], np.float64).reshape(3)
    half = 0.5 * np.asarray(box["size"], np.float64).reshape(3) + margin
    a = np.radians(float(box.get("yaw_deg", 0.0)))
    d = pts - c
    u = d[:, 0] * np.cos(a) + d[:, 1] * np.sin(a)
    v = -d[:, 0] * np.sin(a) + d[:, 1] * np.cos(a)
    return (np.abs(u) <= half[0]) & (np.abs(v) <= half[1]) & (np.abs(d[:, 2]) <= half[2])


def footprint(o: PerceivedObject) -> dict:
    """Position, xy box, sorted xy extent, colour and lowest point of an object, for matching."""
    pts = np.asarray(o.points, dtype=np.float64).reshape(-1, 3)
    aabb = np.asarray(o.aabb, dtype=np.float64).reshape(2, 3)
    ext = np.ptp(pts, axis=0) if len(pts) else aabb[1] - aabb[0]
    color = np.asarray(o.color if o.color is not None else (128, 128, 128), dtype=np.float64)
    return {"c": np.asarray(o.centroid, dtype=np.float64).reshape(3), "lo": aabb[0, :2],
            "hi": aabb[1, :2], "ext": np.sort(ext[:2]), "color": color,
            "zmin": float(pts[:, 2].min()) if len(pts) else float(aabb[0, 2])}


def footprint_fracs(a: dict, b: dict) -> tuple[float, float]:
    """(intersection / a's footprint, intersection / b's footprint) of two xy boxes."""
    w = np.clip(np.minimum(a["hi"], b["hi"]) - np.maximum(a["lo"], b["lo"]), 0.0, None)
    inter = float(w[0] * w[1])
    return (inter / max(float(np.prod(a["hi"] - a["lo"])), 1e-6),
            inter / max(float(np.prod(b["hi"] - b["lo"])), 1e-6))


def similar_size(ref: dict, cand: dict) -> bool:
    """cand's sorted xy extents fit ref's, allowing occlusion to shrink them."""
    e0, e1 = ref["ext"], cand["ext"]
    return bool(np.all(e1 <= 1.35 * e0 + 0.03) and np.all(e1 >= 0.4 * e0 - 0.01))


def footprint_overlap(a: dict, b: dict) -> float:
    """xy footprint intersection over the smaller footprint."""
    w = np.clip(np.minimum(a["hi"], b["hi"]) - np.maximum(a["lo"], b["lo"]), 0.0, None)
    small = min(float(np.prod(a["hi"] - a["lo"])), float(np.prod(b["hi"] - b["lo"])))
    return float(w[0] * w[1]) / small if small > 1e-8 else 0.0


def mass_kg(o: PerceivedObject, mass_fn: MassFn | None) -> float | None:
    """mass_fn's estimate of o, rounded; None without one or when it fails."""
    if mass_fn is None:
        return None
    try:
        m = float(mass_fn(np.asarray(o.verts, dtype=np.float64), np.asarray(o.faces),
                          str(o.label)))
    except Exception:  # noqa: BLE001
        return None
    return round(m, 4) if math.isfinite(m) else None


def object_json(o: PerceivedObject, mass_fn: MassFn | None = None) -> dict:
    """The JSON an API reply carries for one object."""
    def vec(x):
        return [float(v) for v in np.asarray(x, dtype=np.float64).reshape(-1)]

    aabb = np.asarray(o.aabb, dtype=np.float64).reshape(2, 3)
    color = o.color if o.color is not None else (128, 128, 128)
    out = {"id": o.id, "label": str(o.label), "color": [int(c) for c in color],
           "centroid": vec(o.centroid), "aabb": [vec(aabb[0]), vec(aabb[1])],
           "n_points": len(o.points), "kind": str(o.kind)}
    pts = np.asarray(o.points, dtype=np.float64).reshape(-1, 3)
    out["obb"] = obb(pts if len(pts) else np.asarray(o.verts, dtype=np.float64))
    out["source"] = str(o.source)
    mass = mass_kg(o, mass_fn)
    if mass is not None:
        out["est_mass_kg"] = mass
    return out


def object_points(o: PerceivedObject) -> np.ndarray:
    """(N, 6) float32 x, y, z, r, g, b (colour in [0, 1]) of an object's points."""
    pts = np.asarray(o.points, dtype=np.float32).reshape(-1, 3)
    rgb = o.rgb
    if rgb is None or len(rgb) != len(pts):
        rgb = np.tile(np.asarray(o.color, dtype=np.uint8), (len(pts), 1))
    return np.ascontiguousarray(
        np.hstack([pts, np.asarray(rgb, dtype=np.float32) / 255.0]).astype(np.float32))
