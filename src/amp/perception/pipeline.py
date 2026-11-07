#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One perception pass: a fused cloud in, objects with closed meshes and stable ids out."""

from __future__ import annotations

import time

import numpy as np

from amp.contracts.trace import Tracer
from amp.geometry.collision import RobotSpheres
from amp.geometry.embodiment import Embodiment, LinkSpheres
from amp.perception.cloud import (
    Held,
    default_rgb,
    estimate_table_z,
    remove_held,
    remove_robot,
    robot_remnant,
    robot_spheres,
)
from amp.perception.identity import assign_ids, id_num, signature
from amp.perception.mesh import bottoms, extrude, heightmap
from amp.perception.objects import KINDS, PerceivedObject, Perception, label_kind
from amp.perception.params import PerceptionParams, params_for
from amp.perception.segment import absorb, merge_stacked, segment_voxels, voxelize
from amp.perception.views import FreeSpace, viewpoints

MOVED_M = 0.05


def _id_key(k: str) -> tuple[int, str]:
    return id_num(k), k


def pass_deltas(prev: Perception | None, objects: dict) -> tuple[list, list, list]:
    """(new ids, ids dropped, [[id, dist_m]] moved past MOVED_M) of a pass against the previous."""
    before = prev.objects if prev is not None else {}
    new = sorted((k for k in objects if k not in before), key=_id_key)
    lost = sorted((k for k in before if k not in objects), key=_id_key)
    moved = []
    for k in sorted(objects.keys() & before.keys(), key=_id_key):
        d = float(np.linalg.norm(np.asarray(objects[k].centroid, np.float64)
                                 - np.asarray(before[k].centroid, np.float64)))
        if d > MOVED_M:
            moved.append([k, round(d, 3)])
    return new, lost, moved


def _table_z(xyz: np.ndarray, hint: float | None, notes: list[str], p: PerceptionParams) -> float:
    if hint is not None:
        return float(hint)
    if not p.table.search:
        notes.append(f"no support surface; table_z = {p.table.fallback_z:g}")
        return float(p.table.fallback_z)
    est = estimate_table_z(xyz, region=p.table)
    if est is None:
        notes.append(f"table not found; table_z = {p.table.fallback_z:g}")
        return float(p.table.fallback_z)
    notes.append(f"table_z {est[0]:.4f} estimated from {est[1]} points")
    return est[0]


def _segments(pts, col, has_rgb, cams, free, spheres, table_z, p, notes) -> list[dict]:
    """Voxelise, segment, absorb fragments, merge stacks; drop noise and robot remnants."""
    pos, vcol, inv = voxelize(pts.astype(np.float64), col.astype(np.float64), p.voxel)
    view = viewpoints(pos, cams, free)
    lab, pairs = segment_voxels(pos, vcol, has_rgb and p.color_split, view, p)
    lab = absorb(lab, pos, vcol, has_rgb, pairs, p)
    lab = merge_stacked(lab, pos, vcol, has_rgb, pairs, p)
    nvox = np.bincount(lab)
    good = np.flatnonzero(nvox >= p.min_voxels)
    dropped = int((nvox < p.min_voxels).sum())
    if dropped:
        notes.append(f"{dropped} segments under {p.min_voxels} voxels dropped as noise")
    plab = lab[inv]
    order = np.argsort(plab, kind="stable")
    bounds = np.searchsorted(plab[order], np.arange(len(nvox) + 1))
    vorder = np.argsort(lab, kind="stable")
    vb = np.searchsorted(lab[vorder], np.arange(len(nvox) + 1))
    segs, remnants = [], 0
    for g in good:
        pi = order[bounds[g]:bounds[g + 1]]
        vi = vorder[vb[g]:vb[g + 1]]
        if spheres is not None and robot_remnant(pts[pi], spheres, table_z, p):
            remnants += 1
            continue
        segs.append({"pts": pts[pi], "col": col[pi], "vox": pos[vi], "vcol": vcol[vi]})
    if remnants:
        notes.append(f"{remnants} floating segments hugging the robot dropped")
    return segs


def _mesh(pts, hm, bottom, p):
    """(verts, faces) of a segment's height field, clipped to its points' xy box."""
    verts, faces = extrude(hm, bottom, p)
    lo, hi = pts[:, :2].min(axis=0), pts[:, :2].max(axis=0)
    verts[:, :2] = np.clip(verts[:, :2], lo - p.footprint_pad, hi + p.footprint_pad)
    return verts, faces


def perceive(xyz, rgb, *, cams=(), q=None, T_flange_ee=None, table_z_hint: float | None = None,
             held: Held | None = None, prev: Perception | None = None,
             params: PerceptionParams | None = None, now: float | None = None,
             spheres: RobotSpheres | LinkSpheres | None = None, tracer: Tracer | None = None,
             embodiment: Embodiment | None = None) -> Perception:
    """Segment a fused world-frame point cloud into objects with closed meshes."""
    t0 = time.perf_counter()
    p = params_for(embodiment) if params is None else params
    notes: list[str] = []
    xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
    has_rgb = rgb is not None
    rgb = default_rgb(len(xyz), rgb)
    ok = np.isfinite(xyz).all(axis=1)
    xyz, rgb = xyz[ok], rgb[ok]
    table_z = _table_z(xyz, table_z_hint, notes, p)
    keep = xyz[:, 2] > table_z + p.table_margin
    pts, col = xyz[keep], rgb[keep]
    sph = None
    if q is not None and len(pts):
        sph = robot_spheres(q, T_flange_ee, spheres, embodiment=embodiment)
        k = remove_robot(pts, sph, p.robot_inflate)
        notes.append(f"{int((~k).sum())} robot points removed")
        pts, col = pts[k], col[k]
    held_id = None
    if held is not None:
        held_id = str(held[0])
        k = remove_held(pts, held[1], held[2], p.held_margin)
        notes.append(f"{int((~k).sum())} points of held {held_id} removed")
        pts, col = pts[k], col[k]
    free = FreeSpace(cams, xyz, p.free_margin) if cams else None
    segs = _segments(pts, col, has_rgb, cams, free, sph, table_z, p, notes) if len(pts) else []
    hmaps = [heightmap(s["pts"], table_z, p, free) for s in segs]
    keep_i = [i for i, h in enumerate(hmaps) if h is not None]
    segs = [segs[i] for i in keep_i]
    hmaps = [hmaps[i] for i in keep_i]
    sigs, built = [], []
    for s, hm, bot in zip(segs, hmaps, bottoms(hmaps, table_z, p)):
        verts, faces = _mesh(s["pts"], hm, bot, p)
        color = tuple(round(float(c)) for c in s["col"].mean(axis=0))
        centroid = s["vox"].mean(axis=0)
        sigs.append(signature(centroid, s["pts"], color, "object"))
        built.append((s, verts, faces, color, centroid))
    now = time.time() if now is None else float(now)
    ids, nxt, lost = assign_ids(sigs, prev, held_id, now, p)
    objects = {}
    for oid, (s, verts, faces, color, centroid) in sorted(
            zip(ids, built), key=lambda t: id_num(t[0])):
        aabb = np.stack([verts.min(axis=0), verts.max(axis=0)])
        label, kind = label_kind(aabb, color, table_z, p)
        objects[oid] = PerceivedObject(
            id=oid, label=label, color=color,
            points=np.ascontiguousarray(s["pts"], np.float32), verts=verts, faces=faces,
            centroid=np.asarray(centroid, np.float64), aabb=aabb, kind=kind,
            rgb=np.ascontiguousarray(s["col"], np.uint8),
        )
    dt = time.perf_counter() - t0
    notes.append(f"{len(objects)} objects from {len(xyz)} points in {dt * 1000:.0f} ms")
    if tracer is not None:
        added, dropped, moved = pass_deltas(prev, objects)
        tracer.event("server.perception.pass", {
            "points": len(xyz), "objects": sorted(objects), "table_z": table_z,
            "held": held_id, "time_s": dt, "notes": notes, "new": added, "lost": dropped,
            "moved": moved})
    return Perception(table_z, objects, dt, notes, nxt, lost, now)


def object_from_points(oid: str, pts, rgb, table_z: float, *, label: str | None = None,
                       kind: str | None = None, source: str = "agent",
                       params: PerceptionParams | None = None,
                       embodiment: Embodiment | None = None) -> PerceivedObject | None:
    """A PerceivedObject meshed from points as perceive() meshes a segment (None if too few)."""
    p = params_for(embodiment) if params is None else params
    pts = np.asarray(pts, np.float32).reshape(-1, 3)
    rgb = default_rgb(len(pts), rgb, grey=128)
    ok = np.isfinite(pts).all(axis=1) & (pts[:, 2] > table_z + p.min_thickness)
    pts, rgb = pts[ok], rgb[ok]
    if len(pts) < 3:
        return None
    hm = heightmap(pts.astype(np.float64), table_z, p, None)
    if hm is None:
        return None
    verts, faces = _mesh(pts, hm, bottoms([hm], table_z, p)[0], p)
    color = tuple(round(float(c)) for c in rgb.astype(np.float64).mean(axis=0))
    aabb = np.stack([verts.min(axis=0), verts.max(axis=0)])
    auto_label, auto_kind = label_kind(aabb, color, table_z, p)
    return PerceivedObject(
        id=oid, label=str(label) if label else auto_label, color=color,
        points=np.ascontiguousarray(pts), verts=verts, faces=faces,
        centroid=voxelize(pts.astype(np.float64), rgb, p.voxel)[0].mean(axis=0), aabb=aabb,
        kind=kind if kind in KINDS else auto_kind, rgb=np.ascontiguousarray(rgb), source=source)
