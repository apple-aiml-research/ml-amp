#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The agent's perception tools over a PerceptionState: segment, define, forget, points, pass."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from amp.contracts.trace import Tracer
from amp.geometry.collision import RobotSpheres
from amp.geometry.embodiment import Embodiment, LinkSpheres
from amp.perception.cloud import Held, clean_cloud, depth_cams, fuse_frames
from amp.perception.held import held_from, pick_verdict
from amp.perception.objects import KINDS, PerceivedObject, Perception, in_obb, obb, object_points
from amp.perception.pipeline import object_from_points, perceive
from amp.perception.segment import region_grow
from amp.perception.state import AgentObject, PerceptionState

SEGMENT_RADIUS_M = 0.011
SEGMENT_RADIUS_RANGE_M = (0.002, 0.050)
SEGMENT_SEED_M = 0.015
DEFINE_MIN_POINTS = 10
DEFINE_MAX_POINTS = 200_000
COLOR_FROM_CLOUD_M = 0.01
LABEL_MAX = 80
OWNED_M = 0.004


class PerceptionConflict(Exception):
    """A request the perception state cannot satisfy now (the server's 409)."""


@dataclass
class Scan:
    """One observation as perception needs it: the fused cloud, its cameras and the robot."""

    xyz: np.ndarray
    rgb: np.ndarray | None
    cams: tuple = ()
    q: np.ndarray | None = None
    T_flange_ee: np.ndarray | None = None
    T_world_tcp: np.ndarray | None = None


@dataclass
class HeldCheck:
    """A grasp to confirm on the next pass: after a pick (object named) or a replay (None)."""

    object: str | None = None
    sig0: dict | None = None
    geom: dict | None = None


@dataclass
class PassResult:
    perception: Perception
    held: dict | None = None


def _transform(T, verts) -> np.ndarray:
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    return v @ T[:3, :3].T + T[:3, 3]


def held_mesh(geom: dict, T_world_tcp) -> Held:
    """The world mesh of held geometry carried at T_world_tcp."""
    T = np.asarray(T_world_tcp, np.float64) @ np.asarray(geom["T_tcp_obj"], np.float64)
    return geom["object"], _transform(T, geom["verts"]), geom["faces"]


def perceive_pass(state: PerceptionState, scan: Scan, *, table_z: float,
                  held: Held | None = None, check: HeldCheck | None = None,
                  wanted: Iterable[str] = (), now: float | None = None,
                  spheres: RobotSpheres | LinkSpheres | None = None,
                  tracer: Tracer | None = None) -> PassResult:
    """Perceive scan, settle a pending HeldCheck, and install the result in state."""
    spheres = state.spheres if spheres is None else spheres
    kw = {"cams": scan.cams, "q": scan.q, "T_flange_ee": scan.T_flange_ee,
          "table_z_hint": table_z, "params": state.params, "now": now, "spheres": spheres,
          "tracer": tracer, "embodiment": state.embodiment}
    prev = state.prev()
    p, verdict = None, None
    if check is not None and check.object is not None:
        p0 = perceive(scan.xyz, scan.rgb, held=None, prev=prev, **kw)
        prev_ids = set(prev.objects) if prev is not None else set()
        fresh = {i for i in p0.objects if i not in prev_ids}
        ok, why = pick_verdict(check.object, check.sig0, p0, check.sig0["table_z"],
                               np.asarray(scan.T_world_tcp)[:3, 3], fresh)
        verdict = {**check.geom, "reason": why} if ok else {"reason": why}
        if ok:
            held = held_mesh(check.geom, scan.T_world_tcp)
        else:
            p = p0
    elif check is not None:
        p0 = perceive(scan.xyz, scan.rgb, held=None, prev=prev, **kw)
        found = held_from(p0, scan.T_world_tcp)
        if found is None:
            p = p0
        else:
            verdict = {**found, "reason": f"held: {found['object']} at the TCP after the replay "
                                          "(the gripper stopped on it)"}
            prev = p0
            held = (found["object"], np.asarray(found["verts"]), found["faces"])
    if p is None:
        p = perceive(scan.xyz, scan.rgb, held=held, prev=prev, **kw)
    held_id = held[0] if held is not None else None

    def clean():
        return clean_cloud(scan.xyz, scan.rgb, p.table_z, q=scan.q, T_flange_ee=scan.T_flange_ee,
                           held=held, params=state.params, spheres=spheres,
                           embodiment=state.embodiment)

    state.install(p, held=held_id, clean=clean, wanted=wanted)
    return PassResult(p, verdict)


def scan_from_frames(frames, *, q=None, T_flange_ee=None, T_world_tcp=None,
                     workspace=None, embodiment: Embodiment | None = None,
                     T_world_base=None) -> Scan:
    """A root-frame Scan of an observation's camera frames (amp.link.obs.CameraFrame by name)."""
    xyz, rgb, _ = fuse_frames(frames, workspace=workspace, embodiment=embodiment,
                              T_world_base=T_world_base)
    return Scan(xyz, rgb, depth_cams(frames), q, T_flange_ee, T_world_tcp)


def _current(state: PerceptionState) -> Perception:
    if state.perception is None:
        raise PerceptionConflict("no perception yet")
    return state.perception


def check_label_kind(label, kind) -> None:
    """Raise ValueError unless label is None or 1..LABEL_MAX chars and kind is None or a KIND."""
    if label is not None and (not isinstance(label, str) or not 0 < len(label) <= LABEL_MAX):
        raise ValueError(f"label must be a string of 1..{LABEL_MAX} characters, got {label!r}")
    if kind is not None and kind not in KINDS:
        raise ValueError(f"kind must be 'object' or 'fixture', got {kind!r}")


def box_from_aabb(aabb) -> dict:
    """{"center", "size", "yaw_deg": 0} of [[x, y, z] low, [x, y, z] high]; ValueError if bad."""
    try:
        lo, hi = (np.asarray(aabb[0], dtype=np.float64), np.asarray(aabb[1], dtype=np.float64))
        ok = len(aabb) == 2 and lo.shape == hi.shape == (3,)
    except (TypeError, ValueError, IndexError):
        ok = False
    if not ok or not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)) and np.all(hi > lo)):
        raise ValueError(f"aabb must be [[x, y, z] low, [x, y, z] high], got {aabb!r}")
    return {"center": (0.5 * (lo + hi)).tolist(), "size": (hi - lo).tolist(), "yaw_deg": 0.0}


def box_from_obb(b) -> dict:
    """A checked copy of {"center": [3], "size": [3] > 0, "yaw_deg"}; ValueError if bad."""
    if not isinstance(b, dict):
        raise ValueError(f"obb must be {{'center', 'size', 'yaw_deg'}}, got {b!r}")  # noqa: TRY004
    try:
        c = np.asarray(b.get("center"), dtype=np.float64)
        z = np.asarray(b.get("size"), dtype=np.float64)
        yaw = float(b.get("yaw_deg", 0.0))
    except (TypeError, ValueError):
        raise ValueError(f"obb must be {{'center': [3], 'size': [3], 'yaw_deg'}}, got {b!r}")
    if c.shape != (3,) or z.shape != (3,) or not (np.all(np.isfinite(c)) and np.all(z > 0)
                                                  and math.isfinite(yaw)):
        raise ValueError(f"obb must be {{'center': [3], 'size': [3] > 0, 'yaw_deg'}}, got {b!r}")
    return {"center": c.tolist(), "size": z.tolist(), "yaw_deg": yaw}


def _unowned(state: PerceptionState, pts, rgb, replace):
    """pts without those another agent object (not being replaced) owns."""
    keep = np.ones(len(pts), bool)
    for aid, a in state.agent.items():
        if aid in replace or not len(a.points) or not keep.any():
            continue
        d = cKDTree(np.asarray(a.points, dtype=np.float64)).query(
            np.asarray(pts, dtype=np.float64), distance_upper_bound=OWNED_M)[0]
        keep &= ~np.isfinite(d)
    return pts[keep], rgb[keep]


def add_agent_object(state: PerceptionState, pts, rgb, *, label=None, kind=None, replace=(),
                     made: str = "define_object", held: str | None = None,
                     tracer: Tracer | None = None) -> PerceivedObject:
    """Add an agent object from points, replacing the objects named in replace."""
    p = _current(state).copy()
    for r in replace:
        if r == held:
            raise PerceptionConflict(f"{r} is held; it cannot be replaced")
        if r not in p.objects:
            raise PerceptionConflict(f"unknown object {r!r}; the scene has {sorted(p.objects)}")
    n = state.next_id()
    aid = f"obj{n}"
    pts, rgb = _unowned(state, pts, rgb, replace)
    obj = object_from_points(aid, pts, rgb, p.table_z, label=label, kind=kind,
                             params=state.params)
    if obj is None:
        raise PerceptionConflict(f"{made}: too few points above the table for an object")
    for r in replace:
        p.objects.pop(r, None)
        p.lost.pop(r, None)
        state.agent.pop(r, None)
        state.successors[r] = {"to": aid, "why": f"replaced by agent object {aid}"}
    p.next_id = n + 1
    p.objects[aid] = obj
    state.agent[aid] = AgentObject(obj.points, obj.rgb, label, kind, obj.verts.mean(axis=0))
    p.notes.append(f"agent object {aid} from {made} ({len(obj.points)} points)")
    _commit(state, p, held)
    if tracer is not None:
        tracer.event("server.perception.agent_object", {
            "id": aid, "made": made, "points": len(obj.points), "replaced": list(replace)})
    return obj


def _commit(state: PerceptionState, p: Perception, held: str | None) -> None:
    state.merge_agent(p, held=held)
    state.track_successors(p, held=held)
    state.perception = p


def segment(state: PerceptionState, scan: Scan, point, *, radius: float = SEGMENT_RADIUS_M,
            color_tol: float | None = None, label=None, kind=None, held: Held | None = None,
            tracer: Tracer | None = None) -> PerceivedObject:
    """An agent object region-grown over the cleaned cloud from the point nearest `point`."""
    lo, hi = SEGMENT_RADIUS_RANGE_M
    if not lo <= radius <= hi:
        raise ValueError(f"radius must be in [{lo}, {hi}] m, got {radius!r}")
    if color_tol is not None and not color_tol > 0:
        raise ValueError(f"color_tol must be a positive number or None, got {color_tol!r}")
    check_label_kind(label, kind)
    p = _current(state)
    pts, col = clean_cloud(scan.xyz, scan.rgb, p.table_z, q=scan.q, T_flange_ee=scan.T_flange_ee,
                           held=held, params=state.params,
                           spheres=state.spheres, embodiment=state.embodiment)
    x = np.asarray(point, dtype=np.float64).reshape(3)
    d = np.linalg.norm(pts.astype(np.float64) - x, axis=1) if len(pts) else np.zeros(0)
    if not len(d) or float(d.min()) > max(radius, SEGMENT_SEED_M):
        raise PerceptionConflict("the pixel hits nothing: the point under it is the table or the "
                                 "robot")
    idx = region_grow(pts, col, int(np.argmin(d)), radius, color_tol=color_tol,
                      params=state.params)
    if len(idx) < DEFINE_MIN_POINTS:
        raise PerceptionConflict(f"segment: only {len(idx)} points grew from that pixel")
    return add_agent_object(state, pts[idx], col[idx], label=label, kind=kind, made="segment",
                            held=held[0] if held else None, tracer=tracer)


def define_object(state: PerceptionState, scan: Scan, *, box: dict | None = None, points=None,
                  label=None, kind=None, replace=(), held: Held | None = None,
                  tracer: Tracer | None = None) -> PerceivedObject:
    """An agent object from the cleaned cloud inside a z-up box, or from given points."""
    if (box is None) == (points is None):
        raise ValueError("define_object takes exactly one of a box or points")
    check_label_kind(label, kind)
    given = None
    if points is not None:
        try:
            given = np.asarray(points, dtype=np.float64)
        except (TypeError, ValueError):
            given = None
        if (given is None or given.ndim != 2 or given.shape[1] != 3
                or not 3 <= len(given) <= DEFINE_MAX_POINTS or not np.all(np.isfinite(given))):
            raise ValueError(f"points must be 3..{DEFINE_MAX_POINTS} [x, y, z] points")
    p = _current(state)
    pts, col = clean_cloud(scan.xyz, scan.rgb, p.table_z, q=scan.q, T_flange_ee=scan.T_flange_ee,
                           held=held, params=state.params,
                           spheres=state.spheres, embodiment=state.embodiment)
    if box is not None:
        m = in_obb(pts, box)
        if int(m.sum()) < DEFINE_MIN_POINTS:
            raise PerceptionConflict(f"only {int(m.sum())} cloud points inside the box (the "
                                     "table and the robot are not counted)")
        sel, rgb = pts[m], col[m]
    else:
        sel = given.astype(np.float32)
        rgb = np.full((len(sel), 3), 128, np.uint8)
        if len(pts):
            d, j = cKDTree(pts).query(sel, distance_upper_bound=COLOR_FROM_CLOUD_M)
            ok = np.isfinite(d)
            rgb[ok] = col[j[ok]]
    return add_agent_object(state, sel, rgb, label=label, kind=kind, replace=tuple(replace),
                            made="define_object", held=held[0] if held else None, tracer=tracer)


def forget_object(state: PerceptionState, oid: str, *, held: str | None = None,
                  tracer: Tracer | None = None) -> None:
    """Drop an object; nothing is perceived where it was again."""
    if not isinstance(oid, str) or not oid:
        raise ValueError(f"id must be an object id, got {oid!r}")
    p = _current(state).copy()
    if held == oid:
        raise PerceptionConflict(f"{oid} is held; release it first")
    if oid not in p.objects:
        raise PerceptionConflict(f"unknown object {oid!r}; the scene has {sorted(p.objects)}")
    o = p.objects.pop(oid)
    p.lost.pop(oid, None)
    state.agent.pop(oid, None)
    pts = np.asarray(o.points, dtype=np.float64).reshape(-1, 3)
    if not len(pts):
        pts = np.asarray(o.verts, dtype=np.float64).reshape(-1, 3)
    state.forgotten.append({"id": oid, "obb": obb(pts),
                            "ext": np.sort(np.ptp(pts[:, :2], axis=0))})
    for k in [k for k, e in state.successors.items() if e["to"] == oid]:
        del state.successors[k]
    state.successors.pop(oid, None)
    p.notes.append(f"{oid} forgotten")
    _commit(state, p, held)
    if tracer is not None:
        tracer.event("server.perception.forget", {"id": oid})


def points_of(state: PerceptionState, oid: str, *, held: str | None = None) -> np.ndarray:
    """(N, 6) float32 x, y, z, r, g, b of a current object's points (resolve stale ids first)."""
    p = _current(state)
    o = p.objects.get(oid)
    if o is None:
        if held == oid:
            raise PerceptionConflict(f"{oid} is held: its points are not in the cloud")
        raise PerceptionConflict(f"unknown object {oid!r}; the scene has {sorted(p.objects)}")
    return object_points(o)
