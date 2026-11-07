#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Point clouds for perception: fused from depth frames, cropped, and cleaned of table and robot."""

from __future__ import annotations

import itertools
from collections.abc import Mapping

import numpy as np
from scipy.spatial import cKDTree

from amp.geometry.collision import RobotSpheres, mesh_field
from amp.geometry.embodiment import Embodiment, LinkSpheres
from amp.link.obs import CameraFrame
from amp.link.pointcloud import CameraModel, backproject
from amp.perception.params import (
    PerceptionParams,
    TableRegion,
    embodiment_or_droid,
    params_for,
    table_region,
    workspace_bounds,
)

CLOUD_STRIDE = 1
CLOUD_MAX_DEPTH_M = 2.0
CLOUD_FRAMES = ("root", "world")
Held = tuple[str, np.ndarray, np.ndarray]


def crop_workspace(xyz, rgb, bounds=None, *,
                   embodiment: Embodiment | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Keep the points inside the axis-aligned workspace box (None = the embodiment's)."""
    if bounds is None:
        bounds = workspace_bounds(embodiment_or_droid(embodiment).spec.workspace)
    xyz = np.asarray(xyz)
    keep = in_box(xyz, bounds)
    return xyz[keep], np.asarray(rgb)[keep]


def in_box(xyz, bounds) -> np.ndarray:
    """Mask of the points (N, 3) inside the axis-aligned box ((x0, x1), (y0, y1), (z0, z1))."""
    xyz = np.asarray(xyz)
    keep = np.ones(xyz.shape[0], bool)
    for axis, (lo, hi) in enumerate(bounds):
        keep &= (xyz[:, axis] >= lo) & (xyz[:, axis] <= hi)
    return keep


def transform(xyz, T) -> np.ndarray:
    """Points (N, 3) mapped through the 4x4 pose T, in float64."""
    T = np.asarray(T, np.float64)
    return np.asarray(xyz, np.float64).reshape(-1, 3) @ T[:3, :3].T + T[:3, 3]


def workspace_reach(T_world_cam, workspace) -> float:
    """The farthest the workspace box gets from a camera: no point inside it is deeper."""
    corners = np.array(list(itertools.product(*workspace)), dtype=np.float64)
    origin = np.asarray(T_world_cam, dtype=np.float64)[:3, 3]
    return float(np.linalg.norm(corners - origin, axis=1).max())


def depth_limit(name: str, T_world_cam, workspace, max_depth: float | None, crop: bool,
                spec) -> float:
    """The depth cutoff of camera `name`: max_depth if given; else CLOUD_MAX_DEPTH_M, extended
    to the workspace's reach (T_world_cam in the box's frame) for a body camera cropped to it."""
    if max_depth is not None:
        return float(max_depth)
    if crop and name in body_cameras(spec):
        return max(CLOUD_MAX_DEPTH_M, workspace_reach(T_world_cam, workspace))
    return CLOUD_MAX_DEPTH_M


def body_cameras(spec) -> set[str]:
    """The cameras cut at the workspace reach: static ones, and a based robot's non-wrist ones."""
    return {c.name for c in spec.cameras
            if c.mount == "root" or (spec.base is not None and c.role != "wrist")}


def fuse_frames(frames: Mapping[str, CameraFrame], *, stride: int = CLOUD_STRIDE,
                max_depth: float | None = None, crop: bool = True, workspace=None,
                embodiment: Embodiment | None = None, T_world_base=None, frame: str = "root"):
    """(xyz float32 (N, 3), rgb uint8 (N, 3), {cam: n}) of frames in `frame`, cropped."""
    spec = embodiment_or_droid(embodiment).spec
    if frame not in CLOUD_FRAMES:
        raise ValueError(f"cloud frame {frame!r} not in {CLOUD_FRAMES}")
    if workspace is None:
        workspace = workspace_bounds(spec.workspace)
    box_frame = spec.workspace.frame
    Tb = None if T_world_base is None else np.asarray(T_world_base, np.float64).reshape(4, 4)
    if Tb is None and (frame == "world" or (crop and box_frame == "world")):
        what = "a world cloud" if frame == "world" else "a world-frame workspace crop"
        raise ValueError(f"{what} needs the frame's T_world_base")
    if Tb is not None and not np.isfinite(Tb).all():
        raise ValueError("T_world_base must be a finite 4x4 pose")
    xyzs, rgbs, per_cam = [], [], {}
    for name, fr in frames.items():
        if fr.depth is None or fr.depth_model is None:
            continue
        T = fr.depth_model.T_world_cam
        T_out = T if frame == "root" else Tb @ T
        T_box = T if box_frame == "root" or not crop else Tb @ T
        xyz, rgb = backproject(fr.depth, fr.depth_model.K, T_out, fr.rgb_small, stride=stride,
                               max_depth=depth_limit(name, T_box, workspace, max_depth, crop,
                                                     spec))
        if crop and box_frame == frame:
            xyz, rgb = crop_workspace(xyz, rgb, workspace)
        elif crop:
            to_box = Tb if frame == "root" else np.linalg.inv(Tb)
            keep = in_box(transform(xyz, to_box), workspace)
            xyz, rgb = xyz[keep], np.asarray(rgb)[keep]
        per_cam[name] = len(xyz)
        xyzs.append(np.asarray(xyz, dtype=np.float32).reshape(-1, 3))
        rgbs.append(np.asarray(rgb, dtype=np.uint8).reshape(-1, 3))
    xyz = np.concatenate(xyzs) if xyzs else np.zeros((0, 3), np.float32)
    rgb = np.concatenate(rgbs) if rgbs else np.zeros((0, 3), np.uint8)
    return xyz, rgb, per_cam


def depth_cams(frames: Mapping[str, CameraFrame]) -> tuple[CameraModel, ...]:
    """The depth camera models of the frames that carry depth."""
    return tuple(fr.depth_model for fr in frames.values()
                 if fr.depth is not None and fr.depth_model is not None)


def estimate_table_z(xyz, boxes=(), region: TableRegion | None = None, *,
                     embodiment: Embodiment | None = None) -> tuple[float, int] | None:
    """(table z, n points): the mode of z over the table region, ignoring points near boxes."""
    t = table_region(embodiment_or_droid(embodiment).spec.workspace) if region is None else region
    if not t.search:
        return None
    p = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    keep = ((p[:, 0] >= t.x[0]) & (p[:, 0] <= t.x[1])
            & (np.abs(p[:, 1]) < t.y_abs)
            & (p[:, 2] >= t.z_window[0]) & (p[:, 2] <= t.z_window[1]))
    for lo, hi in boxes:
        lo = np.asarray(lo, float) - t.object_margin
        hi = np.asarray(hi, float) + t.object_margin
        keep &= ~((p >= lo) & (p <= hi)).all(axis=1)
    z = p[keep, 2]
    if len(z) < t.min_points:
        return None
    bins = np.arange(t.z_window[0], t.z_window[1] + 0.001, 0.001)
    hist, edges = np.histogram(z, bins=bins)
    smooth = np.convolve(hist, np.ones(3), mode="same")
    i = int(np.argmax(smooth))
    centre = 0.5 * (edges[i] + edges[i + 1])
    near = z[np.abs(z - centre) <= 0.003]
    zt = float(np.median(near)) if len(near) else centre
    return float(np.clip(zt, *t.z_clamp)), len(z)


def flange_ee(embodiment: Embodiment, arm: str, T_flange_ee=None) -> np.ndarray:
    """The arm's T_flange_ee from a mapping, an array, or the embodiment's tool."""
    ees = [e.name for e in embodiment.spec.end_effectors if e.arm == arm]
    if isinstance(T_flange_ee, Mapping):
        T = next((T_flange_ee[e] for e in ees if e in T_flange_ee), None)
    else:
        T = T_flange_ee if arm == embodiment.spec.arms[0].name else None
    if T is None:
        return embodiment.tool(ees[0]).T_flange_ee if ees else np.eye(4)
    return np.asarray(T, np.float64)


def robot_spheres(q, T_flange_ee=None, model: RobotSpheres | LinkSpheres | None = None, *,
                  embodiment: Embodiment | None = None):
    """(centres (S, 3), radii (S,)) of every robot sphere at q, the static base's included."""
    emb = embodiment_or_droid(embodiment)
    q = np.asarray(q, np.float64).reshape(-1)
    if model is None and emb.whole_body:
        if len(q) < emb.dof:
            raise ValueError(f"{emb.name}: the self-filter needs the full q ({emb.dof} values)")
        centres, radii, _ = emb.whole_spheres(q[:emb.dof])
        return centres, radii
    arms = emb.spec.arms
    if len(arms) == 1:
        qs = {arms[0].name: q[:len(arms[0].joints)]}
    else:
        qs = emb.split(q[:emb.dof])
    centres, radii = [], []
    for i, a in enumerate(arms):
        m = model if (i == 0 and model is not None) else emb.spheres(a.name)
        T = flange_ee(emb, a.name, T_flange_ee)
        centres.append(m.world_batch(qs[a.name][None], T)[0])
        radii.append(m.radius)
    if len(arms) == 1:
        return centres[0], radii[0]
    return np.concatenate(centres), np.concatenate(radii)


def remove_robot(pts: np.ndarray, spheres, inflate: float) -> np.ndarray:
    """Mask of points outside every inflated robot sphere."""
    centres, radius = spheres
    keep = np.ones(len(pts), bool)
    if not len(pts):
        return keep
    hits = cKDTree(pts).query_ball_point(centres, radius + inflate)
    idx = [h for h in hits if h]
    if idx:
        keep[np.unique(np.concatenate(idx))] = False
    return keep


def robot_remnant(pts: np.ndarray, spheres, table_z: float, p: PerceptionParams) -> bool:
    """A segment off the table lying wholly within remnant_dist of the robot: arm, not object."""
    if float(pts[:, 2].min()) < table_z + p.remnant_lift:
        return False
    centres, radius = spheres

    def hugs(sub):
        d = np.linalg.norm(sub[:, None, :] - centres[None], axis=2) - radius[None]
        return bool(d.min(axis=1).max() < p.remnant_dist)

    pts = pts.astype(np.float64)
    return hugs(pts[::max(1, len(pts) // 24)]) and hugs(pts[::max(1, len(pts) // 1500)])


def remove_held(pts: np.ndarray, verts, faces, margin: float) -> np.ndarray:
    """Mask of points neither within margin of nor inside the held object's mesh."""
    keep = np.ones(len(pts), bool)
    v = np.asarray(verts, np.float64).reshape(-1, 3)
    if not len(v) or not len(pts):
        return keep
    lo, hi = v.min(axis=0) - margin, v.max(axis=0) + margin
    near = np.flatnonzero(((pts >= lo) & (pts <= hi)).all(axis=1))
    if not len(near):
        return keep
    fld = mesh_field(v, np.asarray(faces, np.int64).reshape(-1, 3))
    q = pts[near].astype(np.float64)
    d = fld.tree.query(q, distance_upper_bound=margin)[0]
    gone = d < margin
    rest = np.flatnonzero(~gone)
    if len(rest) and len(fld.tree.data) and np.asarray(faces).size:
        gone[rest] = fld.inside(q[rest])
    keep[near[gone]] = False
    return keep


def default_rgb(n: int, rgb, grey: int = 180) -> np.ndarray:
    """rgb as (n, 3) uint8, or a uniform grey without colour."""
    if rgb is None:
        return np.full((n, 3), grey, np.uint8)
    return np.asarray(rgb, np.uint8).reshape(-1, 3)


def clean_cloud(xyz, rgb, table_z: float, *, q=None, T_flange_ee=None, held: Held | None = None,
                params: PerceptionParams | None = None,
                spheres: RobotSpheres | LinkSpheres | None = None,
                embodiment: Embodiment | None = None):
    """(points, colours) of a fused cloud with the table, the robot and the held object removed."""
    p = params_for(embodiment) if params is None else params
    xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
    rgb = default_rgb(len(xyz), rgb)
    keep = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > table_z + p.table_margin)
    pts, col = xyz[keep], rgb[keep]
    if q is not None and len(pts):
        k = remove_robot(pts, robot_spheres(q, T_flange_ee, spheres, embodiment=embodiment),
                         p.robot_inflate)
        pts, col = pts[k], col[k]
    if held is not None and len(pts):
        k = remove_held(pts, held[1], held[2], p.held_margin)
        pts, col = pts[k], col[k]
    return pts, col
