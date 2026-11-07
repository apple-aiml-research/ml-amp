#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Gripper models of declared embodiments: the Robotiq linkage, or a linear parallel jaw."""

from __future__ import annotations

import math
import weakref

import numpy as np

from amp.geometry.collision.checker import gripper_link_names
from amp.geometry.embodiment import (
    ArmToolModel,
    Embodiment,
    joint_motion,
    mesh_arrays,
    parent_joints,
    pose44,
    surface_points,
)
from amp.geometry.gripper import PadSpan
from amp.geometry.gripper import profile as robotiq_profile
from amp.planning import robot
from amp.planning.scene_geom import (
    MARGIN,
    MAX_WIDTH,
    STROKE,
    GripperBody,
    GripperModel,
    default_gripper,
)

_GRID = 101
_VOXEL = 0.003
_SWEEP_STEP = 0.05
_SQUEEZE_M = 0.003
PALM_STEP = 0.002
FINGER_DEPTH, FINGER_Y, FINGER_Z = 0.015, 0.012, (-0.045, 0.012)
_MODELS: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()


def _box_points(lo, hi, step: float) -> np.ndarray:
    """Points on the surface of an axis-aligned box, no farther than step apart."""
    lo, hi = np.asarray(lo, dtype=np.float64), np.asarray(hi, dtype=np.float64)
    axes = [np.linspace(a, b, max(2, math.ceil((b - a) / step) + 1)) for a, b in zip(lo, hi)]
    G = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    on = np.any(np.isclose(G, lo) | np.isclose(G, hi), axis=1)
    return G[on]


class LinearProfile:
    """GripperProfile's interface for a parallel jaw whose pads translate: theta is the closure
    (0 open .. closed_angle = 1 closed), the gap stroke * (1 - theta)."""

    angular = False

    def __init__(self, stroke: float, depth: float, pad_y: float, pad_z, palms=()):
        self.stroke = float(stroke)
        self.closed_angle = 1.0
        self.theta = np.linspace(0.0, 1.0, _GRID)
        self.squeeze = min(1.0, _SQUEEZE_M / self.stroke)
        self._depth, self._y, self._z = float(depth), float(pad_y), (float(pad_z[0]),
                                                                     float(pad_z[1]))
        half = 0.5 * self.stroke
        lo = [(half, -pad_y, pad_z[0]), (-half - depth, -pad_y, pad_z[0])]
        hi = [(half + depth, pad_y, pad_z[1]), (-half, pad_y, pad_z[1])]
        parts = ["finger", "finger"] + ["palm"] * len(palms)
        lo += [p[0] for p in palms]
        hi += [p[1] for p in palms]
        lo, hi = np.array(lo, dtype=np.float64), np.array(hi, dtype=np.float64)
        span = [p[1][2] for p in palms if p[0][0] < 0.0 < p[1][0]]
        self.body = GripperBody(
            parts, lo, hi, pad_inner_x=half, pad_y=pad_y, pad_z=pad_z,
            grip_r=float(np.max(np.abs(np.vstack([lo[:, :2], hi[:, :2]])))),
            knuckle=(half + depth + MARGIN, pad_y + MARGIN), near_x=half - MARGIN - 1e-6,
            depth_z=max([pad_z[0], *span]))
        self._finger = _box_points((0.0, -pad_y, pad_z[0]), (depth, pad_y, pad_z[1]), _VOXEL)
        self._points: dict[int, np.ndarray] = {}
        self._coarse: dict[int, np.ndarray] = {}

    def _clip(self, theta):
        return np.clip(theta, 0.0, self.closed_angle)

    def gap(self, theta):
        return self.stroke * (1.0 - self._clip(theta))

    def theta_for_gap(self, w):
        return np.clip(1.0 - np.asarray(w, dtype=np.float64) / self.stroke, 0.0, 1.0)

    def tip_reach(self, theta):
        return np.zeros_like(np.asarray(theta, dtype=np.float64)) + self._z[1]

    def max_tip_reach(self, theta) -> float:
        return self._z[1]

    def pad_span(self, theta) -> PadSpan:
        g = 0.5 * float(self.gap(float(self._clip(theta))))
        return PadSpan(self._z[0], self._z[1], g, g + self._depth, self._y)

    def sweep_thetas(self, theta, step=_SWEEP_STEP) -> np.ndarray:
        th = float(self._clip(theta))
        n = max(1, math.ceil(th / step))
        return np.linspace(0.0, th, n + 1)

    def finger_points(self, theta, coarse: bool = False) -> np.ndarray:
        """(N,3) finger surface points in the TCP frame at the grid closure nearest theta."""
        k = int(np.argmin(np.abs(self.theta - self._clip(theta))))
        if k not in self._points:
            g = 0.5 * float(self.gap(self.theta[k]))
            left = self._finger + (g, 0.0, 0.0)
            right = left * (-1.0, 1.0, 1.0)
            self._points[k] = np.vstack([left, right]).astype(np.float32)
        if not coarse:
            return self._points[k]
        if k not in self._coarse:
            P = self._points[k].astype(np.float64)
            _, idx = np.unique(np.floor(P / _VOXEL).astype(np.int64), axis=0, return_index=True)
            self._coarse[k] = P[np.sort(idx)]
        return self._coarse[k]


def _tree_poses(e: Embodiment, values: dict) -> dict[str, np.ndarray]:
    """Every described link's pose at the arm's home and the given joint values (others 0)."""
    desc = e.description
    up = parent_joints(desc)
    arm = robot.plan_arm(e)
    known = dict(zip(arm.link_names, arm.fk_links(arm.q_home)))
    out: dict[str, np.ndarray] = {desc.root: np.eye(4)}

    def pose(link):
        if link not in out:
            if link in known:
                out[link] = known[link]
            else:
                j = up[link]
                out[link] = pose(j.parent) @ joint_motion(j, float(values.get(j.name, 0.0)))
        return out[link]

    for lk in desc.links:
        pose(lk.name)
    return out


def _subtree(desc, link: str, stop=()) -> list[str]:
    children: dict[str, list[str]] = {}
    for j in desc.joints:
        children.setdefault(j.parent, []).append(j.child)
    out, todo = [], [link]
    while todo:
        lk = todo.pop()
        if lk in stop or lk in out:
            continue
        out.append(lk)
        todo += children.get(lk, [])
    return out


def _mesh_points(e: Embodiment, links, poses, T_tcp, step: float | None = None) -> np.ndarray:
    """Vertices (and with step, surface points no farther apart) of the links' meshes, TCP
    frame."""
    meshes = {lk.name: lk.meshes for lk in e.description.links}
    inv = np.linalg.inv(T_tcp)
    pts = []
    for name in links:
        T = inv @ poses[name]
        for m in meshes.get(name, ()):
            V, F = mesh_arrays(e.blobs, m)
            V = V.astype(np.float64) if step is None else surface_points(V, F, step)
            pts.append(V @ T[:3, :3].T + T[:3, 3])
    return np.vstack(pts) if pts else np.empty((0, 3))


def _palm_cut(P: np.ndarray, below: float) -> list:
    """One box of conservative points (spheres), cut at the pads' top where the finger roots
    enter it; none when nothing lies behind the pads."""
    if not len(P) or P[:, 2].min() >= below:
        return []
    return [(P.min(0), np.minimum(P.max(0), [np.inf, np.inf, below]))]


def _palm_slabs(P: np.ndarray, inner: float) -> list:
    """The hand's surface as a box per slab across the closing axis: between the pads
    (|x| < inner) and beside them, each as deep as the hand truly reaches there."""
    out = []
    for m in (P[:, 0] <= -inner, np.abs(P[:, 0]) < inner, P[:, 0] >= inner):
        if m.any():
            out.append((P[m].min(0), P[m].max(0)))
    return out


def _linear(e: Embodiment) -> GripperModel:
    ee = e.ee()
    g = ee.gripper
    arm = robot.plan_arm(e)
    T_tcp = arm.fk_flange(arm.q_home) @ pose44(ee.T_flange_ee) @ pose44(ee.T_ee_tcp)
    Rc = canonical_rotation(ee)
    if Rc is not None:
        T_tcp = T_tcp @ Rc
    half = 0.5 * g.stroke_m
    have_mesh = e.description is not None and e.blobs is not None
    fingers, F, poses = [], np.empty((0, 3)), {}
    if have_mesh and g.finger_joints:
        by_joint = {j.name: j.child for j in e.description.joints}
        poses = _tree_poses(e, dict(zip(g.finger_joints, g.finger_open)))
        fingers = [lk for f in g.finger_joints if f in by_joint
                   for lk in _subtree(e.description, by_joint[f])]
        F = _mesh_points(e, fingers, poses, T_tcp)
    if fingers and len(F):
        depth = max(float(np.abs(F[:, 0]).max()) - half, FINGER_DEPTH / 3)
        pad_y = float(np.abs(F[:, 1]).max())
        pad_z = (float(F[:, 2].min()), float(F[:, 2].max()))
        rest = sorted(gripper_link_names(e, arm) - set(fingers))
        palms = _palm_slabs(_mesh_points(e, rest, poses, T_tcp, PALM_STEP), half)
    else:
        depth, pad_y, pad_z = FINGER_DEPTH, FINGER_Y, FINGER_Z
        palms = _palm_cut(_sphere_points(e, T_tcp), pad_z[0])
    prof = LinearProfile(g.stroke_m, depth, pad_y, pad_z, palms)
    return GripperModel(prof, stroke=g.stroke_m, max_width=g.max_width_m)


def _sphere_points(e: Embodiment, T_tcp) -> np.ndarray:
    """The end effector's collision spheres' extreme points in the TCP frame."""
    from amp.geometry.collision.checker import gripper_links

    try:
        sp = e.spheres()
    except ValueError:
        return np.empty((0, 3))
    arm = robot.plan_arm(e)
    names = gripper_links(sp, e)
    sel = np.array([n in names for n in sp.link_names])[sp.link]
    F = sp.frames_batch(arm.q_home)[0][sp.frame[sel]]
    W = np.einsum("sij,sj->si", F[:, :3, :3], sp.local[sel]) + F[:, :3, 3]
    inv = np.linalg.inv(T_tcp)
    C = W @ inv[:3, :3].T + inv[:3, 3]
    r = sp.radius[sel, None]
    return np.vstack([C - r, C + r])


def gripper_model(embodiment: Embodiment | None) -> GripperModel:
    """The GripperModel of the embodiment's first end effector (built once per embodiment)."""
    e = robot.resolve(embodiment)
    if e in _MODELS:
        return _MODELS[e]
    ee = e.ee()
    g = ee.gripper
    if g is None:
        raise ValueError(f"{e.name}: end effector {ee.name} has no gripper: it cannot grasp")
    if g.kind != "parallel_jaw":
        raise ValueError(f"{e.name}: end effector {ee.name} has no gripper to grasp with")
    canonical_rotation(ee, e.name)
    if g.model == "robotiq_2f85_linkage":
        if (g.stroke_m, g.max_width_m) == (STROKE, MAX_WIDTH):
            model = default_gripper()
        else:
            model = GripperModel(robotiq_profile(), stroke=g.stroke_m, max_width=g.max_width_m)
    else:
        model = _linear(e)
    _MODELS[e] = model
    return model


def canonical_rotation(ee, robot_name: str = "") -> np.ndarray | None:
    """The planner's TCP frame (x closing, z approach) in ee's TCP frame, None when they agree."""
    axes = (ee.closing_axis, ee.approach_axis)
    if axes == ("x", "z"):
        return None
    if axes != ("y", "z"):
        who = f"{robot_name}: " if robot_name else ""
        raise ValueError(f"{who}grasp planning needs a TCP that approaches along z and closes "
                         f"along x or y, {ee.name} declares {ee.closing_axis} and "
                         f"{ee.approach_axis}")
    Rc = np.eye(4)
    Rc[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    return Rc


def canonical_tool(tool, Rc: np.ndarray) -> ArmToolModel:
    """tool with its TCP turned by Rc into the planner's frame: the same arm and calibration."""
    out = ArmToolModel(robot.arm_of(tool), tool.T_flange_ee, tool.T_ee_tcp @ Rc, finger_axis="x")
    out.calibrated, out.canonical = tool.calibrated, True
    return out
