#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Collision spheres keyed by link name: the fitted DROID model, or a cover of a chain's meshes."""

from __future__ import annotations

import itertools
import math
import threading
from collections.abc import Mapping

import numpy as np

from amp.contracts.embodiment import RobotDescription
from amp.geometry.embodiment.arm import ChainArm
from amp.geometry.embodiment.description import (
    description_sha256,
    joint_motion,
    mesh_arrays,
    parent_joints,
)

COVER_CELL = 0.04
GRIPPER_CELL = 0.01
_MAX_POSES = 64
_COVER_CACHE: dict[tuple, dict[str, tuple[str, np.ndarray]]] = {}
_COVER_LOCK = threading.RLock()


def surface_points(verts: np.ndarray, faces: np.ndarray, step: float) -> np.ndarray:
    """The vertices and a barycentric grid on every face, no surface point farther than step."""
    V = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    F = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    if not len(F):
        return V
    A, B, C = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    edge = np.max([np.linalg.norm(B - A, axis=1), np.linalg.norm(C - B, axis=1),
                   np.linalg.norm(A - C, axis=1)], axis=0)
    n = np.maximum(1, np.ceil(edge / step)).astype(np.int64)
    out = [V]
    for k in np.unique(n[n > 1]):
        sel = n == k
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing="ij")
        keep = i + j <= k
        u, v = i[keep] / k, j[keep] / k
        w = 1.0 - u - v
        P = (w[None, :, None] * A[sel, None] + u[None, :, None] * B[sel, None]
             + v[None, :, None] * C[sel, None])
        out.append(P.reshape(-1, 3))
    return np.vstack(out)


def cover_points(P: np.ndarray, cell: float, margin: float) -> np.ndarray:
    """(n,4) spheres, one per occupied grid cell, each holding its cell's points plus margin."""
    key = np.floor(P / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.ravel()
    m = inv.max() + 1
    cnt = np.bincount(inv, minlength=m).astype(np.float64)
    c = np.stack([np.bincount(inv, P[:, k], minlength=m) for k in range(3)], axis=1) / cnt[:, None]
    r = np.zeros(m)
    np.maximum.at(r, inv, np.linalg.norm(P - c[inv], axis=1))
    return np.hstack([c, (r + margin)[:, None]])


def _range(j) -> float:
    return max(0.0, j.limits[1] - j.limits[0]) if j.type != "fixed" else 0.0


def _sweep(chain, P: np.ndarray, step: float) -> tuple[list[np.ndarray], float]:
    """Poses of the link in its frame link's frame over the chain's joint limits, and how far any
    point of P can lie from its nearest sampled position (the margin the sampling costs)."""
    moving = [i for i, j in enumerate(chain) if _range(j) > 0.0]
    reach = float(np.linalg.norm(P, axis=1).max()) if len(P) else 0.0
    lever = []
    for i in moving:
        down = sum(float(np.linalg.norm(np.array(j.origin).reshape(4, 4)[:3, 3])) + (
            max(abs(j.limits[0]), abs(j.limits[1])) if j.type == "prismatic" else 0.0)
            for j in chain[i + 1:])
        lever.append(1.0 if chain[i].type == "prismatic" else down + reach)
    k_max = max(2, int(_MAX_POSES ** (1.0 / max(1, len(moving)))))
    samples, slack = {}, 0.0
    for i, lv in zip(moving, lever):
        j = chain[i]
        travel = _range(j) * lv
        k = int(min(k_max, max(2, math.ceil(travel * len(moving) / step) + 1)))
        samples[i] = np.linspace(j.limits[0], j.limits[1], k)
        slack += 0.5 * travel / (k - 1)
    poses = []
    for xs in itertools.product(*(samples.get(i, (0.0,)) for i in range(len(chain)))):
        T = np.eye(4)
        for j, x in zip(chain, xs):
            T = T @ joint_motion(j, float(x))
        poses.append(T)
    return poses, slack


def _attachment(description: RobotDescription, arm: ChainArm, link: str, other_joints: set):
    """(frame link, joints from it down to link), or None if another arm's joint moves the link."""
    up = parent_joints(description)
    path = set(arm.link_names)
    chain = []
    while link not in path and link != description.root:
        j = up[link]
        if j.name in other_joints:
            return None
        chain.append(j)
        link = j.parent
    return link, chain[::-1]


def link_cover(description: RobotDescription, blobs: Mapping, arm: ChainArm,
               arm_joints=(), cell: float = COVER_CELL) -> dict[str, tuple[str, np.ndarray]]:
    """Per described link with meshes: (frame link, (n,4) spheres in that frame), cached by sha."""
    key = (description_sha256(description, blobs), arm.name, tuple(sorted(arm_joints)), cell,
           GRIPPER_CELL)
    with _COVER_LOCK:
        if key not in _COVER_CACHE:
            _COVER_CACHE[key] = _link_cover(description, blobs, arm, arm_joints, cell)
        return _COVER_CACHE[key]


def _link_cover(description, blobs, arm, arm_joints, cell):
    out = {}
    other = set(arm_joints) - set(arm.joints)
    path, flange = set(arm.link_names), arm.link_names[-1]
    for lk in description.links:
        if not lk.meshes:
            continue
        att = _attachment(description, arm, lk.name, other)
        if att is None:
            continue
        frame, chain = att
        c = min(cell, GRIPPER_CELL) if frame == flange and lk.name not in path else cell
        step = c / 4.0
        local = np.vstack([surface_points(*mesh_arrays(blobs, m), step) for m in lk.meshes])
        poses, slack = _sweep(chain, local, step)
        P = np.vstack([local @ T[:3, :3].T + T[:3, 3] for T in poses])
        out[lk.name] = (frame, cover_points(P, c, step + slack))
    return out


def body_cover(description: RobotDescription, blobs: Mapping, q_joints, skip=(),
               cell: float = COVER_CELL) -> dict[str, tuple[str, np.ndarray]]:
    """Per described link with meshes, not in skip: (frame link, (n,4) spheres in its frame), the
    frame the link's nearest ancestor (or itself) a q joint moves, else the root; cached by sha."""
    q_joints, skip = frozenset(q_joints), frozenset(skip)
    key = ("body", description_sha256(description, blobs), tuple(sorted(q_joints)),
           tuple(sorted(skip)), cell)
    with _COVER_LOCK:
        if key not in _COVER_CACHE:
            _COVER_CACHE[key] = _body_cover(description, blobs, q_joints, skip, cell)
        return _COVER_CACHE[key]


def _body_cover(description, blobs, q_joints, skip, cell):
    up = parent_joints(description)
    out = {}
    for lk in description.links:
        if not lk.meshes or lk.name in skip:
            continue
        frame, chain = lk.name, []
        while frame != description.root and up[frame].name not in q_joints:
            chain.append(up[frame])
            frame = up[frame].parent
        step = cell / 4.0
        local = np.vstack([surface_points(*mesh_arrays(blobs, m), step) for m in lk.meshes])
        poses, slack = _sweep(chain[::-1], local, step)
        P = np.vstack([local @ T[:3, :3].T + T[:3, 3] for T in poses])
        out[lk.name] = (frame, cover_points(P, cell, step + slack))
    return out


class LinkSpheres:
    """RobotSpheres' layout (link_names, link_frames, frame, local, radius, link) on a chain arm."""

    def __init__(self, arm: ChainArm, links: Mapping[str, tuple[str, np.ndarray]],
                 source: str = ""):
        self.arm = arm
        self.source = source
        index = {name: i + 1 for i, name in enumerate(arm.link_names)}
        self.link_names: list[str] = []
        self.link_frames: list[str] = []
        frame, local, radius, link = [], [], [], []
        for name, (frame_name, spheres) in links.items():
            s = np.asarray(spheres, dtype=np.float64).reshape(-1, 4)
            idx = len(self.link_names)
            self.link_names.append(name)
            self.link_frames.append(frame_name)
            frame += [index.get(frame_name, 0)] * len(s)
            local.append(s[:, :3])
            radius.append(s[:, 3])
            link += [idx] * len(s)
        if not frame:
            raise ValueError("a sphere model needs at least one sphere (the links have no meshes)")
        self.frame = np.array(frame, dtype=np.int64)
        self.local = np.concatenate(local)
        self.radius = np.concatenate(radius)
        self.link = np.array(link, dtype=np.int64)

    def __len__(self) -> int:
        return len(self.radius)

    def spheres(self, name: str) -> np.ndarray:
        """(n,4) spheres of a link in its frame link's frame."""
        sel = self.link == self.link_names.index(name)
        return np.hstack([self.local[sel], self.radius[sel, None]])

    def frames_batch(self, qs) -> np.ndarray:
        """(N,L+1,4,4): the root, then the arm's link frames."""
        links = self.arm.fk_links_batch(qs)
        out = np.empty((len(links), links.shape[1] + 1, 4, 4))
        out[:, 0] = np.eye(4)
        out[:, 1:] = links
        return out

    def world_batch(self, qs, T_flange_ee=None) -> np.ndarray:
        """Sphere centres (N,S,3) in the robot-root frame; the description poses every link, so
        T_flange_ee is accepted for RobotSpheres' signature and unused."""
        F = self.frames_batch(qs)[:, self.frame]
        return np.einsum("nsij,sj->nsi", F[..., :3, :3], self.local) + F[..., :3, 3]

    def world(self, q, T_flange_ee=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(centres (S,3), radii (S,), link index (S,)) at one configuration."""
        return self.world_batch(q)[0], self.radius.copy(), self.link.copy()
