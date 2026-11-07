#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What grasp and place search share: object samples, obstacles, gripper body, IK."""

from __future__ import annotations

import functools
import hashlib
import math
import threading
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from amp.geometry.gripper import GripperProfile
from amp.geometry.gripper import profile as gripper_profile
from amp.planning.poses import to_tcp_frame, unique_in_order

__all__ = [
    "BODY_CEN",
    "BODY_HI",
    "BODY_LO",
    "BODY_PARTS",
    "CLEAR_CAP",
    "MARGIN",
    "MAX_WIDTH",
    "MESH_INFO",
    "MIN_WIDTH",
    "PAD_INNER_X",
    "PAD_Y",
    "PAD_Z",
    "POINT_DROP",
    "ROBOTIQ_BODY",
    "STROKE",
    "ArmOk",
    "GripperBody",
    "GripperModel",
    "MeshInfoCache",
    "ObjectGeom",
    "ObjectShape",
    "Obstacles",
    "SceneContext",
    "body_of",
    "default_gripper",
    "gripper_contacts",
    "gripper_for",
    "lowest_point",
    "table_note",
]

ArmOk = Callable[[np.ndarray], bool]
STROKE = 0.085
MAX_WIDTH = 0.080
MIN_WIDTH = 0.005
MARGIN = 0.003
CLEAR_CAP = 0.03
POINT_DROP = 0.01
PAD_INNER_X = 0.042
PAD_Y = 0.011
PAD_Z = (-0.023, 0.015)
_SAMPLE_AREA = 9e-6
_MIN_SAMPLES, _MAX_SAMPLES, _MAX_VERTS = 300, 3000, 4000
_GRID_CELL = 0.02
_FINGER_PARTS = (
    ("fingertip pad", 0.042, 0.062, 0.011, -0.023, 0.015),
    ("inner finger", 0.044, 0.074, 0.014, -0.042, -0.014),
    ("inner knuckle", 0.007, 0.056, 0.018, -0.079, -0.023),
    ("outer finger", 0.048, 0.076, 0.014, -0.089, -0.030),
    ("outer knuckle", 0.023, 0.068, 0.008, -0.089, -0.069),
)
_BODY_PARTS = (("palm", 0.042, 0.038, -0.138, -0.044), ("wrist", 0.06, 0.06, -0.26, -0.138))


def _gripper_boxes() -> tuple[list[str], np.ndarray, np.ndarray]:
    names, lo, hi = [], [], []
    for name, x0, x1, y, z0, z1 in _FINGER_PARTS:
        for sx in (1.0, -1.0):
            xs = sorted((sx * x0, sx * x1))
            names.append(name)
            lo.append((xs[0], -y, z0))
            hi.append((xs[1], y, z1))
    for name, x, y, z0, z1 in _BODY_PARTS:
        names.append(name)
        lo.append((-x, -y, z0))
        hi.append((x, y, z1))
    return names, np.array(lo), np.array(hi)


class GripperBody:
    """The open gripper as named boxes in the TCP frame (x closing, z approach)."""

    def __init__(self, parts, lo, hi, *, pad_inner_x: float, pad_y: float, pad_z, grip_r: float,
                 knuckle: tuple[float, float], near_x: float, depth_z: float | None = None):
        self.parts = list(parts)
        self.lo, self.hi = np.asarray(lo, dtype=np.float64), np.asarray(hi, dtype=np.float64)
        self.pad_inner_x, self.pad_y = float(pad_inner_x), float(pad_y)
        self.pad_z = (float(pad_z[0]), float(pad_z[1]))
        self.grip_r = float(grip_r)
        self.grip_top = float(np.max(-self.lo[:, 2]))
        self.knuckle = (float(knuckle[0]), float(knuckle[1]))
        self.depth_limit = (self.pad_z[0] if depth_z is None else float(depth_z)) + MARGIN + 0.001
        self.cen, self.half = 0.5 * (self.lo + self.hi), 0.5 * (self.hi - self.lo)
        self.opart = [*self.parts, "space between the fingers"]
        self.olo = np.vstack([self.lo, [-pad_inner_x, -pad_y, self.pad_z[0]]])
        self.ohi = np.vstack([self.hi, [pad_inner_x, pad_y, self.pad_z[1]]])
        self.uc = 0.5 * (self.olo.min(0) + self.ohi.max(0))
        self.uh = 0.5 * (self.ohi.max(0) - self.olo.min(0))
        self.uball = float(np.linalg.norm(self.uh + CLEAR_CAP)) + 1e-6
        groups = [[j for j, n in enumerate(self.opart) if n not in ("palm", "wrist")]]
        groups += [[j for j, p in enumerate(self.opart) if p == n] for n in ("palm", "wrist")
                   if n in self.opart]
        self.gc = np.array([0.5 * (self.olo[g].min(0) + self.ohi[g].max(0)) for g in groups])
        self.gh = np.array([0.5 * (self.ohi[g].max(0) - self.olo[g].min(0)) for g in groups])
        self.tlo, self.thi = self.lo - MARGIN, self.hi + MARGIN
        self.tlo[0, 0], self.thi[1, 0] = pad_inner_x, -pad_inner_x
        self.tlo_min, self.thi_max = self.tlo.min(0), self.thi.max(0)
        straddle = (self.tlo[:, 0] <= 0) & (self.thi[:, 0] >= 0)
        xmin = np.where(straddle, 0.0, np.minimum(np.abs(self.tlo[:, 0]), np.abs(self.thi[:, 0])))
        near = xmin < near_x
        self.gap_z = float(self.thi[near, 2].max()) if near.any() else -math.inf
        self.gap_x = float(xmin[self.thi[:, 2] > self.gap_z].min())


ROBOTIQ_BODY = GripperBody(*_gripper_boxes(), pad_inner_x=PAD_INNER_X, pad_y=PAD_Y, pad_z=PAD_Z,
                           grip_r=0.06, knuckle=(0.056 + MARGIN, 0.018 + MARGIN), near_x=0.03)
BODY_PARTS, BODY_LO, BODY_HI = ROBOTIQ_BODY.parts, ROBOTIQ_BODY.lo, ROBOTIQ_BODY.hi
BODY_CEN, _HALF = ROBOTIQ_BODY.cen, ROBOTIQ_BODY.half
_OPART, _OLO, _OHI = ROBOTIQ_BODY.opart, ROBOTIQ_BODY.olo, ROBOTIQ_BODY.ohi
_UC, _UH, _UBALL = ROBOTIQ_BODY.uc, ROBOTIQ_BODY.uh, ROBOTIQ_BODY.uball


@dataclass
class _MeshInfo:
    tri: np.ndarray
    bary: np.ndarray
    vert_idx: np.ndarray
    closed: bool
    orient: float


def _is_closed(V: np.ndarray, F: np.ndarray) -> bool:
    """Every edge shared by exactly two triangles, after welding split seam vertices."""
    _, inv = np.unique(np.round(V / 1e-5).astype(np.int64), axis=0, return_inverse=True)
    W = inv.reshape(-1)[F]
    W = W[(W[:, 0] != W[:, 1]) & (W[:, 1] != W[:, 2]) & (W[:, 0] != W[:, 2])]
    if not len(W):
        return False
    e = np.sort(np.concatenate([W[:, [0, 1]], W[:, [1, 2]], W[:, [2, 0]]]), axis=1)
    _, counts = np.unique(e[:, 0] * (int(inv.max()) + 1) + e[:, 1], return_counts=True)
    return bool(np.all(counts == 2))


def _build_mesh_info(V: np.ndarray, F: np.ndarray, seed: int) -> _MeshInfo:
    rng = np.random.default_rng(seed)
    F = F[np.all((F >= 0) & (F < len(V)), axis=1)] if len(F) else F
    tri, bary = np.empty((0, 3), np.int64), np.empty((0, 3))
    closed, orient = False, 1.0
    if len(F):
        a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        total = float(area.sum())
        if total > 0:
            n = int(np.clip(round(total / _SAMPLE_AREA), _MIN_SAMPLES, _MAX_SAMPLES))
            tri = F[rng.choice(len(F), size=n, p=area / total)]
            s, r2 = np.sqrt(rng.random(n)), rng.random(n)
            bary = np.stack([1.0 - s, s * (1.0 - r2), s * r2], axis=1)
            orient = 1.0 if float(np.einsum("ij,ij->", a, np.cross(b, c))) >= 0 else -1.0
            closed = _is_closed(V, F)
    if len(V) <= _MAX_VERTS:
        vert_idx = np.arange(len(V))
    else:
        vert_idx = np.sort(rng.choice(len(V), size=_MAX_VERTS, replace=False))
    return _MeshInfo(tri, bary, vert_idx, closed, orient)


class MeshInfoCache:
    """An LRU of surface samplings keyed by mesh content and seed: searches revisit meshes."""

    def __init__(self, size: int = 64):
        self.size = int(size)
        self._infos: OrderedDict[bytes, _MeshInfo] = OrderedDict()
        self._lock = threading.Lock()

    def get(self, verts, faces, seed: int) -> _MeshInfo:
        V = np.ascontiguousarray(verts, dtype=np.float64).reshape(-1, 3)
        F = np.ascontiguousarray(faces, dtype=np.int64).reshape(-1, 3)
        h = hashlib.blake2b(digest_size=16)
        h.update(V.tobytes())
        h.update(F.tobytes())
        h.update(str(int(seed)).encode())
        key = h.digest()
        with self._lock:
            hit = self._infos.get(key)
            if hit is not None:
                self._infos.move_to_end(key)
                return hit
        info = _build_mesh_info(V, F, int(seed))
        with self._lock:
            self._infos[key] = info
            while len(self._infos) > self.size:
                self._infos.popitem(last=False)
        return info


MESH_INFO = MeshInfoCache()


def _mesh_info(verts, faces, seed: int) -> _MeshInfo:
    return MESH_INFO.get(verts, faces, seed)


class ObjectGeom:
    """One object's world-frame surface: area-weighted samples with normals, plus vertices."""

    def __init__(self, name, samples, normals, pts, closed):
        self.name = name
        self.samples, self.normals, self.pts, self.closed = samples, normals, pts, closed
        self.lo, self.hi = pts.min(0), pts.max(0)
        self._tree = None

    @classmethod
    def of(cls, name: str, obj, seed: int) -> ObjectGeom:
        info = _mesh_info(obj.verts, obj.faces, seed)
        W = np.asarray(obj.world_verts(), dtype=np.float64).reshape(-1, 3)
        if not len(W):
            raise ValueError(f"object {name!r} has no vertices")
        if len(info.tri):
            tri = W[info.tri]
            samples = np.einsum("nk,nkd->nd", info.bary, tri)
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]) * info.orient
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1), 1e-15)[:, None]
        else:
            samples, nrm = W[info.vert_idx], np.zeros((len(info.vert_idx), 3))
        return cls(name, samples, nrm, np.vstack([samples, W[info.vert_idx]]), info.closed)

    def moved(self, T) -> ObjectGeom:
        R, t = T[:3, :3], T[:3, 3]
        return ObjectGeom(
            self.name, self.samples @ R.T + t, self.normals @ R.T, self.pts @ R.T + t, self.closed
        )

    def inside(self, Q) -> np.ndarray:
        """Whether each point lies inside the (closed) mesh: a vote of the nearest normals."""
        Q = np.atleast_2d(Q)
        out = np.zeros(len(Q), dtype=bool)
        if not self.closed or len(self.samples) < 4:
            return out
        m = np.all((Q >= self.lo) & (Q <= self.hi), axis=1)
        if not m.any():
            return out
        if self._tree is None:
            self._tree = cKDTree(self.samples)
        k = min(7, len(self.samples))
        _, idx = self._tree.query(Q[m], k=k)
        d = Q[m][:, None, :] - self.samples[idx]
        votes = np.einsum("nkd,nkd->nk", d, self.normals[idx]) < 0
        out[m] = 2 * votes.sum(1) > k
        return out


class Obstacles:
    """Surface samples of every other object, with a KD-tree, owners and footprints."""

    def __init__(self, geoms: list[ObjectGeom]):
        self.geoms = geoms
        if geoms:
            self.pts = np.vstack([g.pts for g in geoms])
            self.owner = np.concatenate([np.full(len(g.pts), i) for i, g in enumerate(geoms)])
            self.tree = cKDTree(self.pts)
        else:
            self.pts, self.owner, self.tree = np.empty((0, 3)), np.empty(0, np.int64), None
        self.glo = np.array([g.lo for g in geoms]).reshape(-1, 3)
        self.ghi = np.array([g.hi for g in geoms]).reshape(-1, 3)
        self.solid = np.array([g.closed and len(g.samples) >= 4 for g in geoms], dtype=bool)
        self._grid = self._occ = self._rank = None

    def rank(self) -> np.ndarray:
        """Each point's position in the tree's order, which a single-point ball query returns."""
        if self._rank is None:
            r = np.empty(len(self.pts), np.int64)
            r[self.tree.indices] = np.arange(len(self.pts))
            self._rank = r
        return self._rank

    def near_xy(self, lo, hi) -> np.ndarray:
        """Indices (in no particular order) of a superset of the points with lo <= xy <= hi."""
        if self._grid is None:
            o = self.pts[:, :2].min(0)
            span = self.pts[:, :2].max(0) - o
            cell = max(_GRID_CELL, math.sqrt(float(np.prod(span + _GRID_CELL)) / 4e6))
            k = np.floor((self.pts[:, :2] - o) / cell).astype(np.int64)
            n = k.max(0) + 1
            key = k[:, 0] * n[1] + k[:, 1]
            order = np.argsort(key, kind="stable")
            starts = np.searchsorted(key[order], np.arange(n[0] * n[1] + 1))
            self._grid = (o, cell, n, order, starts)
        o, cell, n, order, starts = self._grid
        a = np.maximum(np.floor((np.asarray(lo, dtype=np.float64) - o) / cell), 0)
        b = np.minimum(np.floor((np.asarray(hi, dtype=np.float64) - o) / cell), n - 1)
        if not (a <= b).all():
            return np.empty(0, np.int64)
        a, b = a.astype(np.int64), b.astype(np.int64)
        rows = np.arange(a[0], b[0] + 1) * n[1]
        s, e = starts[rows + a[1]], starts[rows + b[1] + 1]
        if len(rows) == 1:
            return order[s[0]:e[0]]
        return np.concatenate([order[i:j] for i, j in zip(s.tolist(), e.tolist())])

    def near_mask(self, Q) -> np.ndarray:
        """A superset of the points of Q within CLEAR_CAP of an obstacle point."""
        if self._occ is None:
            cell = 0.5 * CLEAR_CAP * (1.0 + 1e-6)
            o = self.pts.min(0) - 3 * cell
            k = np.floor((self.pts - o) / cell).astype(np.int64)
            n = k.max(0) + 4
            if float(np.prod(n.astype(np.float64))) > 3e7:
                self._occ = (None, cell, None)
            else:
                g = np.zeros(tuple(n), dtype=bool)
                g[k[:, 0], k[:, 1], k[:, 2]] = True
                for ax in range(3):
                    for _ in range(2):
                        s = g.copy()
                        idx = [slice(None)] * 3
                        idx[ax] = slice(1, None)
                        jdx = [slice(None)] * 3
                        jdx[ax] = slice(None, -1)
                        s[tuple(idx)] |= g[tuple(jdx)]
                        s[tuple(jdx)] |= g[tuple(idx)]
                        g = s
                self._occ = (o, cell, g)
        o, cell, g = self._occ
        Q = np.atleast_2d(Q)
        if g is None:
            return np.ones(len(Q), dtype=bool)
        k = np.floor((Q - o) / cell)
        ok = np.all((k >= 0) & (k < g.shape), axis=1)
        out = np.zeros(len(Q), dtype=bool)
        ki = k[ok].astype(np.int64)
        out[ok] = g[ki[:, 0], ki[:, 1], ki[:, 2]]
        return out

    def solid_under(self, Q) -> np.ndarray:
        """Indices of the solid geoms whose AABB holds any point of Q (the only ones whose
        `inside` can be true for them)."""
        m = np.all((Q[None] >= self.glo[:, None]) & (Q[None] <= self.ghi[:, None]), axis=2)
        return np.flatnonzero(m.any(1) & self.solid)


@dataclass
class ObjectShape:
    """An object's principal axes and extents, as grasp scoring sees them."""

    centroid: np.ndarray
    u1: np.ndarray
    L1: float
    elongated: bool
    v1: np.ndarray
    Lxy: float
    zmin: float
    zmax: float


def _shape(g: ObjectGeom) -> ObjectShape:
    S = g.samples if len(g.samples) >= 4 else g.pts
    c = S.mean(0)
    D = S - c
    _, vec = np.linalg.eigh(D.T @ D)
    u = vec[:, ::-1]
    ext = np.ptp(D @ u, axis=0)
    _, v2 = np.linalg.eigh(D[:, :2].T @ D[:, :2])
    v1 = np.array([v2[0, 1], v2[1, 1], 0.0])
    return ObjectShape(
        centroid=c,
        u1=u[:, 0],
        L1=float(ext[0]),
        elongated=bool(ext[0] >= 1.5 * max(ext[1], 1e-6)),
        v1=v1,
        Lxy=float(np.ptp(D[:, :2] @ v1[:2])),
        zmin=float(g.lo[2]),
        zmax=float(g.hi[2]),
    )


@dataclass(frozen=True)
class GripperModel:
    """The gripper grasp search plans for: its finger closing profile, its stroke (m) and the
    widest object it closes on (m); a profile with a body (a linear jaw's) brings its own."""

    profile: GripperProfile
    stroke: float = STROKE
    max_width: float = MAX_WIDTH

    @property
    def body(self) -> GripperBody:
        return body_of(self.profile)


def body_of(profile) -> GripperBody:
    """A finger profile's body: its own, else the Robotiq 2F-85's."""
    return getattr(profile, "body", ROBOTIQ_BODY)


@functools.cache
def default_gripper() -> GripperModel:
    """RoboLab's Robotiq 2F-85: the packaged finger profile and an 85 mm stroke."""
    return GripperModel(gripper_profile())


def gripper_for(embodiment=None) -> GripperModel:
    """The gripper model of an embodiment's first end effector; None = DROID's default."""
    if embodiment is None:
        return default_gripper()
    from amp.planning.jaw import gripper_model

    return gripper_model(embodiment)


class SceneContext:
    """One object's geometry, the obstacles around it, its shape, mass and the table, and the
    gripper (gripper_for(embodiment) when None) searching it."""

    def __init__(self, scene, name: str, seed: int,
                 gripper: GripperModel | None = None, embodiment=None):
        if name not in scene.objects:
            raise KeyError(f"unknown object {name!r}")
        self.name = name
        self.gripper = gripper_for(embodiment) if gripper is None else gripper
        self.table_z = float(getattr(scene, "table_z", 0.0))
        self.tgt = ObjectGeom.of(name, scene.objects[name], seed)
        obj = scene.objects[name]
        self.phys = getattr(obj, "physics", None)
        self.com = None
        self.mass = self.friction = None
        if self.phys is not None:
            com = np.asarray(self.phys.com, dtype=np.float64)
            self.com = obj.pose[:3, :3] @ com + obj.pose[:3, 3]
            self.mass, self.friction = self.phys.mass_kg, self.phys.static_friction
        self.obst = Obstacles(
            [ObjectGeom.of(n, o, seed) for n, o in scene.objects.items() if n != name]
        )
        self._shape = None

    @property
    def shape(self) -> ObjectShape:
        if self._shape is None:
            self._shape = _shape(self.tgt)
        return self._shape


def _box_dist(L, lo, hi) -> np.ndarray:
    d = np.maximum(np.maximum(lo[None] - L[:, None], L[:, None] - hi[None]), 0.0)
    return np.sqrt((d * d).sum(-1))


def lowest_point(T, body: GripperBody = ROBOTIQ_BODY) -> tuple[float, str]:
    """(world z, part name) of the gripper body's lowest point at TCP pose T."""
    r = T[2, :3]
    z = T[2, 3] + body.cen @ r - body.half @ np.abs(r)
    j = int(np.argmin(z))
    return float(z[j]), body.parts[j]


def table_note(part: str, clear: float, prefix: str = "") -> str:
    """The note for a gripper part too close to (or below) the table."""
    if clear < 0:
        return f"{prefix}{part} {-clear * 1000:.0f} mm below the table"
    return f"{prefix}{part} only {clear * 1000:.1f} mm above the table"


def gripper_contacts(T, obst: Obstacles, prefix: str = "",
                     body: GripperBody = ROBOTIQ_BODY) -> tuple[list[str], float]:
    """Problems and clearance of the gripper (and the space between its fingers) to obstacles."""
    if obst.tree is None:
        return [], CLEAR_CAP
    uc, uh, uball, olo, ohi, opart = body.uc, body.uh, body.uball, body.olo, body.ohi, body.opart
    R, t = T[:3, :3], T[:3, 3]
    problems, clear = [], CLEAR_CAP
    c = R @ uc + t
    h = np.abs(R) @ (uh + CLEAR_CAP) + 1e-6
    if np.isfinite(obst.tree.query(c, distance_upper_bound=uball)[0]):
        idx = obst.near_xy(c[:2] - h[:2], c[:2] + h[:2])
    else:
        idx = np.empty(0, np.int64)
    idx = idx[np.abs(obst.pts[idx, 2] - c[2]) <= h[2]]
    L = to_tcp_frame(T, obst.pts[idx])
    keep = np.all(np.abs(L - uc) <= uh + CLEAR_CAP, axis=1)
    idx, L = idx[keep], L[keep]
    lb2 = np.full(len(L), np.inf)
    for gc, gh in zip(body.gc, body.gh):
        e = np.maximum(np.abs(L - gc) - gh, 0.0)
        np.minimum(lb2, np.einsum("ij,ij->i", e, e), out=lb2)
    lb = np.sqrt(lb2) - 1e-9
    keep = lb < CLEAR_CAP
    idx, L, lb = idx[keep], L[keep], lb[keep]
    if len(idx) > 64:
        k = np.argpartition(lb, 31)[:32]
        m = min(CLEAR_CAP, float(_box_dist(L[k], olo, ohi).min()))
        keep = lb < max(m, MARGIN)
        idx, L = idx[keep], L[keep]
    if len(idx):
        d = _box_dist(L, olo, ohi)
        clear = min(clear, float(d.min()))
        p, b = np.nonzero(d < MARGIN)
        o = np.lexsort((b, obst.rank()[idx[p]]))
        for pi, bi in zip(p[o], b[o]):
            problems.append(f"{prefix}{opart[bi]} hits {obst.geoms[obst.owner[idx[pi]]].name}")
    centres = body.cen @ R.T + t
    for gi in obst.solid_under(centres):
        g = obst.geoms[gi]
        for j in np.nonzero(g.inside(centres))[0]:
            problems.append(f"{prefix}{body.parts[j]} inside {g.name}")
            clear = 0.0
    return unique_in_order(problems), clear
