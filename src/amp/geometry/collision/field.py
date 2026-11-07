#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A mesh's distance field: dense surface samples in a KD-tree, a ray-winding inside test."""

from __future__ import annotations

import hashlib
import math
import threading
from collections import OrderedDict

import numpy as np
from scipy.spatial import ConvexHull, QhullError, cKDTree

SURFACE_STEP = 0.003
MAX_SURFACE_SAMPLES = 200_000
COLUMN_STEP_RANGE = (0.002, 0.006)
ATTACHED_STEP = 0.01
MAX_ATTACHED_POINTS = 400
COARSE_CELL = 0.01
FIELD_CACHE_SIZE = 32


def _surface_samples(v: np.ndarray, f: np.ndarray, step: float) -> tuple[np.ndarray, float]:
    """Points on every triangle no further than about `step` apart, plus the vertices."""
    tri = v[f]
    lengths = np.stack([np.linalg.norm(tri[:, (i + 2) % 3] - tri[:, (i + 1) % 3], axis=1)
                        for i in range(3)], axis=1)
    first = np.argmax(lengths, axis=1)
    order = np.stack([first, (first + 1) % 3, (first + 2) % 3], axis=1)
    tri = np.take_along_axis(tri, order[:, :, None], axis=1)
    lmax = lengths.max(axis=1)
    while True:
        big = np.flatnonzero(lmax > step)
        t = tri[big]
        a, e1, e2 = t[:, 0], t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]
        n1 = np.maximum(1, np.ceil(np.linalg.norm(e1, axis=1) / step)).astype(np.int64)
        n2 = np.maximum(1, np.ceil(np.linalg.norm(e2, axis=1) / step)).astype(np.int64)
        n3 = np.maximum(1, np.ceil(np.linalg.norm(t[:, 2] - t[:, 1], axis=1) / step)).astype(
            np.int64)
        cnt = (n1 + 1) * (n2 + 1)
        if not len(big) or cnt.sum() + n3.sum() <= 2 * MAX_SURFACE_SAMPLES:
            break
        step *= 1.3
    tid = np.repeat(np.arange(len(big)), cnt)
    k = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
    u = (k // (n2[tid] + 1)) / n1[tid]
    w = (k % (n2[tid] + 1)) / n2[tid]
    keep = u + w <= 1.0 + 1e-9
    tid, u, w = tid[keep], u[keep], w[keep]
    grid = a[tid] + u[:, None] * e1[tid] + w[:, None] * e2[tid]
    eid = np.repeat(np.arange(len(big)), n3 + 1)
    s = (np.arange((n3 + 1).sum()) - np.repeat(np.cumsum(n3 + 1) - (n3 + 1), n3 + 1)) / n3[eid]
    edge = t[eid, 1] + s[:, None] * (t[eid, 2] - t[eid, 1])
    return np.vstack([grid, edge, v]), step


class _Rays:
    """Signed triangle crossings along a grid of rays parallel to one axis."""

    def __init__(self, v: np.ndarray, f: np.ndarray, axis: int, lo: np.ndarray, hi: np.ndarray):
        self.perm = ((1, 2, 0), (2, 0, 1), (0, 1, 2))[axis]
        v, lo, hi = v[:, self.perm], lo[list(self.perm)], hi[list(self.perm)]
        ext = hi - lo
        step = float(np.clip(max(ext[0], ext[1]) / 200.0, *COLUMN_STEP_RANGE))
        self.step, self.lo, self.z_hi = step, lo, float(hi[2])
        self.off = np.array([0.1371 * step, 0.2917 * step])
        self.nx = max(1, math.ceil(ext[0] / step))
        self.ny = max(1, math.ceil(ext[1] / step))
        self.z_lo, self.z_span = float(lo[2]), float(ext[2]) + 1e-9
        ncol = self.nx * self.ny
        tri = v[f]
        o = lo[:2] + self.off
        xy_lo = (tri[..., :2].min(axis=1) - o) / step - 0.5
        xy_hi = (tri[..., :2].max(axis=1) - o) / step - 0.5
        i0 = np.maximum(np.ceil(xy_lo).astype(np.int64), 0)
        i1 = np.minimum(np.floor(xy_hi).astype(np.int64), [self.nx - 1, self.ny - 1])
        span = np.maximum(i1 - i0 + 1, 0)
        cnt = span[:, 0] * span[:, 1]
        tid = np.repeat(np.arange(len(f)), cnt)
        k = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        cx = i0[tid, 0] + k // np.maximum(span[tid, 1], 1)
        cy = i0[tid, 1] + k % np.maximum(span[tid, 1], 1)
        px, py = o[0] + (cx + 0.5) * step, o[1] + (cy + 0.5) * step
        a, b, c = tri[tid, 0], tri[tid, 1], tri[tid, 2]
        d = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
        ok = np.abs(d) > 1e-15
        dd = np.where(ok, d, 1.0)
        wb = ((px - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (py - a[:, 1])) / dd
        wc = ((b[:, 0] - a[:, 0]) * (py - a[:, 1]) - (px - a[:, 0]) * (b[:, 1] - a[:, 1])) / dd
        wa = 1.0 - wb - wc
        hit = ok & (wa >= 0) & (wb >= 0) & (wc >= 0)
        z = wa * a[:, 2] + wb * b[:, 2] + wc * c[:, 2]
        col = (cx * self.ny + cy)[hit]
        zn = np.clip((z[hit] - self.z_lo) / self.z_span, 0.0, 0.999999)
        key = col * 2.0 + zn
        order = np.argsort(key, kind="stable")
        self.keys = key[order]
        self.col_start = np.searchsorted(self.keys, np.arange(ncol + 1) * 2.0)
        self.wind = np.concatenate([[0], np.cumsum(np.sign(d[hit][order]).astype(np.int32))])

    def windings(self, p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(up, down): signed crossings on each side of p, +1 per face whose normal points away."""
        p = p[:, self.perm]
        ij = np.floor((p[:, :2] - self.lo[:2] - self.off) / self.step).astype(np.int64)
        valid = (
            (ij[:, 0] >= 0) & (ij[:, 0] < self.nx) & (ij[:, 1] >= 0) & (ij[:, 1] < self.ny)
            & (p[:, 2] >= self.z_lo) & (p[:, 2] <= self.z_hi)
        )
        up = np.zeros(len(p), dtype=np.int32)
        dn = np.zeros(len(p), dtype=np.int32)
        if valid.any():
            col = ij[valid, 0] * self.ny + ij[valid, 1]
            idx = np.searchsorted(self.keys, col * 2.0 + (p[valid, 2] - self.z_lo) / self.z_span)
            base = self.wind[self.col_start[col]]
            below = self.wind[idx] - base
            up[valid] = self.wind[self.col_start[col + 1]] - base - below
            dn[valid] = -below
        return up, dn


def _watertight(v: np.ndarray, f: np.ndarray) -> bool:
    """Every edge (vertices welded by position) is used exactly once in each direction."""
    _, weld = np.unique(v, axis=0, return_inverse=True)
    g = weld.ravel()[f]
    g = g[(g[:, 0] != g[:, 1]) & (g[:, 1] != g[:, 2]) & (g[:, 2] != g[:, 0])]
    n = np.int64(len(v))
    key = np.sort(np.concatenate([g[:, 0] * n + g[:, 1], g[:, 1] * n + g[:, 2],
                                  g[:, 2] * n + g[:, 0]]))
    rev = np.sort((key % n) * n + key // n)
    return bool(len(key)) and bool(np.all(key[1:] != key[:-1])) and np.array_equal(key, rev)


class MeshField:
    """Unsigned distance (KD-tree over dense surface samples) and a ray-winding inside test."""

    def __init__(self, verts: np.ndarray, faces: np.ndarray):
        self.verts = verts
        self.lo, self.hi = verts.min(axis=0), verts.max(axis=0)
        if len(faces):
            samples, self.step = _surface_samples(verts, faces, SURFACE_STEP)
        else:
            samples, self.step = verts, SURFACE_STEP
        self.samples = samples
        self.tree = cKDTree(samples)
        self.watertight = _watertight(verts, faces)
        axes = (2,) if self.watertight else (2, 0, 1)
        self._rays = [_Rays(verts, faces, a, self.lo, self.hi) for a in axes]
        self._attach = None
        self._coarse = None

    def coarse(self) -> tuple[cKDTree, float]:
        """(tree over one sample per COARSE_CELL cell, eps): d is in [d_coarse - eps, d_coarse]."""
        if self._coarse is None:
            key = np.floor((self.samples - self.lo) / COARSE_CELL).astype(np.int64)
            key = (key[:, 0] * 4096 + key[:, 1]) * 4096 + key[:, 2]
            _, idx, inv = np.unique(key, return_index=True, return_inverse=True)
            rep = self.samples[idx]
            eps = float(np.linalg.norm(self.samples - rep[inv.ravel()], axis=1).max())
            self._coarse = (cKDTree(rep), eps + 1e-9)
        return self._coarse

    def inside(self, p) -> np.ndarray:
        """Whether root-frame points lie inside the mesh."""
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        up, dn = self._rays[0].windings(p)
        if self.watertight:
            return dn != 0
        out = (up >= 1) & (dn >= 1)
        idx = np.flatnonzero(out)
        if len(idx):
            q, side = p[idx], np.zeros(len(idx), dtype=bool)
            for rays in self._rays[1:]:
                u, d = rays.windings(q)
                side |= (u >= 1) & (d >= 1)
            out[idx] = side
        return out

    def attach_points(self) -> tuple[np.ndarray, float, np.ndarray]:
        """(surface points ~ATTACHED_STEP apart, their radius, convex-hull vertices), root frame."""
        if self._attach is None:
            step = ATTACHED_STEP
            while True:
                _, idx = np.unique(np.floor(self.samples / step), axis=0, return_index=True)
                if len(idx) <= MAX_ATTACHED_POINTS:
                    break
                step *= 1.25
            try:
                hull = self.verts[ConvexHull(self.verts).vertices]
            except (QhullError, ValueError):
                hull = self.verts
            self._attach = (self.samples[np.sort(idx)], 0.5 * step, hull)
        return self._attach


class FieldCache:
    """An LRU of MeshFields keyed by mesh content: planners rebuild worlds over the same meshes."""

    def __init__(self, size: int = FIELD_CACHE_SIZE):
        self.size = int(size)
        self._fields: OrderedDict[str, MeshField] = OrderedDict()
        self._lock = threading.Lock()

    def __len__(self) -> int:
        return len(self._fields)

    def get(self, verts, faces) -> MeshField | None:
        """The distance field of a root-frame mesh, built on a miss; None for an empty one."""
        v = np.ascontiguousarray(verts, dtype=np.float64).reshape(-1, 3)
        f = np.ascontiguousarray(faces, dtype=np.int64).reshape(-1, 3)
        if not len(v):
            return None
        if len(f) and (f.min() < 0 or f.max() >= len(v)):
            raise ValueError("face index out of range")
        key = hashlib.sha1(v.tobytes() + b"|" + f.tobytes()).hexdigest()  # nosec B324  # nosemgrep: python.lang.security.insecure-hash-algorithms.insecure-hash-algorithm-sha1
        with self._lock:
            field = self._fields.get(key)
            if field is not None:
                self._fields.move_to_end(key)
                return field
        field = MeshField(v, f)
        with self._lock:
            self._fields[key] = field
            while len(self._fields) > self.size:
                self._fields.popitem(last=False)
        return field


FIELDS = FieldCache()


def mesh_field(verts, faces, cache: FieldCache | None = None) -> MeshField | None:
    """The (cached) distance field of a root-frame mesh; None for an empty one."""
    return (FIELDS if cache is None else cache).get(verts, faces)
