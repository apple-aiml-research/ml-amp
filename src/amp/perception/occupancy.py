#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A perceived 2-D occupancy grid of the floor, and a swept-footprint check of a base move."""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass

import numpy as np

_EPS = 1e-9
MEMORY_FRAMES = 30
MEMORY_RANGE_M = 2.0
SELF_HIDDEN_NEAR_M = 0.10
_LENS_M = 0.15
_SELF_DEPTH_TOL_M = 0.06


@dataclass(frozen=True)
class OccupancyGrid:
    """A floor grid: cell (i, j) spans origin + [j, j+1), [i, i+1) x resolution."""

    origin_xy: tuple[float, float]
    resolution: float
    occupied: np.ndarray
    known: np.ndarray

    @classmethod
    def from_points(cls, points_world, floor_z, z_band=(0.05, 2.0), resolution=0.05,
                    extent_m=8.0) -> OccupancyGrid:
        """The extent_m-square grid over the points' floor and band xy, snapped to resolution."""
        res, lo_z, hi_z = float(resolution), float(z_band[0]), float(z_band[1])
        if not (res > 0 and 0 < lo_z < hi_z and extent_m > 0):
            raise ValueError("resolution > 0, extent_m > 0 and 0 < z_band[0] < z_band[1]")
        n = max(1, round(float(extent_m) / res))
        p = np.asarray(points_world, np.float64).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        h = p[:, 2] - float(floor_z)
        floor = np.abs(h) < lo_z
        band = (h >= lo_z) & (h <= hi_z)
        xy = p[floor | band, :2]
        centre = 0.5 * (xy.min(axis=0) + xy.max(axis=0)) if len(xy) else np.zeros(2)
        origin = np.round(np.floor((centre - 0.5 * n * res) / res) * res, 9)
        occupied, seen = np.zeros((n, n), bool), np.zeros((n, n), bool)
        for mask, grid in ((floor, seen), (band, occupied)):
            ij = np.floor((p[mask, :2] - origin) / res).astype(np.int64)[:, ::-1]
            ij = ij[((ij >= 0) & (ij < n)).all(axis=1)]
            grid[ij[:, 0], ij[:, 1]] = True
        return cls((float(origin[0]), float(origin[1])), res, occupied, seen | occupied)

    @property
    def free(self) -> np.ndarray:
        """Cells known and not occupied."""
        return self.known & ~self.occupied

    def swept_free(self, footprint_xy, T_world_base, delta,
                   step_m=0.02) -> tuple[bool, str | None]:
        """(True, None) if the footprint, swept by delta from T_world_base, stays on free cells."""
        hull = convex_hull(np.asarray(footprint_xy, np.float64).reshape(-1, 2))
        if len(hull) < 3:
            raise ValueError("footprint_xy is a polygon of at least 3 non-collinear vertices")
        if not step_m > 0:
            raise ValueError("step_m must be positive")
        T = np.asarray(T_world_base, np.float64).reshape(4, 4)
        dx, dy, dyaw = (float(v) for v in np.asarray(delta, np.float64).reshape(3))
        p0, yaw0 = T[:2, 3], math.atan2(T[1, 0], T[0, 0])
        c, s = math.cos(yaw0), math.sin(yaw0)
        move = np.array([c * dx - s * dy, s * dx + c * dy])
        dist = math.hypot(dx, dy)
        radius = float(np.linalg.norm(hull, axis=1).max())
        n = max(1, math.ceil((dist + radius * abs(dyaw)) / float(step_m) - 1e-12))
        t = np.linspace(0.0, 1.0, n + 1)
        centres = p0 + t[:, None] * move
        hit, cells = self._overlaps(hull, centres, yaw0 + t * dyaw)
        H, W = self.occupied.shape
        i, j = cells[..., 0], cells[..., 1]
        inside = (i >= 0) & (i < H) & (j >= 0) & (j < W)
        ic, jc = np.clip(i, 0, H - 1), np.clip(j, 0, W - 1)
        ids = (i - i.min()) * (int(j.max() - j.min()) + 1) + (j - j.min())
        under = np.isin(ids, ids[0][hit[0]])
        occ = hit & inside & self.occupied[ic, jc]
        unknown = hit & ~(inside & self.known[ic, jc]) & ~under
        bad = (occ | unknown).reshape(len(t), -1).any(axis=1)
        if not bad.any():
            return True, None
        k = int(np.argmax(bad))
        kind, mask = ("occupied", occ[k]) if occ[k].any() else ("unknown", unknown[k])
        ci, cj = i[k][mask], j[k][mask]
        cx = self.origin_xy[0] + (cj + 0.5) * self.resolution
        cy = self.origin_xy[1] + (ci + 0.5) * self.resolution
        pick = int(np.lexsort((cj, ci, np.hypot(cx - centres[k, 0], cy - centres[k, 1])))[0])
        i_, j_ = int(ci[pick]), int(cj[pick])
        if not (0 <= i_ < H and 0 <= j_ < W):
            kind = "unknown (off the grid)"
        when = ("at the start pose" if k == 0 else
                f"after {t[k] * dist:.3f} m, {math.degrees(t[k] * dyaw):.1f} deg of the move "
                f"(sample {k}/{n})")
        return False, (f"{kind} cell ({i_}, {j_}) at ({cx[pick]:.3f}, {cy[pick]:.3f}) m under "
                       f"the footprint {when}")

    def _overlaps(self, hull, centres, yaws) -> tuple[np.ndarray, np.ndarray]:
        """(overlap (S, m, m) bool, cells (S, m, m, 2) int i, j) of the hull at each pose."""
        res, (ox, oy) = self.resolution, self.origin_xy
        radius = float(np.linalg.norm(hull, axis=1).max())
        m = math.ceil(2 * radius / res) + 2
        c, s = np.cos(yaws), np.sin(yaws)
        R = np.stack([np.stack([c, -s], -1), np.stack([s, c], -1)], -2)
        V = centres[:, None, :] + np.einsum("sab,kb->ska", R, hull)
        j0 = np.floor((centres[:, 0] - radius - ox) / res).astype(np.int64)
        i0 = np.floor((centres[:, 1] - radius - oy) / res).astype(np.int64)
        off = np.arange(m)
        i = i0[:, None, None] + off[None, :, None]
        j = j0[:, None, None] + off[None, None, :]
        i, j = np.broadcast_arrays(i, j)
        x0, y0 = ox + j * res, oy + i * res
        lo, hi = V.min(axis=1), V.max(axis=1)
        hit = ((x0 < hi[:, 0, None, None] - _EPS) & (x0 + res > lo[:, 0, None, None] + _EPS)
               & (y0 < hi[:, 1, None, None] - _EPS) & (y0 + res > lo[:, 1, None, None] + _EPS))
        edges = np.roll(V, -1, axis=1) - V
        normal = np.stack([edges[..., 1], -edges[..., 0]], -1)
        offset = np.einsum("ska,ska->sk", normal, V)
        nx, ny = normal[:, None, None, :, 0], normal[:, None, None, :, 1]
        near = ((x0[..., None] + np.where(nx > 0, 0.0, res)) * nx
                + (y0[..., None] + np.where(ny > 0, 0.0, res)) * ny)
        scale = np.hypot(nx, ny)
        hit &= ~(near - offset[:, None, None, :] >= -_EPS * scale).any(axis=-1)
        return hit, np.stack([i, j], -1)


def convex_hull(xy: np.ndarray) -> np.ndarray:
    """The counter-clockwise convex hull (K, 2) of points xy (Andrew's monotone chain)."""
    pts = np.unique(np.asarray(xy, np.float64).reshape(-1, 2), axis=0)
    if len(pts) < 3:
        return pts

    def chain(points):
        out = []
        for p in points:
            while len(out) >= 2 and _cross(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = chain(pts), chain(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _cross(o, a, b) -> float:
    return float((a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]))


class OccupancyMemory:
    """One episode's world-frame floor cells from earlier frames."""

    def __init__(self, resolution: float = 0.05, max_frames: int = MEMORY_FRAMES,
                 max_range: float = MEMORY_RANGE_M):
        self.resolution, self.max_frames, self.max_range = resolution, max_frames, max_range
        self._lock = threading.Lock()
        self.reset()

    def reset(self, token=None) -> None:
        """Forget every cell; what is stored from now on belongs to episode token."""
        with self._lock:
            self.token = token
            self.frame = 0
            self._ij = np.zeros((0, 2), np.int64)
            self._occ = np.zeros(0, bool)
            self._seen = np.zeros(0, np.int64)
            self._stamp = np.zeros(0, np.int64)
            self._top = 0
            self._at = np.zeros((0, 2))

    def __len__(self) -> int:
        return len(self._occ)

    def _offset(self, grid: OccupancyGrid) -> tuple[int, int]:
        if not math.isclose(grid.resolution, self.resolution):
            raise ValueError(f"memory is at {self.resolution} m, got a {grid.resolution} m grid")
        return (round(grid.origin_xy[1] / self.resolution),
                round(grid.origin_xy[0] / self.resolution))

    def _expire(self, base_xy) -> None:
        keep = (self.frame - self._seen < self.max_frames) & (
            np.linalg.norm(self._at - np.asarray(base_xy, float), axis=1) <= self.max_range)
        self._ij, self._occ, self._seen, self._stamp, self._at = (
            self._ij[keep], self._occ[keep], self._seen[keep], self._stamp[keep], self._at[keep])

    def observe(self, grid: OccupancyGrid, base_xy, token=None, stamp=None) -> bool:
        """Store the grid of frame stamp, seen from base_xy; False if token is not the memory's."""
        oi, oj = self._offset(grid)
        with self._lock:
            if token != self.token:
                return False
            stamp = self._top + 1 if stamp is None else int(stamp)
            self._top = max(self._top, stamp)
            self.frame += 1
            ij = np.argwhere(grid.known) + (oi, oj)
            occ = grid.occupied[grid.known]
            self._expire(base_xy)
            take = np.ones(len(ij), bool)
            drop = np.zeros(len(self._occ), bool)
            if len(ij) and len(self._occ):
                size = int(max(np.abs(ij).max(), np.abs(self._ij).max())) + 1
                old_k = (self._ij[:, 0] + size) * (2 * size + 1) + self._ij[:, 1] + size
                new_k = (ij[:, 0] + size) * (2 * size + 1) + ij[:, 1] + size
                order = np.argsort(old_k)
                pos = np.minimum(np.searchsorted(old_k[order], new_k), len(order) - 1)
                hit = np.flatnonzero(old_k[order][pos] == new_k)
                o = order[pos[hit]]
                wins = (stamp > self._stamp[o]) | (
                    (stamp == self._stamp[o]) & (occ[hit] | ~self._occ[o]))
                take[hit[~wins]] = False
                drop[o[wins]] = True
            n = int(take.sum())
            self._ij = np.vstack([self._ij[~drop], ij[take]])
            self._occ = np.concatenate([self._occ[~drop], occ[take]])
            self._seen = np.concatenate([self._seen[~drop], np.full(n, self.frame, np.int64)])
            self._stamp = np.concatenate([self._stamp[~drop], np.full(n, stamp, np.int64)])
            self._at = np.vstack([self._at[~drop], np.tile(np.asarray(base_xy, float), (n, 1))])
        return True

    def merge(self, grid: OccupancyGrid, base_xy, token=None) -> OccupancyGrid:
        """The grid with its unknown cells filled in from memory of episode token."""
        oi, oj = self._offset(grid)
        with self._lock:
            if token != self.token:
                return grid
            self._expire(base_xy)
            ij = self._ij - (oi, oj)
            H, W = grid.known.shape
            ok = (ij[:, 0] >= 0) & (ij[:, 0] < H) & (ij[:, 1] >= 0) & (ij[:, 1] < W)
            ij, occ = ij[ok], self._occ[ok]
        known, occupied = grid.known.copy(), grid.occupied.copy()
        empty = ~known[ij[:, 0], ij[:, 1]]
        ij, occ = ij[empty], occ[empty]
        known[ij[:, 0], ij[:, 1]] = True
        occupied[ij[:, 0], ij[:, 1]] = occ
        return OccupancyGrid(grid.origin_xy, grid.resolution, occupied, known)


def self_hidden(points, cameras, spheres, depth_tol: float = _SELF_DEPTH_TOL_M):
    """Per root-frame point, the body sphere hiding it from every camera seeing it, else -1."""
    P = np.asarray(points, np.float64).reshape(-1, 3)
    C, R = (np.asarray(a, np.float64) for a in spheres)
    out = np.full(len(P), -1, np.int64)
    in_view = np.zeros(len(P), bool)
    blocked = np.ones(len(P), bool)
    who = np.full(len(P), -1, np.int64)
    for T, K, depth in cameras:
        T, K, depth = np.asarray(T, np.float64), np.asarray(K, np.float64), np.asarray(depth)
        depth = depth[:, :, 0] if depth.ndim == 3 else depth
        pc = (P - T[:3, 3]) @ T[:3, :3]
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u, v = K[0, 0] * pc[:, 0] / z + K[0, 2], K[1, 1] * pc[:, 1] / z + K[1, 2]
        h, w = depth.shape
        seen = (z > 0.05) & (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)
        if not seen.any():
            continue
        own = np.linalg.norm(C - T[:3, 3], axis=1) < R
        c, r = C[~own], R[~own]
        if not len(c):
            counts = np.ones(len(seen), bool)
            in_view[counts & seen] = True
            blocked[counts & seen] = False
            continue
        d_obs = np.zeros(len(P))
        idx = np.flatnonzero(seen)
        d_obs[idx] = depth[np.rint(v[idx]).astype(int), np.rint(u[idx]).astype(int)]
        length = np.linalg.norm(P[idx] - T[:3, 3], axis=1)
        ray = (P[idx] - T[:3, 3]) / length[:, None]
        oc = c[None] - T[:3, 3]
        tca = np.einsum("nsj,nj->ns", np.broadcast_to(oc, (len(idx), *oc.shape[1:])), ray)
        d2 = (oc ** 2).sum(-1) - tca ** 2
        half = np.sqrt(np.maximum(r[None] ** 2 - d2, 0.0))
        hit = (d2 < r[None] ** 2) & (tca + half > 0) & (tca - half < length[:, None])
        t_in = np.where(hit, np.maximum(tca - half, 0.0), np.inf)
        t_out = np.where(hit, np.minimum(tca + half, length[:, None]), -np.inf)
        first = t_in.argmin(axis=1)
        any_hit = hit.any(axis=1)
        zr = ray @ T[:3, :3][:, 2]
        z_in, z_out = t_in.min(axis=1) * zr, t_out.max(axis=1) * zr
        d = d_obs[idx]
        sees_body = any_hit & (d > 0) & (d >= z_in - depth_tol) & (d <= z_out + depth_tol)
        counts = ~(any_hit & (t_in.min(axis=1) < _LENS_M))
        in_view[idx[counts]] = True
        blocked[idx[counts]] &= sees_body[counts]
        newly = counts & sees_body & (who[idx] < 0)
        who[idx[newly]] = np.flatnonzero(~own)[first[newly]]
    ok = in_view & blocked
    out[ok] = who[ok]
    return out
