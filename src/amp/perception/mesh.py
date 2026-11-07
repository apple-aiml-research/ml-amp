#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Height maps of segments and their closed, outward-wound 2.5D extrusions."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from amp.perception.params import PerceptionParams
from amp.perception.views import FreeSpace


@dataclass
class HeightMap:
    lo: np.ndarray
    cell: float
    mask: np.ndarray
    top: np.ndarray
    zmin: np.ndarray
    ext: np.ndarray | None = None
    hull: np.ndarray | None = None
    xy: np.ndarray | None = None

    def support_at(self, xy: np.ndarray) -> np.ndarray:
        """Top height at world xy over the row- and column-filled footprint (nan elsewhere)."""
        if self.hull is None:
            m = self.mask
            rows = (np.cumsum(m, axis=1) > 0) & (np.cumsum(m[:, ::-1], axis=1)[:, ::-1] > 0)
            cols = (np.cumsum(m, axis=0) > 0) & (np.cumsum(m[::-1], axis=0)[::-1] > 0)
            self.hull = m | rows | cols
        ij = np.floor((xy - self.lo) / self.cell).astype(np.int64)
        nx, ny = self.mask.shape
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < nx) & (ij[:, 1] >= 0) & (ij[:, 1] < ny)
        out = np.full(len(xy), np.nan)
        i, j = ij[ok, 0], ij[ok, 1]
        out[ok] = np.where(self.hull[i, j], self.top[i, j], np.nan)
        return out

    def centres(self) -> np.ndarray:
        nx, ny = self.mask.shape
        ii, jj = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
        return np.stack([self.lo[0] + (ii + 0.5) * self.cell,
                         self.lo[1] + (jj + 0.5) * self.cell], axis=-1)


def heightmap(pts: np.ndarray, table_z: float, p: PerceptionParams,
               free: FreeSpace | None) -> HeightMap | None:
    xy = pts[:, :2].astype(np.float64)
    z = pts[:, 2].astype(np.float64)
    cell = p.cell
    for _ in range(4):
        lo = xy.min(axis=0) - 2.5 * cell
        ij = np.floor((xy - lo) / cell).astype(np.int64)
        nx, ny = ij.max(axis=0) + 3
        flat = ij[:, 0] * ny + ij[:, 1]
        n_cells = len(np.unique(flat))
        if n_cells <= p.max_cells:
            break
        cell *= float(np.sqrt(n_cells / p.max_cells)) * 1.02
    top = np.full(nx * ny, -np.inf)
    zmin = np.full(nx * ny, np.inf)
    np.maximum.at(top, flat, z)
    np.minimum.at(zmin, flat, z)
    top, zmin = top.reshape(nx, ny), zmin.reshape(nx, ny)
    ext = np.full((4, nx * ny), np.inf)
    np.minimum.at(ext[0], flat, xy[:, 0])
    np.minimum.at(ext[1], flat, -xy[:, 0])
    np.minimum.at(ext[2], flat, xy[:, 1])
    np.minimum.at(ext[3], flat, -xy[:, 1])
    ext = ext.reshape(4, nx, ny)
    obs = np.isfinite(top)
    st = np.ones((3, 3), bool)
    m = obs.copy()
    if p.close_iters > 0:
        m |= ndimage.binary_closing(obs, structure=st, iterations=p.close_iters)
    filled = ndimage.binary_fill_holes(m)
    holes = filled & ~m
    if holes.any() and free is None:
        hl, nh = ndimage.label(holes)
        sizes = np.bincount(hl.ravel(), minlength=nh + 1)
        big = sizes > p.max_hole_cells
        big[0] = False
        filled &= ~big[hl]
    new = filled & ~obs
    _, (ii, jj) = ndimage.distance_transform_edt(~obs, return_indices=True)
    top = np.where(obs, top, top[ii, jj])
    zmin = np.where(obs, zmin, zmin[ii, jj])
    if free is not None and new.any():
        ci, cj = np.nonzero(new)
        step = max(cell, 0.004)
        hc = top[ci, cj]
        n_lev = int(np.ceil((hc.max() - table_z) / step)) + 1
        lev = hc[:, None] - step * np.arange(n_lev)[None, :]
        valid = lev > table_z + p.min_thickness
        cx = lo[0] + (ci + 0.5) * cell
        cy = lo[1] + (cj + 0.5) * cell
        q = np.stack([np.repeat(cx, n_lev), np.repeat(cy, n_lev), lev.ravel()], axis=1)
        vi = valid.ravel()
        fr = np.ones(len(q), bool)
        fr[vi] = free.free(q[vi])
        solid = ~fr.reshape(lev.shape) & valid
        has = solid.any(axis=1)
        k0 = np.argmax(solid, axis=1)
        top[ci, cj] = np.where(has, np.minimum(hc, lev[np.arange(len(ci)), k0] + 0.5 * step), hc)
        floor = holes[ci, cj] & ~has & p.floor_at_support
        floor_z = table_z + max(p.table_margin, 2 * p.min_thickness)
        top[ci[floor], cj[floor]] = floor_z
        filled[ci[~has & ~floor], cj[~has & ~floor]] = False
        low = holes[ci, cj] & has
        top[ci[low], cj[low]] = np.maximum(top[ci[low], cj[low]], floor_z)
    mask = filled & (top > table_z + p.min_thickness)
    mask, top = fix_pinches(mask, top)
    if not mask.any():
        return None
    return HeightMap(lo, cell, mask, top, zmin, ext, xy=xy)


def fix_pinches(mask: np.ndarray, top: np.ndarray):
    """Fill diagonal-only 2x2 contacts so every grid corner of the footprint is manifold."""
    mask, top = mask.copy(), top.copy()
    for _ in range(16):
        a, b, c, d = mask[:-1, :-1], mask[1:, :-1], mask[:-1, 1:], mask[1:, 1:]
        p1 = a & d & ~b & ~c
        p2 = b & c & ~a & ~d
        if not (p1.any() or p2.any()):
            break
        for pm, full, empty in ((p1, ((0, 0), (1, 1)), ((1, 0), (0, 1))),
                                (p2, ((1, 0), (0, 1)), ((0, 0), (1, 1)))):
            i, j = np.nonzero(pm)
            if not len(i):
                continue
            h = np.minimum(top[i + full[0][0], j + full[0][1]], top[i + full[1][0], j + full[1][1]])
            for di, dj in empty:
                ti, tj = i + di, j + dj
                top[ti, tj] = np.where(mask[ti, tj], top[ti, tj], h)
                mask[ti, tj] = True
    return mask, top


def _tighten(hm: HeightMap, mask: np.ndarray, ci, cj, x, y, corners, pad: float,
             min_w: float = 0.001):
    """Move footprint-boundary corners in from the cell grid to the points' own extent."""
    M = np.pad(mask, 1)
    E = np.pad(np.where(mask[None], hm.ext, np.inf), ((0, 0), (1, 1), (1, 1)),
               constant_values=np.inf)
    a, b, c, d = (ci, cj), (ci + 1, cj), (ci, cj + 1), (ci + 1, cj + 1)
    left_empty = ~M[a] & ~M[c]
    right_empty = ~M[b] & ~M[d]
    below_empty = ~M[a] & ~M[b]
    above_empty = ~M[c] & ~M[d]
    x, y = x.copy(), y.copy()
    xmin = np.minimum(E[0][b], E[0][d])
    xmax = -np.minimum(E[1][a], E[1][c])
    ymin = np.minimum(E[2][c], E[2][d])
    ymax = -np.minimum(E[3][a], E[3][b])
    m = left_empty & ~right_empty & np.isfinite(xmin)
    x[m] = np.maximum(x[m], xmin[m] - pad)
    m = right_empty & ~left_empty & np.isfinite(xmax)
    x[m] = np.minimum(x[m], xmax[m] + pad)
    m = below_empty & ~above_empty & np.isfinite(ymin)
    y[m] = np.maximum(y[m], ymin[m] - pad)
    m = above_empty & ~below_empty & np.isfinite(ymax)
    y[m] = np.minimum(y[m], ymax[m] + pad)
    occ = M[a].astype(int) + M[b] + M[c] + M[d]
    conv = np.flatnonzero((occ >= 1) & (occ <= 3))
    if len(conv) and hm.xy is not None and len(hm.xy):
        g = np.stack([hm.lo[0] + ci[conv] * hm.cell, hm.lo[1] + cj[conv] * hm.cell], axis=1)
        dist, j = cKDTree(hm.xy).query(g, distance_upper_bound=1.5 * hm.cell)
        ok = np.isfinite(dist)
        q = hm.xy[j[ok]]
        u = g[ok] - q
        u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
        x[conv[ok]], y[conv[ok]] = (q + pad * u).T
    c00, c10, c11, c01 = corners
    for lo_c, hi_c, arr in ((c00, c10, x), (c01, c11, x), (c00, c01, y), (c10, c11, y)):
        gap = arr[hi_c] - arr[lo_c]
        bad = gap < min_w
        if bad.any():
            mid = 0.5 * (arr[hi_c[bad]] + arr[lo_c[bad]])
            np.minimum.at(arr, lo_c[bad], mid - 0.5 * min_w)
            np.maximum.at(arr, hi_c[bad], mid + 0.5 * min_w)
    return x, y


def extrude(hm: HeightMap, bottom: np.ndarray, p: PerceptionParams):
    """Closed, outward-wound mesh of the height field over the mask, down to the bottom field."""
    mask = hm.mask
    T = np.pad(np.where(mask, hm.top, -np.inf), 1, constant_values=-np.inf)
    B = np.pad(np.where(mask, bottom, np.inf), 1, constant_values=np.inf)
    tc = np.maximum.reduce([T[:-1, :-1], T[1:, :-1], T[:-1, 1:], T[1:, 1:]])
    bc = np.minimum.reduce([B[:-1, :-1], B[1:, :-1], B[:-1, 1:], B[1:, 1:]])
    used = np.isfinite(tc)
    tc = np.where(used, np.maximum(tc, bc + p.min_thickness), 0.0)
    cid = np.full(used.shape, -1, np.int64)
    nu = int(used.sum())
    cid[used] = np.arange(nu)
    ci, cj = np.nonzero(used)
    x = hm.lo[0] + ci * hm.cell
    y = hm.lo[1] + cj * hm.cell
    i, j = np.nonzero(mask)
    c00, c10, c11, c01 = cid[i, j], cid[i + 1, j], cid[i + 1, j + 1], cid[i, j + 1]
    if hm.ext is not None and p.tight_footprint:
        x, y = _tighten(hm, mask, ci, cj, x, y, (c00, c10, c11, c01), p.footprint_pad)
    verts = np.vstack([np.stack([x, y, tc[ci, cj]], 1), np.stack([x, y, bc[ci, cj]], 1)])
    faces = [np.stack([c00, c10, c11], 1), np.stack([c00, c11, c01], 1),
             np.stack([c00, c11, c10], 1) + nu, np.stack([c00, c01, c11], 1) + nu]
    M = np.pad(mask, 1)
    for (u, v), (di, dj) in (((c00, c10), (0, -1)), ((c10, c11), (1, 0)),
                             ((c11, c01), (0, 1)), ((c01, c00), (-1, 0))):
        out = ~M[i + 1 + di, j + 1 + dj]
        uu, vv = u[out], v[out]
        faces.append(np.stack([vv, uu, uu + nu], 1))
        faces.append(np.stack([vv, uu + nu, vv + nu], 1))
    return verts, np.vstack(faces).astype(np.int64)


def bottoms(hmaps: list[HeightMap], table_z: float, p: PerceptionParams) -> list[np.ndarray]:
    """Per-cell extrusion floor: the table (or, without floor_at_support, the lowest seen point),
    or the top of another segment directly underneath."""
    out = []
    boxes = []
    for hm in hmaps:
        nx, ny = hm.mask.shape
        boxes.append((hm.lo, hm.lo + hm.cell * np.array([nx, ny])))
    for a, hm in enumerate(hmaps):
        bot = np.full(hm.mask.shape, table_z) if p.floor_at_support else hm.zmin.copy()
        cen = None
        for b, other in enumerate(hmaps):
            if a == b or np.any(boxes[a][0] > boxes[b][1]) or np.any(boxes[b][0] > boxes[a][1]):
                continue
            if cen is None:
                cen = hm.centres()
            i0, j0 = np.maximum(np.floor((boxes[b][0] - hm.lo) / hm.cell).astype(int), 0)
            i1, j1 = np.ceil((boxes[b][1] - hm.lo) / hm.cell).astype(int) + 1
            win = (slice(i0, i1), slice(j0, j1))
            sub = cen[win]
            if not sub.size:
                continue
            t = other.support_at(sub.reshape(-1, 2)).reshape(sub.shape[:2])
            ok = (np.isfinite(t) & hm.mask[win] & (t <= hm.zmin[win] + p.support_gap)
                  & (t > bot[win]))
            bot[win] = np.where(ok, t, bot[win])
        bot = np.minimum(bot, hm.top - p.min_thickness)
        out.append(np.maximum(bot, table_z))
    return out
