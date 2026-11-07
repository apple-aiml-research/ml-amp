#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Voxel segmentation: neighbour graphs split at concave creases and colour edges, then merged."""

from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from amp.perception.color import color_feat
from amp.perception.params import PerceptionParams


def components(n: int, edges: np.ndarray) -> np.ndarray:
    if not len(edges):
        return np.arange(n)
    g = coo_matrix((np.ones(len(edges), np.int8), (edges[:, 0], edges[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)[1]


def voxelize(xyz: np.ndarray, rgb: np.ndarray, size: float):
    k = np.floor(xyz / size).astype(np.int64)
    k -= k.min(axis=0)
    dims = k.max(axis=0) + 1
    code = (k[:, 0] * dims[1] + k[:, 1]) * dims[2] + k[:, 2]
    _, first, inv, cnt = np.unique(code, return_index=True, return_inverse=True,
                                   return_counts=True)
    inv = inv.ravel()
    n = len(cnt)
    pos = np.stack([np.bincount(inv, xyz[:, a], n) for a in range(3)], axis=1) / cnt[:, None]
    return pos, rgb[first], inv


def _normals(pos: np.ndarray, tree: cKDTree, k: int, views) -> tuple[np.ndarray, np.ndarray]:
    """(PCA normals over the k nearest voxels, sign confidence in [0, 1])."""
    centres, seen = views
    k = int(min(k, len(pos)))
    if k < 3:
        return np.tile([0.0, 0.0, 1.0], (len(pos), 1)), np.ones(len(pos))
    nb = tree.query(pos, k=k)[1]
    q = pos[nb] - pos[nb].mean(axis=1, keepdims=True)
    n = np.linalg.eigh(np.einsum("nki,nkj->nij", q, q))[1][:, :, 0]
    d = centres[None, :, :] - pos[:, None, :]
    d /= np.linalg.norm(d, axis=2, keepdims=True)
    dots = np.einsum("ni,nci->nc", n, d)
    score = np.where(seen, np.abs(dots), -1.0)
    best = np.argmax(score, axis=1)
    sgn = np.sign(dots[np.arange(len(pos)), best])
    n *= np.where(sgn == 0, 1.0, sgn)[:, None]
    return n, np.abs(dots[np.arange(len(pos)), best])


def _concave(pos: np.ndarray, n: np.ndarray, conf: np.ndarray, pairs: np.ndarray,
             p: PerceptionParams):
    """Pairs across a sharp concave crease (where one object rests on or against another)."""
    i, j = pairs[:, 0], pairs[:, 1]
    conv = np.einsum("ni,ni->n", n[i] - n[j], pos[i] - pos[j])
    sure = (conf[i] > p.normal_conf) & (conf[j] > p.normal_conf)
    return sure & (conv < 0) & (np.einsum("ni,ni->n", n[i], n[j])
                                < np.cos(np.radians(p.concave_deg)))


def _merge_regions(lab: np.ndarray, pairs: np.ndarray, feat: np.ndarray, geo: np.ndarray,
                   solid: np.ndarray, p: PerceptionParams) -> np.ndarray:
    """Merge tiny regions into neighbours, and similar ones of one part."""
    n = int(lab.max()) + 1
    _, geo = np.unique(geo, return_inverse=True)
    geo = geo.ravel()
    a, b = lab[pairs[:, 0]], lab[pairs[:, 1]]
    m = a != b
    a, b = np.minimum(a[m], b[m]), np.maximum(a[m], b[m])
    for _ in range(30):
        if not len(a):
            break
        size = np.bincount(lab, minlength=n)
        mean = np.stack([np.bincount(lab, feat[:, k], n) for k in range(feat.shape[1])], 1)
        mean /= np.maximum(size, 1)[:, None]
        rgeo = -1 - np.arange(n)
        rgeo[lab[solid]] = geo[solid]
        code, cnt = np.unique(a * n + b, return_counts=True)
        ra, rb = code // n, code % n
        close = np.linalg.norm(mean[ra] - mean[rb], axis=1) < p.color_merge
        same = rgeo[ra] == rgeo[rb]
        join = close & same
        small = size < p.min_region_voxels
        if not join.any():
            gsize = np.bincount(geo[solid], minlength=int(geo.max()) + 1)
            rg = np.where(rgeo >= 0, gsize[np.maximum(rgeo, 0)], 0)
            small |= (size < p.patch_voxels) & (size < p.patch_frac * rg)
        src, dst = np.concatenate([ra, rb]), np.concatenate([rb, ra])
        c2 = np.concatenate([cnt, cnt])
        patch = (size[src] >= p.min_region_voxels) & small[src]
        c2 = np.where(patch & (rgeo[src] != rgeo[dst]), -1, c2)
        sel = small[src] & (c2 >= 0)
        src, dst, c2 = src[sel], dst[sel], c2[sel]
        if len(src):
            o = np.lexsort((-c2, src))
            src, dst = src[o], dst[o]
            first = np.r_[True, src[1:] != src[:-1]]
            tiny = np.stack([src[first], dst[first]], axis=1)
        else:
            tiny = np.zeros((0, 2), np.int64)
        edges = np.vstack([np.stack([ra[join], rb[join]], axis=1), tiny])
        if not len(edges):
            break
        new = components(n, edges)
        lab, a, b = new[lab], new[a], new[b]
        m = a != b
        a, b = np.minimum(a[m], b[m]), np.maximum(a[m], b[m])
        n = int(new.max()) + 1
    return lab


def segment_voxels(pos: np.ndarray, col: np.ndarray, use_color: bool, view: np.ndarray,
             p: PerceptionParams):
    """(voxel labels, neighbour pairs)."""
    n = len(pos)
    if n == 0:
        return np.zeros(0, np.int64), np.zeros((0, 2), np.int64)
    tree = cKDTree(pos)
    pairs = tree.query_pairs(p.cluster_tol, output_type="ndarray")
    if not len(pairs):
        return np.arange(n), pairs
    if p.concave_split:
        nrm, conf = _normals(pos, tree, p.normal_k, view)
        conc = _concave(pos, nrm, conf, pairs, p)
    else:
        conc = np.zeros(len(pairs), bool)
    nconc = np.bincount(pairs[conc].ravel(), minlength=n)
    solid = nconc < p.crease_pairs
    keep = ~conc & solid[pairs[:, 0]] & solid[pairs[:, 1]]
    geo = components(n, pairs[keep])
    feat = color_feat(col, p.intensity_weight)
    if use_color:
        keep &= np.linalg.norm(feat[pairs[:, 0]] - feat[pairs[:, 1]], axis=1) < p.color_edge
    else:
        feat = np.zeros((n, 1))
    lab = components(n, pairs[keep])
    return _merge_regions(lab, pairs, feat, geo, solid, p), pairs


def _touching(lab: np.ndarray, pairs: np.ndarray) -> set:
    a, b = lab[pairs[:, 0]], lab[pairs[:, 1]]
    m = a != b
    return set(zip(np.minimum(a[m], b[m]).tolist(), np.maximum(a[m], b[m]).tolist()))


def absorb(lab: np.ndarray, pos: np.ndarray, col: np.ndarray, has_rgb: bool, pairs: np.ndarray,
            p: PerceptionParams) -> np.ndarray:
    """Fold small fragments (a sparsely sampled grazing face) into a much larger, close segment."""
    _, lab = np.unique(lab, return_inverse=True)
    lab = lab.ravel()
    n = int(lab.max()) + 1 if len(lab) else 0
    size = np.bincount(lab, minlength=n)
    small = np.flatnonzero(size < p.absorb_voxels)
    if not len(small) or n < 2:
        return lab
    feat = color_feat(col, p.intensity_weight)
    mean = np.stack([np.bincount(lab, feat[:, k], n) for k in range(feat.shape[1])], 1)
    mean /= np.maximum(size, 1)[:, None]
    big = np.flatnonzero(size >= p.absorb_voxels)
    if not len(big):
        return lab
    touching = _touching(lab, pairs)
    in_big = np.isin(lab, big)
    tree = cKDTree(pos[in_big])
    big_lab = lab[in_big]
    target = np.arange(n)
    for r in small:
        d, j = tree.query(pos[lab == r], distance_upper_bound=p.absorb_dist)
        hit = np.isfinite(d)
        if not hit.any():
            continue
        cand, cnt = np.unique(big_lab[j[hit]], return_counts=True)
        t = int(cand[np.argmax(cnt)])
        if size[t] < 4 * size[r] or (min(r, t), max(r, t)) in touching:
            continue
        if has_rgb and np.linalg.norm(mean[t] - mean[r]) > p.color_merge:
            continue
        target[r] = t
    return target[lab]


def merge_stacked(lab: np.ndarray, pos: np.ndarray, col: np.ndarray, has_rgb: bool,
                   pairs: np.ndarray, p: PerceptionParams) -> np.ndarray:
    """Join close same-coloured segments sharing a footprint: one object split by gaps."""
    _, lab = np.unique(lab, return_inverse=True)
    lab = lab.ravel()
    n = int(lab.max()) + 1 if len(lab) else 0
    if n < 2:
        return lab
    feat = color_feat(col, p.intensity_weight)
    size = np.bincount(lab, minlength=n)
    mean = np.stack([np.bincount(lab, feat[:, k], n) for k in range(feat.shape[1])], 1)
    mean /= np.maximum(size, 1)[:, None]
    order = np.argsort(lab, kind="stable")
    bounds = np.searchsorted(lab[order], np.arange(n + 1))
    lo = np.stack([pos[order[bounds[i]:bounds[i + 1]], :2].min(0) for i in range(n)])
    hi = np.stack([pos[order[bounds[i]:bounds[i + 1]], :2].max(0) for i in range(n)])
    lo2, hi2 = lo - 0.5 * p.voxel, hi + 0.5 * p.voxel
    inter = np.clip(np.minimum(hi2[:, None], hi2[None]) - np.maximum(lo2[:, None], lo2[None]),
                    0, None).prod(axis=2)
    area = (hi2 - lo2).prod(axis=1)
    frac = inter / np.minimum(area[:, None], area[None])
    ratio = np.minimum(size[:, None], size[None]) / np.maximum(size[:, None], size[None])
    cand = np.argwhere(np.triu((frac >= p.overlap_frac) & (ratio <= p.fragment_ratio), 1))
    if has_rgb and len(cand):
        d = np.linalg.norm(mean[cand[:, 0]] - mean[cand[:, 1]], axis=1)
        cand = cand[d < p.color_merge]
    touching = _touching(lab, pairs)
    edges = []
    for a, b in cand:
        if (min(a, b), max(a, b)) in touching:
            continue
        pa = pos[order[bounds[a]:bounds[a + 1]]]
        pb = pos[order[bounds[b]:bounds[b + 1]]]
        if len(pa) < len(pb):
            pa, pb = pb, pa
        if np.isfinite(cKDTree(pa).query(pb, distance_upper_bound=p.overlap_dist)[0]).any():
            edges.append((a, b))
    near = set(touching) | {(int(a), int(b)) for a, b in np.argwhere(np.triu(inter > 0, 1))}
    edges += _side_by_side(lab, pos, mean, size, near, order, bounds, has_rgb, p)
    if not edges:
        return lab
    return components(n, np.array(edges))[lab]


def _side_by_side(lab, pos, mean, size, cand, order, bounds, has_rgb, p) -> list:
    """Pairs of close same-coloured parts on one base and footprint: one object split apart."""
    out = []
    for a, b in cand:
        if min(size[a], size[b]) < p.min_region_voxels:
            continue
        if has_rgb and np.linalg.norm(mean[a] - mean[b]) >= p.color_merge:
            continue
        pa, pb = pos[order[bounds[a]:bounds[a + 1]]], pos[order[bounds[b]:bounds[b + 1]]]
        if abs(float(pa[:, 2].min()) - float(pb[:, 2].min())) > p.same_base:
            continue
        ca = {tuple(c) for c in np.floor(pa[:, :2] / 0.01).astype(np.int64).tolist()}
        cb = {tuple(c) for c in np.floor(pb[:, :2] / 0.01).astype(np.int64).tolist()}
        if len(ca & cb) < p.shared_footprint * min(len(ca), len(cb)):
            continue
        if len(pa) < len(pb):
            pa, pb = pb, pa
        if np.isfinite(cKDTree(pa).query(pb, distance_upper_bound=p.overlap_dist)[0]).any():
            out.append((a, b))
    return out


def region_grow(pts, rgb, seed: int, radius: float, *, color_tol=None, reach: float = 0.6,
                params=None) -> np.ndarray:
    """Indices of pts linked to pts[seed] by voxel links under radius (and color_tol)."""
    p = params or PerceptionParams()
    pts = np.asarray(pts, np.float64).reshape(-1, 3)
    near = np.flatnonzero(np.all(np.abs(pts - pts[seed]) <= reach, axis=1))
    sub = pts[near]
    col = np.asarray(rgb, np.float64).reshape(-1, 3)[near]
    pos, vcol, inv = voxelize(sub, col, min(p.voxel, 0.5 * radius))
    pairs = cKDTree(pos).query_pairs(radius, output_type="ndarray")
    if color_tol is not None and len(pairs):
        feat = color_feat(vcol, p.intensity_weight)
        pairs = pairs[np.linalg.norm(feat[pairs[:, 0]] - feat[pairs[:, 1]], axis=1)
                      < float(color_tol)]
    lab = components(len(pos), pairs)
    s = int(np.flatnonzero(near == seed)[0])
    return near[lab[inv] == lab[inv[s]]]
