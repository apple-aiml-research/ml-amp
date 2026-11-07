#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What the cameras saw: seen-through free space and which cameras saw each voxel."""

from __future__ import annotations

import numpy as np
from scipy import ndimage

DEFAULT_VIEW = np.array([0.9, 0.0, 0.5])


class FreeSpace:
    """Seen-through space: a point is free if some camera observed a surface beyond it."""

    def __init__(self, cams, xyz: np.ndarray, margin: float):
        self.margin = margin
        self.views = []
        self.raw = []
        for cam in cams:
            uv, z = cam.project(xyz)
            ok = np.isfinite(uv).all(axis=1) & (z > 0.05)
            uv, z = uv[ok], z[ok]
            for f in (1, 2, 3, 4, 6, 8):
                buf = self._zbuf(cam, uv, z, f)
                filled = np.isfinite(buf)
                between = np.zeros_like(filled)
                between[:, 1:-1] |= filled[:, :-2] & filled[:, 2:]
                between[1:-1, :] |= filled[:-2, :] & filled[2:, :]
                if filled.sum() < 16 or (between & ~filled).sum() <= 0.03 * filled.sum():
                    break
            raw = buf
            buf = np.where(np.isfinite(buf), buf, 0.0)
            buf = ndimage.minimum_filter(buf, size=3, mode="nearest")
            self.views.append((cam, f, buf))
            self.raw.append(raw)

    @staticmethod
    def _zbuf(cam, uv: np.ndarray, z: np.ndarray, f: int) -> np.ndarray:
        w, h = (cam.width + f - 1) // f, (cam.height + f - 1) // f
        iu = np.floor((uv[:, 0] + 0.5) / f).astype(np.int64)
        iv = np.floor((uv[:, 1] + 0.5) / f).astype(np.int64)
        ok = (iu >= 0) & (iu < w) & (iv >= 0) & (iv < h)
        buf = np.full(h * w, np.inf)
        np.minimum.at(buf, iv[ok] * w + iu[ok], z[ok])
        return buf.reshape(h, w)

    def free(self, pts: np.ndarray) -> np.ndarray:
        out = np.zeros(len(pts), bool)
        for cam, f, buf in self.views:
            uv, z = cam.project(pts)
            h, w = buf.shape
            with np.errstate(invalid="ignore"):
                iu = np.floor((uv[:, 0] + 0.5) / f)
                iv = np.floor((uv[:, 1] + 0.5) / f)
            ok = np.isfinite(iu) & np.isfinite(iv) & (z > 0.05)
            ok &= (iu >= 0) & (iu < w) & (iv >= 0) & (iv < h)
            idx = np.flatnonzero(ok)
            seen = buf[iv[idx].astype(np.int64), iu[idx].astype(np.int64)]
            out[idx] |= seen > z[idx] + self.margin
        return out


def viewpoints(pos: np.ndarray, cams, free: FreeSpace | None):
    """(camera centres (C, 3), (V, C) which cameras saw each voxel; the nearest if none did)."""
    if not cams:
        return DEFAULT_VIEW[None], np.ones((len(pos), 1), bool)
    centres = np.array([c.T_world_cam[:3, 3] for c in cams])
    seen = np.zeros((len(pos), len(cams)), bool)
    if free is not None:
        for c, ((cam, f, _), raw) in enumerate(zip(free.views, free.raw)):
            uv, z = cam.project(pos)
            h, w = raw.shape
            with np.errstate(invalid="ignore"):
                iu, iv = np.floor((uv[:, 0] + 0.5) / f), np.floor((uv[:, 1] + 0.5) / f)
            ok = np.isfinite(iu) & np.isfinite(iv) & (iu >= 0) & (iu < w) & (iv >= 0) & (iv < h)
            idx = np.flatnonzero(ok)
            seen[idx, c] = np.abs(raw[iv[idx].astype(np.int64), iu[idx].astype(np.int64)]
                                  - z[idx]) < 0.01
    none = ~seen.any(axis=1)
    near = np.argmin(np.linalg.norm(pos[:, None] - centres[None], axis=2), axis=1)
    seen[np.flatnonzero(none), near[none]] = True
    return centres, seen
