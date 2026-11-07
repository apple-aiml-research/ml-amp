#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Pinhole camera models in the robot-root frame, depth back-projection and depth picking."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_GL_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])
_WORLD_TO_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CONVENTIONS = ("opengl", "world", "ros")


def _quat_to_matrix(q_wxyz) -> np.ndarray:
    w, x, y, z = np.asarray(q_wxyz, dtype=np.float64) / np.linalg.norm(q_wxyz)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


@dataclass
class CameraModel:
    """A pinhole camera: intrinsics K (pixels) and its optical frame in the robot-root frame."""

    name: str
    width: int
    height: int
    K: np.ndarray
    T_world_cam: np.ndarray

    def __post_init__(self) -> None:
        self.width, self.height = int(self.width), int(self.height)
        self.K = np.asarray(self.K, dtype=np.float64).reshape(3, 3)
        self.T_world_cam = np.asarray(self.T_world_cam, dtype=np.float64).reshape(4, 4)

    def scaled(self, width: int, height: int) -> CameraModel:
        """The same camera at another resolution."""
        K = self.K.copy()
        K[0] *= width / self.width
        K[1] *= height / self.height
        K[2] = (0.0, 0.0, 1.0)
        return CameraModel(self.name, width, height, K, self.T_world_cam.copy())

    def project(self, pts) -> tuple[np.ndarray, np.ndarray]:
        """World points (N, 3) -> (pixel uv (N, 2), optical depth z (N,))."""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        R, t = self.T_world_cam[:3, :3], self.T_world_cam[:3, 3]
        pc = (p - t) @ R
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            uvw = pc @ self.K.T
            uv = uvw[:, :2] / uvw[:, 2:3]
        return uv, z

    def to_json(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "K": self.K.tolist(),
            "T_world_cam": self.T_world_cam.tolist(),
        }

    @classmethod
    def from_json(cls, name: str, d: dict) -> CameraModel:
        return cls(name, d["width"], d["height"], d["K"], d["T_world_cam"])


def intrinsics_from_aperture(
    focal_length: float,
    horizontal_aperture: float,
    vertical_aperture: float,
    width: int,
    height: int,
) -> np.ndarray:
    """Isaac pinhole camera intrinsics in pixels."""
    return np.array([
        [width * focal_length / horizontal_aperture, 0.0, width / 2.0],
        [0.0, height * focal_length / vertical_aperture, height / 2.0],
        [0.0, 0.0, 1.0],
    ])


def opengl_to_optical(R_gl) -> np.ndarray:
    """Rotate an OpenGL-convention camera orientation to OpenCV optical axes."""
    return np.asarray(R_gl, dtype=np.float64) @ _GL_TO_OPTICAL


def camera_from_offset(
    name: str,
    *,
    pos,
    rot_wxyz,
    convention: str,
    focal_length: float,
    horizontal_aperture: float,
    vertical_aperture: float,
    width: int,
    height: int,
    T_world_parent=None,
) -> CameraModel:
    """Camera model from Isaac Lab TiledCameraCfg.OffsetCfg semantics (pose in the parent prim)."""
    R = _quat_to_matrix(rot_wxyz)
    if convention == "opengl":
        R = opengl_to_optical(R)
    elif convention == "world":
        R = R @ _WORLD_TO_OPTICAL
    elif convention != "ros":
        raise ValueError(f"unknown camera convention {convention!r}, expected one of {CONVENTIONS}")
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(pos, dtype=np.float64).reshape(3)
    if T_world_parent is not None:
        T = np.asarray(T_world_parent, dtype=np.float64).reshape(4, 4) @ T
    K = intrinsics_from_aperture(
        focal_length, horizontal_aperture, vertical_aperture, width, height
    )
    return CameraModel(name, width, height, K, T)


def _depth2d(depth) -> np.ndarray:
    d = np.asarray(depth, dtype=np.float32)
    if d.ndim == 3 and d.shape[2] == 1:
        d = d[:, :, 0]
    if d.ndim != 2:
        raise ValueError(f"expected an (h, w) depth map, got shape {d.shape}")
    return d


def _valid(z: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    with np.errstate(invalid="ignore"):
        return np.isfinite(z) & (z >= min_depth) & (z <= max_depth)


def backproject(
    depth,
    K_depth,
    T_world_cam,
    rgb_small=None,
    *,
    stride: int = 1,
    min_depth: float = 0.05,
    max_depth: float = 3.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Valid depth pixels -> (xyz (N, 3) float32 world, rgb (N, 3) uint8; grey without rgb)."""
    d = _depth2d(depth)
    K = np.asarray(K_depth, dtype=np.float64)
    T = np.asarray(T_world_cam, dtype=np.float64)
    vs = np.arange(0, d.shape[0], stride)
    us = np.arange(0, d.shape[1], stride)
    z = d[::stride, ::stride].astype(np.float64)
    uu, vv = np.meshgrid(us, vs)
    ok = _valid(z, min_depth, max_depth)
    z, uu, vv = z[ok], uu[ok], vv[ok]
    pc = np.stack([(uu - K[0, 2]) / K[0, 0] * z, (vv - K[1, 2]) / K[1, 1] * z, z], axis=1)
    xyz = (pc @ T[:3, :3].T + T[:3, 3]).astype(np.float32)
    if rgb_small is None:
        rgb = np.full((xyz.shape[0], 3), 180, np.uint8)
    else:
        rgb = np.asarray(rgb_small)[::stride, ::stride, :3][ok].astype(np.uint8)
    return xyz, rgb


def _point_at(K: np.ndarray, T: np.ndarray, u: float, v: float, z: float) -> np.ndarray:
    pc = np.array([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z])
    return T[:3, :3] @ pc + T[:3, 3]


def pick_point(
    depth,
    K_depth,
    T_world_cam,
    u_d: float,
    v_d: float,
    *,
    window: int = 5,
    min_depth: float = 0.05,
    max_depth: float = 3.0,
) -> tuple[np.ndarray | None, float | None, np.ndarray | None]:
    """(world point, depth, camera-facing unit normal | None) at the nearest valid depth pixel."""
    d = _depth2d(depth)
    K = np.asarray(K_depth, dtype=np.float64)
    T = np.asarray(T_world_cam, dtype=np.float64)
    h, w = d.shape
    r = max(int(window) // 2, 0)
    ci, cj = round(float(v_d)), round(float(u_d))
    v0, v1 = max(ci - r, 0), min(ci + r + 1, h)
    u0, u1 = max(cj - r, 0), min(cj + r + 1, w)
    if v0 >= v1 or u0 >= u1:
        return None, None, None
    ok = _valid(d[v0:v1, u0:u1], min_depth, max_depth)
    if not ok.any():
        return None, None, None
    vv, uu = np.nonzero(ok)
    vv, uu = vv + v0, uu + u0
    best = int(np.argmin((uu - u_d) ** 2 + (vv - v_d) ** 2))
    pu, pv = int(uu[best]), int(vv[best])
    z = float(d[pv, pu])
    point = _point_at(K, T, pu, pv, z)
    tol = max(0.02, 0.1 * z)

    def neighbour(du: int, dv: int) -> np.ndarray | None:
        nu, nv = pu + du, pv + dv
        if not (0 <= nu < w and 0 <= nv < h):
            return None
        zn = float(d[nv, nu])
        if not _valid(np.array(zn), min_depth, max_depth) or abs(zn - z) > tol:
            return None
        return _point_at(K, T, nu, nv, zn)

    def tangent(du: int, dv: int) -> np.ndarray | None:
        fwd, back = neighbour(du, dv), neighbour(-du, -dv)
        if fwd is not None and back is not None:
            return fwd - back
        if fwd is not None:
            return fwd - point
        if back is not None:
            return point - back
        return None

    tu, tv = tangent(1, 0), tangent(0, 1)
    if tu is None or tv is None:
        return point, z, None
    n = np.cross(tu, tv)
    norm = np.linalg.norm(n)
    if norm <= 1e-12:
        return point, z, None
    n /= norm
    return point, z, (-n if np.dot(n, T[:3, 3] - point) < 0 else n)
