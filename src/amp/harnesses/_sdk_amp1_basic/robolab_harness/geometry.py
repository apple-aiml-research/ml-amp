#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Poses, rotations, cameras and views. Pose7 = [x, y, z, qw, qx, qy, qz], robot-root frame."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import _robot

TOP_DOWN_QUAT = (0.0, 0.0, 1.0, 0.0)


def quat_to_mat(q) -> np.ndarray:
    """wxyz quaternion -> 3x3 rotation."""
    w, x, y, z = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R) -> np.ndarray:
    """3x3 rotation -> wxyz quaternion with w >= 0."""
    R = np.asarray(R, dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.asarray(q)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def quat_mul(a, b) -> np.ndarray:
    """Hamilton product a*b (wxyz)."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def rot_x(deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_quat(roll_deg, pitch_deg, yaw_deg) -> np.ndarray:
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (fixed-axis x, then y, then z) -> wxyz."""
    return mat_to_quat(rot_z(yaw_deg) @ rot_y(pitch_deg) @ rot_x(roll_deg))


def quat_to_rpy(q) -> np.ndarray:
    """wxyz -> (roll, pitch, yaw) degrees, the inverse of rpy_to_quat."""
    R = quat_to_mat(q)
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def pose_to_T(pose7) -> np.ndarray:
    """Pose7 -> 4x4."""
    p = np.asarray(pose7, dtype=np.float64).reshape(7)
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(p[3:])
    T[:3, 3] = p[:3]
    return T


def T_to_pose(T) -> np.ndarray:
    """4x4 -> Pose7."""
    T = np.asarray(T, dtype=np.float64)
    return np.concatenate([T[:3, 3], mat_to_quat(T[:3, :3])])


def make_pose(pos, quat=TOP_DOWN_QUAT) -> list:
    """Pose7 from a position and a wxyz quaternion."""
    q = np.asarray(quat, dtype=np.float64)
    p = np.asarray(pos, dtype=np.float64).reshape(3)
    return [float(v) for v in (*p, *(q / np.linalg.norm(q)))]


def pose_from_rpy(pos, rpy_deg) -> list:
    """Pose7 from a position and (roll, pitch, yaw) degrees."""
    return make_pose(pos, rpy_to_quat(*rpy_deg))


def top_down(pos, yaw_deg=0.0) -> list:
    """Pose7 with the gripper pointing straight down; the finger axis at yaw_deg, horizontal.

    yaw 0 closes the fingers along world x (quat [0, 0, 1, 0]), yaw 90 along world y.
    """
    if _robot.ACTIVE.default_axes:
        return make_pose(pos, mat_to_quat(rot_z(yaw_deg) @ quat_to_mat(TOP_DOWN_QUAT)))
    return make_pose(pos, mat_to_quat(rot_z(yaw_deg) @ _tool_frame([-1.0, 0, 0], [0, 0, -1.0])))


def _tool_frame(closing, approach) -> np.ndarray:
    """The rotation whose columns put closing and approach on the active robot's TCP axes."""
    rob = _robot.ACTIVE
    c, a, b = rob.closing, rob.approach, rob.width_axis
    x, z = np.asarray(closing, np.float64).reshape(3), np.asarray(approach, np.float64).reshape(3)
    R = np.zeros((3, 3))
    R[:, c], R[:, a] = x, z
    # A right-handed frame: e_b = e_c x e_a when (c, a, b) is a cyclic order of (x, y, z).
    R[:, b] = np.cross(x, z) if (c, a) in ((0, 1), (1, 2), (2, 0)) else np.cross(z, x)
    return R


def finger_yaw(pose7) -> float:
    """Yaw (deg, in [-90, 90)) of the finger axis (TCP +x) projected on the horizontal plane."""
    x = quat_to_mat(np.asarray(pose7)[3:7])[:, _robot.ACTIVE.closing]
    yaw = math.degrees(math.atan2(-x[1], -x[0]))
    return (yaw + 90.0) % 180.0 - 90.0


def approach_axis(pose7) -> np.ndarray:
    """The gripper's approach direction (TCP +z) in the world."""
    return quat_to_mat(np.asarray(pose7)[3:7])[:, _robot.ACTIVE.approach]


def offset_pose(pose7, dx=0.0, dy=0.0, dz=0.0, frame="world") -> list:
    """Translate a Pose7 by (dx, dy, dz) in the world frame, or in the TCP frame (frame="tool")."""
    p = np.asarray(pose7, dtype=np.float64).copy()
    d = np.array([dx, dy, dz], dtype=np.float64)
    if frame == "tool":
        d = quat_to_mat(p[3:]) @ d
    elif frame != "world":
        raise ValueError("frame must be 'world' or 'tool'")
    p[:3] += d
    return p.tolist()


def pose_from_axes(pos, approach, closing=None) -> list:
    """Pose7 whose TCP +z is `approach` and +x (finger closing axis) is `closing` made orthogonal
    to it; closing=None picks a horizontal one."""
    z = np.asarray(approach, dtype=np.float64).reshape(3)
    z = z / np.linalg.norm(z)
    if closing is None:
        x = np.cross(z, [0.0, 0.0, 1.0])
        if np.linalg.norm(x) < 1e-6:
            x = np.array([-1.0, 0.0, 0.0])
    else:
        x = np.asarray(closing, dtype=np.float64).reshape(3)
    x = x - z * float(x @ z)
    if np.linalg.norm(x) < 1e-6:
        raise ValueError("closing axis is parallel to the approach")
    x /= np.linalg.norm(x)
    if not _robot.ACTIVE.default_axes:
        return make_pose(pos, mat_to_quat(_tool_frame(x, z)))
    return make_pose(pos, mat_to_quat(np.column_stack([x, np.cross(z, x), z])))


def side_pose(pos, azimuth_deg, tilt_deg=0.0, closing="horizontal") -> list:
    """Pose7 approaching along azimuth_deg, horizontal (0 = along +x, away from the robot),
    tilted down by tilt_deg (90 = straight down); fingers close horizontally or vertically."""
    az, t = math.radians(azimuth_deg), math.radians(tilt_deg)
    a = np.array([math.cos(t) * math.cos(az), math.cos(t) * math.sin(az), -math.sin(t)])
    if closing == "horizontal":
        return pose_from_axes(pos, a, [math.sin(az), -math.cos(az), 0.0])
    if closing == "vertical":
        return pose_from_axes(pos, a, [0.0, 0.0, 1.0])
    raise ValueError("closing must be 'horizontal' or 'vertical'")


def waypoint(t, pose7, gripper=None) -> dict:
    """Plan JSON waypoint {"t", "pos", "quat", "gripper"} from a Pose7 (gripper 0 open, 1 shut)."""
    p = np.asarray(pose7, dtype=np.float64).reshape(7)
    wp = {"t": float(t), "pos": p[:3].tolist(), "quat": (p[3:] / np.linalg.norm(p[3:])).tolist()}
    if gripper is not None:
        wp["gripper"] = int(round(float(gripper)))  # noqa: RUF046
    return wp


def waypoint_rpy(t, pos, rpy_deg, gripper=None) -> dict:
    """Waypoint from a position and (roll, pitch, yaw) degrees."""
    return waypoint(t, pose_from_rpy(pos, rpy_deg), gripper)


def waypoint_top_down(t, pos, yaw_deg=0.0, gripper=None) -> dict:
    """Waypoint pointing straight down with the finger axis at yaw_deg (see top_down)."""
    return waypoint(t, top_down(pos, yaw_deg), gripper)


@dataclass
class Camera:
    """Pinhole camera: K in pixels of a width x height image, T_world_cam in OpenCV optical axes."""

    name: str
    width: int
    height: int
    K: np.ndarray
    T_world_cam: np.ndarray

    def __post_init__(self) -> None:
        self.width, self.height = int(self.width), int(self.height)
        self.K = np.asarray(self.K, dtype=np.float64).reshape(3, 3)
        self.T_world_cam = np.asarray(self.T_world_cam, dtype=np.float64).reshape(4, 4)

    @classmethod
    def from_obs(cls, name: str, obs: dict, depth: bool = False) -> Camera:
        """The camera `name` of an observation; depth=True gives the depth image's model."""
        c = obs["cameras"][name]
        if depth:
            return cls(name, c["depth_width"], c["depth_height"], c["K_depth"], c["T_world_cam"])
        return cls(name, c["width"], c["height"], c["K"], c["T_world_cam"])

    @property
    def position(self) -> np.ndarray:
        return self.T_world_cam[:3, 3].copy()

    def scaled(self, width: int, height: int) -> Camera:
        """The same camera at another resolution."""
        K = self.K.copy()
        K[0] *= width / self.width
        K[1] *= height / self.height
        K[2] = (0.0, 0.0, 1.0)
        return Camera(self.name, width, height, K, self.T_world_cam.copy())

    def to_cam(self, pts) -> np.ndarray:
        """World points (N, 3) -> camera-frame points (N, 3)."""
        p = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        return (p - self.T_world_cam[:3, 3]) @ self.T_world_cam[:3, :3]

    def project(self, pts) -> tuple[np.ndarray, np.ndarray]:
        """World points (N, 3) -> (pixels uv (N, 2), optical depth z (N,)); z <= 0 is behind."""
        pc = self.to_cam(pts)
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = (pc @ self.K.T)[:, :2] / z[:, None]
        return uv, z

    def unproject(self, u, v, depth) -> np.ndarray:
        """Pixel(s) (u, v) at optical depth z -> world point(s) (..., 3)."""
        u, v, z = (np.asarray(a, dtype=np.float64) for a in (u, v, depth))
        K = self.K
        pc = np.stack([(u - K[0, 2]) / K[0, 0] * z, (v - K[1, 2]) / K[1, 1] * z, z], axis=-1)
        return pc @ self.T_world_cam[:3, :3].T + self.T_world_cam[:3, 3]

    def ray(self, u, v) -> tuple[np.ndarray, np.ndarray]:
        """(origin, unit direction) of the ray through pixel (u, v), world frame."""
        d = self.T_world_cam[:3, :3] @ np.linalg.solve(self.K, np.array([u, v, 1.0]))
        return self.position, d / np.linalg.norm(d)


def backproject(depth, cam: Camera, min_depth=0.05, max_depth=3.0, stride=1) -> np.ndarray:
    """Valid pixels of a depth map (H, W) seen by cam (its depth model) -> world points (N, 3)."""
    d = np.asarray(depth, dtype=np.float64)[::stride, ::stride]
    vv, uu = np.mgrid[0 : d.shape[0], 0 : d.shape[1]]
    with np.errstate(invalid="ignore"):
        ok = np.isfinite(d) & (d >= min_depth) & (d <= max_depth)
    return cam.unproject(uu[ok] * stride, vv[ok] * stride, d[ok])


def _look_at_matrix(eye, target, up) -> np.ndarray:
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye
    f /= np.linalg.norm(f)
    up = np.asarray(up, float)
    r = np.cross(f, up)
    if np.linalg.norm(r) < 1e-6:
        r = np.cross(f, [1.0, 0.0, 0.0] if abs(f[0]) < 0.9 else [0.0, 1.0, 0.0])
    r /= np.linalg.norm(r)
    d = np.cross(f, r)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([r, d, f])
    T[:3, 3] = eye
    return T


STANDARD_VIEWS = {
    # name: (azimuth_deg, elevation_deg, distance)
    "iso": (215.0, 38.0, 1.45),
    "top": (180.0, 90.0, 1.35),
    "front": (0.0, 22.0, 1.15),
    "side": (90.0, 22.0, 1.15),
    "back": (180.0, 25.0, 1.3),
}
DEFAULT_TARGET = (0.5, 0.0, 0.05)


@dataclass
class View:
    """A virtual camera: look-at (eye, target, up, vertical fov) or a real camera by name."""

    eye: tuple | None = None
    target: tuple | None = None
    up: tuple = (0.0, 0.0, 1.0)
    fov_deg: float = 50.0
    camera_name: str | None = None
    camera_model: Camera | None = field(default=None, repr=False)

    @classmethod
    def look_at(cls, target=DEFAULT_TARGET, azimuth_deg=215.0, elevation_deg=38.0, distance=1.4,
                fov_deg=50.0) -> View:
        """Orbit view: the eye `distance` m from target at azimuth (deg, counter-clockwise from
        world +x: 0 looks back at the robot from in front of it, 180 from behind the robot) and
        elevation (deg above the horizontal; 90 is straight down with robot-forward up)."""
        az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
        t = np.asarray(target, dtype=np.float64)
        eye = t + distance * np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                                       math.sin(el)])
        up = (0.0, 0.0, 1.0)
        if abs(elevation_deg) > 80.0:
            # Near the pole the image "up" is the horizontal direction away from the eye.
            up = (-math.cos(az), -math.sin(az), 0.0) if elevation_deg > 0 else \
                (math.cos(az), math.sin(az), 0.0)
        return cls(eye=tuple(eye), target=tuple(t), up=up, fov_deg=fov_deg)

    @classmethod
    def standard(cls, name: str, target=DEFAULT_TARGET, distance=None, fov_deg=50.0) -> View:
        """One of STANDARD_VIEWS: "iso", "top", "front", "side", "back"."""
        az, el, dist = STANDARD_VIEWS[name]
        return cls.look_at(target, az, el, dist if distance is None else distance, fov_deg)

    @classmethod
    def camera(cls, name: str, model: Camera | None = None) -> View:
        """A real camera ("ego_l", "ego_r", "wrist"): its own K and current pose, resolved at
        render time from the latest observation unless a Camera is given."""
        return cls(camera_name=name, camera_model=model)

    def resolve(self, width: int, height: int, cameras: dict | None = None) -> Camera:
        """The pinhole camera of this view at width x height."""
        if self.camera_name is not None or self.camera_model is not None:
            cam = self.camera_model
            if cam is None:
                if not cameras or self.camera_name not in cameras:
                    raise KeyError(f"no camera {self.camera_name!r} to render from")
                cam = cameras[self.camera_name]
            return cam.scaled(width, height)
        T = _look_at_matrix(self.eye, self.target, self.up)
        f = 0.5 * height / math.tan(math.radians(self.fov_deg) / 2)
        K = np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]])
        return Camera("view", width, height, K, T)


def _cross2(a, b) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _hull2d(xy) -> np.ndarray:
    """Convex hull (counter-clockwise) of 2D points, monotone chain."""
    pts = np.unique(np.asarray(xy, dtype=np.float64).reshape(-1, 2), axis=0)
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and _cross2(out[-1] - out[-2], p - out[-2]) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(pts[::-1])
    return np.asarray(lower[:-1] + upper[:-1])


def _wrap_yaw(yaw) -> float:
    return (float(yaw) + 90.0) % 180.0 - 90.0


@dataclass
class OBB:
    """A z-up oriented box: centre, size along its own (x, y, z), and yaw_deg of its x axis in the
    horizontal plane. fit_obb makes x the long horizontal side."""

    center: np.ndarray
    size: np.ndarray
    yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        self.center = np.asarray(self.center, dtype=np.float64).reshape(3)
        self.size = np.asarray(self.size, dtype=np.float64).reshape(3)
        self.yaw_deg = float(self.yaw_deg)

    @classmethod
    def from_json(cls, d) -> OBB | None:
        if d is None or isinstance(d, OBB):
            return d
        return cls(d["center"], d["size"], d.get("yaw_deg", 0.0))

    def to_json(self) -> dict:
        return {"center": np.round(self.center, 4).tolist(),
                "size": np.round(self.size, 4).tolist(), "yaw_deg": round(self.yaw_deg, 2)}

    @property
    def R(self) -> np.ndarray:
        return rot_z(self.yaw_deg)

    @property
    def long_axis(self) -> np.ndarray:
        """Unit horizontal vector along the longer side of the footprint."""
        return self.R[:, 0] if self.size[0] >= self.size[1] else self.R[:, 1]

    @property
    def short_axis(self) -> np.ndarray:
        return self.R[:, 1] if self.size[0] >= self.size[1] else self.R[:, 0]

    @property
    def long_yaw_deg(self) -> float:
        """Yaw (deg, [-90, 90)) of the long side."""
        a = self.long_axis
        return _wrap_yaw(math.degrees(math.atan2(a[1], a[0])))

    @property
    def top_z(self) -> float:
        return float(self.center[2] + self.size[2] / 2)

    @property
    def bottom_z(self) -> float:
        return float(self.center[2] - self.size[2] / 2)

    def corners(self) -> np.ndarray:
        """(8, 3) corners."""
        s = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        return self.center + (s * self.size / 2) @ self.R.T

    def footprint(self) -> np.ndarray:
        """(4, 2) footprint corners, counter-clockwise."""
        s = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * self.size[:2] / 2
        return self.center[:2] + s @ self.R[:2, :2].T

    def aabb(self) -> np.ndarray:
        c = self.corners()
        return np.array([c.min(0), c.max(0)])

    def local(self, pts) -> np.ndarray:
        """World points (N, 3) in the box frame (origin at the centre)."""
        return (np.asarray(pts, dtype=np.float64).reshape(-1, 3) - self.center) @ self.R

    def contains(self, pts, margin=0.0) -> np.ndarray:
        """Bool (N,): points inside the box grown by margin (m)."""
        return np.all(np.abs(self.local(pts)) <= self.size / 2 + margin, axis=1)


def fit_obb(points, robust=0.0) -> OBB:
    """z-up OBB of points (N, 3): minimum-area footprint rectangle, x the long side. robust > 0
    trims that fraction of points at each extreme (e.g. 0.01 against stray points)."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(p) == 0:
        raise ValueError("fit_obb needs points")
    lo_q, hi_q = 100 * robust, 100 * (1 - robust)
    xy = p[:, :2]
    hull = _hull2d(xy)
    best = (np.inf, 0.0)
    if len(hull) >= 3:
        for i in range(len(hull)):
            e = hull[(i + 1) % len(hull)] - hull[i]
            if np.linalg.norm(e) < 1e-9:
                continue
            ang = math.atan2(e[1], e[0])
            c, s = math.cos(ang), math.sin(ang)
            q = hull @ np.array([[c, -s], [s, c]])
            area = np.ptp(q[:, 0]) * np.ptp(q[:, 1])
            if area < best[0]:
                best = (area, ang)
    yaw = best[1]
    c, s = math.cos(yaw), math.sin(yaw)
    q = xy @ np.array([[c, -s], [s, c]])
    lo, hi = np.percentile(q, lo_q, axis=0), np.percentile(q, hi_q, axis=0)
    zlo, zhi = np.percentile(p[:, 2], lo_q), np.percentile(p[:, 2], hi_q)
    size = np.array([hi[0] - lo[0], hi[1] - lo[1], zhi - zlo])
    mid = (lo + hi) / 2
    center_xy = np.array([[c, -s], [s, c]]) @ mid
    yaw_deg = math.degrees(yaw)
    if size[1] > size[0]:
        size[[0, 1]] = size[[1, 0]]
        yaw_deg += 90.0
    return OBB([center_xy[0], center_xy[1], (zlo + zhi) / 2], size, _wrap_yaw(yaw_deg))


def pca_axes(points) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(mean (3,), axes (3, 3) as columns, longest first, half-extents (3,) along them)."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    mean = p.mean(0)
    _, _, vt = np.linalg.svd(p - mean, full_matrices=False)
    axes = vt.T
    if np.linalg.det(axes) < 0:
        axes[:, 2] *= -1
    proj = (p - mean) @ axes
    return mean, axes, (proj.max(0) - proj.min(0)) / 2


def fit_circle(xy) -> tuple[float, float, float]:
    """Least-squares circle (cx, cy, r) through 2D points (N, 2), e.g. a rim's xy."""
    p = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    A = np.column_stack([2 * p, np.ones(len(p))])
    b = (p ** 2).sum(1)
    (cx, cy, k), *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(cx), float(cy), float(math.sqrt(max(k + cx * cx + cy * cy, 0.0)))


def heightmap(xyz, lo, hi, res=0.005) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Max z per res-sized xy cell over [lo, hi] (xy corners): (H (ny, nx), NaN = empty, cell-centre
    xs (nx,), ys (ny,))."""
    p = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    lo, hi = np.asarray(lo, float)[:2], np.asarray(hi, float)[:2]
    n = np.maximum(1, np.ceil((hi - lo) / res).astype(int))
    ij = np.floor((p[:, :2] - lo) / res).astype(int)
    ok = np.all((ij >= 0) & (ij < n), axis=1)
    H = np.full((n[1], n[0]), -np.inf)
    np.maximum.at(H, (ij[ok, 1], ij[ok, 0]), p[ok, 2])
    H[~np.isfinite(H)] = np.nan
    return H, lo[0] + (np.arange(n[0]) + 0.5) * res, lo[1] + (np.arange(n[1]) + 0.5) * res


def points_in(xyz, aabb=None, obb=None, center=None, radius=None, margin=0.0) -> np.ndarray:
    """Bool mask (N,) of points inside an aabb [[lo], [hi]], an OBB, and/or a sphere."""
    p = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    m = np.ones(len(p), bool)
    if aabb is not None:
        a = np.asarray(aabb, float)
        m &= np.all((p >= a[0] - margin) & (p <= a[1] + margin), axis=1)
    if obb is not None:
        m &= OBB.from_json(obb).contains(p, margin)
    if center is not None:
        r = 0.0 if radius is None else float(radius)
        m &= np.linalg.norm(p - np.asarray(center, float).reshape(3), axis=1) <= r + margin
    return m


def _clip_poly(subject, clip) -> np.ndarray:
    """Sutherland-Hodgman: convex polygon `subject` clipped by convex ccw `clip`."""
    out = [np.asarray(p, float) for p in subject]
    clip = np.asarray(clip, float)
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break

        def side(p, a=a, b=b):
            return _cross2(b - a, p - a)

        for j in range(len(inp)):
            p, q = inp[j], inp[(j + 1) % len(inp)]
            sp, sq = side(p), side(q)
            if sp >= 0:
                out.append(p)
            if (sp >= 0) != (sq >= 0):
                out.append(p + (q - p) * (sp / (sp - sq)))
    return np.asarray(out)


def _area(poly) -> float:
    if len(poly) < 3:
        return 0.0
    x, y = poly[:, 0], poly[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2)


def _as_obb(o) -> OBB:
    if isinstance(o, OBB):
        return o
    if isinstance(o, dict):
        return OBB.from_json(o)
    if getattr(o, "obb", None) is not None:
        return o.obb
    if getattr(o, "aabb", None) is not None:
        a = np.asarray(o.aabb, float)
        return OBB((a[0] + a[1]) / 2, a[1] - a[0], 0.0)
    raise ValueError(f"no box for {o!r}")


def footprint_overlap(a, b) -> float:
    """Fraction (0..1) of a's footprint area that lies over b's (Obj, OBB or OBB dict)."""
    fa, fb = _as_obb(a).footprint(), _as_obb(b).footprint()
    area = _area(fa)
    return 0.0 if area <= 0 else min(1.0, _area(_clip_poly(fa, fb)) / area)


