#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Offline z-buffered software renderer and image annotation, numpy + Pillow only."""

from __future__ import annotations

import math
from contextlib import nullcontext
from dataclasses import dataclass, field

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import _robot
from .geometry import OBB, Camera, View, fit_obb, pose_to_T, quat_to_mat

BACKGROUND = (46, 50, 58)
NEAR = 0.02
STROKE = 0.085
ROBOT_RGB = (196, 200, 208)
GHOST_RGB = (110, 160, 255)
DEFAULT_LAYERS = ("cloud", "objects", "robot", "gizmo", "labels")
ALL_LAYERS = ("cloud", "objects", "robot", "frames", "gripper", "path", "cameras", "gizmo",
              "labels")
# Distinct, saturated colours for object ids (obj1 -> index 0).
PALETTE = [(255, 99, 71), (65, 145, 255), (60, 200, 90), (255, 200, 40), (190, 90, 255),
           (0, 210, 210), (255, 130, 200), (255, 150, 30), (150, 220, 60), (120, 120, 255),
           (220, 60, 140), (60, 170, 170)]


def id_color(obj_id) -> tuple:
    """Stable colour for an object id ("obj3" -> PALETTE[2])."""
    s = str(obj_id)
    digits = "".join(ch for ch in s if ch.isdigit())
    i = int(digits) - 1 if digits else sum(map(ord, s))
    return PALETTE[i % len(PALETTE)]


def font(size: int = 16):
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def _ramp(t) -> np.ndarray:
    """Colour ramp (dark blue -> cyan -> green -> yellow -> red) for t in [0, 1] -> uint8 (N, 3)."""
    stops = np.array([[48, 18, 160], [30, 140, 255], [40, 220, 130], [250, 230, 40],
                      [230, 40, 30]], dtype=np.float64)
    t = np.clip(np.asarray(t, dtype=np.float64), 0, 1) * (len(stops) - 1)
    i = np.minimum(t.astype(int), len(stops) - 2)
    f = (t - i)[..., None]
    return (stops[i] * (1 - f) + stops[i + 1] * f).astype(np.uint8)


@dataclass
class Obj:
    """A perceived (or agent-defined) object: id, label, colour, centroid, aabb [[lo], [hi]], an
    oriented box `obb`, a world mesh and `source` ("perception" | "agent")."""

    id: str
    label: str = ""
    color: tuple | None = None
    centroid: np.ndarray | None = None
    aabb: np.ndarray | None = None
    verts: np.ndarray | None = field(default=None, repr=False)
    faces: np.ndarray | None = field(default=None, repr=False)
    kind: str = "object"
    n_points: int | None = None
    obb: OBB | None = None
    source: str | None = None
    pose: list | None = field(default=None, repr=False)     # scene pose of the mesh (Pose7)

    def __post_init__(self) -> None:
        self.obb = OBB.from_json(self.obb)
        if self.obb is None and self.verts is not None and len(self.verts) >= 3:
            self.obb = fit_obb(self.verts)

    @property
    def size(self) -> np.ndarray:
        """aabb extent (dx, dy, dz), m."""
        return np.asarray(self.aabb[1]) - np.asarray(self.aabb[0])

    @property
    def top(self) -> np.ndarray:
        """The aabb's top-face centre, a 3-vector [x, y, z]; the height alone is .top_z."""
        lo, hi = np.asarray(self.aabb[0]), np.asarray(self.aabb[1])
        return np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2]])

    @property
    def top_z(self) -> float:
        return float(np.asarray(self.aabb[1])[2])

    @property
    def bottom_z(self) -> float:
        return float(np.asarray(self.aabb[0])[2])

    def to_json(self) -> dict:
        return {"id": self.id, "label": self.label, "kind": self.kind,
                "color": None if self.color is None else [int(c) for c in self.color],
                "centroid": None if self.centroid is None else np.round(self.centroid, 4).tolist(),
                "aabb": None if self.aabb is None else np.round(self.aabb, 4).tolist(),
                "size_m": None if self.aabb is None else np.round(self.size, 4).tolist(),
                "obb": None if self.obb is None else self.obb.to_json(),
                "source": self.source,
                "n_points": self.n_points,
                "n_faces": 0 if self.faces is None else int(len(self.faces))}  # noqa: RUF046


@dataclass
class SceneData:
    """Everything the renderer draws; Episode.scene() fills it from the live episode."""

    cloud_xyz: np.ndarray | None = None
    cloud_rgb: np.ndarray | None = None
    cloud_cam: np.ndarray | None = None     # per-point index into cloud_cams
    cloud_cams: list = field(default_factory=list)
    objects: list = field(default_factory=list)
    spheres: np.ndarray | None = None       # (S, 4) x, y, z, r of the robot at the current q
    cameras: dict = field(default_factory=dict)          # name -> Camera (current poses)
    depth_cameras: dict = field(default_factory=dict)    # name -> Camera of its depth image
    tcp_pose: list | None = None
    gripper: float = 0.0                    # measured opening, 0 open .. 1 closed
    held: str | None = None


class Canvas:
    """Colour + depth buffers of one camera; every primitive is depth-tested against both."""

    def __init__(self, cam: Camera, background=BACKGROUND):
        self.cam = cam
        self.W, self.H = cam.width, cam.height
        self.z = np.full(self.W * self.H, np.inf, np.float32)
        self.rgb = np.empty((self.W * self.H, 3), np.uint8)
        self.rgb[:] = background
        self.written = np.zeros(self.W * self.H, bool)
        self.labels: list = []
        self.f = float(cam.K[0, 0])

    def fragments(self, u, v, z, rgb) -> None:
        """Integer pixel coordinates with depth and colour (N, 3) or (3,) -> buffers."""
        u, v = np.asarray(u, np.int64), np.asarray(v, np.int64)
        z = np.asarray(z, np.float32)
        ok = (u >= 0) & (u < self.W) & (v >= 0) & (v < self.H) & (z > NEAR) & np.isfinite(z)
        pix = (v * self.W + u)[ok]
        z = z[ok]
        rgb = np.asarray(rgb, np.uint8)
        if rgb.ndim == 2:
            rgb = rgb[ok]
        front = z < self.z[pix]
        pix, z = pix[front], z[front]
        if rgb.ndim == 2:
            rgb = rgb[front]
        if not len(pix):
            return
        np.minimum.at(self.z, pix, z)
        win = z <= self.z[pix]
        self.rgb[pix[win]] = rgb[win] if rgb.ndim == 2 else rgb
        self.written[pix[win]] = True

    # ---- primitives ------------------------------------------------------------------------

    def points(self, xyz, rgb, world_size=None, radius_px=1.0, max_radius=10) -> None:
        """Splats: a disc per point, world_size (m, scalar or per point) wide, else radius_px."""
        xyz = np.asarray(xyz, np.float64).reshape(-1, 3)
        if not len(xyz):
            return
        uv, z = self.cam.project(xyz)
        rgb = np.broadcast_to(np.asarray(rgb, np.uint8), (len(xyz), 3))
        ok = (z > NEAR) & np.isfinite(uv).all(1)
        ok &= (uv[:, 0] > -max_radius) & (uv[:, 0] < self.W + max_radius)
        ok &= (uv[:, 1] > -max_radius) & (uv[:, 1] < self.H + max_radius)
        uv, z, rgb = uv[ok], z[ok], rgb[ok]
        if world_size is not None:
            ws = np.broadcast_to(np.asarray(world_size, np.float64), ok.shape)[ok]
            r = 0.5 * ws * self.f / z
        else:
            r = np.full(len(z), float(radius_px))
        ri = np.clip(np.round(r - 0.25), 0, max_radius).astype(int)
        ui, vi = np.round(uv[:, 0]).astype(np.int64), np.round(uv[:, 1]).astype(np.int64)
        for rad in np.unique(ri):
            sel = ri == rad
            offs = _disc(rad)
            uu = (ui[sel, None] + offs[None, :, 0]).ravel()
            vv = (vi[sel, None] + offs[None, :, 1]).ravel()
            zz = np.repeat(z[sel], len(offs))
            cc = np.repeat(rgb[sel], len(offs), axis=0)
            self.fragments(uu, vv, zz, cc)

    def triangles(self, tris, rgb, shade=True) -> None:
        """World triangles (N, 3, 3), colour (3,) or (N, 3): flat-shaded, two-sided."""
        tris = np.asarray(tris, np.float64).reshape(-1, 3, 3)
        if not len(tris):
            return
        rgb = np.broadcast_to(np.asarray(rgb, np.float64), (len(tris), 3))
        pc = self.cam.to_cam(tris.reshape(-1, 3)).reshape(-1, 3, 3)
        ok = (pc[:, :, 2] > NEAR).all(1)
        pc, rgb = pc[ok], rgb[ok]
        if shade and len(pc):
            n = np.cross(pc[:, 1] - pc[:, 0], pc[:, 2] - pc[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            view = -pc.mean(1)
            view /= np.linalg.norm(view, axis=1, keepdims=True)
            up = self.cam.T_world_cam[:3, :3].T @ np.array([0.3, -0.4, 1.0]) / 1.118
            light = 0.45 + 0.4 * np.abs((n * view).sum(1)) + 0.2 * np.abs(n @ up)
            rgb = rgb * np.clip(light, 0, 1.15)[:, None]
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        K = self.cam.K
        uv = pc[:, :, :2] / pc[:, :, 2:3] * K[[0, 1], [0, 1]] + K[:2, 2]
        lo = np.floor(uv.min(1)).astype(np.int64)
        hi = np.ceil(uv.max(1)).astype(np.int64)
        lo = np.maximum(lo, 0)
        hi = np.minimum(hi, [self.W - 1, self.H - 1])
        vis = (hi >= lo).all(1)
        uv, pc, rgb, lo, hi = uv[vis], pc[vis], rgb[vis], lo[vis], hi[vis]
        size = (hi - lo).max(1) + 1
        bucket = np.ceil(np.log2(np.maximum(size, 1))).astype(int)
        for b in np.unique(bucket):
            S = 1 << int(b)
            idx = np.nonzero(bucket == b)[0]
            per = max(1, 3_000_000 // (S * S))
            for k in range(0, len(idx), per):
                self._raster(uv[idx[k:k + per]], pc[idx[k:k + per], :, 2], rgb[idx[k:k + per]],
                             lo[idx[k:k + per]], S)

    def _raster(self, uv, z, rgb, lo, S) -> None:
        gy, gx = np.mgrid[0:S, 0:S]
        px = lo[:, 0:1] + gx.ravel()[None]
        py = lo[:, 1:2] + gy.ravel()[None]
        x0, y0 = uv[:, 0, 0:1], uv[:, 0, 1:2]
        x1, y1 = uv[:, 1, 0:1], uv[:, 1, 1:2]
        x2, y2 = uv[:, 2, 0:1], uv[:, 2, 1:2]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        good = np.abs(area[:, 0]) > 1e-9
        w0 = (x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)
        w1 = (x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)
        with np.errstate(divide="ignore", invalid="ignore"):
            b0, b1 = w0 / area, w1 / area
            b2 = 1 - b0 - b1
            eps = -1e-6
            inside = (b0 >= eps) & (b1 >= eps) & (b2 >= eps) & good[:, None]
            invz = b0 / z[:, 0:1] + b1 / z[:, 1:2] + b2 / z[:, 2:3]
        ti, si = np.nonzero(inside)
        self.fragments(px[ti, si], py[ti, si], 1.0 / invz[ti, si], rgb[ti])

    def spheres(self, spheres, rgb=ROBOT_RGB) -> None:
        """Ray-traced shaded spheres (S, 4): x, y, z, r; colour (3,) or (S, 3)."""
        sp = np.asarray(spheres, np.float64).reshape(-1, 4)
        if not len(sp):
            return
        cols = np.broadcast_to(np.asarray(rgb, np.float64), (len(sp), 3))
        c = self.cam.to_cam(sp[:, :3])
        r = sp[:, 3]
        K = self.cam.K
        ok = c[:, 2] > r + NEAR
        Kinv = np.linalg.inv(K)
        light = np.array([-0.35, -0.6, -0.72])
        light /= np.linalg.norm(light)
        for ci, ri, col in zip(c[ok], r[ok], cols[ok]):
            uc = K[0, 0] * ci[0] / ci[2] + K[0, 2]
            vc = K[1, 1] * ci[1] / ci[2] + K[1, 2]
            rp = 1.25 * K[0, 0] * ri / math.sqrt(max(ci[2] ** 2 - ri ** 2, 1e-9)) + 1
            u0, u1 = max(int(uc - rp), 0), min(int(uc + rp) + 1, self.W - 1)
            v0, v1 = max(int(vc - rp), 0), min(int(vc + rp) + 1, self.H - 1)
            if u1 < u0 or v1 < v0:
                continue
            vv, uu = np.mgrid[v0:v1 + 1, u0:u1 + 1]
            d = np.stack([uu.ravel(), vv.ravel(), np.ones(uu.size)], 1) @ Kinv.T
            a = (d * d).sum(1)
            b = -2 * d @ ci
            cc = ci @ ci - ri * ri
            disc = b * b - 4 * a * cc
            hit = disc >= 0
            if not hit.any():
                continue
            t = (-b[hit] - np.sqrt(disc[hit])) / (2 * a[hit])
            p = d[hit] * t[:, None]
            n = (p - ci) / ri
            vdir = -p / np.linalg.norm(p, axis=1, keepdims=True)
            diff = np.clip(n @ -light, 0, 1)
            h = vdir - light
            h /= np.linalg.norm(h, axis=1, keepdims=True)
            spec = np.clip((n * h).sum(1), 0, 1) ** 24
            shade = 0.38 + 0.62 * diff
            out = col[None] * shade[:, None] + 90 * spec[:, None]
            self.fragments(uu.ravel()[hit], vv.ravel()[hit], t,
                           np.clip(out, 0, 255).astype(np.uint8))

    def lines(self, a, b, rgb, width=2, bias=0.004) -> None:
        """World segments a[i] -> b[i] (N, 3); colour (3,) or (N, 3); width in pixels."""
        a = np.asarray(a, np.float64).reshape(-1, 3)
        b = np.asarray(b, np.float64).reshape(-1, 3)
        if not len(a):
            return
        cols = np.broadcast_to(np.asarray(rgb, np.uint8), (len(a), 3))
        pa, pb = self.cam.to_cam(a), self.cam.to_cam(b)
        za, zb = pa[:, 2], pb[:, 2]
        keep = (za > NEAR) | (zb > NEAR)
        pa, pb, cols = pa[keep], pb[keep], cols[keep]
        za, zb = pa[:, 2], pb[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            s = np.clip((NEAR - za) / (zb - za), 0, 1)
            s2 = np.clip((NEAR - zb) / (za - zb), 0, 1)
        pa = np.where((za < NEAR)[:, None], pa + (pb - pa) * s[:, None], pa)
        pb = np.where((pb[:, 2] < NEAR)[:, None], pb + (pa - pb) * s2[:, None], pb)
        K = self.cam.K
        ua = pa[:, :2] / pa[:, 2:3] * K[[0, 1], [0, 1]] + K[:2, 2]
        ub = pb[:, :2] / pb[:, 2:3] * K[[0, 1], [0, 1]] + K[:2, 2]
        n = np.clip(np.ceil(np.linalg.norm(ub - ua, axis=1)), 1, 4000).astype(int) + 1
        seg = np.repeat(np.arange(len(n)), n)
        t = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)) / np.repeat(n - 1, n).clip(1)
        p = pa[seg] + (pb[seg] - pa[seg]) * t[:, None]
        z = p[:, 2] * (1 - bias)
        u = p[:, 0] / p[:, 2] * K[0, 0] + K[0, 2]
        v = p[:, 1] / p[:, 2] * K[1, 1] + K[1, 2]
        offs = _disc(max(0, (int(width) - 1) / 2))
        ui, vi = np.round(u).astype(np.int64), np.round(v).astype(np.int64)
        self.fragments((ui[:, None] + offs[None, :, 0]).ravel(),
                       (vi[:, None] + offs[None, :, 1]).ravel(),
                       np.repeat(z, len(offs)), np.repeat(cols[seg], len(offs), axis=0))

    def polyline(self, pts, rgb, width=2) -> None:
        pts = np.asarray(pts, np.float64).reshape(-1, 3)
        if len(pts) >= 2:
            rgb = np.asarray(rgb, np.uint8)
            self.lines(pts[:-1], pts[1:], rgb[:-1] if rgb.ndim == 2 else rgb, width)

    def label(self, xyz, text, color=(255, 255, 255), always=True, size=None, dot=True,
              text_color=None) -> None:
        """Queue a text label at a world point; drawn on top after rasterization."""
        uv, z = self.cam.project(np.asarray(xyz, np.float64).reshape(1, 3))
        if z[0] <= NEAR or not np.isfinite(uv).all():
            return
        u, v = float(uv[0, 0]), float(uv[0, 1])
        if not (0 <= u < self.W and 0 <= v < self.H):
            return
        if not always:
            zi = self.z[int(v) * self.W + int(u)]
            if np.isfinite(zi) and zi < z[0] - 0.03:
                return
        col = tuple(int(c) for c in color)
        if text_color is not None:
            col = (col, tuple(int(c) for c in text_color))
        self.labels.append((u, v, str(text), col, size, dot))

    def blend(self, other: Canvas, alpha: float) -> None:
        """Composite another canvas's written pixels over this one with the given opacity."""
        m = other.written & (other.z < self.z)
        self.rgb[m] = (alpha * other.rgb[m] + (1 - alpha) * self.rgb[m]).astype(np.uint8)
        self.labels.extend(other.labels)

    def image(self, label_scale=1.0) -> Image.Image:
        img = Image.fromarray(self.rgb.reshape(self.H, self.W, 3))
        draw_labels(img, self.labels, label_scale)
        return img

    def overlay_canvas(self) -> Canvas:
        """An empty canvas sharing this one's depth, for semi-transparent layers."""
        c = Canvas(self.cam)
        c.z = self.z.copy()
        return c


_DISCS: dict = {}


def _disc(r) -> np.ndarray:
    key = float(r)
    if key not in _DISCS:
        R = int(math.ceil(key))  # noqa: RUF046
        g = np.mgrid[-R:R + 1, -R:R + 1].reshape(2, -1).T
        _DISCS[key] = g[(g ** 2).sum(1) <= (key + 0.5) ** 2]
    return _DISCS[key]


def draw_labels(img: Image.Image, labels, scale=1.0) -> None:
    """Draw (u, v, text, colour, size, dot) labels with outlines, nudging overlapping ones."""
    if not labels:
        return
    draw = ImageDraw.Draw(img)
    base = max(12, int(round(img.height / 34 * scale)))  # noqa: RUF046
    placed = []
    for u, v, text, color, size, dot in labels:
        fnt = font(size or base)
        color, tcolor = (color if isinstance(color[0], tuple) else (color, color))
        if dot:
            r = max(2, base // 6)
            draw.ellipse([u - r, v - r, u + r, v + r], fill=color, outline=(0, 0, 0))
        w, h = draw.textbbox((0, 0), text, font=fnt)[2:]
        g = base * 0.4
        spots = [(u + g, v - h - g), (u + g, v + g), (u - w - g, v - h - g), (u - w - g, v + g),
                 (u - w / 2, v - h - 2 * g), (u - w / 2, v + 2 * g)]
        x, y, k = spots[0][0], spots[0][1], 0
        for k in range(len(spots) + 10):
            x, y = spots[k] if k < len(spots) else (spots[0][0], spots[0][1] + (k - 5) * (h + 3))
            box = (x - 3, y - 1, x + w + 3, y + h + 3)
            if not any(_overlap(box, p) for p in placed):
                break
        if k >= len(spots):
            draw.line([(u, v), (x, y + h / 2)], fill=color, width=1)
        x = min(max(x, 1), img.width - w - 2)
        y = min(max(y, 1), img.height - h - 2)
        placed.append((x, y, x + w, y + h))
        if tcolor != color:
            draw.rectangle([x - 3, y - 1, x + w + 3, y + h + 3], fill=(20, 22, 28), outline=color,
                           width=2)
        draw.text((x, y), text, fill=tcolor, font=fnt, stroke_width=max(2, base // 8),
                  stroke_fill=(0, 0, 0))


def _overlap(a, b) -> bool:
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


# ---- glyphs ---------------------------------------------------------------------------------

def _box_tris(center, size, R=np.eye(3), t=np.zeros(3)) -> np.ndarray:  # noqa: B008
    """(12, 3, 3) triangles of a box (local centre/size), then rotated by R and moved by t."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    w = corners @ np.asarray(R).T + t
    return w[np.array(faces)]


def _ee_scope(ee):
    """The glyph helpers follow end effector `ee`'s axes and stroke; None: whatever is active."""
    return _robot.bound_ee(ee) if ee else nullcontext()


def _scene_tcps(scene) -> list:
    """(ee, tcp_pose, opening or None) per TCP to draw: each end effector of a multi-ee scene (set by
    Episode.scene()), else the one TCP."""
    ees = getattr(scene, "end_effectors", None)
    if ees:
        return [(n, e["tcp_pose"], e.get("gripper", 0.0)) for n, e in ees.items()
                if e.get("tcp_pose") is not None]
    return [] if scene.tcp_pose is None else [(None, scene.tcp_pose, scene.gripper)]


def gripper_tris(tcp_pose, opening=0.0, stem=True) -> np.ndarray:
    """Triangles of the two-finger glyph at a TCP pose: fingers on +/- TCP x, approach +z.

    opening: 0 fully open (85 mm) .. 1 closed, the gripper command/measurement convention.
    """
    T = pose_to_T(tcp_pose)
    R, t = _glyph_R(T[:3, :3]), T[:3, 3]
    stroke = _robot.ACTIVE.stroke_m
    half = (1 - float(np.clip(opening, 0, 1))) * stroke / 2
    parts = [
        _box_tris((half + 0.006, 0, -0.012), (0.012, 0.022, 0.05), R, t),
        _box_tris((-(half + 0.006), 0, -0.012), (0.012, 0.022, 0.05), R, t),
        _box_tris((0, 0, -0.047), (stroke + 0.036, 0.03, 0.02), R, t),
    ]
    if stem:
        parts.append(_box_tris((0, 0, -0.082), (0.034, 0.034, 0.05), R, t))
    return np.concatenate(parts)


def _glyph_R(R) -> np.ndarray:
    """R with the glyph's local x, y, z on the active robot's closing, width and approach axes."""
    rob = _robot.ACTIVE
    if rob.default_axes:
        return R
    return R[:, [rob.closing, rob.width_axis, rob.approach]]


def _box(item) -> dict:
    """A box item (aabb, OBB, pixel box, or a dict of one) as a dict with "aabb" or "box"."""
    if isinstance(item, OBB):
        return {"aabb": item}
    if isinstance(item, dict):
        if "obb" in item and "aabb" not in item:
            return {**item, "aabb": OBB.from_json(item["obb"])}
        return item
    return {"box": item} if np.size(item) == 4 else {"aabb": item}


def _box_anchor(box) -> np.ndarray:
    return box.corners()[7] if isinstance(box, OBB) else np.asarray(box[1], float)


def box_edges(aabb) -> tuple[np.ndarray, np.ndarray]:
    """The 12 edges (a, b) of an axis-aligned box [[lo], [hi]] or of an OBB."""
    if isinstance(aabb, OBB):
        c = aabb.corners()
    else:
        lo, hi = np.asarray(aabb[0], float), np.asarray(aabb[1], float)
        c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                      for z in (lo[2], hi[2])])
    e = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6),
         (3, 7)]
    return c[[i for i, _ in e]], c[[j for _, j in e]]


def frame_segments(pose7, size=0.06):
    """(a, b, colours) of the x (red), y (green), z (blue) axes of a pose."""
    T = pose_to_T(pose7)
    a = np.repeat(T[:3, 3][None], 3, 0)
    b = a + T[:3, :3].T * size
    return a, b, np.array([[235, 50, 50], [50, 220, 70], [60, 110, 255]], np.uint8)


def outside_spheres(xyz, spheres, inflate=0.01) -> np.ndarray:
    """Mask of points outside every (inflated) sphere (S, 4)."""
    xyz = np.asarray(xyz, np.float64).reshape(-1, 3)
    sp = np.asarray(spheres, np.float64).reshape(-1, 4)
    keep = np.ones(len(xyz), bool)
    if not len(sp) or not len(xyz):
        return keep
    r = sp[:, 3] + inflate
    lo, hi = (sp[:, :3] - r[:, None]).min(0), (sp[:, :3] + r[:, None]).max(0)
    cand = np.nonzero(((xyz >= lo) & (xyz <= hi)).all(1))[0]
    try:
        from scipy.spatial import cKDTree

        tree = cKDTree(xyz[cand])
        for c, rad in zip(sp[:, :3], r):
            hit = tree.query_ball_point(c, rad)
            if hit:
                keep[cand[hit]] = False
        return keep
    except ImportError:
        pass
    for k in range(0, len(cand), 4000):
        idx = cand[k:k + 4000]
        d2 = ((xyz[idx, None, :] - sp[None, :, :3]) ** 2).sum(-1)
        keep[idx[(d2 <= r[None] ** 2).any(1)]] = False
    return keep


def spheres_from_links(links) -> np.ndarray:
    """Approximate robot spheres along the 11 FK link points (when A6 is unavailable)."""
    links = np.asarray(links, float).reshape(-1, 3)
    radii = [0.08, 0.07, 0.07, 0.065, 0.06, 0.055, 0.05, 0.045, 0.045, 0.04]
    out = []
    for i in range(len(links) - 1):
        a, b = links[i], links[i + 1]
        n = max(1, int(np.linalg.norm(b - a) / 0.035))
        for s in np.linspace(0, 1, n + 1):
            out.append([*(a + (b - a) * s), radii[min(i, len(radii) - 1)]])
    return np.array(out)


# ---- the scene renderer ---------------------------------------------------------------------

def _cloud_point_size(scene: SceneData, stride=1) -> np.ndarray | float:
    """World footprint of each cloud point: distance to its source camera / depth focal."""
    xyz = scene.cloud_xyz
    if scene.cloud_cam is None or not scene.depth_cameras:
        return 0.006
    size = np.full(len(xyz), 0.006)
    for i, name in enumerate(scene.cloud_cams):
        cam = scene.depth_cameras.get(name)
        if cam is None:
            continue
        sel = scene.cloud_cam == i
        d = np.linalg.norm(xyz[sel] - cam.position, axis=1)
        size[sel] = d / cam.K[0, 0] * stride
    return size * 1.9


def render_scene(scene: SceneData, view: View, layers=DEFAULT_LAYERS, overlays=None,
                 size=(960, 540), object_alpha=None, label_scale=1.0, cloud_stride=1,
                 background=BACKGROUND, title=None) -> Image.Image:
    """Render a SceneData from a View. See sdk_reference.md "Rendering" for layers and overlays."""
    W, H = int(size[0]), int(size[1])
    cam = view.resolve(W, H, scene.cameras)
    cv = Canvas(cam, background)
    layers = set(ALL_LAYERS if layers in ("all", None) else layers)
    ov = dict(overlays or {})
    highlight = set(ov.get("highlight") or ())

    if "cloud" in layers and scene.cloud_xyz is not None and len(scene.cloud_xyz):
        cv.points(scene.cloud_xyz, scene.cloud_rgb,
                  world_size=_cloud_point_size(scene, cloud_stride))
    if "robot" in layers and scene.spheres is not None:
        cv.spheres(scene.spheres, ROBOT_RGB)
    for item in ov.get("meshes") or ():
        cv.triangles(np.asarray(item["verts"])[np.asarray(item["faces"])],
                     item.get("color", (200, 200, 200)))

    if "objects" in layers:
        has_cloud = "cloud" in layers and scene.cloud_xyz is not None and len(scene.cloud_xyz)
        alpha = object_alpha if object_alpha is not None else (0.45 if has_cloud else 1.0)
        target = cv if alpha >= 1.0 else cv.overlay_canvas()
        for o in scene.objects:
            if o.verts is None or o.faces is None or not len(o.faces):
                continue
            col = np.array(id_color(o.id), float)
            if highlight and o.id not in highlight:
                col = col * 0.35 + 60
            target.triangles(np.asarray(o.verts)[np.asarray(o.faces)], col)
        if target is not cv:
            cv.blend(target, alpha)
    ghosts = ov.get("ghosts") or ()
    if ghosts:
        gcv = cv.overlay_canvas()
        for g in ghosts:
            sp = g["spheres"] if isinstance(g, dict) else g
            gcv.spheres(sp, g.get("color", GHOST_RGB) if isinstance(g, dict) else GHOST_RGB)
        cv.blend(gcv, 0.45)

    tcps = _scene_tcps(scene)
    for ee, tcp, opening in tcps:
        if "gripper" in layers and opening is not None:  # None: a tool without fingers
            with _ee_scope(ee):
                cv.triangles(gripper_tris(tcp, opening), (230, 230, 240))
        if "frames" in layers:
            _draw_frame(cv, tcp, 0.07, "TCP" if ee is None else f"TCP {ee}")
    if "cameras" in layers:
        for name, c in scene.cameras.items():
            _draw_frustum(cv, c, name)

    _draw_overlays(cv, ov)

    if "objects" in layers:
        for o in scene.objects:
            if o.aabb is None:
                continue
            text = f"{o.id} {o.label}".strip() if "labels" in layers else o.id
            if o.id == scene.held:
                text += " (held)"
            cv.label(o.top + [0, 0, 0.01], text, id_color(o.id), text_color=(255, 255, 255))
    img = cv.image(label_scale)
    if "gizmo" in layers:
        _gizmo(img, cam)
    if title:
        d = ImageDraw.Draw(img)
        d.text((8, 6), title, fill=(255, 255, 255), font=font(max(12, H // 30)),
               stroke_width=2, stroke_fill=(0, 0, 0))
    return img


def _draw_frame(cv: Canvas, pose7, size=0.06, text=None, color=(255, 255, 255)) -> None:
    a, b, c = frame_segments(pose7, size)
    cv.lines(a, b, c, width=3)
    if text:
        cv.label(np.asarray(pose7)[:3], text, color, dot=False)


def _draw_frustum(cv: Canvas, cam: Camera, name: str, depth=0.12) -> None:
    corners = [(0, 0), (cam.width, 0), (cam.width, cam.height), (0, cam.height)]
    pts = np.array([cam.unproject(u, v, depth) for u, v in corners])
    o = cam.position
    a = [o] * 4 + list(pts)
    b = list(pts) + list(np.roll(pts, -1, axis=0))
    cv.lines(a, b, (255, 220, 120), width=2)
    cv.label(o, name, (255, 220, 120))


def _path_from(plan) -> tuple[np.ndarray, list, list]:
    """(xyz (N, 3), events [(i, text)], segments) of a preview JSON, a pose list or an xyz list."""
    if isinstance(plan, dict):
        tcp = plan.get("tcp_fk") or plan.get("tcp_planned") or []
        pts = np.asarray([p[:3] for p in tcp], float).reshape(-1, 3)
        events = []
        g = plan.get("gripper") or []
        for i in range(1, len(g)):
            if g[i] != g[i - 1]:
                events.append((i, "close" if g[i] >= 0.5 else "open"))
        segs = [(s["start"], s["name"]) for s in plan.get("segments") or []]
        return pts, events, segs
    arr = [p[:3] if not isinstance(p, dict) else p["pos"] for p in plan]
    return np.asarray(arr, float).reshape(-1, 3), [], []


def _draw_path(cv: Canvas, plan, width=3, labels=True) -> None:
    pts, events, segs = _path_from(plan)
    if len(pts) < 2:
        return
    cols = _ramp(np.linspace(0, 1, len(pts)))
    cv.polyline(pts, cols, width)
    cv.label(pts[0], "start", tuple(cols[0]))
    cv.label(pts[-1], "end", tuple(cols[-1]))
    if labels:
        for i, text in events:
            cv.label(pts[min(i, len(pts) - 1)], text, (255, 255, 255))
        for i, name in segs[1:]:
            if name not in ("close", "release"):
                cv.label(pts[min(i, len(pts) - 1)], name, tuple(cols[min(i, len(pts) - 1)]),
                         size=max(10, cv.H // 44))


def _cand_color(c: dict) -> tuple:
    return (60, 220, 90) if c.get("ok", True) else (240, 70, 60)


def _draw_overlays(cv: Canvas, ov: dict) -> None:
    for item in ov.get("boxes") or ():
        d = _box(item)
        if "aabb" not in d:
            continue
        a, b = box_edges(d["aabb"])
        cv.lines(a, b, d.get("color", (255, 255, 0)), width=2)
        if d.get("label"):
            cv.label(_box_anchor(d["aabb"]), d["label"], d.get("color", (255, 255, 0)))
    for item in ov.get("frames") or ():
        d = item if isinstance(item, dict) else {"pose": item}
        _draw_frame(cv, d["pose"], d.get("size", 0.06), d.get("label"))
    for item in ov.get("grippers") or ():
        d = item if isinstance(item, dict) else {"tcp": item}
        with _ee_scope(d.get("ee")):
            tris = gripper_tris(d["tcp"], d.get("opening", 0.0))
        cv.triangles(tris, d.get("color", (230, 230, 240)))
        if d.get("label"):
            cv.label(np.asarray(d["tcp"])[:3], d["label"], d.get("color", (230, 230, 240)))
    glyphs = cv.overlay_canvas()
    for i, c in enumerate(ov.get("grasps") or ()):
        col = _cand_color(c)
        if i > 0:
            col = tuple(int(v * 0.7) for v in col)
        with _ee_scope(c.get("ee")):
            glyphs.triangles(gripper_tris(c["tcp"], 0.0, stem=False), col)
        if c.get("pregrasp") is not None:
            cv.lines([c["pregrasp"][:3]], [c["tcp"][:3]], col, width=1)
        cv.label(np.asarray(c.get("pregrasp") or c["tcp"])[:3], f"g{c.get('id', i)}", col)
    for i, c in enumerate(ov.get("places") or ()):
        col = _cand_color(c)
        with _ee_scope(c.get("ee")):
            glyphs.triangles(gripper_tris(c["tcp"], 0.6, stem=False), col)
        if c.get("object_pose") is not None:
            cv.points([c["object_pose"][:3]], col, radius_px=4)
        cv.label(np.asarray(c["tcp"])[:3], f"p{c.get('id', i)}", col)
    if glyphs.written.any():
        cv.blend(glyphs, 0.7)
    plan = ov.get("plan")
    if plan is not None:
        _draw_path(cv, plan)
    for item in ov.get("paths") or ():
        _draw_path(cv, item, labels=False)
    pts = ov.get("points")
    if pts is not None:
        for d in point_items(pts):
            col = d["color"]
            cv.points([d["xyz"]], col, radius_px=d.get("size", 5))
            if d.get("label"):
                cv.label(d["xyz"], d["label"], col, dot=False)
    for d in ov.get("text") or ():
        if "uv" in d:
            cv.labels.append((d["uv"][0], d["uv"][1], d["text"], tuple(d.get("color", (255,) * 3)),
                              d.get("size"), False))
        else:
            cv.label(d["xyz"], d["text"], d.get("color", (255, 255, 255)), dot=False)


def _gizmo(img: Image.Image, cam: Camera) -> None:
    """World axes x/y/z drawn in the lower-left corner, oriented as the view sees them."""
    draw = ImageDraw.Draw(img)
    L = max(24, img.height // 14)
    o = np.array([L * 1.4, img.height - L * 1.4])
    R = cam.T_world_cam[:3, :3]
    fnt = font(max(11, img.height // 40))
    axes = ((0, "x", (235, 70, 70)), (1, "y", (70, 220, 90)), (2, "z", (90, 140, 255)))
    for axis, name, col in axes:
        d = R.T[:, axis]
        e = o + L * np.array([d[0], d[1]])
        draw.line([tuple(o), tuple(e)], fill=col, width=3)
        draw.text((e[0] + 2, e[1] - 8), name, fill=col, font=fnt, stroke_width=2,
                  stroke_fill=(0, 0, 0))


def point_items(points, labels=None, color=(255, 60, 200)) -> list:
    """Normalise points for drawing: an (N, 3) array, a list of xyz, of dicts {"xyz", "label",
    "color", "size"}, or of Obj (their centroid, labelled by id) -> list of dicts."""
    if points is None:
        return []
    if isinstance(points, np.ndarray):
        seq = list(points.reshape(-1, 3)) if points.size else []
    elif isinstance(points, dict) or hasattr(points, "centroid"):
        seq = [points]
    else:
        seq = list(points)
        if seq and np.isscalar(seq[0]):
            seq = [seq]
    out = []
    for i, p in enumerate(seq):
        if isinstance(p, dict):
            xyz = p.get("xyz", p.get("point", p.get("pos")))
            d = {**p, "xyz": np.asarray(xyz, float).reshape(3)}
        elif hasattr(p, "centroid"):
            d = {"xyz": np.asarray(p.centroid, float).reshape(3), "label": p.id}
        else:
            d = {"xyz": np.asarray(p, float).reshape(3)}
        if labels is not None and i < len(labels) and labels[i] is not None:
            d.setdefault("label", labels[i])
        d["color"] = tuple(d.get("color") or color)
        out.append(d)
    return out


# ---- 2D helpers -----------------------------------------------------------------------------

def annotate_image(img, cam: Camera, points=None, labels=None, boxes=None, plan=None, poses=None,
                   grasps=None, objects=None, depth=None, depth_cam: Camera | None = None,
                   color=(255, 60, 200)) -> Image.Image:
    """Project 3D items into a real camera image (PIL or array) and draw them.

    points: (N, 3) world points labelled by `labels`, or dicts {"xyz", "label", "color"} as in
    render overlays, or Obj (see point_items); boxes: 3D aabbs [[lo], [hi]] or pixel
    boxes [u0, v0, u1, v1] (optionally dicts {"aabb"|"box", "label", "color"}); plan: preview JSON
    or a list of poses; poses: Pose7s drawn as axes; grasps: candidates drawn as finger pairs;
    objects: Obj list (id at the top centre and the aabb). With depth, a point hidden behind the
    visible surface is drawn hollow.
    """
    img = Image.fromarray(np.asarray(img)) if not isinstance(img, Image.Image) else img.copy()
    img = img.convert("RGB")
    if (img.width, img.height) != (cam.width, cam.height):
        cam = cam.scaled(img.width, img.height)
    draw = ImageDraw.Draw(img)
    s = max(1.0, img.height / 360)
    queued = []

    def occluded(p) -> bool:
        if depth is None or depth_cam is None:
            return False
        uv, z = depth_cam.project(np.asarray(p).reshape(1, 3))
        u, v = int(round(uv[0, 0])), int(round(uv[0, 1]))  # noqa: RUF046
        if not (0 <= u < depth_cam.width and 0 <= v < depth_cam.height) or z[0] <= 0:
            return False
        dz = float(np.asarray(depth)[v, u])
        return np.isfinite(dz) and dz > 0 and dz < z[0] - 0.02

    def proj(p):
        uv, z = cam.project(np.asarray(p, float).reshape(-1, 3))
        return uv, z

    def seg3(a, b, col, width):
        uv, z = proj([a, b])
        if (z > NEAR).all():
            draw.line([tuple(uv[0]), tuple(uv[1])], fill=col, width=width)

    if points is not None:
        items = point_items(points, labels, color)
        P = np.asarray([d["xyz"] for d in items], float).reshape(-1, 3)
        uv, z = proj(P)
        for d, p, q, zi in zip(items, P, uv, z):
            if zi <= NEAR:
                continue
            col = tuple(int(c) for c in d["color"])
            r = d.get("size", 5) * s
            hidden = occluded(p)
            draw.ellipse([q[0] - r, q[1] - r, q[0] + r, q[1] + r], outline=(0, 0, 0),
                         fill=None if hidden else col, width=int(2 * s))
            if hidden:
                draw.ellipse([q[0] - r + 2, q[1] - r + 2, q[0] + r - 2, q[1] + r - 2],
                             outline=col, width=int(2 * s))
            if d.get("label") is not None:
                queued.append((q[0], q[1], str(d["label"]) + (" (hidden)" if hidden else ""), col,
                               None, False))
    for item in boxes or ():
        d = _box(item)
        col = tuple(d.get("color", (255, 230, 0)))
        if "box" in d:
            u0, v0, u1, v1 = d["box"]
            draw.rectangle([u0, v0, u1, v1], outline=col, width=int(2 * s))
            anchor = (u0, v0)
        else:
            a, b = box_edges(d["aabb"])
            for p, q in zip(a, b):
                seg3(p, q, col, int(2 * s))
            uv, _ = proj(_box_anchor(d["aabb"]))
            anchor = tuple(uv[0])
        if d.get("label"):
            queued.append((anchor[0], anchor[1], d["label"], col, None, False))
    for o in objects or ():
        col = id_color(o.id)
        if o.aabb is not None:
            a, b = box_edges(o.aabb)
            for p, q in zip(a, b):
                seg3(p, q, col, max(1, int(s)))
            uv, z = proj(o.top)
            if z[0] > NEAR:
                queued.append((uv[0, 0], uv[0, 1], o.id, col, None, True))
    for pose in poses or ():
        a, b, cols = frame_segments(pose, 0.05)
        for p, q, c in zip(a, b, cols):
            seg3(p, q, tuple(int(x) for x in c), int(3 * s))
    for i, c in enumerate(grasps or ()):
        T = pose_to_T(c["tcp"])
        col = _cand_color(c)
        with _ee_scope(c.get("ee")):
            half = _robot.ACTIVE.stroke_m / 2 + 0.006
            Rg = _glyph_R(T[:3, :3])
        tips = [Rg @ np.array([sx * half, 0, 0.012]) + T[:3, 3] for sx in (-1, 1)]
        roots = [Rg @ np.array([sx * half, 0, -0.037]) + T[:3, 3] for sx in (-1, 1)]
        for tip, root in zip(tips, roots):
            seg3(root, tip, col, int(3 * s))
        seg3(roots[0], roots[1], col, int(3 * s))
        if c.get("pregrasp") is not None:
            seg3(c["pregrasp"][:3], T[:3, 3], col, max(1, int(s)))
        uv, z = proj(T[:3, 3])
        if z[0] > NEAR:
            queued.append((uv[0, 0], uv[0, 1], f"g{c.get('id', i)}", col, None, False))
    if plan is not None:
        pts, events, _ = _path_from(plan)
        if len(pts) >= 2:
            uv, z = proj(pts)
            cols = _ramp(np.linspace(0, 1, len(pts)))
            for k in range(len(pts) - 1):
                if z[k] > NEAR and z[k + 1] > NEAR:
                    draw.line([tuple(uv[k]), tuple(uv[k + 1])], fill=tuple(int(x) for x in cols[k]),
                              width=int(3 * s))
            for k, text in [(0, "start"), (len(pts) - 1, "end"), *events]:
                k = min(k, len(pts) - 1)
                if z[k] > NEAR:
                    queued.append((uv[k, 0], uv[k, 1], text, tuple(int(x) for x in cols[k]), None,
                                   True))
    draw_labels(img, queued, 1.0)
    return img


def crop(img, center_uv, size=(320, 180), zoom=None) -> Image.Image:
    """A size=(w, h) region of an image around center_uv (full-resolution pixels), optionally
    enlarged by `zoom` (default: to at least 640 px wide) for close inspection."""
    img = Image.fromarray(np.asarray(img)) if not isinstance(img, Image.Image) else img
    w, h = int(size[0]), int(size[1])
    u0 = int(round(center_uv[0] - w / 2))  # noqa: RUF046
    v0 = int(round(center_uv[1] - h / 2))  # noqa: RUF046
    out = img.crop((u0, v0, u0 + w, v0 + h))
    if zoom is None:
        zoom = max(1.0, 640 / w)
    if zoom != 1:
        out = out.resize((int(w * zoom), int(h * zoom)), Image.LANCZOS)
    return out


def grid(images, cols=2, titles=None, pad=4, background=(20, 20, 24)) -> Image.Image:
    """Tile images (PIL or arrays) into one, `cols` per row, with optional titles."""
    ims = [Image.fromarray(np.asarray(i)) if not isinstance(i, Image.Image) else i for i in images]
    ims = [i.convert("RGB") for i in ims]
    if not ims:
        raise ValueError("grid needs at least one image")
    cw, ch = max(i.width for i in ims), max(i.height for i in ims)
    rows = math.ceil(len(ims) / cols)
    out = Image.new("RGB", (cols * cw + (cols + 1) * pad, rows * ch + (rows + 1) * pad), background)
    draw = ImageDraw.Draw(out)
    fnt = font(max(12, ch // 26))
    for k, im in enumerate(ims):
        r, c = divmod(k, cols)
        x, y = pad + c * (cw + pad), pad + r * (ch + pad)
        out.paste(im, (x, y))
        if titles and k < len(titles) and titles[k]:
            draw.text((x + 6, y + 4), str(titles[k]), fill=(255, 255, 255), font=fnt,
                      stroke_width=2, stroke_fill=(0, 0, 0))
    return out


def depth_colormap(depth, lo=None, hi=None, legend=True) -> Image.Image:
    """A depth map (metres) as a colour image: near red .. far blue, invalid black, legend."""
    d = np.asarray(depth, np.float64)
    ok = np.isfinite(d) & (d > 0)
    if not ok.any():
        return Image.new("RGB", (d.shape[1], d.shape[0]))
    lo = float(np.percentile(d[ok], 1)) if lo is None else lo
    hi = float(np.percentile(d[ok], 99)) if hi is None else hi
    t = np.clip((d - lo) / max(hi - lo, 1e-6), 0, 1)
    rgb = _ramp(1 - t)
    rgb[~ok] = 0
    img = Image.fromarray(rgb)
    if legend:
        img = img.resize((max(img.width, 640), int(img.height * max(img.width, 640) / img.width)),
                         Image.NEAREST)
        dr = ImageDraw.Draw(img)
        dr.text((6, 4), f"near {lo:.2f} m (red) .. far {hi:.2f} m (blue); black = no depth",
                fill=(255, 255, 255), font=font(14), stroke_width=2, stroke_fill=(0, 0, 0))
    return img


def tcp_axes(pose7):
    """(origin, x, y, z) unit axes of a pose: x = finger closing axis, z = approach."""
    R = _glyph_R(quat_to_mat(np.asarray(pose7)[3:7]))
    return np.asarray(pose7)[:3], R[:, 0], R[:, 1], R[:, 2]
