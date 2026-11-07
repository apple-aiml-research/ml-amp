#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RoboDojo's two X5A arms as a /sim/robot description: X5A.urdf and STL meshes under the root."""

from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from amp.contracts.embodiment import JointSpec, LinkSpec, RobotDescription
from amp.robots.robodojo import embodiment as E

Mesh = tuple[np.ndarray, np.ndarray]
URDF_NAME = "X5A.urdf"
_STL_FACET = np.dtype([("normal", "<f4", 3), ("verts", "<f4", (3, 3)), ("attr", "<u2")])


def _xyz(text: str | None, default=(0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    if text is None:
        return default
    v = tuple(float(x) for x in text.split())
    if len(v) != 3:
        raise ValueError(f"urdf: {text!r} is not three floats")
    return v


def _origin(elem: ET.Element | None) -> np.ndarray:
    """A URDF <origin xyz rpy> as a 4x4; URDF's rpy is Rz(yaw) Ry(pitch) Rx(roll)."""
    T = np.eye(4)
    if elem is None:
        return T
    r, p, y = _xyz(elem.get("rpy"))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y),
                              math.sin(y))
    T[:3, :3] = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]) @ \
        np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]) @ \
        np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    T[:3, 3] = _xyz(elem.get("xyz"))
    return T


def base_pose(side: str) -> np.ndarray:
    """ROOT -> <side>_base_link: the root is turned as the bases are, so this is an offset only."""
    T = np.eye(4)
    T[:3, 3] = E.BASE_POS[side]
    return T


def load_stl(path: str | Path) -> Mesh:
    """An STL's (verts, faces), binary or ASCII; one vertex per facet corner, no welding."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + _STL_FACET.itemsize * count == len(data) and count:
            facets = np.frombuffer(data, dtype=_STL_FACET, offset=84, count=count)
            return _indexed(facets["verts"].reshape(-1, 3))
    return _indexed(_ascii_verts(data, path))


def _ascii_verts(data: bytes, path: str | Path) -> np.ndarray:
    rows = []
    for line in data.decode("utf-8", "replace").splitlines():
        parts = line.split()
        if parts and parts[0] == "vertex":
            if len(parts) != 4:
                raise ValueError(f"{path}: ASCII STL vertex line {line.strip()!r} is not 3 floats")
            rows.append([float(x) for x in parts[1:4]])
    if not rows or len(rows) % 3:
        raise ValueError(f"{path}: ASCII STL has {len(rows)} vertices, not a positive multiple "
                         "of 3 (and it is no binary STL)")
    return np.asarray(rows, dtype=np.float64)


def _indexed(verts: np.ndarray) -> Mesh:
    faces = np.arange(len(verts), dtype=np.uint32).reshape(-1, 3)
    return np.ascontiguousarray(verts, dtype="<f4"), faces


def _place(mesh: Mesh, T: np.ndarray, scale: tuple[float, float, float]) -> Mesh:
    """A mesh's verts in its link's frame: the visual's scale, then its origin."""
    verts, faces = mesh
    v = np.asarray(verts, dtype=np.float64) * np.asarray(scale, dtype=np.float64)
    return np.ascontiguousarray(v @ T[:3, :3].T + T[:3, 3], dtype="<f4"), faces


def _visuals(link: ET.Element) -> list[tuple[Path, np.ndarray, tuple[float, float, float]]]:
    out = []
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            continue
        out.append((Path(str(mesh.get("filename"))), _origin(visual.find("origin")),
                    _xyz(mesh.get("scale"), (1.0, 1.0, 1.0))))
    return out


def _urdf(path: Path) -> tuple[dict[str, list], list[tuple]]:
    """One arm's ({link: its visual meshes}, [(name, type, parent, child, origin, axis, limits)])."""
    robot = ET.parse(path).getroot()
    links = {}
    for link in robot.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise ValueError(f"{path}: link {name!r} is missing a name or repeated")
        links[name] = _visuals(link)
    joints = []
    for joint in robot.findall("joint"):
        name, jtype = joint.get("name"), joint.get("type")
        parent, child = joint.find("parent"), joint.find("child")
        if not name or not jtype or parent is None or child is None:
            raise ValueError(f"{path}: joint {name!r} needs a type, a parent and a child")
        plink, clink = parent.get("link"), child.get("link")
        if not plink or not clink:
            raise ValueError(f"{path}: joint {name}: parent and child need a link")
        limit, axis = joint.find("limit"), joint.find("axis")
        limits = (0.0, 0.0) if jtype == "fixed" or limit is None else \
            (float(limit.get("lower", 0.0)), float(limit.get("upper", 0.0)))
        xyz = (0.0, 0.0, 1.0) if jtype == "fixed" else \
            _xyz(axis.get("xyz") if axis is not None else None, (1.0, 0.0, 0.0))
        joints.append((name, jtype, plink, clink,
                       tuple(_origin(joint.find("origin")).reshape(-1)), xyz, limits))
    return links, joints


def description(robot_dir: str | Path) -> tuple[RobotDescription, dict[str, Mesh]]:
    """Both arms of <robot_dir>/X5A.urdf under the robot root, with their link meshes."""
    robot_dir = Path(robot_dir)
    links, joints = _urdf(robot_dir / URDF_NAME)
    children = {j[3] for j in joints}
    roots = [name for name in links if name not in children]
    if roots != [E.URDF_ROOT_LINK]:
        raise ValueError(f"{URDF_NAME}: one arm's root must be {E.URDF_ROOT_LINK!r}, got {roots}")
    out_links, out_joints, meshes = [LinkSpec(E.ROOT)], [], {}
    for side in E.SIDES:
        out_joints.append(JointSpec(name=f"{side}_base_mount", type="fixed", parent=E.ROOT,
                                    child=f"{side}_{E.URDF_ROOT_LINK}",
                                    origin=tuple(base_pose(side).reshape(-1))))
        for name, visuals in links.items():
            named = []
            for filename, T, scale in visuals:
                path = robot_dir / filename
                if filename.suffix.lower() != ".stl" or not path.is_file():
                    continue
                key = filename.stem
                if key not in meshes:
                    meshes[key] = _place(load_stl(path), T, scale)
                named.append(key)
            out_links.append(LinkSpec(f"{side}_{name}", tuple(named)))
        for name, jtype, parent, child, origin, axis, limits in joints:
            out_joints.append(JointSpec(name=f"{side}_{name}", type=jtype,
                                        parent=f"{side}_{parent}", child=f"{side}_{child}",
                                        origin=origin, axis=axis, limits=limits))
    return RobotDescription(root=E.ROOT, links=tuple(out_links),
                            joints=tuple(out_joints)), meshes
