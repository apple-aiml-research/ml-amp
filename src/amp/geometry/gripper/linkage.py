#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Robotiq 2F-85 finger linkage: each finger link's pose at a finger angle; the TCP frame."""

from __future__ import annotations

import math

import numpy as np

from amp.geometry.robot_asset import RobotAsset

PADS = ("left_inner_finger", "right_inner_finger")
R_TCP_EE = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def axis_rotation(axis, angle: float) -> np.ndarray:
    """Rodrigues: the rotation by angle about axis."""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * K @ K


def ee_points_to_tcp(T_ee_link: np.ndarray, verts: np.ndarray, tcp_offset: float) -> np.ndarray:
    """A link's vertices posed by T_ee_link, expressed in the TCP frame."""
    P = verts @ T_ee_link[:3, :3].T + T_ee_link[:3, 3]
    P = P @ R_TCP_EE.T
    P[:, 2] -= tcp_offset
    return P


class GripperLinkage:
    """The asset's finger links (ee-frame links with a parent), each geared to theta."""

    def __init__(self, asset: RobotAsset):
        self.closed_angle = asset.closed_angle
        self.links = {n: asset.links[n] for n in asset.finger_links()}

    def poses(self, theta: float) -> dict[str, np.ndarray]:
        """T_ee_link of every finger link at finger angle theta (0 = open)."""
        out: dict[str, np.ndarray] = {}

        def pose(name: str) -> np.ndarray:
            if name not in out:
                lk = self.links[name]
                T_parent = pose(lk.parent) if lk.parent in self.links else np.eye(4)
                J = np.eye(4)
                if lk.gearing != 0.0:
                    piv = np.asarray(lk.pivot, dtype=np.float64)
                    Rj = axis_rotation(lk.axis, lk.gearing * theta)
                    J[:3, :3], J[:3, 3] = Rj, piv - Rj @ piv
                out[name] = T_parent @ J
            return out[name]

        for name in self.links:
            pose(name)
        return out

    def tcp_points(self, theta: float, tcp_offset: float, verts=None) -> dict[str, np.ndarray]:
        """Each finger link's vertices (or verts[name] instead) at theta, in the TCP frame."""
        T = self.poses(theta)
        return {n: ee_points_to_tcp(T[n], lk.verts if verts is None else verts[n], tcp_offset)
                for n, lk in self.links.items() if verts is None or n in verts}

    def pad_faces(self, tol: float) -> dict[str, np.ndarray]:
        """The inner fingers' contact-face vertices: within tol of the face nearest the gap."""
        out = {}
        for n in PADS:
            V = self.links[n].verts
            ay = np.abs(V[:, 1])
            out[n] = V[ay <= ay.min() + tol]
        return out
