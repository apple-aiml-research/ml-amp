#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A robot description's canonical sha256, its mesh blobs, and walks over its joint tree."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping

import numpy as np

from amp.contracts.embodiment import JointSpec, RobotDescription


def mesh_arrays(blobs: Mapping, mesh: str) -> tuple[np.ndarray, np.ndarray]:
    """A mesh's (verts float32 (N,3), faces uint32 (M,3)) from its verts/, faces/ blobs."""
    try:
        v, f = blobs[f"verts/{mesh}"], blobs[f"faces/{mesh}"]
    except KeyError as e:
        raise KeyError(f"mesh {mesh!r}: blob {e.args[0]!r} missing") from None
    return (np.ascontiguousarray(np.asarray(v), dtype="<f4").reshape(-1, 3),
            np.ascontiguousarray(np.asarray(f), dtype="<u4").reshape(-1, 3))


def description_sha256(description: RobotDescription, blobs: Mapping | None = None) -> str:
    """The canonical sha256 of a /sim/robot payload, the one Kinematics.description_sha256 names:
    sha256 over json.dumps(description.to_dict(), sort_keys=True, separators=(",", ":")), then for
    every mesh of every link in description order its verts (float32 LE) then faces (uint32 LE)."""
    h = hashlib.sha256(json.dumps(description.to_dict(), sort_keys=True,
                                  separators=(",", ":")).encode())
    for link in description.links:
        for mesh in link.meshes:
            v, f = mesh_arrays(blobs or {}, mesh)
            h.update(v.tobytes())
            h.update(f.tobytes())
    return h.hexdigest()


def parent_joints(description: RobotDescription) -> dict[str, JointSpec]:
    """Each non-root link's parent joint."""
    return {j.child: j for j in description.joints}


def path_joints(description: RobotDescription, start: str, end: str) -> list[JointSpec]:
    """The joints from link start down to link end, in order; start must be an ancestor of end."""
    up = parent_joints(description)
    out, link = [], end
    while link != start:
        j = up.get(link)
        if j is None:
            raise ValueError(f"link {start!r} is not an ancestor of {end!r}")
        out.append(j)
        link = j.parent
    return out[::-1]


def joint_motion(j: JointSpec, x: float) -> np.ndarray:
    """Parent link -> child link of a joint at value x (rad or m)."""
    T = np.array(j.origin, dtype=np.float64).reshape(4, 4)
    if j.type == "fixed" or x == 0.0:
        return T
    a = np.array(j.axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    M = np.eye(4)
    if j.type == "prismatic":
        M[:3, 3] = x * a
    else:
        K = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
        M[:3, :3] += math.sin(x) * K + (1.0 - math.cos(x)) * K @ K
    return T @ M
