#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""An official CaP-X robot read from its MjModel: the /sim/robot description and meshes."""

from __future__ import annotations

import mujoco
import numpy as np

from amp.contracts.embodiment import JointSpec, LinkSpec, RobotDescription

ROOT = "robot0_base"



MOUNT_PREFIX = "mount0_"
VISUAL_GROUP = 1
_TYPES = {mujoco.mjtJoint.mjJNT_HINGE: "revolute", mujoco.mjtJoint.mjJNT_SLIDE: "prismatic"}
_BOX_FACES = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                       [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]],
                      dtype=np.uint32)


def pose(pos, quat) -> np.ndarray:
    """4x4 of a MuJoCo (pos, wxyz quat)."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=np.float64))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R.reshape(3, 3), pos
    return T


def robot_bodies(m, mount: bool = True, mount_prefix: str = MOUNT_PREFIX) -> list[int]:
    """The root and its subtree in body order; mount=False leaves the mount's bodies out."""
    root = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ROOT)
    if root < 0:
        raise ValueError(f"the model has no {ROOT} body")
    out = [root]
    for b in range(root + 1, m.nbody):
        p = b
        while p not in (0, root):
            p = m.body_parentid[p]
        if p == root and (mount or not m.body(b).name.startswith(mount_prefix)):
            out.append(b)
    return out


def _visual_mesh(m, g) -> tuple[np.ndarray, np.ndarray]:
    if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        v = m.mesh_vert[va:va + vn].astype(np.float64)
        f = m.mesh_face[fa:fa + fn].astype(np.uint32)
    elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
        corners = [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
        v, f = np.array(corners, dtype=np.float64) * m.geom_size[g], _BOX_FACES
    else:
        raise ValueError(f"visual geom {m.geom(g).name or g} is type {int(m.geom_type[g])}, "
                         "neither a mesh nor a box")
    T = pose(m.geom_pos[g], m.geom_quat[g])
    return (v @ T[:3, :3].T + T[:3, 3]).astype(np.float32), f


def export(m, mount: bool = True, mount_prefix: str = MOUNT_PREFIX
           ) -> tuple[RobotDescription, dict[str, tuple[np.ndarray, np.ndarray]]]:
    """(description, {mesh: (verts float32 in the link frame, faces uint32)}), read only."""
    bodies = robot_bodies(m, mount, mount_prefix)
    keep = set(bodies)
    links, joints, meshes = [], [], {}
    for b in bodies:
        name = m.body(b).name
        names = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == b and m.geom_group[g] == VISUAL_GROUP:
                key = f"{name}.{m.geom(g).name or g}"
                meshes[key] = _visual_mesh(m, g)
                names.append(key)
        links.append(LinkSpec(name=name, meshes=tuple(names)))
        if b == bodies[0]:
            continue
        parent = int(m.body_parentid[b])
        if parent not in keep:
            raise ValueError(f"body {name}: parent {m.body(parent).name} is not described")
        origin = tuple(pose(m.body_pos[b], m.body_quat[b]).reshape(-1))
        js = [j for j in range(m.njnt) if m.jnt_bodyid[j] == b]
        if not js:
            joints.append(JointSpec(name=f"{name}_fixed", type="fixed", parent=m.body(parent).name,
                                    child=name, origin=origin))
            continue
        j = js[0]
        if len(js) > 1 or m.jnt_type[j] not in _TYPES or np.any(m.jnt_pos[j] != 0):
            raise ValueError(f"body {name}: only one hinge or slide joint anchored at the body "
                             "origin maps to a description joint")
        limits = tuple(map(float, m.jnt_range[j])) if m.jnt_limited[j] else (0.0, 0.0)
        joints.append(JointSpec(name=m.joint(j).name, type=_TYPES[m.jnt_type[j]],
                                parent=m.body(parent).name, child=name, origin=origin,
                                axis=tuple(map(float, m.jnt_axis[j])), limits=limits))
    return RobotDescription(root=ROOT, links=tuple(links), joints=tuple(joints)), meshes
