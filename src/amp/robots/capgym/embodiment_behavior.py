#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""capx_b1k_r1pro: CaP-X's R1Pro (BEHAVIOR-1K) as AMP declares it, on embodiment contract 2."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET  # nosemgrep: python.lang.security.use-defused-xml.use-defused-xml
from pathlib import Path

import numpy as np

from amp.contracts.embodiment import (
    ActionSegment,
    ArmSpec,
    BaseSpec,
    CameraSpec,
    DriveSpec,
    EmbodimentSpec,
    EndEffectorSpec,
    GripperSpec,
    JointGroupSpec,
    JointSpec,
    Kinematics,
    LinkSpec,
    RobotDescription,
    WorkspaceSpec,
)

NAME, TASK_SET = "capx_b1k_r1pro", "capgym_b1k"
CONTROL_DT = 1.0 / 30.0
ROOT = "base_link"
BASE, TORSO_GROUP = "base", "torso"
ARMS, HANDS = ("left", "right"), ("left_hand", "right_hand")
BASE_ALL = tuple(f"base_footprint_{c}_joint" for c in ("x", "y", "z", "rx", "ry", "rz"))
BASE_JOINTS = (BASE_ALL[0], BASE_ALL[1], BASE_ALL[5])
TORSO = tuple(f"torso_joint{i}" for i in range(1, 5))
ARM_JOINTS = {a: tuple(f"{a}_arm_joint{i}" for i in range(1, 8)) for a in ARMS}
FINGER_JOINTS = {a: (f"{a}_gripper_finger_joint1", f"{a}_gripper_finger_joint2") for a in ARMS}

OG_JOINTS = (*BASE_ALL, *TORSO,
             *(j for pair in zip(ARM_JOINTS["left"], ARM_JOINTS["right"]) for j in pair),
             *FINGER_JOINTS["left"], *FINGER_JOINTS["right"])

Q_JOINTS = (*ARM_JOINTS["left"], *ARM_JOINTS["right"], *TORSO)

OG_ACTION_DIM = 25
FLANGE_LINKS = {a: f"{a}_arm_link7" for a in ARMS}
EE_LINKS = {a: f"{a}_gripper_link" for a in ARMS}



T_EE_TCP = (-1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, -1.0, -0.06,
            0.0, 0.0, 0.0, 1.0)

TORSO_MIN = (-1.1345, -2.7925, -1.8326, -3.0543)
TORSO_MAX = (1.8326, 2.5307, 1.5708, 3.0543)
TORSO_QD = (2.5, 2.5, 2.5, 2.5)
ARM_MIN = {"left": (-4.4506, -0.1745, -2.356196, -2.0944, -2.356196, -1.047198, -1.5708),
           "right": (-4.4506, -3.1416, -2.356196, -2.0944, -2.356196, -1.047198, -1.5708)}
ARM_MAX = {"left": (1.3090, 3.1416, 2.356196, 0.3491, 2.356196, 1.047198, 1.5708),
           "right": (1.3090, 0.1745, 2.356196, 0.3491, 2.356196, 1.047198, 1.5708)}
ARM_QD = (7.1209, 7.1209, 8.3776, 8.3776, 10.4720, 10.4720, 10.4720)
FINGER_OPEN, FINGER_CLOSED = 0.05, 0.0
Q_HOME_ARM = (0.0,) * 7


BASE_QD = (0.5, 0.5, 1.0)

T_FLANGE_EE = (1.0, 0.0, 0.0, -0.0295,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, -0.16065,
               0.0, 0.0, 0.0, 1.0)
IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
CAMERA_SIZE = 512
CAMERAS = {"head": "zed_link", "left_wrist": "left_realsense_link",
           "right_wrist": "right_realsense_link"}
URDF = "r1pro.urdf"
URDF_SHA256 = "b0d76760cbedfbcbe1ddc280380dd5f497bbca380313bc096d7239a2c50dae61"

DESCRIPTION_SHA256 = "155faa041a5927fd9251ff6f44cee555aaf1e8d1423d83979d3d46e2f2f415dd"


def og_sensor(robot_name: str, camera: str) -> str:
    """OmniGibson's sensor and obs key of a declared camera."""
    return f"{robot_name}:{CAMERAS[camera]}:Camera:0"


def _arm(arm: str) -> tuple[ArmSpec, EndEffectorSpec]:
    spec = ArmSpec(name=arm, joints=ARM_JOINTS[arm], q_min=ARM_MIN[arm], q_max=ARM_MAX[arm],
                   qd_max=ARM_QD, q_home=Q_HOME_ARM, base_link=f"{arm}_arm_base_link",
                   flange_link=FLANGE_LINKS[arm],
                   kinematics=Kinematics("chain", DESCRIPTION_SHA256))

    gripper = GripperSpec(kind="parallel_jaw", model="linear", stroke_m=2 * FINGER_OPEN,
                          max_width_m=2 * FINGER_OPEN, finger_joints=FINGER_JOINTS[arm],
                          finger_open=(FINGER_OPEN,) * 2, finger_closed=(FINGER_CLOSED,) * 2)


    hand = EndEffectorSpec(name=f"{arm}_hand", arm=arm, ee_link=EE_LINKS[arm],
                           T_flange_ee=T_FLANGE_EE, T_ee_tcp=T_EE_TCP, gripper=gripper,
                           closing_axis="y", approach_axis="z")
    return spec, hand


def capx_b1k_r1pro() -> EmbodimentSpec:
    """CaP-X's R1Pro: holonomic base, a 4-joint torso carrying two 7-joint arms, 2-finger hands."""
    (left, lh), (right, rh) = _arm("left"), _arm("right")
    torso = JointGroupSpec(name=TORSO_GROUP, kind="torso", joints=TORSO, q_min=TORSO_MIN,
                           q_max=TORSO_MAX, qd_max=TORSO_QD, q_home=(0.0,) * 4,
                           base_link=ROOT, tip_link="torso_link4", carries=ARMS)
    return EmbodimentSpec(
        name=NAME, arms=(left, right), end_effectors=(lh, rh),
        cameras=tuple(CameraSpec(name, CAMERA_SIZE, CAMERA_SIZE, mount=link, depth=True,
                                 role="scene" if name == "head" else "wrist")
                      for name, link in CAMERAS.items()),

        action=(ActionSegment("base_pose_delta", BASE, 3),
                ActionSegment("joint_pos_abs", TORSO_GROUP, 4),
                ActionSegment("joint_pos_abs", "left", 7), ActionSegment("gripper01", HANDS[0], 1),
                ActionSegment("joint_pos_abs", "right", 7),
                ActionSegment("gripper01", HANDS[1], 1)),
        control_dt=CONTROL_DT,

        drive=DriveSpec(),
        workspace=WorkspaceSpec(support="floor", box=(-0.5, 1.5, -1.0, 1.0, -0.1, 2.0)),
        base=BaseSpec(name=BASE, joints=BASE_JOINTS, qd_max=BASE_QD),
        joint_groups=(torso,))


def _origin(el) -> tuple[float, ...]:
    """A URDF <origin xyz rpy> as a 4x4 row-major pose (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    xyz = [float(v) for v in (el.get("xyz", "0 0 0") if el is not None else "0 0 0").split()]
    r, p, y = [float(v) for v in (el.get("rpy", "0 0 0") if el is not None else "0 0 0").split()]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), \
        math.cos(y), math.sin(y)
    R = ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
         (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
         (-sp, cp * sr, cp * cr))
    return (*R[0], xyz[0], *R[1], xyz[1], *R[2], xyz[2], 0.0, 0.0, 0.0, 1.0)


def read_obj(path: Path, T: tuple[float, ...] = IDENTITY) -> tuple[np.ndarray, np.ndarray]:
    """(verts float32 Nx3 in the link frame, faces uint32 Mx3) of a Wavefront OBJ, fans split."""
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
            idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
            faces.extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    M = np.reshape(T, (4, 4))
    v = v @ M[:3, :3].T + M[:3, 3]
    return v.astype(np.float32), np.asarray(faces, dtype=np.uint32).reshape(-1, 3)


def describe_urdf(path: str | Path, *, root: str = ROOT,
                  meshes: bool = True) -> tuple[RobotDescription, dict]:
    """(description, visual meshes) of the robot's URDF from base_link, read as is."""
    path = Path(path)
    tree = ET.parse(str(path)).getroot()  # nosec B314  # nosemgrep: python.lang.security.use-defused-xml-parse.use-defused-xml-parse
    joints = []
    for j in tree.findall("joint"):
        kind = j.get("type")
        lim, axis = j.find("limit"), j.find("axis")

        limits = (0.0, 0.0) if kind in ("fixed", "continuous") or lim is None else \
            (float(lim.get("lower", 0.0)), float(lim.get("upper", 0.0)))
        joints.append(JointSpec(
            name=j.get("name"), type="revolute" if kind == "continuous" else kind,
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            origin=_origin(j.find("origin")),
            axis=tuple(float(v) for v in axis.get("xyz").split()) if axis is not None
            else (0.0, 0.0, 1.0), limits=limits))
    keep, grew = {root}, True
    while grew:
        grew = False
        for js in joints:
            if js.parent in keep and js.child not in keep:
                keep.add(js.child)
                grew = True
    links, blobs = [], {}
    for lk in tree.findall("link"):
        name = lk.get("name")
        if name not in keep:
            continue
        names = []
        for i, vis in enumerate(lk.findall("visual")):
            mesh = vis.find("geometry/mesh")
            if mesh is None or not meshes:
                continue
            key = f"{name}.{i}"
            blobs[key] = read_obj(path.parent / mesh.get("filename"), _origin(vis.find("origin")))
            names.append(key)
        links.append(LinkSpec(name, tuple(names)))
    return (RobotDescription(root=root, links=tuple(links),
                             joints=tuple(js for js in joints if js.child in keep)), blobs)


EMBODIMENTS = {TASK_SET: capx_b1k_r1pro}
