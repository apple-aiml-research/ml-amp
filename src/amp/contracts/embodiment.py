#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A robot's embodiment as its backend declares it: arms, end effectors, cameras, action layout."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass, field

EMBODIMENT_FORMAT = "amp-embodiment-1"
EMBODIMENT_FORMAT_2 = "amp-embodiment-2"
EMBODIMENT_FORMATS = (EMBODIMENT_FORMAT, EMBODIMENT_FORMAT_2)
DROID = "droid_panda_robotiq"
ANY = "*"

KINEMATICS_KINDS = ("dh_craig", "chain")
GRIPPER_KINDS = ("parallel_jaw", "none")
GRIPPER_MODELS = ("robotiq_2f85_linkage", "linear", "")
ACTION_KINDS = ("joint_pos_abs", "gripper01", "base_pose_delta")
CAMERA_ROLES = ("scene", "wrist")
SUPPORTS = ("table", "floor", "none")
AXES = ("x", "y", "z")
JOINT_GROUP_KINDS = ("torso",)
BASE_KINDS = ("holonomic_planar",)
BASE_DIMS = {"holonomic_planar": 3}
WORKSPACE_FRAMES = ("root", "world")

_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_.-]*")


def _pose(v, what: str) -> tuple[float, ...]:
    t = tuple(float(x) for x in v)
    if len(t) != 16 or not all(math.isfinite(x) for x in t):
        raise ValueError(f"{what}: a pose is 16 finite floats (4x4 row-major)")
    return t


def _floats(v, n: int, what: str) -> tuple[float, ...]:
    t = tuple(float(x) for x in v)
    if len(t) != n or not all(math.isfinite(x) for x in t):
        raise ValueError(f"{what}: {n} finite floats, got {len(t)}")
    return t


def _name(v: str, what: str) -> str:
    if not isinstance(v, str) or not _NAME.fullmatch(v):
        raise ValueError(f"{what} {v!r} must match {_NAME.pattern}")
    return v


@dataclass(frozen=True)
class Kinematics:
    """dh_craig: AMP's built-in Panda chain (DROID); chain: the backend's /sim/robot joint tree."""

    kind: str
    description_sha256: str = ""

    def __post_init__(self):
        if self.kind not in KINEMATICS_KINDS:
            raise ValueError(f"kinematics {self.kind!r} not in {KINEMATICS_KINDS}")
        if self.kind == "chain" and not re.fullmatch(r"[0-9a-f]{64}", self.description_sha256):
            raise ValueError("a chain names the sha256 of the /sim/robot description it comes from")


@dataclass(frozen=True)
class Singularity:
    """Keep |q[joint]| at least min_abs, pushing back within zone (the Panda shoulder: 1, 0.10)."""

    joint: int
    min_abs: float
    zone: float


@dataclass(frozen=True)
class ArmSpec:
    name: str
    joints: tuple[str, ...]
    q_min: tuple[float, ...]
    q_max: tuple[float, ...]
    qd_max: tuple[float, ...]
    q_home: tuple[float, ...]
    base_link: str
    flange_link: str
    kinematics: Kinematics
    singularities: tuple[Singularity, ...] = ()

    def __post_init__(self):
        _name(self.name, "arm")
        n = len(self.joints)
        if n < 1 or len(set(self.joints)) != n:
            raise ValueError(f"arm {self.name}: joints must be non-empty and unique")
        for f in ("q_min", "q_max", "qd_max", "q_home"):
            object.__setattr__(self, f, _floats(getattr(self, f), n, f"arm {self.name} {f}"))
        if any(lo > hi for lo, hi in zip(self.q_min, self.q_max)):
            raise ValueError(f"arm {self.name}: q_min above q_max")
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "singularities", tuple(self.singularities))
        for s in self.singularities:
            if not 0 <= s.joint < n:
                raise ValueError(f"arm {self.name}: singularity joint {s.joint} out of range")


@dataclass(frozen=True)
class JointGroupSpec:
    """Joints outside every arm, the description's path base_link -> tip_link; the arms it carries
    hang from tip_link, so a carried arm's chain from the root runs through these joints first."""

    name: str
    kind: str
    joints: tuple[str, ...]
    q_min: tuple[float, ...]
    q_max: tuple[float, ...]
    qd_max: tuple[float, ...]
    q_home: tuple[float, ...]
    base_link: str
    tip_link: str
    carries: tuple[str, ...]

    def __post_init__(self):
        _name(self.name, "joint group")
        if self.kind not in JOINT_GROUP_KINDS:
            raise ValueError(f"joint group {self.name}: kind {self.kind!r} not in "
                             f"{JOINT_GROUP_KINDS}")
        n = len(self.joints)
        if n < 1 or len(set(self.joints)) != n:
            raise ValueError(f"joint group {self.name}: joints must be non-empty and unique")
        for f in ("q_min", "q_max", "qd_max", "q_home"):
            object.__setattr__(self, f, _floats(getattr(self, f), n, f"joint group {self.name} {f}"))
        if any(lo > hi for lo, hi in zip(self.q_min, self.q_max)):
            raise ValueError(f"joint group {self.name}: q_min above q_max")
        object.__setattr__(self, "joints", tuple(self.joints))
        object.__setattr__(self, "carries", tuple(self.carries))
        if not self.carries or len(set(self.carries)) != len(self.carries):
            raise ValueError(f"joint group {self.name}: a torso carries one or more distinct arms")


@dataclass(frozen=True)
class BaseSpec:
    """A mobile base."""

    name: str
    joints: tuple[str, ...]
    qd_max: tuple[float, ...]
    kind: str = "holonomic_planar"

    def __post_init__(self):
        _name(self.name, "base")
        if self.kind not in BASE_KINDS:
            raise ValueError(f"base {self.name}: kind {self.kind!r} not in {BASE_KINDS}")
        n = BASE_DIMS[self.kind]
        object.__setattr__(self, "joints", tuple(self.joints))
        if len(self.joints) != n or len(set(self.joints)) != n:
            raise ValueError(f"base {self.name}: {self.kind} names {n} distinct joints")
        object.__setattr__(self, "qd_max", _floats(self.qd_max, n, f"base {self.name} qd_max"))
        if any(v <= 0 for v in self.qd_max):
            raise ValueError(f"base {self.name}: qd_max must be positive")

    @property
    def dim(self) -> int:
        return BASE_DIMS[self.kind]


@dataclass(frozen=True)
class GripperSpec:
    """Readings and commands are 0 open .. 1 closed on the wire."""

    kind: str = "parallel_jaw"
    model: str = "linear"
    stroke_m: float = 0.0
    max_width_m: float = 0.0
    closed_empty: float = 1.0
    held_below: float = 1.0
    finger_joints: tuple[str, ...] = ()
    finger_open: tuple[float, ...] = ()
    finger_closed: tuple[float, ...] = ()
    tip_reach_m: tuple[float, ...] = ()

    def __post_init__(self):
        if self.kind not in GRIPPER_KINDS:
            raise ValueError(f"gripper {self.kind!r} not in {GRIPPER_KINDS}")
        if self.model not in GRIPPER_MODELS:
            raise ValueError(f"gripper model {self.model!r} not in {GRIPPER_MODELS}")
        if self.kind == "parallel_jaw" and not 0 < self.max_width_m <= self.stroke_m:
            raise ValueError("a parallel jaw needs 0 < max_width_m <= stroke_m")
        n = len(self.finger_joints)
        object.__setattr__(self, "finger_joints", tuple(self.finger_joints))
        object.__setattr__(self, "finger_open", _floats(self.finger_open, n, "finger_open"))
        object.__setattr__(self, "finger_closed", _floats(self.finger_closed, n, "finger_closed"))
        reach = tuple(self.tip_reach_m)
        if reach:
            reach = _floats(reach, 2, "tip_reach_m")
            if any(x < 0 for x in reach):
                raise ValueError("tip_reach_m: (open, closing) metres, non-negative")
        object.__setattr__(self, "tip_reach_m", reach)


@dataclass(frozen=True)
class EndEffectorSpec:
    name: str
    arm: str
    ee_link: str
    T_flange_ee: tuple[float, ...]
    T_ee_tcp: tuple[float, ...]
    gripper: GripperSpec | None
    closing_axis: str = "x"
    approach_axis: str = "z"

    def __post_init__(self):
        _name(self.name, "end effector")
        object.__setattr__(self, "T_flange_ee", _pose(self.T_flange_ee, f"{self.name} T_flange_ee"))
        object.__setattr__(self, "T_ee_tcp", _pose(self.T_ee_tcp, f"{self.name} T_ee_tcp"))
        if self.closing_axis not in AXES or self.approach_axis not in AXES or \
                self.closing_axis == self.approach_axis:
            raise ValueError(f"{self.name}: closing and approach axes must be distinct of {AXES}")


@dataclass(frozen=True)
class CameraSpec:
    """Intrinsics and T_world_cam travel with every frame; the spec fixes identity and size."""

    name: str
    width: int
    height: int
    mount: str = "root"
    depth: bool = True
    role: str = "scene"

    def __post_init__(self):
        _name(self.name, "camera")
        if self.width < 1 or self.height < 1:
            raise ValueError(f"camera {self.name}: size must be positive")
        if self.role not in CAMERA_ROLES:
            raise ValueError(f"camera {self.name}: role {self.role!r} not in {CAMERA_ROLES}")


@dataclass(frozen=True)
class ActionSegment:
    """One action: joint_pos_abs (absolute joint targets, rad), gripper01 (0 open to 1 closed) and
    base_pose_delta (dx m, dy m, dyaw rad in the base frame)."""

    kind: str
    target: str
    dim: int

    def __post_init__(self):
        if self.kind not in ACTION_KINDS:
            raise ValueError(f"action {self.kind!r} not in {ACTION_KINDS}")
        if self.dim < 1:
            raise ValueError("an action segment has at least one dimension")


def _tau(tau_s, tau_band_s, what: str) -> tuple[float, float]:
    lo, hi = tau_band_s
    if not 0 < lo <= tau_s <= hi:
        raise ValueError(f"{what}: need 0 < tau_band_s[0] <= tau_s <= tau_band_s[1]")
    return float(lo), float(hi)


@dataclass(frozen=True)
class GroupDriveSpec:
    """A joint group's or the base's drive priors; an unset one is unknown until measured."""

    target: str
    tau_s: float | None = None
    tau_band_s: tuple[float, ...] = ()
    hold_offset_max: tuple[float, ...] = ()
    step_clamp: tuple[float, ...] = ()

    def __post_init__(self):
        what = f"drive {self.target}"
        object.__setattr__(self, "tau_band_s", tuple(self.tau_band_s))
        if (self.tau_s is None) != (not self.tau_band_s):
            raise ValueError(f"{what}: tau_s and tau_band_s are given together or not at all")
        if self.tau_s is not None:
            object.__setattr__(self, "tau_s", float(self.tau_s))
            object.__setattr__(self, "tau_band_s", _tau(self.tau_s, _floats(
                self.tau_band_s, 2, f"{what} tau_band_s"), what))
        for f in ("hold_offset_max", "step_clamp"):
            v = tuple(getattr(self, f))
            object.__setattr__(self, f, _floats(v, len(v), f"{what} {f}"))
        if any(x <= 0 for x in self.step_clamp) or any(x < 0 for x in self.hold_offset_max):
            raise ValueError(f"{what}: step_clamp positive, hold_offset_max non-negative")


@dataclass(frozen=True)
class DriveSpec:
    """How the robot's controller tracks a joint target: the executor's priors."""

    tau_s: float = 0.2
    tau_band_s: tuple[float, float] = (0.16, 0.23)
    hold_offset_max: tuple[float, ...] = ()
    step_clamp: tuple[float, ...] = ()
    groups: tuple[GroupDriveSpec, ...] = ()

    def __post_init__(self):
        object.__setattr__(self, "tau_band_s", _tau(self.tau_s, self.tau_band_s, "drive"))
        object.__setattr__(self, "hold_offset_max", tuple(float(x) for x in self.hold_offset_max))
        object.__setattr__(self, "step_clamp", tuple(float(x) for x in self.step_clamp))
        object.__setattr__(self, "groups", tuple(self.groups))


@dataclass(frozen=True)
class WorkspaceSpec:
    """Where the robot works, in the robot-root frame (x forward, y left, z up), or in the world
    (frame "world", z up) for a robot whose base moves its root."""

    support: str = "table"
    box: tuple[float, ...] = (-0.3, 1.2, -0.8, 0.8, -0.1, 1.0)
    table_region: tuple[float, ...] = ()
    frame: str = "root"

    def __post_init__(self):
        if self.support not in SUPPORTS:
            raise ValueError(f"support {self.support!r} not in {SUPPORTS}")
        if self.frame not in WORKSPACE_FRAMES:
            raise ValueError(f"workspace frame {self.frame!r} not in {WORKSPACE_FRAMES}")
        b = _floats(self.box, 6, "workspace box")
        if b[0] >= b[1] or b[2] >= b[3] or b[4] >= b[5]:
            raise ValueError("workspace box: each min must be below its max")
        object.__setattr__(self, "box", b)
        object.__setattr__(self, "table_region", tuple(self.table_region))
        if self.table_region:
            r = _floats(self.table_region, 7, "table_region")
            if r[0] >= r[1] or r[2] <= 0 or r[3] >= r[4] or r[5] > r[6]:
                raise ValueError("table_region: x_lo < x_hi, y_abs > 0, z_lo < z_hi, clamp lo <= hi")
            object.__setattr__(self, "table_region", r)


@dataclass(frozen=True)
class EmbodimentSpec:
    name: str
    arms: tuple[ArmSpec, ...]
    end_effectors: tuple[EndEffectorSpec, ...]
    cameras: tuple[CameraSpec, ...]
    action: tuple[ActionSegment, ...]
    control_dt: float
    drive: DriveSpec = field(default_factory=DriveSpec)
    workspace: WorkspaceSpec = field(default_factory=WorkspaceSpec)
    format: str = ""
    base: BaseSpec | None = None
    joint_groups: tuple[JointGroupSpec, ...] = ()

    def __post_init__(self):
        _name(self.name, "embodiment")
        object.__setattr__(self, "joint_groups", tuple(self.joint_groups))
        v2 = bool(self.base is not None or self.joint_groups or self.drive.groups
                  or self.workspace.frame != "root")
        need = EMBODIMENT_FORMAT_2 if v2 else EMBODIMENT_FORMAT
        if not self.format:
            object.__setattr__(self, "format", need)
        if self.format not in EMBODIMENT_FORMATS:
            raise ValueError(f"embodiment format {self.format!r} is not one of "
                             f"{EMBODIMENT_FORMATS}")
        if v2 and self.format == EMBODIMENT_FORMAT:
            raise ValueError(f"embodiment format {self.format!r}: a base, joint groups or a world "
                             f"workspace need {EMBODIMENT_FORMAT_2}")
        for f in ("arms", "end_effectors", "cameras", "action"):
            object.__setattr__(self, f, tuple(getattr(self, f)))
        if not self.arms or not self.cameras or not self.action:
            raise ValueError(f"{self.name}: an embodiment has an arm, a camera and an action")
        arms = {a.name: a for a in self.arms}
        ees = {e.name: e for e in self.end_effectors}
        cams = [c.name for c in self.cameras]
        if len(arms) != len(self.arms) or len(ees) != len(self.end_effectors) or \
                len(set(cams)) != len(cams):
            raise ValueError(f"{self.name}: arm, end-effector and camera names must be unique")
        for e in self.end_effectors:
            if e.arm not in arms:
                raise ValueError(f"end effector {e.name}: unknown arm {e.arm!r}")
        groups = self._check_groups(arms)
        bases = 0
        for s in self.action:
            if s.kind == "joint_pos_abs":
                chain = arms.get(s.target) or groups.get(s.target)
                if chain is None or s.dim != len(chain.joints):
                    what = "an arm or joint group" if groups else "an arm"
                    raise ValueError(f"action {s.kind}/{s.target}: {what}, dim = its joints")
            elif s.kind == "base_pose_delta":
                bases += 1
                if self.base is None or s.target != self.base.name or s.dim != self.base.dim \
                        or bases > 1:
                    raise ValueError(f"action {s.kind}/{s.target}: the base, once, dim = its "
                                     "axes")
            elif s.target not in ees or s.dim != 1 or ees[s.target].gripper is None:
                raise ValueError(f"action {s.kind}/{s.target}: an end effector with a gripper, "
                                 "dim 1")
        if self.drive.hold_offset_max and len(self.drive.hold_offset_max) != self.arm_dof:
            raise ValueError(f"{self.name}: drive.hold_offset_max needs one value per joint")
        if self.drive.step_clamp and len(self.drive.step_clamp) != self.arm_dof:
            raise ValueError(f"{self.name}: drive.step_clamp needs one value per joint")
        self._check_group_drives(groups)
        if self.workspace.frame == "world" and self.base is None:
            raise ValueError(f"{self.name}: a world-frame workspace needs a base")
        if not self.control_dt > 0:
            raise ValueError(f"{self.name}: control_dt must be positive")

    def _check_groups(self, arms: dict) -> dict:
        """The joint groups by name, checked against the arms and the base."""
        groups = {g.name: g for g in self.joint_groups}
        names = [*arms, *groups, *([self.base.name] if self.base else [])]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: arm, joint group and base names must be distinct")
        joints = [j for c in (*self.arms, *self.joint_groups) for j in c.joints]
        if self.base is not None:
            joints += self.base.joints
        if (groups or self.base is not None) and len(set(joints)) != len(joints):
            raise ValueError(f"{self.name}: a joint belongs to one arm, joint group or base")
        carried = [a for g in self.joint_groups for a in g.carries]
        if len(set(carried)) != len(carried):
            raise ValueError(f"{self.name}: an arm hangs from one joint group at most")
        for g in self.joint_groups:
            for a in g.carries:
                if a not in arms:
                    raise ValueError(f"joint group {g.name}: unknown arm {a!r}")
                if arms[a].kinematics.kind != "chain":
                    raise ValueError(f"joint group {g.name}: arm {a} must be a chain to hang "
                                     "from it")
        return groups

    def _check_group_drives(self, groups: dict) -> None:
        sizes = {n: len(g.joints) for n, g in groups.items()}
        if self.base is not None:
            sizes[self.base.name] = self.base.dim
        seen = set()
        for d in self.drive.groups:
            if d.target not in sizes or d.target in seen:
                raise ValueError(f"drive {d.target}: one entry per joint group or base")
            seen.add(d.target)
            for f in ("hold_offset_max", "step_clamp"):
                v = getattr(d, f)
                if v and len(v) != sizes[d.target]:
                    raise ValueError(f"drive {d.target}: {f} needs one value per joint or axis")

    @property
    def dof(self) -> int:
        """The proprio q length: every arm's joints, then every joint group's."""
        return self.arm_dof + sum(len(g.joints) for g in self.joint_groups)

    @property
    def arm_dof(self) -> int:
        return sum(len(a.joints) for a in self.arms)

    @property
    def action_dim(self) -> int:
        return sum(s.dim for s in self.action)

    def camera(self, name: str) -> CameraSpec:
        return next(c for c in self.cameras if c.name == name)

    def to_dict(self) -> dict:
        d = asdict(self)
        if self.base is None:
            del d["base"]
        if not self.joint_groups:
            del d["joint_groups"]
        if not self.drive.groups:
            del d["drive"]["groups"]
        if self.workspace.frame == "root":
            del d["workspace"]["frame"]
        for ee in d["end_effectors"]:
            if ee["gripper"] is not None and not ee["gripper"]["tip_reach_m"]:
                del ee["gripper"]["tip_reach_m"]
        return d

    def sha256(self) -> str:
        """Over the canonical JSON (sorted keys, compact), as amp.link hashes a description."""
        text = json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(text.encode()).hexdigest()

    @classmethod
    def from_dict(cls, d: dict) -> EmbodimentSpec:
        d = dict(d)
        arms = []
        for a in d.pop("arms"):
            a = dict(a)
            a["kinematics"] = Kinematics(**a["kinematics"])
            a["singularities"] = tuple(Singularity(**s) for s in a.get("singularities", ()))
            arms.append(ArmSpec(**a))
        ees = []
        for e in d.pop("end_effectors"):
            e = dict(e)
            e["gripper"] = None if e.get("gripper") is None else GripperSpec(**e["gripper"])
            ees.append(EndEffectorSpec(**e))
        drive = dict(d.pop("drive", {}))
        if "tau_band_s" in drive:
            drive["tau_band_s"] = tuple(drive["tau_band_s"])
        if "groups" in drive:
            drive["groups"] = tuple(GroupDriveSpec(**g) for g in drive["groups"])
        base = d.pop("base", None)
        return cls(arms=tuple(arms), end_effectors=tuple(ees),
                   cameras=tuple(CameraSpec(**c) for c in d.pop("cameras")),
                   action=tuple(ActionSegment(**s) for s in d.pop("action")),
                   drive=DriveSpec(**drive), workspace=WorkspaceSpec(**d.pop("workspace", {})),
                   base=None if base is None else BaseSpec(**base),
                   joint_groups=tuple(JointGroupSpec(**g) for g in d.pop("joint_groups", ())),
                   **d)


DESCRIPTION_FORMAT = "amp-robot-description-1"
JOINT_TYPES = ("revolute", "prismatic", "fixed")


@dataclass(frozen=True)
class JointSpec:
    """One joint of a robot description's kinematic tree, at q = 0."""

    name: str
    type: str
    parent: str
    child: str
    origin: tuple[float, ...]
    axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    limits: tuple[float, float] = (0.0, 0.0)

    def __post_init__(self):
        if self.type not in JOINT_TYPES:
            raise ValueError(f"joint {self.name}: type {self.type!r} not in {JOINT_TYPES}")
        object.__setattr__(self, "origin", _pose(self.origin, f"joint {self.name} origin"))
        object.__setattr__(self, "axis", _floats(self.axis, 3, f"joint {self.name} axis"))
        object.__setattr__(self, "limits", _floats(self.limits, 2, f"joint {self.name} limits"))


@dataclass(frozen=True)
class LinkSpec:
    """A link and its visual meshes: /sim/robot blobs verts/<mesh> (float32 Nx3, metres, in this
    link's frame) and faces/<mesh> (uint32 Mx3)."""

    name: str
    meshes: tuple[str, ...] = ()

    def __post_init__(self):
        object.__setattr__(self, "meshes", tuple(self.meshes))


@dataclass(frozen=True)
class RobotDescription:
    """The /sim/robot header: the robot's kinematic tree, read from the backend's own model as is."""

    root: str
    links: tuple[LinkSpec, ...]
    joints: tuple[JointSpec, ...]
    format: str = DESCRIPTION_FORMAT

    def __post_init__(self):
        if self.format != DESCRIPTION_FORMAT:
            raise ValueError(f"description format {self.format!r} is not {DESCRIPTION_FORMAT}")
        object.__setattr__(self, "links", tuple(self.links))
        object.__setattr__(self, "joints", tuple(self.joints))
        names = {lk.name for lk in self.links}
        if self.root not in names or len(names) != len(self.links):
            raise ValueError("description: unique link names, the root among them")
        children = [j.child for j in self.joints]
        if len(set(children)) != len(children) or self.root in children:
            raise ValueError("description: a tree (each link has at most one parent, root none)")
        for j in self.joints:
            if j.parent not in names or j.child not in names:
                raise ValueError(f"joint {j.name}: links {j.parent!r}, {j.child!r} not described")

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> RobotDescription:
        d = dict(d)
        return cls(links=tuple(LinkSpec(**lk) for lk in d.pop("links")),
                   joints=tuple(JointSpec(**j) for j in d.pop("joints")), **d)


def accepted(names) -> tuple[str, ...]:
    """A harness's or server's accepted embodiments, validated: embodiment names or ANY."""
    names = (names,) if isinstance(names, str) else tuple(names)
    if not names or any(n != ANY and not _NAME.fullmatch(n) for n in names):
        raise ValueError(f"embodiments {names!r}: non-empty names, or {ANY!r}")
    return names


def supports(accepted: tuple[str, ...], embodiment: str) -> bool:
    """Whether a harness or server that accepts these embodiments can drive this one."""
    return ANY in accepted or embodiment in accepted
