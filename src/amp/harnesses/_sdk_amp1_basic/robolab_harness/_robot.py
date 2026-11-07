#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot an episode drives, from observe()["embodiment"]; absent or DROID: DROID's constants."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, replace

DROID_NAME = "droid_panda_robotiq"
_AXIS = {"x": 0, "y": 1, "z": 2}


@dataclass(frozen=True)
class Arm:
    """One arm: its joints, and its slice [start, stop) of the full q (arms in declaration order)."""

    name: str
    joints: tuple
    start: int
    stop: int


@dataclass(frozen=True)
class EE:
    """One end effector: its arm, TCP axes, gripper and wrist camera."""

    name: str
    arm: str
    closing: int
    approach: int
    gripper_kind: str
    gripper_model: str
    stroke_m: float
    max_width_m: float
    closed_empty: float
    held_below: float
    wrist_camera: str | None
    has_gripper: bool = True  # False: a tool without fingers (gripper null); axes and stroke are defaults


@dataclass(frozen=True)
class Robot:
    """What the SDK needs of an embodiment; the defaults are DROID (Panda + Robotiq 2F-85).

    The scalar fields are the primary (first) end effector's; `arms` / `end_effectors` hold every one.
    """

    name: str = DROID_NAME
    droid: bool = True
    cameras: tuple = ("ego_l", "ego_r", "wrist")
    scene_cameras: tuple = ("ego_l", "ego_r")
    wrist_camera: str | None = "wrist"
    dof: int = 7
    gripper_kind: str = "parallel_jaw"
    gripper_model: str = "robotiq_2f85_linkage"
    stroke_m: float = 0.085
    max_width_m: float = 0.085
    closed_empty: float = 1.0
    held_below: float = 1.0
    closing: int = 0  # the TCP axis index the fingers close along
    approach: int = 2  # the TCP axis index pointing into the grasp
    has_gripper: bool = True
    arms: tuple = ()
    end_effectors: tuple = ()

    @property
    def multi(self) -> bool:
        return len(self.end_effectors) > 1

    @property
    def full_dof(self) -> int:
        """The length of the full q: every arm's joints, concatenated in declaration order."""
        return self.arms[-1].stop if self.arms else self.dof

    @property
    def primary_ee(self) -> str | None:
        return self.end_effectors[0].name if self.end_effectors else None

    def _ee(self, name: str, strict: bool = True) -> EE | None:
        found = next((e for e in self.end_effectors if e.name == name), None)
        if found is None and strict and self.end_effectors:
            raise ValueError(f"no end effector {name!r}; have {[e.name for e in self.end_effectors]}")
        return found

    def arm_dof(self, ee: str | None = None) -> int:
        """The number of joints of an end effector's arm (default the primary's)."""
        e = self._ee(ee, strict=False) if ee else None
        arm = next((a for a in self.arms if e is not None and a.name == e.arm), None)
        return len(arm.joints) if arm is not None and arm.joints else self.dof

    def q_slice(self, ee: str | None = None) -> tuple[int, int]:
        """[lo, hi) of an end effector's arm in the full q (the primary's when ee is None)."""
        e = self._ee(ee, strict=False) if ee else None
        arm = next((a for a in self.arms if e is not None and a.name == e.arm), None)
        return (arm.start, arm.stop) if arm is not None else (0, self.dof)

    def for_ee(self, name: str | None, strict: bool = True) -> Robot:
        """This robot as seen from one end effector: its axes, gripper, wrist camera and arm dof."""
        e = self._ee(name, strict) if name else None
        if e is None:
            return self
        return replace(self, dof=self.arm_dof(name), gripper_kind=e.gripper_kind,
                       gripper_model=e.gripper_model, stroke_m=e.stroke_m,
                       max_width_m=e.max_width_m, closed_empty=e.closed_empty,
                       held_below=e.held_below, closing=e.closing, approach=e.approach,
                       wrist_camera=e.wrist_camera, has_gripper=e.has_gripper)

    @property
    def default_axes(self) -> bool:
        return self.closing == 0 and self.approach == 2

    @property
    def width_axis(self) -> int:
        return 3 - self.closing - self.approach

    def gap_m(self, reading: float) -> float:
        """The finger gap of a 0 open .. 1 closed reading under a linear model."""
        return self.stroke_m * (1.0 - float(reading))


DROID = Robot()
ACTIVE = DROID
_FULL = DROID  # the whole robot; ACTIVE is it, or its view from the bound end effector
_BOUND: str | None = None


def _float(v, default: float) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def _end_effector(e: dict, i: int, n: int, cams: list, wrists: list) -> EE:
    g = e.get("gripper")
    has_gripper = not ("gripper" in e and g is None)
    g = g if isinstance(g, dict) else {}
    closing = _AXIS.get(str(e.get("closing_axis", "x")), 0)
    approach = _AXIS.get(str(e.get("approach_axis", "z")), 2)
    if closing == approach or not has_gripper:
        closing, approach = 0, 2
    stroke = _float(g.get("stroke_m"), 0.0)
    name = str(e.get("name") or f"ee{i}")
    arm = str(e.get("arm") or "")
    # Which wrist camera rides on this ee: declared, else named for it, else the same position.
    wrist = e.get("wrist_camera")
    if wrist not in wrists:
        wrist = next((str(c["name"]) for c in cams if c.get("role") == "wrist"
                      and c.get("ee") == name), None)
    if wrist is None and n > 1:
        wrist = next((w for w in wrists if name in w or (arm and arm in w)), None)
        if wrist is None and wrists:
            wrist = wrists[i] if len(wrists) == n else (wrists[0] if i == 0 else None)
    elif wrist is None and wrists:
        wrist = wrists[0]
    return EE(name=name, arm=arm, closing=closing, approach=approach,
              gripper_kind=str(g.get("kind") or "parallel_jaw") if has_gripper else "none",
              gripper_model=str(g.get("model") or ""), stroke_m=stroke,
              max_width_m=_float(g.get("max_width_m"), stroke),
              closed_empty=_float(g.get("closed_empty"), 1.0),
              held_below=_float(g.get("held_below"), 1.0), wrist_camera=wrist,
              has_gripper=has_gripper)


def from_embodiment(emb) -> Robot:
    """The Robot of an observation's "embodiment" dict; None, malformed or DROID's name: DROID."""
    if not isinstance(emb, dict) or emb.get("name") in (None, DROID_NAME):
        return DROID
    cams = [c for c in emb.get("cameras") or () if isinstance(c, dict) and c.get("name")]
    names = tuple(str(c["name"]) for c in cams)
    scene = tuple(str(c["name"]) for c in cams if c.get("role", "scene") == "scene")
    wrists = [str(c["name"]) for c in cams if c.get("role") == "wrist"]
    arm_dicts = [a for a in emb.get("arms") or () if isinstance(a, dict)]
    dof = len(arm_dicts[0].get("joints") or ()) if arm_dicts else 0
    arms, lo = [], 0
    for i, a in enumerate(arm_dicts):
        joints = tuple(str(j) for j in a.get("joints") or ())
        arms.append(Arm(str(a.get("name") or f"arm{i}"), joints, lo, lo + len(joints)))
        lo += len(joints)
    ee_dicts = [e for e in emb.get("end_effectors") or () if isinstance(e, dict)]
    ees = tuple(_end_effector(e, i, len(ee_dicts), cams, wrists) for i, e in enumerate(ee_dicts))
    p = ees[0] if ees else _end_effector({}, 0, 0, cams, wrists)
    return Robot(name=str(emb["name"]), droid=False, cameras=names,
                 scene_cameras=scene or names, wrist_camera=p.wrist_camera, dof=dof or 7,
                 gripper_kind=p.gripper_kind, gripper_model=p.gripper_model, stroke_m=p.stroke_m,
                 max_width_m=p.max_width_m, closed_empty=p.closed_empty, held_below=p.held_below,
                 closing=p.closing, approach=p.approach, has_gripper=p.has_gripper,
                 arms=tuple(arms), end_effectors=ees)


def activate(robot: Robot) -> Robot:
    """Make robot the one the module-level helpers (top_down, finger_yaw, glyphs) follow."""
    global ACTIVE, _FULL
    _FULL = robot
    ACTIVE = robot.for_ee(_BOUND, strict=False)
    return robot


@contextmanager
def bound_ee(name: str | None):
    """Inside the block the helpers follow end effector `name`'s axes and gripper (an EEHandle call)."""
    global ACTIVE, _BOUND
    prev = _BOUND
    _BOUND = name
    ACTIVE = _FULL.for_ee(name, strict=False)
    try:
        yield
    finally:
        _BOUND = prev
        ACTIVE = _FULL.for_ee(prev, strict=False)
