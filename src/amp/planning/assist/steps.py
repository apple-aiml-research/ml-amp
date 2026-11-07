#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The assist plan model: step specs as the model and server exchange them; planned steps."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from amp.geometry.kinematics import pose7
from amp.geometry.world import pose_from7
from amp.planning import options as opt
from amp.planning.trajectory import PlanResult

KINDS = ("pick", "place")


def as_pose(T, name) -> np.ndarray:
    """A copy of T as a finite 4x4 transform, or ValueError naming it."""
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    return T.copy()


def pose7_json(T) -> list:
    """T's pose7 for JSON: position to 5 digits, quaternion to 6."""
    p = pose7(np.asarray(T, dtype=np.float64))
    return [round(float(v), 5) for v in p[:3]] + [round(float(v), 6) for v in p[3:]]


def check_place_args(kind, name, target, point) -> np.ndarray | None:
    """Validate a step's target and point for its kind; the point as an array, or None."""
    if kind == "pick":
        if target is not None or point is not None:
            raise ValueError("a pick step takes no target or point")
        return None
    if (target is None) == (point is None):
        raise ValueError("a place step needs exactly one of target or point")
    if target is not None and target == name:
        raise ValueError(f"target must be another object than {name!r}")
    if point is None:
        return None
    p = np.asarray(point, dtype=np.float64).reshape(-1)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError(f"point must be 3 finite numbers, got {point!r}")
    return p


@dataclass
class StepSpec:
    """A pick or place step: the object, where to (target or table point), and its anchor."""

    id: int
    kind: str
    object: str
    target: str | None = None
    point: np.ndarray | None = None
    anchor: np.ndarray | None = None
    options: dict = field(default_factory=dict)

    def __post_init__(self):
        if isinstance(self.id, bool) or not isinstance(self.id, (int, np.integer)):
            raise ValueError(f"step id must be an integer, got {self.id!r}")  # noqa: TRY004
        self.id = int(self.id)
        if self.kind not in KINDS:
            raise ValueError(f"step kind must be one of {list(KINDS)}, got {self.kind!r}")
        if not isinstance(self.object, str) or not self.object:
            raise ValueError(f"step object must be a name, got {self.object!r}")
        if self.target is not None and not isinstance(self.target, str):
            raise ValueError(f"step target must be a name, got {self.target!r}")
        self.point = check_place_args(self.kind, self.object, self.target, self.point)
        if self.anchor is not None:
            self.anchor = as_pose(self.anchor, "anchor")
        if not isinstance(self.options, dict):
            raise ValueError("step options must be a JSON object")  # noqa: TRY004
        self.options = opt.validate(self.kind, self.options)

    def to_json(self) -> dict:
        """The step spec JSON; the anchor is kept at full precision."""
        anchor = None if self.anchor is None else [
            float(v) for v in pose7(np.asarray(self.anchor, dtype=np.float64))]
        out = {"id": self.id, "kind": self.kind, "object": self.object}
        if self.kind == "pick":
            out["grasp"] = anchor
        else:
            out["target"] = self.target
            out["point"] = None if self.point is None else [float(v) for v in self.point]
            out["release"] = anchor
        out["options"] = dict(self.options)
        return out

    @classmethod
    def from_json(cls, d) -> StepSpec:
        if not isinstance(d, dict):
            raise ValueError("a step spec must be a JSON object")  # noqa: TRY004
        kind = d.get("kind")
        key = "grasp" if kind == "pick" else "release"
        raw = d.get(key)
        anchor = None if raw is None else pose_from7(raw, key)
        options = d.get("options")
        return cls(
            id=d.get("id"), kind=kind, object=d.get("object"), target=d.get("target"),
            point=d.get("point"), anchor=anchor, options={} if options is None else options,
        )


@dataclass
class PlannedStep:
    """A step in a plan: its spec, the states around it, its plan, and why it is infeasible."""

    spec: StepSpec
    before: object = None
    result: PlanResult | None = None
    tcp: np.ndarray | None = None
    after: object = None
    ok: bool = False
    message: str = ""
    release_height: float | None = None
    key: tuple | None = field(default=None, repr=False)

    @property
    def id(self) -> int:
        return self.spec.id

    def clear(self) -> None:
        self.result = self.tcp = self.after = self.release_height = self.key = None
        self.ok, self.message = False, ""
