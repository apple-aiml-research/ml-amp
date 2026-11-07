#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Assist request options as typed dataclasses, and the JSON parsing that validates them."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

SETTLE_S = 0.3
MAX_SETTLE_S = 2.0
MAX_ASSIST_DIST_M = 0.3
MAX_GRIP_HOLD_S = 5.0
ASSIST_KINDS = ("pick", "place", "lift", "release", "move", "push")
STOP_AFTER = ("lift_test", "close")
PROPOSAL_MODES = ("v1", "v2")


@dataclass(frozen=True)
class PickOptions:
    approach: float = 0.10
    lift: float = 0.10
    speed_scale: float = 0.5
    settle_s: float = SETTLE_S
    close_s: float | None = None
    lift_test_m: float | None = None
    stop_after: str | None = None


@dataclass(frozen=True)
class PlaceOptions:
    approach: float = 0.10
    retreat: float = 0.10
    speed_scale: float = 0.5
    settle_s: float = SETTLE_S
    open_s: float | None = None
    insert: bool = False


@dataclass(frozen=True)
class LiftOptions:
    height_m: float = 0.10
    settle_s: float = SETTLE_S
    speed_scale: float | None = None


@dataclass(frozen=True)
class ReleaseOptions:
    retreat_m: float = 0.08
    settle_s: float = SETTLE_S
    open_s: float | None = None


@dataclass(frozen=True)
class MoveOptions:
    speed_scale: float = 0.5
    settle_s: float = SETTLE_S


@dataclass(frozen=True)
class PushOptions:
    height_m: float = 0.02
    speed_scale: float = 0.5
    settle_s: float = SETTLE_S
    close_s: float | None = None


Options = PickOptions | PlaceOptions | LiftOptions | ReleaseOptions | MoveOptions | PushOptions
OPTIONS: dict[str, type] = {"pick": PickOptions, "place": PlaceOptions, "lift": LiftOptions,
                            "release": ReleaseOptions, "move": MoveOptions, "push": PushOptions}
_DURATIONS = {"pick": ("close_s",), "place": ("open_s",), "release": ("open_s",),
              "push": ("close_s",)}
_EXTRA = {("pick", "stop_after"): STOP_AFTER, ("pick", "lift_test_m"): "number",
          ("place", "insert"): "bool", ("lift", "speed_scale"): "number"}
_DEFAULT_KEYS = {"pick": ("approach", "lift", "speed_scale"),
                 "place": ("approach", "retreat", "speed_scale"), "lift": ("height_m",),
                 "release": ("retreat_m",), "move": ("speed_scale",),
                 "push": ("height_m", "speed_scale")}


def number(value, name) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return float(value)


def allowed_keys(kind) -> set[str]:
    return set(_DEFAULT_KEYS[kind]) | set(_DURATIONS.get(kind, ())) | {
        k for (kd, k) in _EXTRA if kd == kind}


def validate(kind, raw) -> dict:
    """The keys of raw, validated for kind (a step stores only what it was given)."""
    if kind not in OPTIONS:
        raise ValueError(f"kind must be one of {list(ASSIST_KINDS)}, got {kind!r}")
    raw = {} if raw is None else raw
    if not isinstance(raw, dict):
        raise ValueError("options must be a JSON object")  # noqa: TRY004
    allowed = allowed_keys(kind)
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown {kind} options {sorted(unknown)}; allowed {sorted(allowed)}")
    out = {}
    for key, value in raw.items():
        want = _EXTRA.get((kind, key))
        if want == "bool":
            if not isinstance(value, bool):
                raise ValueError(f"options.{key} must be true or false, got {value!r}")
            out[key] = value
            continue
        if isinstance(want, tuple):
            if value is not None and value not in want:
                raise ValueError(f"options.{key} must be one of {list(want)} or null, "
                                 f"got {value!r}")
            out[key] = value
            continue
        v = number(value, f"options.{key}")
        if key in _DURATIONS.get(kind, ()):
            if not 0.0 <= v <= MAX_GRIP_HOLD_S:
                raise ValueError(f"options.{key} must be in [0, {MAX_GRIP_HOLD_S}] s, got {v}")
        elif key == "speed_scale":
            if not 0.0 < v <= 1.0:
                raise ValueError(f"options.speed_scale must be in (0, 1], got {v}")
        elif not 0.0 <= v <= MAX_ASSIST_DIST_M:
            raise ValueError(f"options.{key} must be in [0, {MAX_ASSIST_DIST_M}] m, got {v}")
        out[key] = v
    return out


def parse(kind, raw=None, *, settle_s: float = SETTLE_S) -> Options:
    """Request options for kind, validated, over the defaults and the server's settle_s."""
    given = validate(kind, raw)
    return OPTIONS[kind](**{**given, "settle_s": float(settle_s)})


def name_of(body, key) -> str:
    v = body.get(key)
    if not isinstance(v, str) or not v:
        raise ValueError(f"{key!r} must be an object name, got {v!r}")
    return v


def point3(v, name) -> np.ndarray:
    if not isinstance(v, list) or len(v) != 3:
        raise ValueError(f"{name} must be 3 numbers, got {v!r}")
    return np.array([number(x, name) for x in v])


def target_or_point(body, name, *, exactly_one) -> tuple[str | None, np.ndarray | None]:
    """A place's target object or table point from a request body."""
    target, point = body.get("target"), body.get("point")
    if target is not None:
        target = name_of(body, "target")
        if target == name:
            raise ValueError(f"target must be another object than {name!r}")
    if point is not None:
        if not isinstance(point, list) or len(point) != 3:
            raise ValueError(f"point must be 3 numbers, got {point!r}")
        point = np.array([number(v, "point") for v in point])
    if target is not None and point is not None:
        raise ValueError("give either target or point, not both")
    if exactly_one and target is None and point is None:
        raise ValueError("give a target object or a table point")
    return target, point


def force(body) -> bool:
    f = body.get("force", False)
    if not isinstance(f, bool):
        raise ValueError(f"force must be true or false, got {f!r}")  # noqa: TRY004
    return f
