#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot a plan is for: an Embodiment (None = DROID), and the arm a tool drives."""

from __future__ import annotations

import functools

import numpy as np

from amp.geometry.embodiment import Arm, Embodiment


@functools.cache
def droid() -> Embodiment:
    """The DROID runtime planning defaults to, built once."""
    return Embodiment.droid()


def resolve(embodiment: Embodiment | None) -> Embodiment:
    return droid() if embodiment is None else embodiment


def arm_of(tool) -> Arm:
    """The arm a tool drives: an ArmToolModel's own, the built-in Panda for kinematics.ToolModel."""
    arm = getattr(tool, "arm", None)
    return droid().arm() if arm is None else arm


def plan_arm(embodiment: Embodiment | None, ee: str | None = None) -> Arm:
    """The arm of end effector ee (None: the embodiment's default, a view's own), the one plans move."""
    e = resolve(embodiment)
    return e.arm(e.ee(ee).arm)


def check_tool(embodiment: Embodiment | None, tool, ee: str | None = None) -> Arm:
    """The plan arm of ee, after checking that tool drives it (ValueError otherwise)."""
    want = plan_arm(embodiment, ee)
    got = arm_of(tool)
    if got is not want and got.spec != want.spec:
        name = resolve(embodiment).name
        raise ValueError(f"the tool drives arm {got.name!r} ({got.spec.kinematics.kind}), not "
                         f"{name}'s {want.name!r}: plan with the embodiment's own tool")
    return want


def lims(arm: Arm, margin: float) -> tuple[np.ndarray, np.ndarray]:
    """(q_min + margin, q_max - margin) of an arm."""
    key = float(margin)
    cache = arm.__dict__.setdefault("_planning_lims", {})
    if key not in cache:
        cache[key] = (arm.q_min + key, arm.q_max - key)
    return cache[key]
