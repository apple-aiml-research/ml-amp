#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CaP-X simulator families: each one's task sets, adapter module, venv and GPU use."""

from __future__ import annotations

import os
from dataclasses import dataclass

from amp import benchmarks

NODE_SWITCH_ENV = "AMP_CAPX"
NODE_DISABLED_ENV = "AMP_CAPX_DISABLED"


@dataclass(frozen=True)
class Family:
    """A CaP-X simulator family: its adapter module and venv (several may share one venv)."""

    name: str
    task_sets: tuple[str, ...]
    module: str
    python_env: str

    disabled_env: str
    gpu: str

    @property
    def process_per_task(self) -> bool:
        """Its backend builds one task per process (benchmarks.process_per_task)."""
        return any(benchmarks.process_per_task(ts) for ts in self.task_sets)

    def disabled(self) -> str | None:
        """Why the host turned this family off, by the host-wide switches or its own; else None."""
        if os.environ.get(NODE_SWITCH_ENV, "").strip() == "0":
            return f"switched off by {NODE_SWITCH_ENV}=0"
        return os.environ.get(NODE_DISABLED_ENV) or os.environ.get(self.disabled_env) or None

    def python(self, override: str | None = None) -> str:
        """override or $python_env ("" if unset: check_pythons refuses); ValueError if the family is off."""
        why = self.disabled()
        if why:
            raise ValueError(f"CaP-X {self.name} is off on this host: {why}")
        return override or os.environ.get(self.python_env) or ""


FAMILIES = (
    Family("libero", ("capgym_libero",), "amp.robots.capgym.official",
           "AMP_CAPX_LIBERO_PYTHON",
           "AMP_CAPX_LIBERO_DISABLED", "egl"),
    Family("robosuite", ("capgym_robosuite_cube", "capgym_robosuite_nut"),
           "amp.robots.capgym.official_robosuite", "AMP_CAPX_ROBOSUITE_PYTHON",
           "AMP_CAPX_ROBOSUITE_DISABLED", "egl"),

    Family("robosuite_two_arm", ("capgym_robosuite_two_arm_lift",
                                 "capgym_robosuite_two_arm_handover"),
           "amp.robots.capgym.official_robosuite_two_arm", "AMP_CAPX_ROBOSUITE_PYTHON",
           "AMP_CAPX_ROBOSUITE_DISABLED", "egl"),
    Family("robosuite_wipe", ("capgym_robosuite_wipe",),
           "amp.robots.capgym.official_robosuite_wipe", "AMP_CAPX_ROBOSUITE_PYTHON",
           "AMP_CAPX_ROBOSUITE_DISABLED", "egl"),

    Family("behavior", ("capgym_b1k",), "amp.robots.capgym.official_behavior",
           "AMP_CAPX_BEHAVIOR_PYTHON",
           "AMP_CAPX_BEHAVIOR_DISABLED", "cuda"),
)
TASK_SETS = tuple(ts for f in FAMILIES for ts in f.task_sets)
NAMES = tuple(f.name for f in FAMILIES)
_BY_TASK_SET = {ts: f for f in FAMILIES for ts in f.task_sets}


def family(task_set: str) -> Family:
    """The family serving task_set; ValueError for one no family serves."""
    f = _BY_TASK_SET.get(task_set)
    if f is None:
        raise ValueError(f"no CaP-X family serves task set {task_set!r}; known: "
                         f"{', '.join(TASK_SETS)}")
    return f


def flag(fam: Family) -> str:
    """The amp eval capgym flag that gives fam its interpreter."""
    return f"--{fam.name.split('_')[0]}-python"

