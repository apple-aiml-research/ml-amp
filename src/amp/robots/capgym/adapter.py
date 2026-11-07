#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""SimAdapter: what the capgym link client needs of a simulator, and the types it exchanges."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from amp.contracts.embodiment import EmbodimentSpec, RobotDescription
from amp.robots.capgym.families import TASK_SETS

__all__ = ["TASK_SETS"]



SUCCESS_CHECKS = ("step", "end")
Mesh = tuple[np.ndarray, np.ndarray]


class EnvError(RuntimeError):
    """The benchmark's own env raised during an action: that action fails, the episode goes on."""

    def __init__(self, cause: Exception):
        super().__init__(f"{type(cause).__name__}: {cause}")
        self.cause = cause


def env_call(fn, *args, **kwargs):
    """fn(*args, **kwargs), a call into the benchmark's env; an Exception it raises is an EnvError."""

    try:
        return fn(*args, **kwargs)
    except EnvError:
        raise
    except Exception as exc:
        raise EnvError(exc) from exc


@dataclass
class CameraImage:
    """One rendered camera: OpenCV optical frame, depth as optical z in metres."""

    rgb: np.ndarray
    depth: np.ndarray | None
    K: np.ndarray
    T_root_cam: np.ndarray


@dataclass
class Frame:
    """The robot's state at one control step, in the robot-root frame."""

    q: np.ndarray
    grippers: dict[str, float]
    ee: dict[str, tuple[np.ndarray, np.ndarray]]
    cameras: dict[str, CameraImage]
    T_world_base: np.ndarray | None = None
    body_contacts: tuple[str, ...] | None = None
    contacts_unavailable: bool = False

    @classmethod
    def single(cls, q, gripper: float, ee_pos, ee_quat, cameras: dict[str, CameraImage],
               ee: str = "ee") -> Frame:
        """A one-end-effector frame."""
        return cls(q=np.asarray(q, dtype=np.float64), grippers={ee: float(gripper)},
                   ee={ee: (np.asarray(ee_pos, dtype=np.float64),
                            np.asarray(ee_quat, dtype=np.float64))}, cameras=cameras)


class SimAdapter(Protocol):
    """One simulator process: a task library, one scene up at a time, the embodiment's actions."""

    def embodiment(self) -> EmbodimentSpec | None:
        """The robot it simulates, declared in the hello; None = DROID."""
        ...

    def robot_description(self) -> tuple[RobotDescription, dict[str, Mesh]] | None:
        """The /sim/robot payload a chain embodiment names; None when it declares none."""
        ...

    def success_checks(self) -> str:
        """One of SUCCESS_CHECKS."""
        ...

    def tasks(self) -> list[dict]:
        """Catalog entries {name, instruction, attributes, episode_length_s}."""
        ...

    def load(self, task: str) -> None:
        """Build or switch to the task's scene; cheap if it is already up."""
        ...

    def reset(self, episode: int) -> None:
        """Start the loaded task's episode `episode`; ValueError naming the task and its count
        when out of range (never wrap)."""
        ...

    def horizon(self) -> int | None:
        """Control steps the reset episode may still take under the benchmark's own horizon;
        None = the client's (episode_length_s / control_dt, or --max-steps; none if the task has
        no episode_length_s either)."""
        ...

    def control_dt(self) -> float:
        """Seconds of simulated time per apply()."""
        ...

    def observe(self) -> Frame:
        """The current state and camera renders."""
        ...

    def apply(self, action: Sequence[float]) -> None:
        """Track one action for one control_dt; the env's own raise is an EnvError (env_call)."""
        ...

    def success(self) -> tuple[bool, float]:
        """(success, score in [0, 1]) of the current state."""
        ...

    def ended(self) -> bool:
        """Optional (the client uses getattr): the benchmark ended the episode itself; the client
        then stops after that apply() and scores the state it ended in. None does at this pin."""
        ...

    def close(self) -> None:
        """Release the scene so the next load() rebuilds it; idempotent, never raises. Called on
        every abort and failed job (jobs.serve unloads the worker), so each costs a rebuild."""
        ...


class AdapterFactory(Protocol):
    """The make_adapter a simulator module (amp.robots.capgym.official) exposes."""

    def __call__(self, *, task_set: str, cameras: str, seed: int | None) -> SimAdapter:
        """The adapter for one task set; an adapter with official trials (CaP-X) ignores seed."""
        ...

