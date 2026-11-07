#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CapgymBackend: a CaP-X simulator as a slot's long-lived link client, rendering with EGL."""

from __future__ import annotations

import importlib
import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from amp.contracts.embodiment import EmbodimentSpec
from amp.contracts.robot import EpisodeRef, SlotSpec
from amp.link.wire import jsonable
from amp.robots.base import ISAAC_ABSENT, pythonpath
from amp.robots.capgym import affinity, sandbox
from amp.robots.capgym.adapter import TASK_SETS
from amp.robots.capgym.client import DEFAULT_POLICY_LABEL
from amp.robots.capgym.families import NAMES as FAMILY_NAMES
from amp.robots.capgym.families import family
from amp.robots.robolab.records import ingest_results, result_artifacts, result_row

LIBERO_CONFIG_ENV = "LIBERO_CONFIG_PATH"



def _pinned_as_asked(pin: dict) -> bool:
    """Whether the backend's process runs on exactly the CPUs it was asked to pin."""
    try:
        return bool(pin.get("pinned")) and (affinity.cpulist(str(pin["pinned"]))
                                            == affinity.cpulist(str(pin.get("requested") or "")))
    except ValueError:
        return False


@dataclass(frozen=True)
class CapgymBackend:
    """python -m amp.robots.capgym in benchmark mode, writing a RoboLab-shaped results folder."""

    task_sets: ClassVar[tuple[str, ...]] = TASK_SETS


    python: str | Mapping[str, str] | None = None
    task_set: str = "capgym_libero"
    name: str = field(default="capgym", init=False)

    def __post_init__(self):
        if isinstance(self.python, Mapping):
            stray = sorted(set(self.python) - set(FAMILY_NAMES))
            if stray:
                raise ValueError(f"capgym python is keyed by family ({', '.join(FAMILY_NAMES)}); "
                                 f"{', '.join(map(str, stray))} is not one")
            bad = [k for k, v in self.python.items() if not isinstance(v, str) or not v]
            if bad:
                raise ValueError(f"capgym python for {bad[0]!r} must be a non-empty path")
            object.__setattr__(self, "python", dict(self.python))
        elif self.python is not None and not isinstance(self.python, str):
            raise ValueError(f"capgym python must be a path or a {{family: path}} map, not "
                             f"{self.python!r}")

    @property
    def embodiment(self) -> EmbodimentSpec | None:
        """The boot task set's, as embodiment_for gives it."""
        return self.embodiment_for(self.task_set)

    def embodiment_for(self, task_set: str) -> EmbodimentSpec | None:
        """What the hello declares for task_set: its family's module's."""
        return importlib.import_module(family(task_set).module).EMBODIMENTS[task_set]()

    def python_for(self, task_set: str) -> str:
        """task_set's family's interpreter; ValueError when the host turned that family off."""
        fam = family(task_set)
        over = self.python.get(fam.name) if isinstance(self.python, Mapping) else self.python
        return fam.python(over)

    def command(self, slot: SlotSpec, server_url: str) -> list[str]:
        if not slot.results_dir:
            raise ValueError(f"slot {slot.slot_id}: the capgym backend needs a results_dir")
        task_set = slot.task_set or self.task_set
        if task_set not in self.task_sets:
            raise ValueError(f"slot {slot.slot_id}: task set {task_set!r} is not one of "
                             f"{self.task_sets}")
        return [self.python_for(task_set), "-m", "amp.robots.capgym", "--server", server_url,
                "--benchmark", "--adapter", "official", "--output-dir", slot.results_dir,
                "--cameras", slot.cameras, "--info", slot.info_mode,
                "--episodes-per-task", "1", "--policy-label", DEFAULT_POLICY_LABEL,
                "--max-steps", "0", "--seed", "0", "--task-set", task_set]

    def wipe_on_exit(self, slot: SlotSpec) -> tuple[str, ...]:
        """The sandbox's FRESH dirs, emptied when this slot's process is gone, however it died."""
        if not slot.results_dir:
            return ()
        return sandbox.fresh_paths(Path(os.path.abspath(slot.results_dir)).parent)

    def env_unset(self, slot: SlotSpec) -> tuple[str, ...]:
        """A MuJoCo family drops the Isaac names ISAAC_ABSENT lists (OMNI_* stays); BEHAVIOR none."""
        if family(slot.task_set or self.task_set).gpu == "cuda":
            return ()
        return ISAAC_ABSENT

    def cwd(self, slot: SlotSpec) -> str | None:
        return None

    def warmup(self) -> list[str] | None:
        return None

    def env(self, slot: SlotSpec) -> dict[str, str]:
        """The slot's env plus its sandbox, port block and family's GPU."""
        if not slot.results_dir:
            raise ValueError(f"slot {slot.slot_id}: the capgym backend needs a results_dir")
        fam = family(slot.task_set or self.task_set)
        out = {**slot.env, "PYTHONPATH": pythonpath(inherited=slot.env.get("PYTHONPATH", "")),
               "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}

        out.update(libero_config(slot))

        out.update(sandbox.sandbox_env(Path(os.path.abspath(slot.results_dir)).parent))
        if fam.gpu == "cuda":

            out.pop("MUJOCO_EGL_DEVICE_ID", None)
            out.pop("OMNIGIBSON_REMOTE_STREAMING", None)
            out.update(sandbox.cuda_env(Path(os.path.abspath(slot.results_dir)).parent,
                                        slot.gpu, slot.env.get("CUDA_VISIBLE_DEVICES")))
            return out
        out.update({"MUJOCO_GL": "egl", "PYOPENGL_PLATFORM": "egl",


                    "CUDA_VISIBLE_DEVICES": "",


                    "MUJOCO_EGL_DEVICE_ID": sandbox.egl_device(
                        slot.gpu, os.environ.get(sandbox.EGL_MAP_ENV))})
        return out

    def ingest(self, episode: EpisodeRef, tracer) -> None:
        ingest_results(f"robot/{self.name}", episode, tracer)
        capx = (result_row(episode) or {}).get("capx")
        pin = capx.get("cpu_affinity") if isinstance(capx, dict) else None
        if isinstance(pin, dict):
            tracer.child(f"robot/{self.name}").event(
                f"robot.{self.name}.cpu_affinity", jsonable(pin),
                lvl="info" if _pinned_as_asked(pin) else "warn")
        if isinstance(capx, dict) and capx.get("boundary_v") is not None:
            b = capx.get("boundary")
            b = b if isinstance(b, dict) else {"ok": None, "error": "no boundary record"}
            tracer.child(f"robot/{self.name}").event(
                f"robot.{self.name}.boundary", jsonable(b),
                lvl="info" if b.get("ok") is True else "error")

    def artifacts(self, episode: EpisodeRef) -> list[Path]:
        return result_artifacts(episode)


def libero_config(slot: SlotSpec) -> dict[str, str]:
    """{LIBERO_CONFIG_PATH: the slot's or this process's}, empty when neither sets it."""
    path = slot.env.get(LIBERO_CONFIG_ENV) or os.environ.get(LIBERO_CONFIG_ENV)
    return {LIBERO_CONFIG_ENV: path} if path else {}
