#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RoboDojoBackend: the RoboDojo launcher as a slot's long-lived link client."""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from amp.benchmarks.robodojo import TASK_SET
from amp.contracts.embodiment import EmbodimentSpec
from amp.contracts.robot import EpisodeRef, SlotSpec
from amp.robots.base import ISAAC_ABSENT, SRC_DIR, pythonpath
from amp.robots.capgym import sandbox
from amp.robots.capgym.client import DEFAULT_POLICY_LABEL
from amp.robots.robodojo.client import PORT_ENV
from amp.robots.robodojo.embodiment import robodojo_dual_x5
from amp.robots.robolab.records import ingest_results, result_artifacts

ROOT_ENV = "AMP_ROBODOJO_ROOT"
PYTHON_ENV = "AMP_ROBODOJO_PYTHON"
XPL_PYTHON_ENV = "AMP_ROBODOJO_XPL_PYTHON"
OVERLAY_DIR = "robodojo_overlay"
PORT_OFFSET = 1
SETUP = "docs/robodojo.md, section 1"


def default_root(environ: Mapping[str, str] | None = None) -> Path:
    """$AMP_ROBODOJO_ROOT, else this repository's external/robodojo."""
    env = os.environ if environ is None else environ
    given = (env.get(ROOT_ENV) or "").strip()
    return absolute(given) if given else SRC_DIR.parent / "external" / "robodojo"


def interpreter(given: str | None, name: str, environ: Mapping[str, str] | None = None) -> str:
    """given, else $<name>, a path made absolute (a bare name stays for PATH); ValueError naming
    the setup when neither is set."""
    env = os.environ if environ is None else environ
    out = (given or env.get(name) or "").strip()
    if not out:
        raise ValueError(f"${name} is not set: point it at the venv's python ({SETUP})")
    return str(absolute(out)) if os.sep in out else out


def absolute(path: str | os.PathLike) -> Path:
    """path made absolute against the cwd, symlinks kept: a venv's python must stay its own."""
    return Path(os.path.abspath(os.path.expanduser(path)))


def kit_cache_env(slot_dir: str | os.PathLike) -> dict[str, str]:
    """Kit's OptiX, CUDA JIT and torch-extension caches under the slot's sandbox."""
    sb = Path(os.path.abspath(slot_dir)) / sandbox.SANDBOX
    return {"OPTIX_CACHE_PATH": str(sb / "home" / ".cache" / "optix"),
            "CUDA_CACHE_PATH": str(sb / "home" / ".nv" / "ComputeCache"),
            "TORCH_EXTENSIONS_DIR": str(sb / "torch_extensions")}


def overlay_root(results_dir: str | os.PathLike) -> Path:
    """The slot's overlay root, under its results dir's parent (the slot dir)."""
    return Path(os.path.abspath(results_dir)).parent / OVERLAY_DIR


@dataclass(frozen=True)
class RoboDojoBackend:
    """python -m amp.robots.robodojo in benchmark mode, writing a RoboLab-shaped results folder."""

    task_sets: ClassVar[tuple[str, ...]] = (TASK_SET,)
    python: str | None = None
    xpl_python: str | None = None
    robodojo_root: str | None = None
    episodes_per_task: int = 1
    policy_label: str = DEFAULT_POLICY_LABEL
    takeover: bool = False
    name: str = field(default="robodojo", init=False)

    @property
    def declared(self) -> EmbodimentSpec:
        """robodojo_dual_x5, as the launcher's hello declares it (its measured sha aside)."""
        return robodojo_dual_x5()

    @property
    def root(self) -> Path:
        return absolute(self.robodojo_root) if self.robodojo_root else default_root()

    def command(self, slot: SlotSpec, server_url: str) -> list[str]:
        if not slot.results_dir:
            raise ValueError(f"slot {slot.slot_id}: the robodojo backend needs a results_dir")
        task_set = slot.task_set or TASK_SET
        if task_set not in self.task_sets:
            raise ValueError(f"slot {slot.slot_id}: task set {task_set!r} is not one of "
                             f"{self.task_sets}")
        cmd = [self.python_for(task_set), "-m", "amp.robots.robodojo", "--server", server_url,
               "--benchmark", "--output-dir", slot.results_dir,
               "--robodojo-root", str(self.root),
               "--xpl-python", interpreter(self.xpl_python, XPL_PYTHON_ENV), "--overlay-root",
               str(overlay_root(slot.results_dir)), "--gpu", str(self.gpu_index(slot)),
               "--episodes-per-task", str(self.episodes_per_task),
               "--policy-label", self.policy_label]
        if self.takeover:
            cmd.append("--takeover")
        return cmd

    @staticmethod
    def gpu_index(slot: SlotSpec) -> int:
        """The slot's PHYSICAL GPU, which is what eval_policy.sh's --device_id must name."""
        first = (slot.env.get("CUDA_VISIBLE_DEVICES") or "").split(",")[0].strip()
        if first.isdigit():
            return int(first)
        return max(slot.gpu, 0)

    def python_for(self, task_set: str) -> str:
        """RoboDojo's interpreter; ValueError when it is not configured."""
        if task_set not in self.task_sets:
            raise ValueError(f"task set {task_set!r} is not one of {self.task_sets}")
        return interpreter(self.python, PYTHON_ENV)

    def cwd(self, slot: SlotSpec) -> str | None:
        return None

    def warmup(self) -> list[str] | None:
        return None

    def wipe_on_exit(self, slot: SlotSpec) -> tuple[str, ...]:
        """The sandbox's FRESH dirs; the overlay and the shader, OptiX and Kit caches stay."""
        if not slot.results_dir:
            return ()
        return sandbox.fresh_paths(Path(os.path.abspath(slot.results_dir)).parent)

    def env_unset(self, slot: SlotSpec) -> tuple[str, ...]:
        """RoboLab's Isaac 5.0 names; eval_env sets the Kit paths RoboDojo's Isaac 5.1 needs."""
        return ISAAC_ABSENT

    def env(self, slot: SlotSpec) -> dict[str, str]:
        """The slot's env plus AMP's src, its sandbox, its policy-server port and its GPU."""
        if not slot.results_dir:
            raise ValueError(f"slot {slot.slot_id}: the robodojo backend needs a results_dir")
        out = {**slot.env, "PYTHONPATH": pythonpath(inherited=slot.env.get("PYTHONPATH", ""))}
        slot_dir = Path(os.path.abspath(slot.results_dir)).parent
        out.update(sandbox.sandbox_env(slot_dir))
        out.update(kit_cache_env(slot_dir))
        out.update(sandbox.port_env(slot.server_port))
        try:
            out[PORT_ENV] = str(sandbox.port_block(slot.server_port)[PORT_OFFSET])
        except ValueError:
            pass
        out.setdefault("CUDA_VISIBLE_DEVICES", str(self.gpu_index(slot)))
        return out

    def ingest(self, episode: EpisodeRef, tracer) -> None:
        ingest_results(f"robot/{self.name}", episode, tracer)

    def artifacts(self, episode: EpisodeRef) -> list[Path]:
        return result_artifacts(episode)
