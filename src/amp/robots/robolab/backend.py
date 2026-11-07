#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RoboLabBackend: one RoboLab run.py process per task, with AMP's client, on one GPU."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.eval import INSTRUCTION_TYPES
from amp.contracts.robot import EpisodeRef
from amp.robots.base import pythonpath, server_address
from amp.robots.robolab.client import ClientConfig
from amp.robots.robolab.records import ingest_results, result_artifacts

RUN_MODULE = "amp.robots.robolab.run"
KIT_ARGS = "--/renderer/multiGpu/enabled=false"
ACTIVE_GPU_ARG = "--/renderer/activeGpu"


@dataclass(frozen=True)
class RoboLabBackend:
    """The run.py command and environment for one task's runs, and ingest of its files."""

    robolab_root: str
    python: str
    policy_label: str = "amp"
    depth_stride: int = 4
    name: str = field(default="robolab", init=False)

    def client_config(self) -> ClientConfig:
        return ClientConfig(depth_stride=self.depth_stride)

    def run_command(self, *, task: str, num_runs: int, output_dir: str, server_url: str,
                    driver_socket: str, gpu_index: int | None = None,
                    instruction_type: str = "default") -> list[str]:
        """One official run.py process: --task task --num-runs num_runs into output_dir."""
        host, port = server_address(server_url)
        if instruction_type not in INSTRUCTION_TYPES:
            raise ValueError(f"instruction_type {instruction_type!r} not in {INSTRUCTION_TYPES}")
        if not Path(output_dir).is_absolute():
            raise ValueError(f"output_dir {output_dir!r} must be absolute: RoboLab joins a "
                             "relative one under its own checkout")
        if isinstance(num_runs, bool) or not isinstance(num_runs, int) or num_runs < 1:
            raise ValueError(f"num_runs {num_runs!r}: expected an int >= 1")
        kit_args = KIT_ARGS
        if gpu_index is not None:
            if isinstance(gpu_index, bool) or not isinstance(gpu_index, int) or gpu_index < 0:
                raise ValueError(f"gpu_index {gpu_index!r}: expected an nvidia-smi index >= 0")
            kit_args += f" {ACTIVE_GPU_ARG}={gpu_index}"
        cmd = [self.python, "-m", RUN_MODULE,
                "--headless", "--device", "cuda:0", "--num-envs", "1",
                f"--kit_args={kit_args}",
                "--task", task, "--num-runs", str(num_runs),
                "--output-folder-name", str(output_dir),
                "--policy", self.policy_label,
                "--remote-host", host, "--remote-port", str(port),
                "--amp-config", self.client_config().to_json(),
                "--amp-driver", str(driver_socket)]
        if instruction_type != "default":
            cmd += ["--instruction-type", instruction_type]
        return cmd

    def env(self, *, gpu_uuid: str, base: Mapping[str, str]) -> dict[str, str]:
        """base, with this repo's src and RoboLab on PYTHONPATH, pinned to one GPU by UUID."""
        if not gpu_uuid:
            raise ValueError("gpu_uuid is required: a Kit sees exactly one GPU")
        path = pythonpath(self.robolab_root, inherited=base.get("PYTHONPATH", ""))
        return {**base, "PYTHONPATH": path, "CUDA_DEVICE_ORDER": "PCI_BUS_ID",
                "CUDA_VISIBLE_DEVICES": gpu_uuid}

    def ingest(self, episode: EpisodeRef, tracer) -> None:
        ingest_results(f"robot/{self.name}", episode, tracer)

    def artifacts(self, episode: EpisodeRef) -> list[Path]:
        out = result_artifacts(episode)
        cfg = episode.results_dir / "env_cfg.json"
        return [*out, cfg] if cfg.is_file() else out
