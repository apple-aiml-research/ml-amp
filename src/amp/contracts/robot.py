#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Robot backends: the link clients that put a robot (mock, RoboLab in Isaac, later real) behind the
robot server."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

LINK_PROTOCOL_VERSION = 1
LINK_CONTENT_TYPE = "application/x-teleop-frame"
ACTION_DIM = 8


@dataclass(frozen=True)
class SlotSpec:
    """One slot of a run: a robot (backend process) plus its robot server, on one GPU."""

    slot_id: str
    gpu: int
    index: int
    server_port: int
    cameras: str = "droid"
    info_mode: str = "standard"
    results_dir: str = ""
    env: dict[str, str] = field(default_factory=dict)
    task_set: str | None = None
    official: bool = False


@dataclass(frozen=True)
class EpisodeRef:
    """An episode as a backend sees it after the fact."""

    episode_id: str
    task: str
    episode: int
    episode_dir: Path
    results_dir: Path
    run_id: str = ""


class RobotBackend(Protocol):
    name: str

    def command(self, slot: SlotSpec, server_url: str) -> list[str]:
        """The long-lived link-client process for one slot."""
        ...

    def env(self, slot: SlotSpec) -> dict[str, str]: ...

    def cwd(self, slot: SlotSpec) -> str | None:
        """The command's working directory; None = the supervisor's choice."""
        ...

    def warmup(self) -> list[str] | None:
        """A one-off command run before any slot starts (e.g. shader caches)."""
        ...

    def ingest(self, episode: EpisodeRef, tracer: Any) -> None:
        """Turn the backend's own logs for this episode into robot/<name> trace events."""
        ...

    def artifacts(self, episode: EpisodeRef) -> list[Path]:
        """The backend's files to upload for this episode (RoboLab: videos, hdf5, result row)."""
        ...


ROBOT_RESULT_FILE = "robot_result.json"
RESULTS_LOCK = ".teleop/results.lock"
