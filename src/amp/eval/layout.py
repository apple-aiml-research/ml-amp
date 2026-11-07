#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One harness configuration's run directory, in the layout amp.contracts.eval pins."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from amp.contracts.eval import AGENTIC_PREFIX, LIVE_PREFIX, RUN_CONFIG_FILE, unit_name

META_FILE = "meta.json"
AGENT_OUTPUT = "agent.out"
ROBOLAB_RESULTS = "episode_results.jsonl"


@dataclass(frozen=True)
class RunLayout:
    """Paths under one harness configuration's run root."""

    root: Path

    def __post_init__(self):
        object.__setattr__(self, "root", Path(self.root))

    @property
    def agentic(self) -> Path:
        return self.root / AGENTIC_PREFIX

    @property
    def episodes(self) -> Path:
        return self.agentic / "episodes"

    @property
    def units(self) -> Path:
        return self.agentic / "units"

    @property
    def attempts(self) -> Path:
        return self.agentic / "attempts"

    @property
    def run_config(self) -> Path:
        return self.agentic / RUN_CONFIG_FILE

    def live_dir(self, episode_id: str) -> Path:
        return self.episodes / f"{LIVE_PREFIX}{episode_id}"

    def final_name(self, task: str, episode_id: str) -> str:
        return f"{task}__{episode_id}"

    def unit_path(self, task: str, episode: int) -> Path:
        return self.units / f"{unit_name(task, episode)}.json"

    @property
    def official(self) -> Path:
        return self.agentic / "official"

    def attempt_path(self, task: str, episode: int) -> Path:
        return self.attempts / f"{unit_name(task, episode)}.json"

    def official_path(self, task: str, episode: int) -> Path:
        """An official unit's attempt record: attempt 1's, the one attempt it runs."""
        return self.official / f"{unit_name(task, episode)}__a1.json"

    def robolab_dir(self, run_id: str, task: str, episode: int) -> Path:
        """RoboLab's own outputs for one unit, in the folder the result viewer's census reads."""
        return self.root / f"robolab_{run_id}_{task}_r{int(episode)}"

    def rel(self, path: Path) -> str:
        return Path(path).relative_to(self.root).as_posix()


def unit_rel(task: str, episode: int) -> str:
    return f"{AGENTIC_PREFIX}/units/{unit_name(task, episode)}.json"

