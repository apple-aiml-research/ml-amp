#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""An episode's artifacts: its own dir (trace, reports, logs) and the robot backend's files, staged
into the run layout so the run dir is a mirror of what the store gets."""

from __future__ import annotations

import contextlib
import json
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.robot import ROBOT_RESULT_FILE, EpisodeRef
from amp.fs.safe_files import safe_copy, safe_read_bytes, walk

from .layout import ROBOLAB_RESULTS, RunLayout
from .store import excluded


@dataclass
class Collected:
    """Run-root-relative paths to export, in order; RoboLab's row file comes last."""

    episode: list[str] = field(default_factory=list)
    robolab: list[str] = field(default_factory=list)
    refused: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    @property
    def rels(self) -> list[str]:
        return self.episode + self.robolab


def _inside(path: Path, root: Path) -> bool:
    try:
        Path(path).absolute().relative_to(Path(root).absolute())
    except ValueError:
        return False
    return True


def stage_backend(layout: RunLayout, ref: EpisodeRef, backend, run_id: str,
                  halt: Callable[[], bool] | None = None) -> Collected:
    """Copy the backend's files for this episode into robolab_<run>_<Task>_r<k>/<Task>/ and
    its results row into that folder's episode_results.jsonl; halt() true stops before each."""
    out = Collected()
    halt = halt or (lambda: False)
    try:
        paths = list(backend.artifacts(ref)) if backend is not None else []
    except Exception as e:  # noqa: BLE001
        out.errors.append(f"backend artifacts: {type(e).__name__}: {e}")
        return out
    folder = layout.robolab_dir(run_id, ref.task, ref.episode)
    row_file = None
    for p in paths:
        p = Path(p)
        if _inside(p, ref.episode_dir):
            if p.name == ROBOT_RESULT_FILE:
                row_file = p
            continue
        if halt():
            return out
        dst = folder / ref.task / p.name
        try:
            dst.parent.mkdir(parents=True, exist_ok=True)
            safe_copy(p, p.parent, dst)
        except OSError as e:
            out.refused.append(f"{p}: {e}")
            continue
        out.robolab.append(layout.rel(dst))
    if row_file is not None and not halt():
        try:
            row = json.loads(safe_read_bytes(row_file, ref.episode_dir))
            if isinstance(row, dict) and ref.episode_id:
                row["source_episode_id"] = ref.episode_id
            folder.mkdir(parents=True, exist_ok=True)
            (folder / ROBOLAB_RESULTS).write_text(json.dumps(row) + "\n")
            out.robolab.append(layout.rel(folder / ROBOLAB_RESULTS))
        except (OSError, ValueError) as e:
            out.errors.append(f"results row: {e}")
    out.robolab.sort(key=lambda r: (r.endswith("/" + ROBOLAB_RESULTS), r))
    return out


def episode_files(layout: RunLayout, ep_dir: Path) -> Collected:
    """The episode dir's regular files minus NEVER_UPLOAD; links are refused, never followed."""
    out = Collected()
    for rel, why in walk(ep_dir):
        if excluded(rel):
            continue
        if why is not None:
            out.refused.append(f"{rel}: {why}")
            continue
        out.episode.append(layout.rel(ep_dir / rel))
    return out


def collect(layout: RunLayout, ep_dir: Path, ref: EpisodeRef, backend, run_id: str) -> Collected:
    """Everything of one episode that may be shared, staged; backend files first so a row file
    it writes into the episode dir is listed too."""
    staged = stage_backend(layout, ref, backend, run_id)
    files = episode_files(layout, ep_dir)
    return Collected(episode=files.episode, robolab=staged.robolab,
                     refused=staged.refused + files.refused, errors=staged.errors)


def collect_adopted(layout: RunLayout, ref: EpisodeRef, backend, run_id: str,
                    halt: Callable[[], bool] | None = None) -> Collected:
    """An adopted episode's robot outputs, staged as collect() stages them: the backend's files and
    the results row it writes into ref.episode_dir, not that dir's other files."""
    made = not ref.episode_dir.exists()
    ref.episode_dir.mkdir(parents=True, exist_ok=True)
    out = stage_backend(layout, ref, backend, run_id, halt)
    row = ref.episode_dir / ROBOT_RESULT_FILE
    if row.is_file() and not row.is_symlink():
        out.episode.append(layout.rel(row))
    elif made:
        with contextlib.suppress(OSError):
            ref.episode_dir.rmdir()
    return out
