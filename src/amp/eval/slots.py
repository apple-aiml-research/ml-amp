#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What the episode runner needs from a slot beyond the Slot protocol."""

from __future__ import annotations

import socket
from dataclasses import dataclass
from pathlib import Path

import amp


@dataclass(frozen=True)
class SlotPaths:
    gateway_state_dir: Path | None
    server_state_dir: Path | None
    results_dir: Path | None
    node_id: str
    log_dir: Path | None = None


def slot_paths(slot) -> SlotPaths:
    nid = slot.node_id or socket.gethostname()
    return SlotPaths(Path(slot.gateway_state_dir), Path(slot.server_state_dir),
                     Path(slot.results_dir), str(nid), Path(slot.log_dir))


def own_src() -> Path:
    """The src dir this process imports amp from."""
    return Path(amp.__file__).resolve().parents[1]


def retarget(slot, task_set: str, task: str, official: bool = False) -> tuple[bool, str | None]:
    """(moved, why not): retarget the slot's backend (CapgymSlot.ensure_task) as the local runner does."""
    try:
        return bool(slot.ensure_task(task_set, task, official=official)), None
    except Exception as e:  # noqa: BLE001
        return False, f"task set {task_set!r} for task {task!r}: {type(e).__name__}: {e}"
