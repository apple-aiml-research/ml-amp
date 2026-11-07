#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The task recipe a harness's overlays add to the instruction the server serves (task_notes)."""

from __future__ import annotations

import functools
import importlib
import json
from pathlib import Path

HARNESS_PACKAGE = "amp.harnesses"

HEADING = "TASK RECIPE:"
OVERLAYS = "_overlays"
DATA_DIR = "task_notes"
ALIASES = "aliases.json"
PERCEPTION = "perception.json"
RECIPE, NOTES = "robodojo_recipe", "robodojo_notes"


def package_dir(harness: str) -> Path | None:
    """The harness's directory in amp.harnesses, or None when it has none."""
    if not harness or Path(harness).name != harness:
        return None
    path = list(importlib.import_module(HARNESS_PACKAGE).__path__)
    return next((d for p in path if (d := Path(p) / harness).is_dir()), None)


def applied(harness: str) -> list[str]:
    """The overlays applied to harness, in order (its perception.json); [] for a plain one."""
    pkg = package_dir(harness)
    p = None if pkg is None else pkg / PERCEPTION
    if p is None or not p.is_file():
        return []
    return list(json.loads(p.read_text()).get("overlays") or [])


def overlays_root() -> Path | None:
    """amp.harnesses' _overlays directory, found by package path (robot_server may not import it)."""
    path = list(importlib.import_module(HARNESS_PACKAGE).__path__)
    return next((d for p in path if (d := Path(p) / OVERLAYS).is_dir()), None)


@functools.cache
def _found(harness: str) -> tuple[tuple[Path, ...], str]:
    """(each applied overlay's task_notes source dir, in order; why there are none, or "")."""
    root = overlays_root()
    names = applied(harness) if root is not None else []
    if NOTES in names and RECIPE not in names:
        return (), f"{harness!r} applies {NOTES} without {RECIPE}, whose text it extends"
    dirs = tuple(d for n in names if (d := root / n / DATA_DIR).is_dir())
    return dirs, "" if dirs else f"no overlay of {harness!r} ships {DATA_DIR}/"


def roots(harness: str) -> tuple[Path, ...]:
    """Each overlay's task_notes directory, in the order the overlays were applied."""
    return _found(harness)[0]


def problem(harness: str) -> str:
    """Why harness has no recipe to serve, "" when it has one."""
    return _found(harness)[1]


@functools.cache
def _aliases(root: Path) -> dict:
    p = root / ALIASES
    return json.loads(p.read_text(encoding="utf-8")) if p.is_file() else {}


@functools.cache
def _text(root: Path, task: str) -> str:
    """One overlay's text for a task (through its aliases), "" when it ships none."""
    base = str(_aliases(root).get(task) or task.removesuffix("_random"))
    if not base or Path(base).name != base:
        return ""
    p = root / f"{base}.md"
    return p.read_text(encoding="utf-8") if p.is_file() else ""


def recipe(harness: str, task: str) -> str:
    """The task's recipe: every overlay's text for it, in the order they were applied."""
    return "".join(_text(r, task) for r in roots(harness)) if task else ""


def with_recipe(instruction: str, task: str, harness: str) -> str:
    """The instruction then the task's recipe, as the official entry composes its goal; the
    instruction unchanged when no overlay has text for the task."""
    text = recipe(harness, task)
    if not text.strip():
        return instruction
    block = f"{HEADING}\n{text.rstrip()}"
    return f"{instruction}\n\n{block}" if instruction else block


def clear() -> None:
    """Drop the cached directories, aliases and texts."""
    for f in (_found, _aliases, _text):
        f.cache_clear()
