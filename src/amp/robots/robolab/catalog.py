#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The benchmark catalog from RoboLab's registered envs: each task's instruction and time limit."""

from __future__ import annotations

import ast
import functools
import os
import sys
from pathlib import Path
from typing import Any


def resolve_instruction(instruction: Any, instruction_type: str = "default") -> str:
    """RoboLab's resolve_instruction, tolerant: a dict falls back to "default", then any value."""
    if isinstance(instruction, dict):
        value = instruction.get(instruction_type, instruction.get("default"))
        if value is None and instruction:
            value = next(iter(instruction.values()))
        return "" if value is None else str(value)
    return "" if instruction is None else str(instruction)


def _literal_class_attrs(node: ast.ClassDef) -> dict:
    """Literal class-level assignments of a parsed class (no import)."""
    out = {}
    for st in node.body:
        target = value = None
        if isinstance(st, ast.Assign) and len(st.targets) == 1:
            target, value = st.targets[0], st.value
        elif isinstance(st, ast.AnnAssign) and st.value is not None:
            target, value = st.target, st.value
        if isinstance(target, ast.Name):
            try:
                out[target.id] = ast.literal_eval(value)
            except (ValueError, SyntaxError, TypeError):
                pass
    return out


@functools.cache
def task_index(task_dir: str) -> dict[str, dict]:
    """Class name -> its literal class attributes, over every task file under task_dir."""
    index: dict[str, dict] = {}
    for path in sorted(Path(task_dir).rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        try:
            tree = ast.parse(path.read_text())
        except (OSError, SyntaxError, UnicodeDecodeError):
            continue
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and node.name not in index:
                attrs = _literal_class_attrs(node)
                if "instruction" in attrs:
                    index[node.name] = attrs
    return index


def task_metadata(name: str, info: dict | None, task_dir: str | None) -> dict:
    """instruction / episode_length_s from the Task class: its loaded module, else its file."""
    stem = Path(str((info or {}).get("task_file_name") or "")).stem
    for mod in list(sys.modules.values()):
        f = getattr(mod, "__file__", None)
        cls = getattr(mod, name, None) if stem and f and Path(f).stem == stem else None
        if isinstance(getattr(cls, "instruction", None), (dict, str)):
            return {"instruction": cls.instruction,
                    "episode_length_s": getattr(cls, "episode_length_s", None)}
    if task_dir and os.path.isdir(task_dir):
        attrs = task_index(task_dir).get(name)
        if attrs:
            return {"instruction": attrs["instruction"],
                    "episode_length_s": attrs.get("episode_length_s")}
    return {}


def catalog_tasks(names, get_env_info, task_dir: str | None,
                  instruction_type: str = "default") -> list[dict]:
    """The /sim/catalog task list for the registered env names, each named by its Task class."""
    tasks, seen = [], set()
    for name in names:
        if name in seen:
            raise ValueError(f"task {name!r} is registered twice")
        seen.add(name)
        info = get_env_info(name) or {}
        cfg = info.get("config_class")
        meta = task_metadata(name, info, task_dir)
        length = meta.get("episode_length_s", getattr(cfg, "episode_length_s", None))
        raw = meta.get("instruction", getattr(cfg, "instruction", ""))
        row = {
            "name": name,
            "instruction": resolve_instruction(raw, instruction_type),
            "attributes": list(getattr(cfg, "_task_attributes", None) or []),
            "episode_length_s": None if length is None else float(length),
        }
        if isinstance(raw, dict):
            row["instructions"] = {str(k): str(v) for k, v in raw.items() if isinstance(v, str)}
        tasks.append(row)
    return tasks
