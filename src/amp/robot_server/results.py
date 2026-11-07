#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Benchmark bookkeeping: the catalog, RoboLab's saved records, teleop records, api/tasks and runs."""

from __future__ import annotations

import fcntl
import json
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.robot import RESULTS_LOCK
from amp.fs.atomic import write_json_atomic
from amp.link.wire import jsonable

RECORD_FORMAT = "teleop-episode-1"
RESULTS_FILE = "episode_results.jsonl"
MAX_RUN_ID = 128


@dataclass
class Task:
    name: str
    instruction: str = ""
    attributes: list = field(default_factory=list)
    episode_length_s: float | None = None
    instructions: dict = field(default_factory=dict)

    def to_json(self) -> dict:
        out = {"name": self.name, "instruction": self.instruction,
               "attributes": list(self.attributes), "episode_length_s": self.episode_length_s}
        if self.instructions:
            out["instructions"] = dict(self.instructions)
        return out


@dataclass
class Catalog:
    tasks: list[Task]
    output_dir: str | None = None
    episodes_per_task: int = 1
    policy_label: str = ""

    def task(self, name) -> Task | None:
        return next((t for t in self.tasks if t.name == name), None)

    @property
    def root(self) -> Path | None:
        """output_dir when it exists on this filesystem, else None."""
        if not self.output_dir:
            return None
        p = Path(self.output_dir)
        return p if p.is_dir() else None


def safe_component(name) -> bool:
    """name is one plain path component: no separators, no '.'/'..', no NUL, not hidden."""
    return (isinstance(name, str) and bool(name) and "/" not in name and "\\" not in name
            and "\x00" not in name and not name.startswith("."))


def parse_catalog(body: dict, default_label: str) -> Catalog:
    """Validate a /sim/catalog body; ValueError on anything malformed."""
    raw = body.get("tasks")
    if not isinstance(raw, list) or not raw:
        raise ValueError("catalog needs a non-empty 'tasks' list")
    tasks, seen = [], set()
    for i, t in enumerate(raw):
        if not isinstance(t, dict):
            raise ValueError(f"tasks[{i}] must be an object")  # noqa: TRY004
        name = t.get("name")
        if not safe_component(name):
            raise ValueError(f"tasks[{i}].name must be a plain name, got {name!r}")
        if name in seen:
            raise ValueError(f"task {name!r} is listed twice")
        seen.add(name)
        attrs = t.get("attributes") or []
        if not isinstance(attrs, list):
            raise ValueError(f"tasks[{i}].attributes must be a list")  # noqa: TRY004
        length = t.get("episode_length_s")
        if length is not None:
            if isinstance(length, bool) or not isinstance(length, (int, float)) \
                    or not math.isfinite(length):
                raise ValueError(f"tasks[{i}].episode_length_s must be a number or null")
            length = float(length)
        variants = t.get("instructions") or {}
        if not isinstance(variants, dict) or not all(
                isinstance(k, str) and isinstance(v, str) for k, v in variants.items()):
            raise ValueError(f"tasks[{i}].instructions must map wording names to strings")
        tasks.append(Task(name, str(t.get("instruction") or ""), attrs, length, dict(variants)))
    n = body.get("episodes_per_task", 1)
    if isinstance(n, bool) or not isinstance(n, int) or n < 1:
        raise ValueError(f"episodes_per_task must be a positive integer, got {n!r}")
    out = body.get("output_dir")
    if out is not None and not isinstance(out, str):
        raise ValueError("output_dir must be a string or null")
    label = body.get("policy_label") or default_label
    if not isinstance(label, str):
        raise ValueError("policy_label must be a string")  # noqa: TRY004
    return Catalog(tasks, out or None, n, label)


def record_key(rec) -> tuple[str, int] | None:
    """(task, episode) of an episode_results.jsonl record, or None if it has none."""
    if not isinstance(rec, dict):
        return None
    env, cls = rec.get("env_name"), rec.get("task_name")
    task = cls if isinstance(env, str) and isinstance(cls, str) and cls and env.startswith(cls) \
        else env or cls
    ep = rec.get("episode", rec.get("run"))
    if not isinstance(task, str) or isinstance(ep, bool):
        return None
    try:
        return task, int(ep)
    except (TypeError, ValueError):
        return None


def read_results(path) -> list[dict]:
    """The records of an episode_results.jsonl, skipping unparseable lines; [] if absent."""
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []
    out = []
    for line in text.splitlines():
        try:
            rec = json.loads(line) if line.strip() else None
        except json.JSONDecodeError:
            continue
        if isinstance(rec, dict):
            out.append(rec)
    return out


class ResultsFile:
    """episode_results.jsonl, re-read only when its size or mtime changes."""

    def __init__(self, path):
        self.path = Path(path)
        self._sig = None
        self._records: dict = {}

    def records(self) -> dict:
        """{(task, episode): record}; a later line for the same key wins."""
        try:
            st = self.path.stat()
            sig = (st.st_mtime_ns, st.st_size)
        except OSError:
            sig = None
        if sig != self._sig:
            self._sig = sig
            out = {}
            for rec in read_results(self.path) if sig else ():
                key = record_key(rec)
                if key is not None:
                    out[key] = rec
            self._records = out
        return self._records


def next_free_episode(taken) -> int:
    k, taken = 0, set(taken)
    while k in taken:
        k += 1
    return k


def task_status(n_wanted: int, records: list[dict]) -> str:
    if not records:
        return "todo"
    if any(r.get("success") is not True for r in records):
        return "failed"
    return "partial" if len(records) < n_wanted else "success"


def progress(catalog: Catalog, records: dict, running: str | None = None):
    """(per-task [(task, status, records sorted by episode)], done, succeeded)."""
    by_task: dict[str, list] = {}
    for (task, _), rec in records.items():
        by_task.setdefault(task, []).append(rec)
    rows, done, ok = [], 0, 0
    for t in catalog.tasks:
        recs = sorted(by_task.get(t.name, []), key=lambda r: record_key(r)[1])
        base = task_status(catalog.episodes_per_task, recs)
        done += len(recs) >= catalog.episodes_per_task
        ok += base == "success"
        rows.append((t, "running" if t.name == running else base, recs))
    return rows, done, ok


def teleop_path(root, task: str, k: int) -> Path:
    return Path(root) / task / f"teleop_{int(k)}.json"


def check_run_id(run) -> str:
    """A run_id as select accepts it: "" (unscoped) or one plain path component."""
    if run is None or run == "":
        return ""
    if not safe_component(run) or len(run) > MAX_RUN_ID or run in (".", ".."):
        raise ValueError(f"run_id must be a plain name (a path segment), got {run!r}")
    return run


class Results:
    """The catalog plus every saved record: RoboLab's results file, /sim/saved, teleop records."""

    def __init__(self, default_label: str):
        self.default_label = default_label
        self.catalog: Catalog | None = None
        self._files: dict[str, ResultsFile] = {}
        self.saved: dict = {}

    def set_catalog(self, body: dict) -> Catalog:
        cat = parse_catalog(body, self.default_label)
        self.catalog = cat
        self._files = {}
        return cat

    @property
    def root(self) -> Path | None:
        return self.catalog.root if self.catalog is not None else None

    def run_root(self, run: str = "") -> Path | None:
        root = self.root
        return None if root is None else (root / run if run else root)

    def records(self, run: str = "") -> dict:
        """{(task, k): record} of a run: its results file overlaid with /sim/saved records."""
        root = self.run_root(run)
        disk = {}
        if root is not None:
            f = self._files.get(run)
            if f is None:
                f = self._files[run] = ResultsFile(root / RESULTS_FILE)
            disk = f.records()
        mine = {(t, k): r for (rr, t, k), r in self.saved.items() if rr == run}
        return {**disk, **mine}

    def save_record(self, task: str, k: int, record: dict, run: str = "") -> None:
        self.saved[(run, task, k)] = jsonable(record)

    def taken(self, task: str, run: str = "") -> set[int]:
        return {e for (t, e) in self.records(run) if t == task}

    def teleop_file(self, rec: dict, run: str = "") -> Path | None:
        """The record's path in the run's results folder; None without a local folder."""
        root = self.run_root(run)
        return None if root is None else teleop_path(root, rec["task"], rec["episode"])

    def write_teleop(self, path: Path, rec: dict) -> None:
        """Write teleop_<k>.json under the results lock the backend shares."""
        path.parent.mkdir(parents=True, exist_ok=True)
        lock = self.root / RESULTS_LOCK
        lock.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(lock, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o644)
        try:
            os.fchmod(fd, 0o644)
            fcntl.flock(fd, fcntl.LOCK_EX)
            write_json_atomic(path, jsonable(rec))
        finally:
            os.close(fd)

    def tasks_json(self, running: str | None) -> dict:
        """GET api/tasks: every catalog task with its status and saved (unscoped) episodes."""
        cat = self.catalog
        rows, done, ok = progress(cat, self.records(), running)
        tasks = []
        for t, status, recs in rows:
            episodes = [{"episode": record_key(r)[1], "success": r.get("success"),
                         "score": r.get("score"), "episode_step": r.get("episode_step"),
                         "events": r.get("events")} for r in recs]
            tasks.append({**t.to_json(), "status": status, "episodes": episodes})
        return {"ok": True, "policy_label": cat.policy_label, "output_dir": cat.output_dir,
                "episodes_per_task": cat.episodes_per_task, "done": done, "succeeded": ok,
                "tasks": tasks}

    def run_json(self, task: str, k: int, run: str = "") -> dict | None:
        """GET api/runs[/<run>]/<task>/<k>, or None for an unknown task."""
        if self.catalog.task(task) is None:
            return None
        return {"ok": True, "record": self.records(run).get((task, k))}

    def summary(self) -> tuple[int, int]:
        _, done, ok = progress(self.catalog, self.records())
        return done, ok
