#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A RoboLab results folder: episode files, redo set-aside, episode_results.jsonl and its lock."""

from __future__ import annotations

import fcntl
import json
import os
import re
import time
from collections.abc import Iterator
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from amp.contracts.robot import RESULTS_LOCK, ROBOT_RESULT_FILE, EpisodeRef
from amp.fs.atomic import write_bytes_atomic, write_json_atomic
from amp.fs.safe_files import safe_read_text
from amp.link.wire import jsonable

RESULTS_FILE = "episode_results.jsonl"
SUPERSEDED = "superseded"
_STAMPED = re.compile(r"^\d{8}T\d{6}Z_ep(\d+)(?:_\d+)?$")
_RUN_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def check_run(run) -> str:
    """A job's "run" as a single safe path component; "" when absent (unscoped)."""
    if run is None or run == "":
        return ""
    if not isinstance(run, str) or not _RUN_ID.match(run) or run in (".", ".."):
        raise ValueError(f"job run {run!r} is not a plain run id ([A-Za-z0-9][A-Za-z0-9._-]*)")
    return run


def run_dir(output_dir: str, run: str) -> str:
    """Where a run's episodes go: <output_dir>/<run>, or output_dir itself when unscoped."""
    return os.path.join(output_dir, check_run(run)) if run else output_dir


def results_roots(output_dir: str) -> list[str]:
    """output_dir plus every run subdir that holds an episode_results.jsonl."""
    roots = [output_dir]
    if os.path.isdir(output_dir):
        for name in sorted(os.listdir(output_dir)):
            sub = os.path.join(output_dir, name)
            if not os.path.islink(sub) and os.path.isfile(os.path.join(sub, RESULTS_FILE)):
                roots.append(sub)
    return roots


def episode_file_names(names, k: int) -> list[str]:
    """The names among `names` that belong to episode k of a RoboLab task folder."""
    k = int(k)
    exact = {f"run_{k}.hdf5", f"run_{k}.hdf5_failed.hdf5", f"log_{k}_env0.json", f"teleop_{k}.json",
             f"run_{k}_state.json",
             f"robodojo_result_{k}.json"}
    video = re.compile(rf"^.+_{k}(_viewport)?\.mp4$")
    return sorted(n for n in names if n in exact or video.match(n))


def episode_files(task_dir: str, k: int) -> list[str]:
    if not os.path.isdir(task_dir):
        return []
    names = [n for n in os.listdir(task_dir) if os.path.isfile(os.path.join(task_dir, n))]
    return episode_file_names(names, k)


def utc_stamp(now: float | None = None) -> str:
    t = datetime.fromtimestamp(time.time() if now is None else now, tz=timezone.utc)  # noqa: UP017
    return t.strftime("%Y%m%dT%H%M%SZ")


def set_aside(task_dir: str, k: int, *, now: float | None = None) -> str | None:
    """Move episode k's files into <task>/superseded/<UTC ts>_ep<k>/; that dir, or None if none."""
    names = episode_files(task_dir, k)
    if not names:
        return None
    base = os.path.join(task_dir, SUPERSEDED, f"{utc_stamp(now)}_ep{int(k)}")
    dest, n = base, 1
    while os.path.exists(dest):
        n += 1
        dest = f"{base}_{n}"
    os.makedirs(dest)
    for name in names:
        os.replace(os.path.join(task_dir, name), os.path.join(dest, name))
    return dest


def discard_attempt(task_dir: str, k: int, moved: str | None) -> None:
    """Delete the files an unsaved attempt at episode k left, and move the set-aside ones back."""
    for name in episode_files(task_dir, k):
        os.remove(os.path.join(task_dir, name))
    if moved is None or not os.path.isdir(moved):
        return
    for name in os.listdir(moved):
        os.replace(os.path.join(moved, name), os.path.join(task_dir, name))
    os.rmdir(moved)
    parent = os.path.dirname(moved)
    if not os.listdir(parent):
        os.rmdir(parent)


def parse_records(text: str) -> list[dict]:
    """episode_results.jsonl lines as dicts, skipping blank and malformed lines as RoboLab does."""
    out = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue
        if isinstance(rec, dict):
            out.append(rec)
    return out


def load_records(path: str) -> list[dict]:
    if not os.path.exists(path):
        return []
    with open(path) as f:
        return parse_records(f.read())


def write_records(path: str, records: list[dict]) -> None:
    """Rewrite episode_results.jsonl atomically; NaN metrics stay as RoboLab writes them."""
    write_bytes_atomic(path, "".join(json.dumps(r) + "\n" for r in records).encode())


def _is(rec: dict, task: str, k: int) -> bool:
    return rec.get("episode") == k and task in (rec.get("env_name"), rec.get("task_name"))


def find_record(records: list[dict], task: str, k: int) -> dict | None:
    """The LAST record for (task, episode k), or None."""
    for rec in reversed(records):
        if _is(rec, task, k):
            return rec
    return None


def keep_latest(path: str, task: str, k: int,
                extra: dict | None = None) -> tuple[list[dict], dict | None]:
    """Keep only the last record for (task, k), updated with `extra`; (all records, that one)."""
    records = load_records(path)
    latest = find_record(records, task, k)
    kept = [r for r in records if not _is(r, task, k) or r is latest]
    if latest is not None and extra:
        latest.update(extra)
    if len(kept) != len(records) or (latest is not None and extra):
        write_records(path, kept)
    return kept, latest


@contextmanager
def results_lock(output_dir: str) -> Iterator[None]:
    """Hold an exclusive flock on <output_dir>/RESULTS_LOCK (blocking) for the block."""
    path = os.path.join(output_dir, RESULTS_LOCK)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        yield
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def recover_all(output_dir: str) -> list[str]:
    """recover_interrupted_redos over the unscoped folder and every run's."""
    out = []
    for root in results_roots(output_dir):
        prefix = "" if root == output_dir else f"{os.path.basename(root)}/"
        records = load_records(os.path.join(root, RESULTS_FILE))
        out += [prefix + what for what in recover_interrupted_redos(root, records)]
    return out


def recover_interrupted_redos(output_dir: str, records: list[dict]) -> list[str]:
    """Put back a set-aside episode whose redo died mid-run (its files gone, its record kept)."""
    restored = []
    saved = {(r.get(f), r.get("episode")) for r in records for f in ("env_name", "task_name")}
    for task in sorted(os.listdir(output_dir)) if os.path.isdir(output_dir) else []:
        sup = os.path.join(output_dir, task, SUPERSEDED)
        if not os.path.isdir(sup):
            continue
        newest: dict[int, str] = {}
        for d in sorted(os.listdir(sup)):
            m = _STAMPED.match(d)
            if m and os.path.isdir(os.path.join(sup, d)):
                newest[int(m.group(1))] = os.path.join(sup, d)
        for k, src in sorted(newest.items()):
            task_dir = os.path.join(output_dir, task)
            finished = os.path.isfile(os.path.join(task_dir, f"log_{k}_env0.json"))
            if (task, k) in saved and not finished:
                for name in episode_files(task_dir, k):
                    os.remove(os.path.join(task_dir, name))
                for name in os.listdir(src):
                    os.replace(os.path.join(src, name), os.path.join(task_dir, name))
                os.rmdir(src)
                restored.append(f"{task} episode {k}")
    return restored


_JSON_LIMIT = 64 << 20


def _read_json(path: Path, root: Path):
    return json.loads(safe_read_text(path, root, limit=_JSON_LIMIT))


def result_row(episode: EpisodeRef) -> dict | None:
    """The episode's row of <results>/../episode_results.jsonl, or None."""
    results = episode.results_dir.parent / RESULTS_FILE
    try:
        rows = parse_records(safe_read_text(results, episode.results_dir.parent))
    except OSError:
        return None
    return find_record(rows, episode.task, episode.episode)


def ingest_results(src: str, episode: EpisodeRef, tracer) -> None:
    """RoboLab's own logs for one episode as `src` trace events (robot.<name>.*)."""
    tr = tracer.child(src)
    prefix = src.replace("/", ".")
    k, task_dir = episode.episode, episode.results_dir
    files = {n: task_dir / n for n in episode_files(str(task_dir), k)}
    tr.event(f"{prefix}.files", {"task": episode.task, "episode": k, "run_id": episode.run_id,
                                 "files": [
        {"name": n, "bytes": p.lstat().st_size} for n, p in files.items()]})
    missing = []
    for name, kind in ((f"log_{k}_env0.json", "log"), (f"teleop_{k}.json", "teleop_record")):
        if name not in files:
            missing.append(name)
            continue
        try:
            body = _read_json(files[name], task_dir)
        except (OSError, ValueError) as exc:
            tr.event(f"{prefix}.unreadable", {"file": name, "error": str(exc)}, lvl="warn")
            continue
        summary = {k2: v for k2, v in body.items() if not isinstance(v, (dict, list))} \
            if isinstance(body, dict) else {}
        tr.event(f"{prefix}.{kind}", {"file": name, **jsonable(summary)},
                 blobs=[tr.blob(body, mime="application/json", role=kind, compress=True)])
    row = result_row(episode)
    if row is None:
        missing.append(RESULTS_FILE)
    else:
        tr.event(f"{prefix}.result", jsonable(row))
    if missing:
        tr.event(f"{prefix}.missing", {"files": missing}, lvl="warn")


def result_artifacts(episode: EpisodeRef) -> list[Path]:
    """Episode k's videos, run_<k>.hdf5, logs and its results row (written into the episode dir)."""
    out = [episode.results_dir / n for n in episode_files(str(episode.results_dir),
                                                           episode.episode)]
    row = result_row(episode)
    if row is not None:
        path = episode.episode_dir / ROBOT_RESULT_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        write_json_atomic(path, jsonable(row), indent=1)
        out.append(path)
    return out
