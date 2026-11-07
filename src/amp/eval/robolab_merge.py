#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Merge the slots' RoboLab output folders with RoboLab's own tools, then check the row census."""

from __future__ import annotations

import os
import shutil
import subprocess
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from amp.robots.robolab.records import load_records

RESULTS = "episode_results.jsonl"
NON_ROBOLAB = ("teleop_*.json", ".teleop", "superseded")


class MergeError(RuntimeError):
    """A RoboLab tool failed, or the merged rows are not tasks x N with episodes 0..N-1."""


@dataclass
class Census:
    rows: int
    by_task: dict[str, list[int]] = field(default_factory=dict)
    problems: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.problems


def row_task(row: dict) -> str | None:
    t = row.get("env_name") or row.get("task_name")
    return t if isinstance(t, str) else None


def census(rows: Iterable[dict], tasks: Sequence[str], num_runs: int) -> Census:
    """Check rows == len(tasks) x num_runs and each task's episodes == {0..num_runs-1}."""
    rows = list(rows)
    by: dict[str, list[int]] = {t: [] for t in tasks}
    out = Census(rows=len(rows), by_task=by)
    for r in rows:
        t, k = row_task(r), r.get("episode")
        if t not in by:
            out.problems.append(f"row for a task outside the run: {t!r}")
            continue
        by[t].append(k)
    want = list(range(num_runs))
    for t, ks in by.items():
        if sorted(ks) != want:
            dup = sorted({k for k in ks if ks.count(k) > 1})
            missing = sorted(set(want) - set(ks))
            out.problems.append(f"{t}: episodes {sorted(ks)}, missing {missing}"
                                + (f", duplicated {dup}" if dup else ""))
    if len(rows) != len(tasks) * num_runs:
        out.problems.append(f"{len(rows)} rows, expected {len(tasks)} x {num_runs}")
    return out


def _ignored(_dir: str, names: list[str]) -> set[str]:
    import fnmatch

    return {n for n in names if any(fnmatch.fnmatchcase(n, p) for p in NON_ROBOLAB)}


def link_copy(src: Path, dest: Path) -> None:
    """A hard-link copy of src at dest (cp -al), without the non-RoboLab files."""
    shutil.copytree(src, dest, copy_function=os.link, ignore=_ignored, dirs_exist_ok=False)
    for d, subdirs, files in os.walk(dest, topdown=False):
        if Path(d) != dest and not files and not os.listdir(d):
            os.rmdir(d)


def run_tool(python: str, script: Path, args: Sequence[str], log: Path | None = None
             ) -> subprocess.CompletedProcess:
    p = subprocess.run([python, str(script), *args], capture_output=True, text=True,
                       stdin=subprocess.DEVNULL, check=False, cwd=str(script.parent.parent))
    if log is not None:
        log.parent.mkdir(parents=True, exist_ok=True)
        with open(log, "a") as f:
            f.write(f"$ {script.name} {' '.join(args)}\n{p.stdout}{p.stderr}\n")
    return p


def merge(sources: Sequence[Path], dest: Path, *, robolab_root: str, python: str,
          tasks: Sequence[str], num_runs: int, work: Path | None = None,
          log: Path | None = None) -> dict:
    """Merge one harness configuration's slot folders into dest with compile_results.py --merge;
    read_results.py summarizes it."""
    root = Path(robolab_root)
    sources = [Path(s) for s in sources if (Path(s) / RESULTS).is_file()]
    if not sources:
        raise MergeError("no RoboLab output folder to merge")
    if dest.exists():
        raise MergeError(f"{dest} exists: merge into a fresh folder")
    work = work or dest.parent / f".{dest.name}.copies"
    if work.exists():
        shutil.rmtree(work)
    copies = []
    for i, s in enumerate(sources):
        c = work / f"part{i}"
        link_copy(s, c)
        copies.append(c)
    p = run_tool(python, root / "analysis" / "compile_results.py",
                 [*map(str, copies), "--merge", str(dest), "--keep", "-y"], log)
    if p.returncode != 0:
        raise MergeError(f"compile_results.py --merge exited {p.returncode}: "
                         f"{(p.stderr or p.stdout).strip()[-600:]}")
    shutil.rmtree(work, ignore_errors=True)
    summary = run_tool(python, root / "analysis" / "read_results.py", [str(dest)], log)
    if summary.returncode != 0:
        raise MergeError(f"read_results.py exited {summary.returncode}: "
                         f"{(summary.stderr or summary.stdout).strip()[-600:]}")
    c = census(load_records(str(dest / RESULTS)), tasks, num_runs)
    if not c.ok:
        raise MergeError("merged rows fail the census: " + "; ".join(c.problems[:20]))
    return {"dest": str(dest), "rows": c.rows, "tasks": len(tasks), "num_runs": num_runs,
            "summary": summary.stdout}
