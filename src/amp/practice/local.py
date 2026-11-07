#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Practice jobs on this machine: a submit runs `amp eval robodojo` from a thread of the driver, and
the drivers read its run folder (units, episodes, traces) as they would read any finished job's."""

from __future__ import annotations

import fcntl
import json
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

from amp.contracts.eval import Arm

JOB_FILE = "job.json"
LOG_FILE = "job.log"
LOCK_FILE = ".practice.lock"
TERMINAL = frozenset({"done", "failed"})
RELAUNCHES = 3

_threads: dict[str, threading.Thread] = {}
_guard = threading.Lock()


def profile(cfg: dict):
    """The study's agent profile: what practice episodes run and the debrief continues as."""
    from amp.eval.robolab_official import make_profile

    return make_profile(cfg["model"], cfg.get("reasoning"), cfg.get("prompt", "amp1"))


def arm_id(cfg: dict) -> str:
    """The arm folder amp eval writes for the study's harness, model, reasoning and prompt."""
    return Arm(cfg["harness"], profile(cfg)).id


def eval_argv(cfg: dict, *, tasks: list[str], corpus: str, exp_tag: str,
              episodes: int | None = None, layouts: list[int] | None = None,
              workspace: Path | None = None, sha: str | None = None, shared: bool = False,
              notebook: int | None = None, max_bytes: int | None = None,
              workspaces: Path | None = None) -> list[str]:
    """amp eval robodojo's argv for one job, without --output (workspaces: a notebooks manifest)."""
    argv = [sys.executable, "-m", "amp", "eval", "robodojo", "--harness", cfg["harness"],
            "--model", cfg["model"], "--prompt", cfg.get("prompt", "amp1"),
            "--corpus", corpus, "--task", ",".join(tasks)]
    if cfg.get("reasoning"):
        argv += ["--reasoning", str(cfg["reasoning"])]
    argv += (["--layouts", ",".join(str(k) for k in layouts)] if layouts is not None
             else ["--episodes", str(episodes)])
    if workspace is not None or workspaces is not None:
        if workspace is not None:
            argv += ["--workspace", str(workspace)]
        else:
            argv += ["--workspaces", str(workspaces)]
        if sha:
            argv += ["--workspace-sha", sha]
        if shared:
            argv.append("--shared-workspace")
        if notebook:
            argv += ["--workspace-notebook", str(notebook)]
        if max_bytes:
            argv += ["--workspace-max-bytes", str(max_bytes)]
    argv += ["--gpus", str(cfg.get("gpus", "auto"))]
    return argv + ["--exp-tag", exp_tag]


def split_tag(argv: list[str]) -> tuple[list[str], str]:
    """(argv without --exp-tag, the tag): amp eval takes no tag; the job id carries it."""
    if "--exp-tag" not in argv:
        raise ValueError("a practice job's argv names its --exp-tag")
    i = argv.index("--exp-tag")
    return argv[:i] + argv[i + 2:], argv[i + 1]


def read_job(job_dir: Path) -> dict | None:
    try:
        return json.loads((Path(job_dir) / JOB_FILE).read_text())
    except (OSError, ValueError):
        return None


def write_job(job_dir: Path, rec: dict) -> None:
    p = Path(job_dir) / JOB_FILE
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(rec, indent=1))
    os.replace(tmp, p)


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _running_here(job_dir: Path) -> bool:
    with _guard:
        t = _threads.get(str(job_dir))
        return t is not None and t.is_alive()


def start(job_dir: Path) -> None:
    """Run the job in a thread of this process; it waits its turn on the runs folder's lock."""
    job_dir = Path(job_dir).resolve()
    with _guard:
        t = _threads.get(str(job_dir))
        if t is not None and t.is_alive():
            return
        rec = read_job(job_dir) or {}
        rec.update(launches=int(rec.get("launches", 0)) + 1, state="queued", owner=os.getpid())
        write_job(job_dir, rec)
        t = threading.Thread(target=run_job, args=(job_dir,), name=f"practice-{job_dir.name}",
                             daemon=True)
        _threads[str(job_dir)] = t
        t.start()


def run_job(job_dir: Path) -> int:
    """Wait for the runs folder's lock, then run amp eval to the end; its exit code."""
    job_dir = Path(job_dir)
    rec = read_job(job_dir)
    if rec is None:
        return 2
    with open(job_dir.parent / LOCK_FILE, "a") as lock, open(job_dir / LOG_FILE, "ab") as log:
        fcntl.flock(lock, fcntl.LOCK_EX)
        rec.update(state="running", started_at=time.time())
        write_job(job_dir, rec)
        argv = rec["argv"] + ["--output", str(job_dir), "--job-id", rec["job"]]
        log.write(f"[practice] {' '.join(argv)}\n".encode())
        log.flush()
        child = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT,  # nosec B603
                                 stdin=subprocess.DEVNULL, pass_fds=(lock.fileno(),))
        rec["child"] = child.pid
        write_job(job_dir, rec)
        rc = child.wait()
    rec.update(state="done" if rc == 0 else "failed", returncode=rc, ended_at=time.time())
    write_job(job_dir, rec)
    return rc


class LocalRunner:
    """The drivers' runner: runner(argv) records the job, starts it and returns its id at once."""

    def __init__(self, runs: Path):
        self.runs = Path(runs)

    def __call__(self, argv: list[str]) -> str:
        argv, tag = split_tag(list(argv))
        job = f"{tag}-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:6]}"
        d = self.runs / job
        d.mkdir(parents=True)
        write_job(d, {"job": job, "exp_tag": tag, "argv": argv, "state": "queued",
                      "created_at": time.time(), "launches": 0})
        start(d)
        return job


class LocalPool:
    """The drivers' view of the jobs under one runs folder."""

    def __init__(self, cfg: dict, runs: Path | None = None):
        self.runs = Path(runs if runs is not None else cfg["runs"])
        self.arm = arm_id(cfg)

    def job_dir(self, job: str) -> Path:
        return self.runs / job

    def units_dir(self, job: str) -> Path:
        return self.job_dir(job) / self.arm / "agentic" / "units"

    def count(self, job: str) -> int:
        """Unit records written; while the job runs, only final ones (amp eval retries a unit
        that ended infra_error, rewriting its record)."""
        from amp.contracts.eval import is_retryable

        rows = self.units(job)
        if (read_job(self.job_dir(job)) or {}).get("state") in TERMINAL:
            return len(rows)
        return sum(1 for r in rows if not is_retryable(r))

    def units(self, job: str) -> list[dict]:
        out = []
        for p in sorted(self.units_dir(job).glob("*.json")):
            try:
                out.append(json.loads(p.read_text()))
            except (OSError, ValueError):
                continue
        return out

    def job_state(self, job: str) -> str | None:
        """queued, running, done or failed; a job no live driver runs is resumed here."""
        d = self.job_dir(job)
        rec = read_job(d)
        if rec is None:
            return None
        if rec.get("state") in TERMINAL or _running_here(d.resolve()):
            return rec.get("state")
        owner, child = rec.get("owner"), rec.get("child")
        if isinstance(owner, int) and owner != os.getpid() and _alive(owner):
            return rec.get("state")
        if isinstance(child, int) and _alive(child):
            return "running"
        if int(rec.get("launches", 0)) > RELAUNCHES:
            rec.update(state="failed", error="left unfinished too often")
            write_job(d, rec)
            return "failed"
        start(d)
        return "queued"

    def find_job(self, exp_tag: str, since: float) -> str | None:
        """The earliest job with this tag created at or after since, or None."""
        got = [r for r in (read_job(p.parent) for p in self.runs.glob(f"*/{JOB_FILE}"))
               if r and r.get("exp_tag") == exp_tag and r.get("created_at", 0) >= since]
        return min(got, key=lambda r: r["created_at"])["job"] if got else None

    def fetch(self, row: dict, dest: Path) -> list[str]:
        """robot_result.json, trace/agent.jsonl and its blobs copied to dest; the paths missing."""
        from amp.practice import history as H

        return H.fetch_episode(row.get("artifact_uri") or "", dest)

    def agent_trace(self, row: dict) -> str | None:
        p = Path(row.get("artifact_uri") or "") / "trace" / "agent.jsonl"
        try:
            return p.read_text()
        except OSError:
            return None
