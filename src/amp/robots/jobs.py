#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Benchmark mode, link side: send the catalog, run the jobs /sim/next hands out, report saves."""

from __future__ import annotations

import os
import time
import traceback
from collections.abc import Callable
from dataclasses import asdict, dataclass, field
from typing import Any, Protocol

from amp.contracts.eval import INSTRUCTION_TYPES
from amp.link.client import EpisodeAborted, LinkError
from amp.robots.robolab.records import check_run

LINK_RETRY_S = max(0.05, float(os.environ.get("AMP_TEST_LINK_RETRY_S") or 2.0))
MAX_REFUSED_HELLOS = 3
MAX_HELLO_MISSES = 3
CATALOG_TASK_KEYS = ("name", "instruction", "attributes", "episode_length_s")


class WorkerStopped(RuntimeError):
    """The worker says it must take no more jobs (its stop_reason()), e.g. a sensor gone for good."""


@dataclass
class Catalog:
    """The /sim/catalog body: the benchmark's tasks and the results folder."""

    policy_label: str
    output_dir: str
    episodes_per_task: int
    tasks: list[dict]

    def names(self) -> set[str]:
        return {t["name"] for t in self.tasks}


@dataclass
class Stats:
    saved: int = 0
    aborted: int = 0
    failed: int = 0
    records: list[dict] = field(default_factory=list)


class Worker(Protocol):
    """One robot running one job at a time; `loaded` is the task it has up (None when none)."""

    loaded: str | None

    def run_job(self, job: dict, task: str, k: int, run: str) -> dict:
        """Run and save episode k under run ("" = unscoped); its record. EpisodeAborted if so."""
        ...

    def unload(self) -> None:
        """Drop the loaded task after a failure or at the end; never raises."""
        ...


class Link(Protocol):
    def hello(self, *, takeover: bool = False) -> dict: ...

    def send_catalog(self, **catalog) -> dict: ...

    def next_job(self, loaded: str | None, *, failed: dict | None = None) -> dict | None: ...

    def report_saved(self, *, job_id, task: str, episode: int, record: dict) -> dict: ...


def _log(message: str) -> None:
    print(f"[amp.robots] {message}", flush=True)


def check_job(job: dict, catalog: Catalog) -> tuple[str, int, str]:
    """(task, episode, run) of a job, or ValueError naming what is wrong with it."""
    task, k = job.get("task"), job.get("episode")
    if task not in catalog.names():
        raise ValueError(f"job names task {task!r}, which is not in the catalog")
    if not isinstance(k, int) or isinstance(k, bool) or k < 0:
        raise ValueError(f"job episode must be a non-negative int, got {k!r}")
    itype = job.get("instruction_type")
    if itype is not None and itype not in INSTRUCTION_TYPES:
        raise ValueError(f"job instruction_type must be one of {list(INSTRUCTION_TYPES)}, "
                         f"got {itype!r}")
    return task, k, check_run(job.get("run"))


def failure(job: dict, exc: BaseException) -> dict:
    """The /sim/next "failed" entry for a job that raised."""
    return {**{f: job.get(f) for f in ("job_id", "task", "episode")},
            **({"run": job["run"]} if job.get("run") is not None else {}),
            **({"void": True} if getattr(exc, "void", False) is True else {}),
            "error": f"{type(exc).__name__}: {exc}"}


def serve(
    link: Link,
    catalog: Catalog,
    worker: Worker,
    *,
    retry_s: float = LINK_RETRY_S,
    max_refused: int = MAX_REFUSED_HELLOS,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str], None] = _log,
    resume: dict | None = None,
) -> Stats:
    """Serve jobs until the server says quit; a link failure re-hellos and resends the catalog."""
    stats = Stats()
    need_hello, need_catalog, failed, refused, misses = False, True, None, 0, 0

    def say_hello() -> bool:
        """Send a hello; False when it failed (a 409 count ends serve; else misses counts it)."""
        nonlocal need_hello, refused, misses
        try:
            link.hello()
        except LinkError as again:
            log(f"/sim/hello failed: {again}")
            if again.status == 409:
                misses = 0
                refused += 1
                if refused >= max_refused:
                    message = (f"/sim/hello refused {refused} times in a row: another "
                               "sim backend owns this robot server's link; this one stops")
                    log(message)
                    raise LinkError(message, again.status, again.body) from again
            else:
                misses += 1
            need_hello = True
            return False
        need_hello, refused, misses = False, 0, 0
        return True

    try:
        while True:
            stop = getattr(worker, "stop_reason", None)
            why = stop() if callable(stop) else None
            if why:
                log(f"taking no more jobs: {why}")
                raise WorkerStopped(why)
            if (need_hello or getattr(link, "hello_required", False) is True) and not say_hello():
                if misses < MAX_HELLO_MISSES:
                    sleep(retry_s * 2 ** max(misses - 1, 0))
                    continue
                if misses == MAX_HELLO_MISSES:
                    log(f"/sim/hello failed {misses} times in a row: serving without it; "
                        "retrying it before every request")
            try:
                if need_catalog:
                    link.send_catalog(**asdict(catalog))
                    need_catalog = False
                    log(f"catalog: {len(catalog.tasks)} tasks, results in {catalog.output_dir}")
                if resume is not None:
                    job, resume = resume, None
                else:
                    job = link.next_job(worker.loaded, failed=failed)
            except LinkError as exc:
                log(f"link failed, retrying in {retry_s:g} s: {exc}")
                sleep(retry_s)
                need_hello = need_catalog = True
                continue
            failed = None
            if job is None:
                continue
            if job.get("quit"):
                log(f"quit (job {job.get('job_id')})")
                return stats
            if getattr(link, "hello_required", False) is True and not say_hello():
                log(f"job {job.get('job_id')} kept: its server has no hello on record")
            outcome, record = _run(job, catalog, worker, log)
            if outcome == "failed":
                stats.failed += 1
                failed = record
                continue
            if outcome == "aborted":
                stats.aborted += 1
                continue
            stats.saved += 1
            stats.records.append(record)
            task, k = job["task"], job["episode"]
            where = f" (run {job['run']})" if job.get("run") else ""
            log(f"{task} episode {k}{where} saved: success={record.get('success')} "
                f"score={record.get('score')}")
            try:
                link.report_saved(job_id=job.get("job_id"), task=task, episode=k, record=record)
            except LinkError as exc:
                log(f"/sim/saved for {task} episode {k} failed: {exc}")
    finally:
        worker.unload()


def _run(job: dict, catalog: Catalog, worker: Worker, log) -> tuple[str, Any]:
    """("saved", record), ("aborted", None) or ("failed", its /sim/next failure entry)."""
    try:
        task, k, run = check_job(job, catalog)
        return "saved", worker.run_job(job, task, k, run)
    except EpisodeAborted as exc:
        worker.unload()
        log(f"{job.get('task')} episode {job.get('episode')} aborted "
            f"({exc.reason or 'no reason'}); nothing saved")
        return "aborted", None
    except Exception as exc:  # noqa: BLE001
        log(f"job {job} failed:\n{traceback.format_exc()}")
        worker.unload()
        return "failed", failure(job, exc)
