#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Local mode: a study on this machine's slots, N at a time, in the run layout under runs_dir."""

from __future__ import annotations

import threading
from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.eval import (
    DELIVERED_STATUSES,
    EpisodeResult,
    NodeRuntime,
    is_retryable,
)
from amp.contracts.trace import NullTracer

from .episode import RunnerOptions, run_unit
from .finalize import finalize_run, write_run_config
from .layout import RunLayout
from .provenance import runtime_facts
from .slots import retarget, slot_paths
from .store import read_record, write_json
from .study import Study, StudyUnit, expand

MAX_ATTEMPTS = 3


@dataclass
class LocalRun:
    root: Path
    results: list[EpisodeResult] = field(default_factory=list)

    @property
    def ok(self) -> int:
        return sum(r.status in DELIVERED_STATUSES for r in self.results)


class _Todo:
    """The units still to run, (unit, attempt, task_set), in order; a slot takes what it serves."""

    def __init__(self):
        self._items: list[tuple[StudyUnit, int, str]] = []
        self._lock = threading.Lock()

    def put(self, item: tuple[StudyUnit, int, str]) -> None:
        with self._lock:
            self._items.append(item)

    def take(self, slot) -> tuple[StudyUnit, int, str] | None:
        serves = getattr(slot, "serves", None)
        with self._lock:
            for i, item in enumerate(self._items):
                if not callable(serves) or serves(item[2]):
                    return self._items.pop(i)
        return None

    def drain(self) -> list[tuple[StudyUnit, int, str]]:
        with self._lock:
            out, self._items = self._items, []
            return out

    def waiting(self) -> Counter[str]:
        """Units left by task set."""
        with self._lock:
            return Counter(ts for _, _, ts in self._items)


def run_local(study: Study, *, launcher, runs_dir, runtime: NodeRuntime, parallel: int = 1,
              tracer=None, options: RunnerOptions | None = None,
              abort: threading.Event | None = None, retries: int = MAX_ATTEMPTS - 1,
              on_result: Callable[[EpisodeResult], None] | None = None,
              provenance: dict | None = None, root: Path | None = None) -> LocalRun:
    """Run every unit of study with run_unit on up to `parallel` of launcher.slots, then finalize the run."""
    root = Path(root) if root is not None else Path(runs_dir) / study.job_id
    layouts = {a.id: RunLayout(root / a.id) for a in study.arms}
    for arm in study.arms:
        write_run_config(layouts[arm.id], study, arm)
    run = LocalRun(root)
    abort = abort or threading.Event()
    kw = {"study": study, "layouts": layouts, "run": run, "parallel": parallel,
          "runtime": runtime, "tracer": tracer or NullTracer(),
          "options": options, "abort": abort, "retries": retries, "on_result": on_result,
          "provenance": runtime_facts() if provenance is None else provenance}
    _run_units(expand(study), launcher, **kw)
    finalize_run(study, root)
    return run


def _run_units(units: list[StudyUnit], launcher, *, study: Study, layouts: dict, run: LocalRun,
               parallel: int, runtime: NodeRuntime, tracer,
               options: RunnerOptions | None, abort: threading.Event, retries: int,
               on_result, provenance: dict) -> None:
    slots = list(launcher.slots)[:max(1, int(parallel))]
    if not slots:
        raise ValueError("the launcher has no slots")
    lock = threading.Lock()
    todo = _Todo()
    for u in units:
        prior = read_record(layouts[u.ref.arm_id].root, u.ref.task, u.ref.episode)
        if prior is not None and not is_retryable(prior):
            res = EpisodeResult.from_record(prior)
            if res.status in DELIVERED_STATUSES:
                res.status = "resumed"
            run.results.append(res)
            continue
        todo.put((u, 1, study.request(u.ref).task_set))
    next_set = getattr(launcher, "next_task_set", None)

    def not_retargeted(slot, req, why: str) -> EpisodeResult:
        res = EpisodeResult(job_id=req.job_id, arm_id=req.arm.id, task=req.task,
                            episode=req.episode, status="infra_error", node=slot_paths(slot).node_id,
                            slot=slot.slot_id, error=why)
        write_json(layouts[req.arm.id].unit_path(req.task, req.episode), res.to_record())
        tracer.event("eval.episode.end", res.to_record())
        return res

    def one(slot, u: StudyUnit, attempt: int, task_set: str) -> None:
        req = study.request(u.ref)
        capgym = study.eval_name == "capgym"
        why = None
        if capgym or req.protocol.official:
            moved, why = retarget(slot, req.task_set, req.task,
                                  req.protocol.official and not capgym)
            if moved:
                tracer.event("eval.local.retarget", {"slot": slot.slot_id, "task": req.task,
                                                     "task_set": req.task_set})
        if why is not None:
            res = not_retargeted(slot, req, why)
        else:
            res = run_unit(req, slot, runtime=runtime, run_dir=layouts[u.ref.arm_id].root,
                           tracer=tracer, abort=abort, options=options, provenance=provenance)
        if is_retryable(res.to_record()) and attempt <= retries and not abort.is_set():
            todo.put((u, attempt + 1, task_set))
            return
        with lock:
            run.results.append(res)
        if on_result is not None:
            on_result(res)

    def worker(slot) -> None:
        while not abort.is_set():
            item = todo.take(slot)
            if item is None:
                waiting = todo.waiting()
                to = next_set(slot, waiting) if callable(next_set) and waiting else None
                if to is None:
                    return
                try:
                    slot.switch_to(to)
                except Exception as e:  # noqa: BLE001
                    tracer.event("eval.local.switch_error", {
                        "slot": slot.slot_id, "task_set": to,
                        "error": f"{type(e).__name__}: {e}"}, lvl="error")
                    return
                continue
            one(slot, *item)

    threads = [threading.Thread(target=worker, args=(s,), name=f"amp-local-{s.slot_id}",
                                daemon=True) for s in slots]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if abort.is_set():
        return
    for u, _, task_set in todo.drain():
        req = study.request(u.ref)
        res = not_retargeted(slots[0], req, f"no slot is left to serve task set {task_set!r}")
        with lock:
            run.results.append(res)
        if on_result is not None:
            on_result(res)
