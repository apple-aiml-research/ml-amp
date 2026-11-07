#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A study (harness configurations x a corpus under one EvalProtocol) and its expansion."""

from __future__ import annotations

import dataclasses
import functools
from dataclasses import dataclass, field
from typing import NamedTuple

from amp.contracts.eval import Arm, EpisodeRequest, EvalProtocol


@dataclass(frozen=True)
class UnitRef:
    """One (job, harness configuration, task, episode) unit of a study."""

    job_id: str
    arm_id: str
    task: str
    episode: int


class Unit(NamedTuple):
    """One (task, episode) of a study, in the shape of amp.benchmarks.Unit."""

    task: str
    episode: int
    task_set: str
    est_s: float


def _task_set_of(benchmark: str | None, task: str) -> str:
    if not benchmark:
        return "benchmark"
    from amp import benchmarks

    return benchmarks.task_set_of(benchmark, task)


class StudyError(ValueError):
    """A study that cannot be expanded."""


@dataclass(frozen=True)
class Study:
    """Harness configurations x a corpus (or, with corpus=None, exactly tasks x range(episodes)) on a benchmark."""

    arms: tuple[Arm, ...]
    corpus: str | None = None
    tasks: tuple[str, ...] | None = None
    episodes: int | None = None
    protocol: EvalProtocol = field(default_factory=EvalProtocol)
    job_id: str = "local"
    code_ref: str = "local"
    note: str = ""
    benchmark: str | None = None
    layouts: tuple[int, ...] | None = None
    est_s: tuple[tuple[str, float], ...] | None = None

    def __post_init__(self):
        object.__setattr__(self, "arms", tuple(self.arms))
        if self.tasks is not None:
            object.__setattr__(self, "tasks", tuple(self.tasks))
        if self.layouts is not None:
            object.__setattr__(self, "layouts", tuple(self.layouts))
        if self.est_s is not None:
            object.__setattr__(self, "est_s", tuple((str(t), float(v)) for t, v in self.est_s))
        if not self.arms:
            raise StudyError("a study needs at least one harness configuration")
        if len({a.id for a in self.arms}) != len(self.arms):
            raise StudyError("harness configuration ids must be unique")
        if self.corpus is None and not self.tasks:
            raise StudyError("a study without a corpus must name its tasks")
        if self.benchmark is not None:
            from amp import benchmarks

            try:
                if self.corpus is None:
                    benchmarks.get(self.benchmark)
                else:
                    benchmarks.resolve(self.benchmark, self.corpus)
            except KeyError as e:
                raise StudyError(e.args[0]) from None
            except benchmarks.CorpusError as e:
                raise StudyError(str(e)) from None

    def arm(self, arm_id: str) -> Arm:
        return next(a for a in self.arms if a.id == arm_id)

    @property
    def eval_name(self) -> str | None:
        """The benchmark this study runs; None for listed tasks with no benchmark named."""
        if self.benchmark is not None or self.corpus is None:
            return self.benchmark
        from amp import benchmarks

        try:
            return benchmarks.for_corpus(self.corpus).name
        except benchmarks.CorpusError:
            return None

    @functools.cached_property
    def corpus_meta(self) -> dict | None:
        """benchmarks.corpus_meta as this study runs it (its subset, its depth); None without."""
        if self.corpus is None:
            return None
        from amp import benchmarks

        try:
            return benchmarks.corpus_meta(self.corpus, self.episodes, self.tasks, self.layouts)
        except benchmarks.CorpusError as e:
            raise StudyError(str(e)) from e

    @functools.cached_property
    def units(self) -> tuple[Unit, ...]:
        """The corpus's units in run order (benchmarks.expand), or the listed tasks as given."""
        if self.corpus is None:
            episodes = 1 if self.episodes is None else int(self.episodes)
            if episodes < 1:
                raise StudyError(f"episodes must be >= 1, got {episodes}")
            sets = {t: _task_set_of(self.benchmark, t) for t in self.tasks}
            est = dict(self.est_s or ())
            return tuple(Unit(t, k, sets[t], float(est.get(t, 0.0)))
                         for t in self.tasks for k in range(episodes))
        from amp import benchmarks

        try:
            return tuple(benchmarks.expand(self.corpus, self.episodes, self.tasks, self.layouts))
        except benchmarks.CorpusError as e:
            raise StudyError(str(e)) from e

    @functools.cached_property
    def _task_sets(self) -> dict[str, str]:
        return {u.task: u.task_set for u in self.units}

    @property
    def task_set(self) -> str:
        """The one library the study's units register in."""
        sets = sorted(set(self._task_sets.values()))
        if len(sets) != 1:
            raise StudyError(f"the study's units span task sets {sets}")
        return sets[0]

    def request(self, ref: UnitRef) -> EpisodeRequest:
        task_set = self._task_sets.get(ref.task)
        if task_set is None:
            raise StudyError(f"{ref.task!r} is not a task of this study")
        return EpisodeRequest(job_id=self.job_id, arm=self.arm(ref.arm_id), task=ref.task,
                              episode=ref.episode, protocol=self.protocol,
                              code_ref=self.code_ref, task_set=task_set)

    def to_dict(self) -> dict:
        return {**dataclasses.asdict(self), "arms": tuple(a.to_dict() for a in self.arms),
                "protocol": self.protocol.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> Study:
        d = dict(d)
        d["arms"] = tuple(Arm.from_dict(a) for a in d["arms"])
        d["protocol"] = EvalProtocol.from_dict(d.get("protocol"))
        return cls(**d)


@dataclass(frozen=True)
class StudyUnit:
    """One unit with its place in the schedule."""

    ref: UnitRef
    ord: int
    harness: str
    endpoint_class: str
    est_s: float


def expand(study: Study) -> list[StudyUnit]:
    """Study -> harness configurations x tasks x episodes, in corpus run order or as listed, the harness configurations interleaved."""
    n = len(study.arms)
    return [StudyUnit(UnitRef(study.job_id, arm.id, u.task, u.episode), i * n + a, arm.harness,
                      arm.agent.endpoint_class, u.est_s)
            for i, u in enumerate(study.units) for a, arm in enumerate(study.arms)]
