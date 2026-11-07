#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The corpus registry, keyed by benchmark; corpus names are unique across them."""

from __future__ import annotations

from . import capgym, robodojo
from .base import MOBILE_BASE_EMBODIMENTS, Benchmark, CorpusError, Unit, UnknownCorpus

__all__ = ["MOBILE_BASE_EMBODIMENTS", "Benchmark", "CorpusError", "Unit", "UnknownCorpus",
           "corpus_meta", "corpus_tasks", "expand", "for_corpus", "for_task_set",
           "gateway_load_timeouts", "get", "is_capgym", "listed_names", "listing", "load_timeout_s",
           "load_timeouts", "process_per_task", "resolve", "row_wait_s", "row_waits", "task_set_of"]

_MODULES = {m.BENCHMARK.name: m for m in (capgym, robodojo)}


def get(name: str) -> Benchmark:
    """The benchmark called `name`; KeyError naming the known ones."""
    m = _MODULES.get(name)
    if m is None:
        raise KeyError(f"unknown benchmark {name!r}: one of {', '.join(_MODULES)}")
    return m.BENCHMARK


def _known(benchmark: str) -> list[str]:
    """Every corpus name the benchmark recognises, runnable here or not."""
    m = _MODULES[benchmark]
    return m.known_corpora() if hasattr(m, "known_corpora") else m.corpus_names()


def listed_names(benchmark: str) -> list[str]:
    """The corpora of `benchmark` that listings (help, errors, docs) show; fixed, no manifest."""
    get(benchmark)
    m = _MODULES[benchmark]
    return m.listed_corpora() if hasattr(m, "listed_corpora") else _known(benchmark)


def _labels(benchmark: str | None = None) -> dict[str, str]:
    """Listed corpus -> its listing label, of one benchmark or all."""
    mods = [_MODULES[get(benchmark).name]] if benchmark else list(_MODULES.values())
    return {n: s for m in mods for n, s in getattr(m, "listed_labels", dict)().items()}


def listing(benchmark: str) -> str:
    """listed_names as one help string, each corpus followed by its label in parentheses."""
    labels = _labels(benchmark)
    return ", ".join(f"{n} ({labels[n]})" if labels.get(n) else n for n in listed_names(benchmark))


def _module(corpus: str):
    for name, m in _MODULES.items():
        if corpus in _known(name):
            return m
    raise UnknownCorpus(corpus, sorted(n for b in _MODULES for n in listed_names(b)),
                        labels=_labels())


def for_corpus(corpus: str) -> Benchmark:
    """The benchmark a corpus belongs to; CorpusError if no benchmark has it."""
    return _module(corpus).BENCHMARK


def for_task_set(task_set: str) -> Benchmark:
    """The benchmark whose backend registers `task_set`; KeyError naming the known ones."""
    for m in _MODULES.values():
        if task_set in m.BENCHMARK.task_sets:
            return m.BENCHMARK
    known = ", ".join(t for m in _MODULES.values() for t in m.BENCHMARK.task_sets)
    raise KeyError(f"unknown task set {task_set!r}: one of {known}")


def is_capgym(*, corpus: str | None = None, task_set: str | None = None) -> bool:
    """Whether a corpus or a task set is CaP-Gym's; one that no benchmark knows is not."""
    try:
        return (for_corpus(corpus) if corpus is not None else for_task_set(task_set)).name == "capgym"
    except (CorpusError, KeyError):
        return False


def load_timeouts() -> dict[str, float]:
    """Each task set's floor on the load timeout, for those that have one."""
    return {ts: float(s) for m in _MODULES.values()
            for ts, s in getattr(m, "LOAD_TIMEOUT_S", {}).items()}


GATEWAY_LOAD_MARGIN_S = 60.0


def gateway_load_timeouts() -> dict[str, float]:
    """load_timeouts() for the gateway: the runner's deadline fires first, normally."""
    return {ts: s + GATEWAY_LOAD_MARGIN_S for ts, s in load_timeouts().items()}


def row_waits() -> dict[str, float]:
    """Each task set's floor on the runner's wait for a ledger row, for those that have one."""
    return {ts: float(s) for m in _MODULES.values()
            for ts, s in getattr(m, "ROW_WAIT_S", {}).items()}


def row_wait_s(task_set: str | None, default: float) -> float:
    """How long a runner waits for an episode's ledger row: the job's, or its task set's floor."""
    return max(float(default), row_waits().get(task_set or "", 0.0))


def load_timeout_s(task_set: str | None, default: float) -> float:
    """The load timeout of an episode in task_set: the job's, or its task set's floor if longer."""
    return max(float(default), load_timeouts().get(task_set or "", 0.0))


def process_per_task(task_set: str | None) -> bool:
    """Whether task_set's backend needs a process per task (a task switch restarts it)."""
    return any(task_set in getattr(m, "PROCESS_PER_TASK", ()) for m in _MODULES.values())


def task_set_of(benchmark: str, task: str) -> str:
    """The task set of `benchmark` that registers `task`."""
    return _MODULES[get(benchmark).name].task_set_of(task)


def corpus_tasks(corpus: str) -> list[str]:
    """The distinct task names of a corpus, in its catalog's order."""
    return _module(corpus).corpus_tasks(corpus)


def _layouts(corpus: str, layouts) -> dict:
    """The layouts keyword for a module that takes it; CorpusError for any other corpus."""
    if layouts is None:
        return {}
    if not getattr(_module(corpus), "TAKES_LAYOUTS", False):
        raise CorpusError(f"--layouts picks CaP-Gym and RoboDojo episodes; "
                          f"{corpus} has none")
    return {"layouts": tuple(layouts)}


def expand(corpus: str, episodes: int | None = None,
           tasks: tuple[str, ...] | None = None, layouts: tuple[int, ...] | None = None) -> list[Unit]:
    """A corpus's units (a task subset and depth, or chosen layouts, optional), in run order."""
    return _module(corpus).expand(corpus, episodes, None if tasks is None else tuple(tasks),
                                  **_layouts(corpus, layouts))


def corpus_meta(corpus: str, episodes: int | None = None,
                tasks: tuple[str, ...] | None = None, layouts: tuple[int, ...] | None = None) -> dict:
    """{"name","task_set","tasks","episodes_per_task","all_tasks", ...}: the corpus, as run."""
    return _module(corpus).corpus_meta(corpus, episodes, None if tasks is None else tuple(tasks),
                                       **_layouts(corpus, layouts))


def resolve(eval_name: str, corpus: str | None) -> tuple[Benchmark, str]:
    """(benchmark, corpus) from the benchmark and --corpus: its default corpus, or one it owns."""
    try:
        bench = get(eval_name)
    except KeyError as e:
        raise CorpusError(e.args[0]) from None
    if corpus is None:
        if bench.default_corpus is None:
            raise CorpusError(f"{bench.name} has no default corpus: pass --corpus")
        return bench, bench.default_corpus
    owner = for_corpus(corpus)
    if owner.name != bench.name:
        raise CorpusError(f"corpus {corpus!r} is a {owner.name} corpus, not {bench.name}: "
                          f"pass one of {listing(bench.name)}")
    return bench, corpus
