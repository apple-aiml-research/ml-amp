#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The types every benchmark module shares; stdlib and contracts only."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

MOBILE_BASE_EMBODIMENTS = frozenset({"capx_b1k_r1pro"})


class CorpusError(ValueError):
    """A corpus that is unknown, unavailable or cannot be expanded; str() says why."""


class UnknownCorpus(CorpusError):
    """No benchmark here has the corpus; `known` lists the ones that do exist."""

    def __init__(self, corpus: str, known: list[str], note: str = "",
                 labels: Mapping[str, str] | None = None):
        self.corpus, self.known, self.note = corpus, list(known), note
        labels = labels or {}
        self.shown = ", ".join(f"{n} ({labels[n]})" if labels.get(n) else n for n in self.known)
        super().__init__(f"unknown corpus {corpus!r}: one of {self.shown}{note}")


@dataclass(frozen=True)
class Benchmark:
    name: str
    task_sets: tuple[str, ...]
    default_corpus: str | None
    embodiments: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class Unit:
    """One (task, episode) of a corpus, in run order."""

    task: str
    episode: int
    task_set: str
    est_s: float
