#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The paper's scores for Figure 4F-K, computed as its own scripts compute them (the leaderboard
formula of tools/paper_experiments/analysis/recompute_all.py and cl_formula.py): success rate,
call ratio, their bootstrap CIs and the block x checkpoint matrix."""
from __future__ import annotations

import math
import random
import statistics
from itertools import pairwise

CATS = ("generalization", "precision", "long-horizon", "memory", "open")
RANDOM = "_random"
RESAMPLES = 2000
BLOCK_ENDS = (10, 20, 30)
LABELS = ("not-seen", "just-seen", "seen-earlier")


def gmean(xs: list[float]) -> float:
    return math.exp(statistics.fmean(math.log(x) for x in xs))


def formula(per: dict, pick: dict, agg=statistics.fmean) -> float | None:
    """The leaderboard score: agg over each category's tasks, then over the five categories, with
    generalization the agg of its standard and randomized settings' scores; None if no setting is
    left."""
    def part(settings):
        v = [per[c] for c in settings if c in per]
        return [agg(v)] if v else []

    gen = pick.get("generalization", ())
    g = part(gen) + part([t + RANDOM for t in gen])
    parts = ([agg(g)] if g else []) + [x for c in CATS[1:] for x in part(pick.get(c, ()))]
    return agg(parts) if parts else None


def tasks_by_cat(dim_of: dict) -> dict:
    """{category: sorted tasks} of {task: category}; the order is the bootstrap's stream order."""
    return {c: sorted(t for t, d in dim_of.items() if d == c) for c in CATS}


def ci(per: dict, by_cat: dict, rng: random.Random, agg=statistics.fmean,
       n: int = RESAMPLES) -> tuple[float, float] | None:
    """The 2.5th and 97.5th percentile (sims[50] and sims[1949] of 2,000) of n bootstraps of the
    tasks within each category, None if a resample leaves no setting."""
    sims = [formula(per, {c: rng.choices(ts, k=len(ts)) for c, ts in by_cat.items()}, agg)
            for _ in range(n)]
    if None in sims:
        return None
    sims.sort()
    return sims[int(0.025 * n)], sims[int(0.975 * n) - 1]


def with_ci(per: dict, by_cat: dict, rng: random.Random, agg=statistics.fmean,
            n: int = RESAMPLES) -> tuple[float, float, float]:
    """(formula, 2.5th, 97.5th percentile), the CI drawn from rng as in ci."""
    bounds = ci(per, by_cat, rng, agg, n)
    if bounds is None:
        raise statistics.StatisticsError("no setting to score")
    return formula(per, by_cat, agg), *bounds


def success_rates(succ: dict, intended: dict) -> dict:
    """{setting: % of its intended episodes that succeeded}; succ {(setting, episode): bool} holds
    the scored episodes only, so an unscored one counts as a failure."""
    return {c: 100 * sum(bool(succ.get((c, e))) for e in es) / len(es)
            for c, es in intended.items()}


def paired(run: dict, base: dict) -> tuple[dict, dict]:
    """({setting: calls with memory}, {setting: calls without}) of the episodes successful in
    both, aligned; run and base are {(setting, episode): (success, calls)}."""
    num: dict[str, list[float]] = {}
    den: dict[str, list[float]] = {}
    for u, (ok, calls) in run.items():
        b = base.get(u)
        if b and ok and b[0] and calls and b[1]:
            num.setdefault(u[0], []).append(calls)
            den.setdefault(u[0], []).append(b[1])
    return num, den


def ratios(num: dict, den: dict) -> dict:
    """{setting: mean of its num / mean of its den}."""
    return {c: statistics.fmean(num[c]) / statistics.fmean(den[c]) for c in num}


def call_ratios(run: dict, base: dict) -> dict:
    """{setting: mean calls with memory / mean calls without} over the episodes successful in
    both; a setting with none is left out."""
    return ratios(*paired(run, base))


def blocks_of(order: list[str]) -> list[list[str]]:
    """K's column blocks of a pass's task order."""
    cuts = [0, *BLOCK_ENDS, len(order)]
    return [order[a:b] for a, b in pairwise(cuts)]


def checkpoint_block(step: int) -> int:
    """The block a checkpoint of `step` places has just finished or entered."""
    return sum(e < step for e in BLOCK_ENDS)


def label(block: int, step: int) -> str:
    """not-seen, just-seen or seen-earlier: the block against the checkpoint's own."""
    return LABELS[(block <= checkpoint_block(step)) + (block < checkpoint_block(step))]


def task_rates(succ: dict, intended: dict, task_of: dict) -> dict:
    """{task: % of its intended episodes (over all its configs) that succeeded}."""
    n: dict[str, int] = {}
    k: dict[str, int] = {}
    for c, es in intended.items():
        t = task_of[c]
        n[t] = n.get(t, 0) + len(es)
        k[t] = k.get(t, 0) + sum(bool(succ.get((c, e))) for e in es)
    return {t: 100 * k[t] / n[t] for t in n}


def matrix_row(succ: dict, intended: dict, task_of: dict, order: list[str]) -> list[float | None]:
    """One matrix row: each block's task mean of task_rates, None for a block with no task."""
    rates = task_rates(succ, intended, task_of)
    return [round(statistics.fmean(rates[t] for t in b), 2) if b else None
            for b in blocks_of(order)]
