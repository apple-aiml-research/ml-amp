#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CaP-Gym's trial limit on the infra-adjusted clock: wall time since the start less model-call retries."""

from __future__ import annotations

from collections.abc import Iterable, Mapping

UNBOUNDED_TRIAL_S = 10**9
TRIAL_CLOCK = "unbounded"


def retry_span(t: float, data: Mapping) -> tuple[float, float] | None:
    """A transient model-call failure and its backoff as a wall-time span; None for any other."""
    if data.get("transient") is not True:
        return None
    elapsed = float(data.get("elapsed_s") or 0.0)
    wait = float(data.get("retry_in_s") or 0.0)
    return (t - max(elapsed, 0.0), t + max(wait, 0.0))


def merged(spans: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for a, b in sorted(s for s in spans if s[1] > s[0]):
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def adjusted_deadline(start: float, limit_s: float, spans: Iterable[tuple[float, float]]) -> float:
    """Wall time at which start + limit_s elapses with every infra span begun before it excluded."""
    at = start + limit_s
    for a, b in merged(spans):
        a = max(a, start)
        if b <= a:
            continue
        if a >= at:
            break
        at += b - a
    return at


def infra_s(start: float, until: float, spans: Iterable[tuple[float, float]]) -> float:
    """Seconds of [start, until] inside infra spans."""
    return sum(max(0.0, min(b, until) - max(a, start)) for a, b in merged(spans))
