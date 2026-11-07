#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The clock planning budgets run on: wall time, or a logical clock that makes searches
reproducible regardless of machine load."""

from __future__ import annotations

import contextlib
import time
from collections.abc import Callable, Iterator
from contextvars import ContextVar

Clock = Callable[[], float]
_CLOCK: ContextVar[Clock] = ContextVar("amp_planning_clock", default=time.perf_counter)


def now() -> float:
    """Seconds on the current planning clock."""
    return _CLOCK.get()()


@contextlib.contextmanager
def use(clock: Clock | None) -> Iterator[None]:
    """Run the block on clock (None: leave the current one)."""
    if clock is None:
        yield
        return
    token = _CLOCK.set(clock)
    try:
        yield
    finally:
        _CLOCK.reset(token)
