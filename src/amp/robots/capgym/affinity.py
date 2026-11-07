#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A BEHAVIOR backend pins itself to its slot's CPUs first; stdlib only, Python 3.10."""

from __future__ import annotations

import os
import sys

AFFINITY_ENV = "AMP_CPU_AFFINITY"

RESULT: dict[str, str | None] = {"requested": None, "pinned": None, "error": None}


def cpulist(text: str) -> list[int]:
    """A kernel CPU list ("1-3,7") as sorted CPU numbers; ValueError on a malformed one."""
    out: set[int] = set()
    for part in text.strip().split(","):
        part = part.strip()
        if not part:
            continue
        lo, sep, hi = part.partition("-")
        a, b = int(lo), int(hi) if sep else int(lo)
        if a < 0 or b < a:
            raise ValueError(f"bad CPU range {part!r}")
        out.update(range(a, b + 1))
    return sorted(out)


def cpustr(cpus) -> str:
    """CPU numbers as a kernel CPU list, ranges collapsed."""
    cpus, out, i = sorted(set(cpus)), [], 0
    while i < len(cpus):
        j = i
        while j + 1 < len(cpus) and cpus[j + 1] == cpus[j] + 1:
            j += 1
        out.append(f"{cpus[i]}-{cpus[j]}" if j > i else str(cpus[i]))
        i = j + 1
    return ",".join(out)


def apply(environ=None) -> list[int] | None:
    """Pin this process to the env's CPU list; the CPUs pinned, or None (unset, or it could not)."""
    raw = (os.environ if environ is None else environ).get(AFFINITY_ENV, "")
    RESULT.update(requested=raw.strip() or None, pinned=None, error=None)
    if not raw.strip():
        return None
    try:
        cpus = cpulist(raw)
        os.sched_setaffinity(0, cpus)
        got = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError, ValueError) as e:
        RESULT["error"] = f"{type(e).__name__}: {e}"
        print(f"[amp.robots.capgym] cpu affinity {raw!r} not applied: {RESULT['error']}",
              file=sys.stderr, flush=True)
        return None
    RESULT["pinned"] = cpustr(got)
    print(f"[amp.robots.capgym] cpu affinity: {RESULT['pinned']}", file=sys.stderr, flush=True)
    return got


def record() -> dict | None:
    """What apply() did, if the slot runner asked this process to pin; None if it did not."""
    return dict(RESULT) if RESULT["requested"] else None
