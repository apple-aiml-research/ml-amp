#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The one place the gateway makes per-episode tracers, and the only way it writes to them."""

from __future__ import annotations

from pathlib import Path

from amp.contracts.trace import REDACT_KEYS, TRACE_DIR, Tracer
from amp.trace.writer import FileTracer

REDACTED = "[redacted]"
_REDACT = frozenset(k.lower() for k in REDACT_KEYS)


def make_tracer(state_dir: str | Path, episode_id: str, src: str = "gateway") -> Tracer:
    """The episode's gateway stream, <state_dir>/episodes/<episode_id>/trace/, for ingest."""
    return FileTracer(Path(state_dir) / "episodes" / episode_id / TRACE_DIR, src,
                      episode_id=episode_id)


def redact(value):
    """A copy of a JSON value with every REDACT_KEYS key's value replaced, at any depth."""
    if isinstance(value, dict):
        return {k: REDACTED if isinstance(k, str) and k.lower() in _REDACT else redact(v)
                for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [redact(v) for v in value]
    return value


def emit(tracer: Tracer, kind: str, data: dict, lvl: str = "info") -> None:
    tracer.event(kind, redact(data), lvl=lvl)


def close_tracer(tracer: Tracer) -> None:
    tracer.close()
