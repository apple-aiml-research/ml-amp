#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Trace context across processes (AMP_TRACE + AMP_TRACE_DIR) and HTTP (X-AMP-Trace)."""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path

from amp.contracts.trace import (
    TRACE_DIR,
    TRACE_DIR_ENV,
    TRACE_ENV,
    NullTracer,
    Tracer,
    parse_context,
    valid_src,
)
from amp.trace.writer import FileTracer

UNKNOWN_EPISODE = "unknown"


def checked_episode_id(value: str | None) -> str | None:
    """The episode id of a TRACE_ENV / TRACE_HEADER value, or None unless it is safe in a path."""
    return parse_context(value or "")[0] or None


def service_trace_dir(state_dir, value: str | None) -> Path | None:
    """<state_dir>/episodes/<episode_id>/trace for a request's header value, or None."""
    episode_id = checked_episode_id(value)
    return Path(state_dir) / "episodes" / episode_id / TRACE_DIR if episode_id else None


def from_env(src: str = "eval", *, environ: Mapping[str, str] | None = None,
             **kw) -> FileTracer | NullTracer:
    """A FileTracer when AMP_TRACE_DIR is set (context from AMP_TRACE), else a NullTracer."""
    env = os.environ if environ is None else environ
    trace_dir = env.get(TRACE_DIR_ENV, "").strip()
    if not trace_dir or not valid_src(src):
        return NullTracer(src)
    episode_id, span = parse_context(env.get(TRACE_ENV, ""))
    return FileTracer(trace_dir, src, episode_id=episode_id or UNKNOWN_EPISODE, base_span=span,
                      **kw)


def env_for_child(tracer: Tracer) -> dict[str, str]:
    """The env vars a child process needs to continue this trace under the current span."""
    ctx = tracer.context()
    if not ctx:
        return {}
    env = {TRACE_ENV: ctx}
    trace_dir = getattr(tracer, "trace_dir", None)
    if trace_dir is not None:
        env[TRACE_DIR_ENV] = str(trace_dir)
    return env


def header_value(tracer: Tracer) -> str:
    """The X-AMP-Trace value for a request made under the current span ("" means omit it)."""
    return tracer.context()


def tracer_from_header(trace_dir, value: str | None, src: str, **kw) -> FileTracer | NullTracer:
    """A tracer for one request: trace_dir, the caller's episode and span. Close it after."""
    episode_id, span = parse_context(value or "")
    if not episode_id or not valid_src(src):
        return NullTracer(src)
    return FileTracer(trace_dir, src, episode_id=episode_id, base_span=span, **kw)
