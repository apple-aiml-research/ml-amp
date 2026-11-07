#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The trace core: the file-backed Tracer, blobs, redaction, propagation, finalize and reading."""

from amp.trace.episode import EpisodeTrace, FinalizeContext, Ingested, kernel_finalizer
from amp.trace.propagate import (
    checked_episode_id,
    env_for_child,
    from_env,
    header_value,
    service_trace_dir,
    tracer_from_header,
)
from amp.trace.reader import Trace, load
from amp.trace.redact import Redactor, register_secret
from amp.trace.writer import DEFAULT_MAX_BYTES, FileTracer, trace_health

__all__ = [
    "DEFAULT_MAX_BYTES",
    "EpisodeTrace",
    "FileTracer",
    "FinalizeContext",
    "Ingested",
    "Redactor",
    "Trace",
    "checked_episode_id",
    "env_for_child",
    "from_env",
    "header_value",
    "kernel_finalizer",
    "load",
    "register_secret",
    "service_trace_dir",
    "trace_health",
    "tracer_from_header",
]
