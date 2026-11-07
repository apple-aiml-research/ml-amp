#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The common diagnostic interface."""

from __future__ import annotations

import contextlib
import contextvars
import hashlib
import itertools
import json
import pickle
import re
import threading
import time
import uuid
from collections.abc import Iterator, Sequence
from dataclasses import asdict, dataclass, field, fields
from typing import Any, Protocol

TRACE_VERSION = 1
TRACE_DIR = "trace"
TRACE_ENV = "AMP_TRACE"
TRACE_DIR_ENV = "AMP_TRACE_DIR"
TRACE_HEADER = "X-AMP-Trace"
KERNEL_STREAM = "kernel_cwd/trace.jsonl"
LEVELS = ("debug", "info", "warn", "error")
SOURCES = ("eval", "agent", "kernel", "gateway", "server", "harness", "robot", "node")
REDACT_KEYS = ("token", "authorization", "api_key", "apikey", "password", "secret",
               "cookie", "x-api-key", "x-goog-api-key", "x-harness-key", "x-amp-node-key")
_BLOB_REF = re.compile(r"blobs/[0-9a-f]{64}\.[a-z0-9]+(?:\.gz)?")

KINDS = {
    "eval.episode.begin": "the episode runner claimed a unit",
    "eval.episode.end": "the unit's EpisodeResult",
    "agent.run.begin": "profile, harness, prompt id, system_sha256",
    "agent.run.end": "end reason and totals",
    "agent.turn.begin": "a model turn starts",
    "agent.turn.end": "a model turn ends",
    "agent.llm.request": "the request as sent: full system once (blob), then per-turn deltas",
    "agent.llm.response": "every content block as returned, thinking and signatures included",
    "agent.llm.usage": "tokens (input, output, thinking, cache), the prompt's tokens against the "
                       "model's context window, latency, cost",
    "agent.llm.retry": "a retried call and why",
    "agent.llm.error": "a call that failed for good",
    "agent.tool.call": "a tool call's input",
    "agent.tool.result": "a tool call's output, images as blobs",
    "agent.context.trim": "what context trimming elided",
    "agent.context.images": "the oldest images replaced by pointers, under the request image caps",
    "agent.context.compact": "the history compacted (summarized) to fit; no harness compacts yet",
    "kernel.start.begin": "the kernel process is starting: backend, python",
    "kernel.start.end": "startup finished",
    "kernel.ready": "startup time, or the startup error",
    "kernel.exec.begin": "code sent to the sandboxed kernel",
    "kernel.exec.result": "its output, images, error and duration",
    "kernel.exec.end": "the exec span closed",
    "kernel.restart": "the kernel was restarted and why",
    "kernel.close": "the kernel stopped; relay summary",
    "sdk.call": "one SDK request as the relay saw it",
    "gateway.request": "a proxied request: policy key, allowed, filtered fields, status, latency",
    "gateway.claim": "a directed claim",
    "gateway.release": "an episode released or discarded",
    "server.request": "a robot-server request and its handler time",
    "server.plan": "a planner call: inputs, result, failure reason",
    "server.execute": "a trajectory executed: tracking errors, end state",
    "server.episode": "the episode state machine",
    "server.log": "a server log line (warnings and errors that are not events of their own)",
    "server.perception.pass": "a perception pass: objects found, timings, held check",
    "server.perception.agent_object": "an object the agent defined or segmented",
    "server.perception.forget": "an object the agent forgot",
}


def stream_name(src: str) -> str:
    return src.replace("/", "__") + ".jsonl"


def valid_src(src: str) -> bool:
    head, _, name = src.partition("/")
    return head in SOURCES and (bool(name) == (head in ("harness", "robot")))


@dataclass(frozen=True)
class BlobRef:
    ref: str
    mime: str
    size: int
    role: str = ""

    def __post_init__(self):
        if self.ref and not _BLOB_REF.fullmatch(self.ref):
            raise ValueError(f"blob ref {self.ref!r} is not blobs/<sha256>.<ext>")


@dataclass(frozen=True)
class Event:
    v: int
    seq: int
    t: float
    mono: float
    src: str
    kind: str
    data: dict[str, Any] = field(default_factory=dict)
    span: str | None = None
    parent: str | None = None
    lvl: str = "info"
    blobs: tuple[BlobRef, ...] = ()
    trusted: bool = True
    proc: str = ""

    def __post_init__(self):
        if not isinstance(self.v, int) or not 1 <= self.v <= TRACE_VERSION:
            raise ValueError(f"trace version {self.v!r} not in 1..{TRACE_VERSION}")
        if not valid_src(self.src):
            raise ValueError(f"trace src {self.src!r}")
        if self.lvl not in LEVELS:
            raise ValueError(f"trace level {self.lvl!r}")

    def to_json(self) -> str:
        d = asdict(self)
        d["blobs"] = [asdict(b) for b in self.blobs]
        return json.dumps(d, separators=(",", ":"))

    @classmethod
    def from_json(cls, line: str, *, trusted: bool | None = None) -> Event:
        """Parse one stream line; unknown fields are ignored and `trusted` overrides the line's."""
        d = json.loads(line)
        names = {f.name for f in fields(cls)}
        d = {k: v for k, v in d.items() if k in names}
        d["blobs"] = tuple(BlobRef(**b) for b in d.get("blobs", ()))
        if trusted is not None:
            d["trusted"] = trusted
        return cls(**d)


class Tracer(Protocol):
    src: str

    def event(self, kind: str, data: dict | None = None, *, lvl: str = "info",
              blobs: Sequence[BlobRef] = (), span: str | None = None,
              verbatim: bool = False) -> None:
        """verbatim: only registered exact secrets are redacted (LLM request/response content)."""
        ...

    def span(self, kind: str, data: dict | None = None) -> contextlib.AbstractContextManager[str]:
        """Emits <kind>.begin, yields the span id, emits <kind>.end with dur_s (and error)."""
        ...

    def blob(self, payload: Any, *, mime: str | None = None, role: str = "",
             compress: bool = False, verbatim: bool = False) -> BlobRef:
        """bytes, str, JSON, a numpy array or a PIL image -> a content-addressed blob."""
        ...

    def child(self, src: str) -> Tracer:
        """Same trace, another source; its events parent under the span open at the call."""
        ...

    def context(self) -> str:
        """The TRACE_ENV / TRACE_HEADER value for work started under the current span."""
        ...

    def close(self) -> None: ...


class NullTracer:
    """Discards everything; the default when no trace is open."""

    def __init__(self, src: str = "eval"):
        self.src = src

    def event(self, kind, data=None, *, lvl="info", blobs=(), span=None, verbatim=False) -> None:
        return None

    @contextlib.contextmanager
    def span(self, kind, data=None) -> Iterator[str]:
        yield ""

    def blob(self, payload, *, mime=None, role="", compress=False, verbatim=False) -> BlobRef:
        return BlobRef(ref="", mime=mime or "application/octet-stream", size=0, role=role)

    def child(self, src: str) -> NullTracer:
        return NullTracer(src)

    def context(self) -> str:
        return ""

    def close(self) -> None:
        return None


def _blob_bytes(payload: Any) -> bytes:
    if isinstance(payload, bytes):
        return payload
    if isinstance(payload, str):
        return payload.encode()
    if hasattr(payload, "tobytes") and hasattr(payload, "dtype"):
        return f"{payload.dtype}|{tuple(payload.shape)}|".encode() + payload.tobytes()
    try:
        return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    except TypeError:
        return pickle.dumps(payload)  # nosemgrep: python.lang.security.deserialization.pickle.avoid-pickle


class MemoryTracer:
    """Keeps events and blobs in memory: the test double for any layer that takes a Tracer."""

    def __init__(self, src: str = "eval", *, episode_id: str = "test", _shared: dict | None = None,
                 _base: str | None = None):
        self.src = src
        self.episode_id = episode_id
        self._base = _base
        self._shared = _shared if _shared is not None else {
            "events": [], "blobs": {}, "seq": {}, "lock": threading.Lock(),
            "stack": contextvars.ContextVar(f"amp_memory_tracer_{id(self)}", default=())}

    @property
    def events(self) -> list[Event]:
        return self._shared["events"]

    @property
    def blobs(self) -> dict[str, bytes]:
        return self._shared["blobs"]

    def kinds(self) -> list[str]:
        return [e.kind for e in self.events]

    def _current(self) -> str | None:
        stack = self._shared["stack"].get()
        return stack[-1] if stack else self._base

    def _emit(self, kind, data, lvl, blobs, span, parent) -> None:
        with self._shared["lock"]:
            seq = self._shared["seq"].setdefault(self.src, itertools.count())
            self.events.append(Event(v=TRACE_VERSION, seq=next(seq), t=time.time(),
                                     mono=time.monotonic(), src=self.src, kind=kind,
                                     data=dict(data or {}), span=span, parent=parent, lvl=lvl,
                                     blobs=tuple(blobs)))

    def event(self, kind, data=None, *, lvl="info", blobs=(), span=None, verbatim=False) -> None:
        self._emit(kind, data, lvl, blobs, span or self._current(), None)

    @contextlib.contextmanager
    def span(self, kind, data=None) -> Iterator[str]:
        sid = uuid.uuid4().hex[:16]
        var = self._shared["stack"]
        parent = self._current()
        self._emit(f"{kind}.begin", data, "info", (), sid, parent)
        token = var.set(var.get() + (sid,))
        t0 = time.monotonic()
        try:
            yield sid
        except BaseException as exc:
            var.reset(token)
            self._emit(f"{kind}.end", {"dur_s": time.monotonic() - t0, "error": repr(exc)},
                       "error", (), sid, parent)
            raise
        var.reset(token)
        self._emit(f"{kind}.end", {"dur_s": time.monotonic() - t0}, "info", (), sid, parent)

    def blob(self, payload, *, mime=None, role="", compress=False, verbatim=False) -> BlobRef:
        data = _blob_bytes(payload)
        sha = hashlib.sha256(data).hexdigest()
        self.blobs[sha] = data
        return BlobRef(ref=f"blobs/{sha}.bin", mime=mime or "application/octet-stream",
                       size=len(data), role=role)

    def child(self, src: str) -> MemoryTracer:
        return MemoryTracer(src, episode_id=self.episode_id, _shared=self._shared,
                            _base=self._current())

    def context(self) -> str:
        return f"{self.episode_id}/{self._current() or ''}"

    def close(self) -> None:
        return None


_CTX_ID = re.compile(r"[A-Za-z0-9_.-]{1,128}")


def _safe_id(v: str) -> bool:
    return bool(_CTX_ID.fullmatch(v)) and v not in (".", "..")


def parse_context(value: str) -> tuple[str, str | None]:
    """'<episode_id>/<span_id>' -> (episode_id, span_id); an unsafe id becomes "" or None."""
    episode_id, _, span = (value or "").partition("/")
    if not _safe_id(episode_id):
        return "", None
    return episode_id, (span if span and _safe_id(span) else None)
