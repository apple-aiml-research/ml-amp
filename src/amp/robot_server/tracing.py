#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Trace routing: one tracer per episode (from X-AMP-Trace), a server stream for everything else."""

from __future__ import annotations

import shutil
import threading
import time
from pathlib import Path

import numpy as np

from amp.contracts.trace import TRACE_DIR, Tracer
from amp.trace.propagate import checked_episode_id, service_trace_dir
from amp.trace.writer import FileTracer

SRC = "server"
SERVER_STREAM_ID = "server"
MAX_OPEN = 4
STATS_EVERY_S = 60.0
ROTATE_EVENTS = 100_000
SERVER_MAX_BYTES, SERVER_KEEP = 64 << 20, 1


class RotatingTracer:
    """The server stream: a FileTracer replaced by a fresh one (the old kept as trace.1) when
    it has written ROTATE_EVENTS events, so a long-lived server never hits the drop cap."""

    def __init__(self, trace_dir: Path, rotate_events: int = ROTATE_EVENTS):
        self._dir, self._limit = Path(trace_dir), int(rotate_events)
        self._lock = threading.Lock()
        self._n = 0
        self._tr = self._open()

    def _open(self) -> FileTracer:
        return FileTracer(self._dir, SRC, episode_id=SERVER_STREAM_ID,
                          max_bytes=SERVER_MAX_BYTES, keep=SERVER_KEEP)

    def _rotate(self) -> None:
        self._tr.close()
        old = self._dir.with_name(self._dir.name + ".1")
        shutil.rmtree(old, ignore_errors=True)
        self._dir.rename(old)
        self._tr, self._n = self._open(), 0

    def event(self, kind, data=None, *, lvl="info", blobs=(), span=None) -> None:
        with self._lock:
            self._n += 1
            if self._n > self._limit:
                self._rotate()
            tr = self._tr
        tr.event(kind, data, lvl=lvl, blobs=blobs, span=span)

    def span(self, kind, data=None):
        return self._tr.span(kind, data)

    def blob(self, payload, *, mime=None, role="", compress=False):
        return self._tr.blob(payload, mime=mime, role=role, compress=compress)

    def child(self, src):
        return self._tr.child(src)

    def context(self) -> str:
        return self._tr.context()

    def close(self) -> None:
        with self._lock:
            self._tr.close()


class RequestStats:
    """Per-route request counts and latencies over one window."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.routes: dict[str, list] = {}

    def add(self, name, status, dt) -> None:
        self.routes.setdefault(name, []).append((dt, status >= 400))

    def json(self) -> dict:
        out = {}
        for name, xs in self.routes.items():
            ms = np.array([x[0] for x in xs]) * 1000.0
            out[name] = {"n": len(xs), "errors": sum(x[1] for x in xs),
                         "p50_ms": round(float(np.percentile(ms, 50)), 3),
                         "p95_ms": round(float(np.percentile(ms, 95)), 3),
                         "max_ms": round(float(ms.max()), 3)}
        return {"route": "*", "window_s": round(time.monotonic() - self.t0, 3), "routes": out}


class TraceRouter:
    """Tracers by episode id; the current episode is the one the last lifecycle call named."""

    def __init__(self, state_dir: Path, *, root: Tracer | None = None):
        self._state_dir = Path(state_dir)
        self._lock = threading.Lock()
        self._root = root
        self._server: Tracer | None = None
        self._episodes: dict[str, Tracer] = {}
        self._current: str | None = None
        self._stats = RequestStats()
        self.stats_every_s = STATS_EVERY_S

    def server(self) -> Tracer:
        """The server-level stream, <state_dir>/trace/, for requests without a trace context."""
        with self._lock:
            if self._server is None:
                self._server = self._root if self._root is not None else RotatingTracer(
                    self._state_dir / TRACE_DIR)
            return self._server

    def for_header(self, value: str | None, *, make_current: bool = False) -> Tracer:
        """The episode's tracer for an X-AMP-Trace value, else the server stream."""
        eid = checked_episode_id(value)
        if eid is None:
            if make_current:
                with self._lock:
                    self._current = None
            return self.server()
        with self._lock:
            tr = self._episodes.get(eid)
            if tr is None:
                if self._root is not None:
                    tr = self._root.child(SRC)
                else:
                    tr = FileTracer(service_trace_dir(self._state_dir, value), SRC,
                                    episode_id=eid)
                self._episodes[eid] = tr
                while len(self._episodes) > MAX_OPEN:
                    old = next(k for k in self._episodes if k != eid)
                    self._close(old)
            if make_current:
                self._current = eid
            return tr

    def count(self, name, status, dt) -> None:
        """Record one polled request; emit the window's aggregate when it is due."""
        with self._lock:
            self._stats.add(name, status, dt)
            due = time.monotonic() - self._stats.t0 >= self.stats_every_s
            stats = self._stats if due else None
            if due:
                self._stats = RequestStats()
        if stats is not None:
            self.server().event("server.request", stats.json())

    def flush(self) -> None:
        with self._lock:
            stats, self._stats = self._stats, RequestStats()
        if stats.routes:
            self.server().event("server.request", stats.json())

    def current(self) -> Tracer:
        with self._lock:
            tr = self._episodes.get(self._current) if self._current else None
        return tr if tr is not None else self.server()

    def _close(self, eid) -> None:
        tr = self._episodes.pop(eid, None)
        if tr is not None and tr is not self._root:
            tr.close()
        if self._current == eid:
            self._current = None

    def close_current(self) -> None:
        """The episode ended: its tracer is closed; later events go to the server stream."""
        with self._lock:
            if self._current is not None:
                self._close(self._current)

    def close_all(self) -> None:
        self.flush()
        with self._lock:
            for eid in list(self._episodes):
                self._close(eid)
            if self._server is not None and self._server is not self._root:
                self._server.close()
            self._server = None
