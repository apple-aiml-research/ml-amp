#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""FileTracer: the on-disk Tracer. One O_APPEND JSONL stream per src, MemoryTracer's semantics."""

from __future__ import annotations

import contextlib
import contextvars
import fcntl
import gzip
import itertools
import json
import os
import socket
import threading
import time
import uuid
import weakref
from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Self

from amp.contracts.trace import LEVELS, TRACE_VERSION, BlobRef, Event, stream_name, valid_src
from amp.trace.blobs import GZIP_SUFFIX, BlobStore, ext_for, is_text_mime
from amp.trace.redact import DEFAULT, Redactor

DEFAULT_MAX_BYTES = 256 << 20
OVERFLOW_KIND = "trace.overflow"
ROTATED_KIND = "trace.rotated"
DROPPED_KIND = "trace.dropped"
ERROR_KIND = "trace.error"
ROTATING_SUFFIX = ".rotating"
ROTATE_RETRY_S = 60.0
ROTATE_CEILING = 2
LOCK_TRIES = 100
_TOTAL_KEYS: tuple[str, ...] = ("rotations", "rotate_errors", "overflowed", "dropped", "errors")
_STACKS: contextvars.ContextVar[Mapping[int, tuple[str, ...]]] = contextvars.ContextVar(
    "amp_trace_stacks", default=MappingProxyType({}))
_KEYS = itertools.count(1)
_TAIL_CHUNK = 1 << 16
_OPEN = os.O_RDWR | os.O_APPEND | os.O_CREAT | os.O_CLOEXEC
_wall = time.time


_PROC: tuple[int, str] = (0, "")
_TOTALS_LOCK = threading.Lock()
_TOTALS: dict[str, int] = dict.fromkeys(_TOTAL_KEYS, 0)


_STREAMS: weakref.WeakSet[_Stream] = weakref.WeakSet()
_SHARED: weakref.WeakValueDictionary[int, _Shared] = weakref.WeakValueDictionary()


def _after_fork_in_child() -> None:
    """Fresh totals, locks and drop counts: the parent still holds and reports its own."""
    global _TOTALS_LOCK, _TOTALS
    _TOTALS_LOCK, _TOTALS = threading.Lock(), dict.fromkeys(_TOTAL_KEYS, 0)
    for st in list(_STREAMS):
        st.lock = threading.Lock()
        st.dropped = st.reported = 0
    for sh in list(_SHARED.values()):
        sh.lock = threading.Lock()


os.register_at_fork(after_in_child=_after_fork_in_child)


def _count(key: str, n: int = 1) -> None:
    with _TOTALS_LOCK:
        _TOTALS[key] += n


def trace_health() -> dict[str, int]:
    """This process's FileTracer totals: rotations, rotate_errors, overflowed, dropped, errors."""
    with _TOTALS_LOCK:
        return dict(_TOTALS)


def rotated_path(path: Path, i: int) -> Path:
    """<name>.jsonl.<i>: the i-th newest rotated file of a stream."""
    return path.with_name(f"{path.name}.{i}")


def proc_id() -> str:
    """This process as "<host>:<pid>:<start_ns>", recomputed in a forked child."""
    global _PROC
    if _PROC[0] != os.getpid():
        _PROC = (os.getpid(), f"{socket.gethostname()}:{os.getpid()}:{time.time_ns()}")
    return _PROC[1]


def new_span_id() -> str:
    return uuid.uuid4().hex[:16]


def _seq_of(line: bytes) -> int | None:
    try:
        seq = json.loads(line).get("seq")
    except (ValueError, AttributeError):
        return None
    return seq if isinstance(seq, int) and not isinstance(seq, bool) else None


def last_seq(fd: int, size: int) -> int:
    """The seq of the last parseable line of an open stream, or -1; reads backwards from the end."""
    pos, head = size, b""
    while pos > 0:
        n = min(_TAIL_CHUNK, pos)
        pos -= n
        lines = (os.pread(fd, n, pos) + head).split(b"\n")
        head = lines[0] if pos > 0 else b""
        for ln in reversed(lines[1:] if pos > 0 else lines):
            seq = _seq_of(ln) if ln.strip() else None
            if seq is not None:
                return seq
    return -1


class _Stream:
    """One JSONL file; a thread lock, flock and resync keep seq unique across processes and restarts."""

    def __init__(self, path: Path, src: str, max_bytes: int, keep: int = 0):
        self.path, self.src, self.max_bytes, self.keep = path, src, max_bytes, max(0, int(keep))
        self.lock = threading.Lock()
        self.fd: int | None = None
        self.ino: tuple[int, int] | None = None
        self.pid = 0
        self.seq = 0
        self.size = -1
        self.needs_newline = False
        self.overflowed = False
        self.dropped = 0
        self.reported = 0
        self.rotations = 0
        self.rotate_errors = 0
        self.retry_at = 0.0
        self.errors = 0
        _STREAMS.add(self)

    @property
    def limit(self) -> int:
        """The size past which non-error events drop: the cap, or a rotating file's ceiling."""
        return self.max_bytes * (ROTATE_CEILING if self.keep else 1)

    def _ensure_open(self) -> int:
        if self.fd is None or self.pid != os.getpid():
            fd = os.open(self.path, _OPEN, 0o644)
            st = os.fstat(fd)
            self.fd, self.pid, self.size, self.ino = fd, os.getpid(), -1, (st.st_dev, st.st_ino)
        return self.fd

    def _lock_current(self) -> int:
        """The fd of the file now at path, flocked; OSError if the path will not hold still."""
        fd = self._ensure_open()
        fcntl.flock(fd, fcntl.LOCK_EX)
        try:
            for _ in range(LOCK_TRIES):
                if _ino(self.path) == self.ino:
                    return fd
                with contextlib.suppress(OSError):
                    self._append_dropped(fd)
                fcntl.flock(fd, fcntl.LOCK_UN)
                os.close(fd)
                self.fd, self.overflowed, fd = None, False, -1
                fd = self._ensure_open()
                fcntl.flock(fd, fcntl.LOCK_EX)
        except BaseException:
            if fd >= 0:
                with contextlib.suppress(OSError):
                    fcntl.flock(fd, fcntl.LOCK_UN)
            raise
        fcntl.flock(fd, fcntl.LOCK_UN)
        raise OSError(f"{self.path} was replaced {LOCK_TRIES} times while being locked")

    def _resync(self, fd: int, size: int) -> None:
        self.seq = last_seq(fd, size) + 1
        self.needs_newline = size > 0 and os.pread(fd, 1, size - 1) != b"\n"
        if not self.keep and size >= self.max_bytes and not self.overflowed:
            self.overflowed = True
            _count("overflowed")
        self.size = size

    def _append(self, fd: int, data: bytes) -> None:
        if self.needs_newline:
            data, self.needs_newline = b"\n" + data, False
        _write_all(fd, data)
        self.size += len(data)
        self.seq += 1

    def _after(self, data: bytes) -> int:
        """The file's size once data is appended, with the newline a torn last line needs."""
        return self.size + len(data) + self.needs_newline

    def _drop(self) -> None:
        self.dropped += 1
        _count("dropped")

    def write(self, build: Callable[[int, float, float], str], lvl: str) -> bool:
        """Append build(seq, t, mono) as one line; False when dropped past the stream's limit."""
        with self.lock:
            fd = self._lock_current()
            try:
                size = os.fstat(fd).st_size
                if size != self.size:
                    self._resync(fd, size)
                if self.keep and self.size >= self.max_bytes:
                    fd = self._rotate(fd)
                if self.overflowed and lvl != "error":
                    self._drop()
                    return False
                seq = self.seq
                data = (build(seq, _wall(), time.monotonic()) + "\n").encode()
                if self.keep and self.size > 0 and self._after(data) > self.max_bytes:
                    fd = self._rotate(fd)
                if (not self.overflowed and self._after(data) > self.limit
                        and (not self.keep or self.size >= self.max_bytes)):
                    self.overflowed = True
                    _count("overflowed")
                    self._append(fd, self._line(OVERFLOW_KIND, "warn", {
                        "max_bytes": self.max_bytes, "limit": self.limit, "size": self.size,
                        "note": "later non-error events are dropped"}))
                    if lvl != "error":
                        self._drop()
                        return False
                if self.seq != seq:
                    data = (build(self.seq, _wall(), time.monotonic()) + "\n").encode()
                self._append(fd, data)
                return True
            finally:
                fcntl.flock(fd, fcntl.LOCK_UN)

    def _rotate(self, fd: int) -> int:
        """Rotate to <name>.1 and go on in a fresh file; on failure a trace.error and the old fd."""
        if time.monotonic() < self.retry_at:
            return fd
        first = rotated_path(self.path, 1)
        tmp = self.path.with_name(self.path.name + ROTATING_SUFFIX)
        new = -1
        try:
            self._append_dropped(fd)
            new = os.open(tmp, _OPEN | os.O_TRUNC, 0o644)
            fcntl.flock(new, fcntl.LOCK_EX)
            line = self._line(ROTATED_KIND, "info", {"from": first.name, "size": self.size,
                                                     "keep": self.keep,
                                                     "max_bytes": self.max_bytes})
            _write_all(new, line)
            if _ino(first) != self.ino:
                self._shift()
                try:
                    os.link(self.path, first)
                except OSError:
                    os.rename(self.path, first)
            os.replace(tmp, self.path)
        except OSError as e:
            if new >= 0:
                os.close(new)
            with contextlib.suppress(OSError):
                os.unlink(tmp)
            self.rotate_errors += 1
            _count("rotate_errors")
            self.retry_at = time.monotonic() + ROTATE_RETRY_S
            with contextlib.suppress(OSError):
                self._append(fd, self._line(ERROR_KIND, "error", {
                    "op": "rotate", "error": f"{type(e).__name__}: {e}"[:2000],
                    "size": self.size, "limit": self.limit, "retry_in_s": ROTATE_RETRY_S}))
            return fd
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
        st = os.fstat(new)
        self.fd, self.ino, self.size = new, (st.st_dev, st.st_ino), len(line)
        self.needs_newline, self.overflowed = False, False
        self.seq += 1
        self.rotations += 1
        _count("rotations")
        return new

    def _shift(self) -> None:
        """Free <name>.1 by shifting up to the lowest free slot; .keep is deleted only if none is."""
        hole = next((i for i in range(1, self.keep + 1)
                     if not os.path.lexists(rotated_path(self.path, i))), None)
        if hole is None:
            hole = self.keep
            os.unlink(rotated_path(self.path, hole))
        for i in range(hole, 1, -1):
            os.replace(rotated_path(self.path, i - 1), rotated_path(self.path, i))

    def _line(self, kind: str, lvl: str, data: dict) -> bytes:
        ev = Event(v=TRACE_VERSION, seq=self.seq, t=_wall(), mono=time.monotonic(),
                   src=self.src, kind=kind, lvl=lvl, data=data, proc=proc_id())
        return (ev.to_json() + "\n").encode()

    def _append_dropped(self, fd: int) -> None:
        """Under the flock: a trace.dropped line past the limit, so the file says what it lost."""
        n = self.dropped - self.reported
        if n <= 0:
            return
        size = os.fstat(fd).st_size
        if size != self.size:
            self._resync(fd, size)
        self._append(fd, self._line(DROPPED_KIND, "warn", {"dropped": n,
                                                           "max_bytes": self.max_bytes}))
        self.reported = self.dropped

    def close(self) -> None:
        with self.lock:
            if self.fd is not None and self.pid == os.getpid():
                with contextlib.suppress(OSError):
                    fcntl.flock(self.fd, fcntl.LOCK_EX)
                    try:
                        self._append_dropped(self.fd)
                    finally:
                        fcntl.flock(self.fd, fcntl.LOCK_UN)
                os.close(self.fd)
            self.fd = None


def _ino(path: Path) -> tuple[int, int] | None:
    try:
        st = os.stat(path)
    except FileNotFoundError:
        return None
    return st.st_dev, st.st_ino


def _write_all(fd: int, data: bytes) -> None:
    view = memoryview(data)
    while view:
        view = view[os.write(fd, view):]


def _close_all(streams: dict[str, _Stream]) -> None:
    for st in list(streams.values()):
        st.close()


@dataclass
class _Shared:
    trace_dir: Path
    episode_id: str
    max_bytes: int
    keep: int
    redactor: Redactor
    blobs: BlobStore
    key: int = field(default_factory=lambda: next(_KEYS))
    streams: dict[str, _Stream] = field(default_factory=dict)
    lock: threading.Lock = field(default_factory=threading.Lock)


class FileTracer:
    """Writes <trace_dir>/<stream_name(src)>, rotated at max_bytes (keep > 0) or capped."""

    def __init__(self, trace_dir, src: str, *, episode_id: str,
                 max_bytes: int = DEFAULT_MAX_BYTES, keep: int = 0,
                 redactor: Redactor | None = None, base_span: str | None = None,
                 _shared: _Shared | None = None):
        if not valid_src(src):
            raise ValueError(f"invalid trace src {src!r}")
        if _shared is None:
            trace_dir = Path(trace_dir)
            os.makedirs(trace_dir, exist_ok=True)
            redactor = redactor or DEFAULT
            _shared = _Shared(trace_dir=trace_dir, episode_id=episode_id, max_bytes=max_bytes,
                              keep=keep, redactor=redactor, blobs=BlobStore(trace_dir, redactor))
            weakref.finalize(_shared, _close_all, _shared.streams)
            _SHARED[_shared.key] = _shared
        self.src = src
        self.base_span = base_span
        self._s = _shared

    @property
    def trace_dir(self) -> Path:
        return self._s.trace_dir

    @property
    def episode_id(self) -> str:
        return self._s.episode_id

    def _stack(self) -> tuple[str, ...]:
        return _STACKS.get().get(self._s.key, ())

    def _set_stack(self, stack: tuple[str, ...]) -> None:
        m = dict(_STACKS.get())
        if stack:
            m[self._s.key] = stack
        else:
            m.pop(self._s.key, None)
        _STACKS.set(m)

    def current_span(self) -> str | None:
        stack = self._stack()
        return stack[-1] if stack else self.base_span

    def _stream(self) -> _Stream:
        with self._s.lock:
            st = self._s.streams.get(self.src)
            if st is None:
                st = _Stream(self.trace_dir / stream_name(self.src), self.src, self._s.max_bytes,
                             self._s.keep)
                self._s.streams[self.src] = st
            return st

    def _payload(self, data, verbatim: bool = False) -> dict:
        if data is None:
            return {}
        return self._s.redactor.value(data if isinstance(data, dict) else {"value": data},
                                      verbatim=verbatim)

    def _write(self, stream: _Stream, kind: str, payload: dict, lvl: str, blobs: tuple,
               span, parent, verbatim: bool = False) -> None:
        red, src, proc = self._s.redactor, self.src, proc_id()

        def build(seq: int, t: float, mono: float) -> str:
            line = Event(v=TRACE_VERSION, seq=seq, t=t, mono=mono, src=src, kind=kind,
                         data=payload, span=span, parent=parent, lvl=lvl, blobs=blobs,
                         proc=proc).to_json()
            if not red.secrets_in(line):
                return line
            return red.exact(line) if verbatim else red.text(line)

        stream.write(build, lvl)

    def _emit(self, kind, data, lvl, blobs, span, parent, verbatim: bool = False) -> None:
        """Write one event; any failure becomes a trace.error event, never an exception."""
        stream = self._stream()
        try:
            kind = str(kind)
            payload = self._payload(data, verbatim)
            if lvl not in LEVELS:
                payload["invalid_lvl"] = self._s.redactor.text(str(lvl))[:64]
                lvl = "info"
            self._write(stream, kind, payload, lvl, tuple(blobs), span, parent, verbatim)
        except OSError:
            stream.errors += 1
            _count("errors")
        except Exception as exc:  # noqa: BLE001
            self._error(stream, "event", kind, exc, span)

    def _error(self, stream: _Stream, op: str, what, exc: BaseException, span) -> None:
        try:
            red = self._s.redactor
            data = {"op": op, "kind": red.text(str(what))[:200],
                    "error": red.text(f"{type(exc).__name__}: {exc}")[:2000]}
            self._write(stream, ERROR_KIND, data, "error", (), span, None)
        except Exception:  # noqa: BLE001
            stream.errors += 1
            _count("errors")

    def event(self, kind: str, data: dict | None = None, *, lvl: str = "info",
              blobs: Sequence[BlobRef] = (), span: str | None = None,
              verbatim: bool = False) -> None:
        """verbatim: only registered secrets and REDACT_KEYS values are redacted."""
        self._emit(kind, data, lvl, blobs, span or self.current_span(), None, verbatim)

    @contextlib.contextmanager
    def span(self, kind: str, data: dict | None = None) -> Iterator[str]:
        sid, parent = new_span_id(), self.current_span()
        self._emit(f"{kind}.begin", data, "info", (), sid, parent)
        self._set_stack(self._stack() + (sid,))
        t0 = time.monotonic()
        try:
            yield sid
        except BaseException as exc:
            self._pop(sid)
            self._emit(f"{kind}.end", {"dur_s": time.monotonic() - t0, "error": repr(exc)},
                       "error", (), sid, parent)
            raise
        self._pop(sid)
        self._emit(f"{kind}.end", {"dur_s": time.monotonic() - t0}, "info", (), sid, parent)

    def _pop(self, sid: str) -> None:
        cur = self._stack()
        self._set_stack(cur[:-1] if cur and cur[-1] == sid else tuple(x for x in cur if x != sid))

    def blob(self, payload, *, mime: str | None = None, role: str = "",
             compress: bool = False, verbatim: bool = False) -> BlobRef:
        """A stored blob; on any failure an empty ref and a trace.error event, never a raise."""
        try:
            if verbatim and isinstance(payload, bytes) and mime and is_text_mime(mime):
                text = payload.decode("utf-8", "replace")
                data = self._s.redactor.exact(text).encode() if self._s.redactor.secrets_in(
                    text) else payload
                ext = ext_for(mime)
                if compress:
                    data, ext = gzip.compress(data, mtime=0), ext + GZIP_SUFFIX
                return self._s.blobs.put_bytes(data, mime=mime, ext=ext, role=role)
            return self._s.blobs.put(payload, mime=mime, role=role, compress=compress)
        except Exception as exc:  # noqa: BLE001
            self._error(self._stream(), "blob", type(payload).__name__, exc, self.current_span())
            return BlobRef(ref="", mime=mime or "application/octet-stream", size=0, role=role)

    def child(self, src: str) -> FileTracer:
        return FileTracer(self.trace_dir, src, episode_id=self.episode_id,
                          base_span=self.current_span(), _shared=self._s)

    def context(self) -> str:
        return f"{self.episode_id}/{self.current_span() or ''}"

    def stats(self) -> dict[str, dict]:
        """Per-src writer state: next seq, bytes, overflowed, dropped, rotations, errors, open fd."""
        with self._s.lock:
            return {src: {"seq": st.seq, "bytes": st.size, "overflowed": st.overflowed,
                          "dropped": st.dropped, "rotations": st.rotations,
                          "rotate_errors": st.rotate_errors, "errors": st.errors,
                          "open": st.fd is not None}
                    for src, st in self._s.streams.items()}

    def close(self) -> None:
        """Close every stream of this tracer and its children; a later event reopens one."""
        with self._s.lock:
            _close_all(self._s.streams)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc) -> None:
        self.close()
