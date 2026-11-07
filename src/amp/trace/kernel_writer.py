#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Kernel-side trace writer: copied into the sandbox next to a harness SDK, so it is one stdlib-only
file with no amp imports."""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import socket
import threading
import time

STREAM = "trace.jsonl"
BLOB_DIR = "blobs"
TRACE_ENV = "AMP_TRACE"
STREAM_ENV = "AMP_KERNEL_STREAM"
VERSION = 1
LEVELS = ("debug", "info", "warn", "error")
MAX_BYTES = 64 << 20
MAX_LINE = 1 << 20
MAX_IMAGE = 16 << 20


class KernelTraceWriter:
    """Appends to stream_path, else <directory>/trace.jsonl, else <cwd>/trace.jsonl; the path is
    pinned at first emit so a chdir cannot move it."""

    def __init__(self, directory: str | None = None, max_bytes: int = MAX_BYTES, *,
                 stream_path: str | None = None, span: str | None = None):
        self.path = os.path.abspath(stream_path) if stream_path else None
        self.dir = os.path.dirname(self.path) if self.path else directory
        self.span_override = span
        self.max_bytes = max_bytes
        self.lock = threading.Lock()
        self.fd = None
        self.pid = 0
        self.seq = 0
        self.size = 0
        self.span = None
        self.proc = ""
        self.overflowed = False

    def _open(self) -> int:
        if self.fd is None or self.pid != os.getpid():
            self.dir = self.dir or os.getcwd()
            self.path = self.path or os.path.join(self.dir, STREAM)
            self.fd = os.open(self.path, os.O_RDWR | os.O_APPEND | os.O_CREAT | os.O_CLOEXEC,
                              0o644)
            self.pid = os.getpid()
            self.size = os.fstat(self.fd).st_size
            self.seq = self._last_seq() + 1
            self.overflowed = self.size >= self.max_bytes
            self.span = self.span_override or _span_of(os.environ.get(TRACE_ENV, ""))
            self.proc = f"{socket.gethostname()}:{self.pid}:{time.time_ns()}"
        return self.fd

    def configure(self, stream_path: str | None = None, span: str | None = None) -> bool:
        """Point the writer at another stream file and/or span ("<episode>/<span>" or a bare span
        id); False if the arguments were unusable."""
        try:
            with self.lock:
                if stream_path:
                    path = os.fspath(stream_path)
                    if not isinstance(path, str):
                        return False
                    path = os.path.abspath(path)
                    if path != self.path:
                        self._close()
                        self.path, self.dir = path, os.path.dirname(path)
                if span is not None:
                    self.span_override = _span_of(str(span)) or None
                    self.span = self.span_override
            return True
        except Exception:  # noqa: BLE001
            return False

    def close(self) -> None:
        """Release the stream's fd; idempotent, and a later emit reopens it. Never raises."""
        with contextlib.suppress(Exception), self.lock:
            self._close()

    def __del__(self):
        with contextlib.suppress(Exception):
            self._close()

    def _close(self) -> None:
        fd, self.fd = self.fd, None
        if fd is not None and self.pid == os.getpid():
            os.close(fd)

    def _last_seq(self) -> int:
        n = min(self.size, 1 << 16)
        for ln in reversed(os.pread(self.fd, n, self.size - n).split(b"\n")):
            try:
                seq = json.loads(ln).get("seq")
            except (ValueError, AttributeError):
                continue
            if isinstance(seq, int):
                return seq
        return -1

    def _image(self, png: bytes, role: str) -> dict | None:
        if len(png) > MAX_IMAGE or self.size + len(png) > self.max_bytes:
            return None
        sha = hashlib.sha256(png).hexdigest()
        d = os.path.join(self.dir, BLOB_DIR)
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, sha + ".png")
        if not os.path.exists(path):
            tmp = f"{path}.{os.getpid()}.tmp"
            with open(tmp, "wb") as f:
                f.write(png)
            os.replace(tmp, path)
            self.size += len(png)
        return {"ref": f"{BLOB_DIR}/{sha}.png", "mime": "image/png", "size": len(png),
                "role": role}

    def _line(self, kind: str, data, lvl: str, blobs: list) -> bytes:
        ev = {"v": VERSION, "seq": self.seq, "t": time.time(), "mono": time.monotonic(),
              "src": "kernel", "kind": kind, "data": data, "span": self.span, "parent": None,
              "lvl": lvl, "blobs": blobs, "trusted": False, "proc": self.proc}
        line = json.dumps(ev, separators=(",", ":"), default=repr)
        if len(line) > MAX_LINE:
            ev["data"] = {"truncated": True, "repr": repr(data)[: MAX_LINE // 2]}
            line = json.dumps(ev, separators=(",", ":"), default=repr)
        if len(line) > MAX_LINE:
            ev["data"] = {"truncated": True}
            line = json.dumps(ev, separators=(",", ":"), default=repr)
        return (line + "\n").encode()

    def emit(self, kind, data=None, image_png_bytes=None, *, lvl="info", role="image") -> bool:
        """Append one event; False if it was dropped (cap reached, bad input, any error)."""
        try:
            with self.lock:
                fd = self._open()
                kind = str(kind)[:200]
                lvl = lvl if lvl in LEVELS else "info"
                data = data if isinstance(data, dict) else ({} if data is None else {"value": data})
                if self.overflowed:
                    return False
                blobs = []
                if image_png_bytes is not None:
                    ref = self._image(bytes(image_png_bytes), str(role)[:64])
                    if ref:
                        blobs.append(ref)
                line = self._line(kind, data, lvl, blobs)
                if self.size + len(line) > self.max_bytes:
                    self.overflowed = True
                    line = self._line("trace.overflow", {"max_bytes": self.max_bytes}, "warn", [])
                    os.write(fd, line)
                    self.size += len(line)
                    self.seq += 1
                    return False
                os.write(fd, line)
                self.size += len(line)
                self.seq += 1
                return True
        except Exception:  # noqa: BLE001
            return False


def _span_of(ctx: str) -> str | None:
    """The span id of "<episode_id>/<span_id>", or of a bare span id."""
    return (ctx.partition("/")[2] if "/" in ctx else ctx) or None


def _default_writer() -> KernelTraceWriter:
    try:
        return KernelTraceWriter(stream_path=os.environ.get(STREAM_ENV) or None)
    except Exception:  # noqa: BLE001
        return KernelTraceWriter()


_WRITER = _default_writer()


def configure(stream_path: str | None = None, span: str | None = None) -> bool:
    """Retarget this process's writer: the stream file and/or the span its events carry."""
    return _WRITER.configure(stream_path, span)


def emit(kind, data=None, image_png_bytes=None, *, lvl="info", role="image") -> bool:
    """Append one event to the configured stream; never raises."""
    return _WRITER.emit(kind, data, image_png_bytes, lvl=lvl, role=role)
