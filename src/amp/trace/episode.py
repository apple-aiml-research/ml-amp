#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The episode's trace lifecycle: open the eval stream, ingest external streams, write the index."""

from __future__ import annotations

import gzip
import hashlib
import os
import zlib
from collections import Counter
from collections.abc import Callable, Iterable
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

from amp.contracts.trace import KERNEL_STREAM, TRACE_DIR, BlobRef, stream_name, valid_src
from amp.fs.atomic import write_bytes_atomic, write_json_atomic
from amp.fs.safe_files import safe_read_bytes, walk
from amp.trace.blobs import BLOB_DIR, EXT_MIME, GZIP_SUFFIX, REF_RE, BlobStore
from amp.trace.reader import load, parse_event, parse_lines
from amp.trace.redact import DEFAULT, Redactor
from amp.trace.writer import DEFAULT_MAX_BYTES, FileTracer

INDEX = "index.json"
UNTRUSTED_SUFFIX = ".untrusted.jsonl"
MAX_INGEST_BLOB = 32 << 20
MAX_INGEST_BLOBS = 256 << 20
UNTRUSTED_EXTS = frozenset({"png", "jpg", "npy", "json", "txt", "bin"})


@dataclass
class Ingested:
    path: str
    src: str
    trusted: bool
    target: str = ""
    events: int = 0
    malformed: int = 0
    rejected: int = 0
    truncated: bool = False
    blobs: int = 0
    blobs_dropped: int = 0
    refused: str | None = None


@dataclass
class FinalizeContext:
    """What a finalizer gets: the dirs and an ingest step."""

    episode_dir: Path
    trace_dir: Path
    redactor: Redactor
    kernel_uid: int | None = None
    max_bytes: int = DEFAULT_MAX_BYTES
    max_blob_bytes: int = MAX_INGEST_BLOBS
    ingested: list[Ingested] = field(default_factory=list)

    def ingest(self, path, src: str, trusted: bool = False) -> Ingested:
        """Copy an external stream into the trace dir (see target_name); never overwrites."""
        res = ingest_stream(self, path, src, trusted)
        self.ingested.append(res)
        return res


Finalizer = Callable[[FinalizeContext], None]


def target_name(src: str, trusted: bool) -> str:
    """The trace-dir file an ingested stream lands in; untrusted ones never share a trusted name."""
    name = stream_name(src)
    return name if trusted else name[: -len(".jsonl")] + UNTRUSTED_SUFFIX


def _inside(path: Path, root: Path) -> bool:
    """Lexically inside root; links are left for safe_files to refuse, never resolved here."""
    p, r = os.path.abspath(path), os.path.abspath(root)
    return p != r and os.path.commonpath([p, r]) == r


def _gunzip_bounded(data: bytes, limit: int) -> bytes | None:
    d = zlib.decompressobj(wbits=31)
    try:
        out = d.decompress(data, limit + 1)
    except zlib.error:
        return None
    return out if d.eof and not d.unused_data and len(out) <= limit else None


def _restore_blob(ctx: FinalizeContext, store: BlobStore, stream_dir: Path, root: Path,
                  b: BlobRef, trusted: bool, budget: list[int]) -> BlobRef | None:
    m = REF_RE.match(b.ref)
    if not m:
        return None
    sha, ext, gz = m.group(1), m.group(2), bool(m.group(3))
    try:
        data = safe_read_bytes(stream_dir / b.ref, root, MAX_INGEST_BLOB + 1,
                               kernel_uid=ctx.kernel_uid)
    except OSError:
        return None
    if len(data) > MAX_INGEST_BLOB or len(data) > budget[0]:
        return None
    if hashlib.sha256(data).hexdigest() != sha:
        return None
    mime = b.mime
    if not trusted:
        ext = ext if ext in UNTRUSTED_EXTS else "bin"
        mime = EXT_MIME[ext]
        if ext in ("txt", "json"):
            text = _gunzip_bounded(data, MAX_INGEST_BLOB) if gz else data
            if text is None:
                return None
            text = ctx.redactor.text(text.decode("utf-8", "replace")).encode()
            data = gzip.compress(text, mtime=0) if gz else text
    budget[0] -= len(data)
    return store.put_bytes(data, mime=mime, ext=ext + (GZIP_SUFFIX if gz else ""),
                           role=str(b.role)[:64])


def ingest_stream(ctx: FinalizeContext, path, src: str, trusted: bool) -> Ingested:
    """Validate and copy one external stream."""
    if not valid_src(src):
        raise ValueError(f"invalid trace src {src!r}")
    path = Path(path)
    path = path if path.is_absolute() else ctx.episode_dir / path
    target = ctx.trace_dir / target_name(src, trusted)
    res = Ingested(path=str(path), src=src, trusted=trusted, target=target.name)
    inside = _inside(path, ctx.episode_dir)
    if not trusted and not inside:
        res.refused = "outside the episode directory"
        return res
    root = ctx.episode_dir if inside else path.parent
    try:
        raw = safe_read_bytes(path, root, ctx.max_bytes + 1, kernel_uid=ctx.kernel_uid)
    except FileNotFoundError:
        res.refused = "missing"
        return res
    except OSError as e:
        res.refused = str(e)
        return res
    if os.path.lexists(target):
        if trusted and os.path.samefile(path, target):
            evs, _, truncated = parse_lines(raw)
            res.events, res.truncated = len(evs), truncated
        else:
            res.refused = f"target {target.name} already exists"
        return res
    try:
        _copy_events(ctx, res, raw, path, root, target, trusted)
    except Exception as e:  # noqa: BLE001
        res.refused = f"error: {e!r}"
    return res


def _copy_events(ctx: FinalizeContext, res: Ingested, raw: bytes, path: Path, root: Path,
                 target: Path, trusted: bool) -> None:
    if len(raw) > ctx.max_bytes:
        raw, res.truncated = raw[: raw.rfind(b"\n", 0, ctx.max_bytes) + 1], True
    store = BlobStore(ctx.trace_dir, ctx.redactor)
    budget = [ctx.max_blob_bytes]
    out = []
    lines = raw.split(b"\n")
    for i, ln in enumerate(lines):
        if not ln.strip():
            continue
        ev = parse_event(ln, trusted=None if trusted else False)
        if ev is None:
            res.truncated = res.truncated or i == len(lines) - 1
            res.malformed += i != len(lines) - 1
            continue
        if ev.src != res.src:
            res.rejected += 1
            continue
        blobs = []
        for b in ev.blobs:
            nb = _restore_blob(ctx, store, path.parent, root, b, trusted, budget)
            res.blobs += nb is not None
            res.blobs_dropped += nb is None
            if nb is not None:
                blobs.append(nb)
        ev = replace(ev, data=ctx.redactor.value(ev.data), blobs=tuple(blobs))
        if not trusted:
            ev = replace(ev, seq=len(out))
        line = ev.to_json()
        out.append(ctx.redactor.text(line) if ctx.redactor.secrets_in(line) else line)
    res.events = len(out)
    write_bytes_atomic(target, "".join(f"{ln}\n" for ln in out).encode())


def kernel_finalizer(ctx: FinalizeContext) -> None:
    """Ingest <episode>/kernel_cwd/trace.jsonl as the untrusted "kernel" stream, if present."""
    if os.path.lexists(ctx.episode_dir / KERNEL_STREAM):
        ctx.ingest(KERNEL_STREAM, "kernel", trusted=False)


def blob_stats(trace_dir: Path) -> tuple[int, int]:
    """(count, bytes) of the well-named regular files in trace/blobs."""
    count = size = 0
    for rel, why in walk(trace_dir, BLOB_DIR):
        if why is None and REF_RE.match(rel):
            count += 1
            size += os.stat(trace_dir / rel, follow_symlinks=False).st_size
    return count, size


def build_index(trace_dir: Path, ingested: Iterable[Ingested] = (),
                finalizer_errors: Iterable[dict] = ()) -> dict:
    """The index: per-stream counts and bytes, kinds, time range, blobs, trust, overflow."""
    tr = load(trace_dir)
    nblobs, blob_bytes = blob_stats(trace_dir)
    ts = [e.t for e in tr.events]
    return {
        "v": 1,
        "streams": {name: {"src": s.src, "events": s.events, "bytes": s.bytes,
                           "malformed": s.malformed, "truncated": s.truncated,
                           "untrusted": s.untrusted, "overflow": s.overflow,
                           "refused": s.refused}
                    for name, s in sorted(tr.streams.items())},
        "events": len(tr.events),
        "kinds": dict(sorted(Counter(e.kind for e in tr.events).items())),
        "time": {"start": min(ts), "end": max(ts)} if ts else None,
        "blobs": {"count": nblobs, "bytes": blob_bytes},
        "untrusted": sum(not e.trusted for e in tr.events),
        "malformed": sum(s.malformed for s in tr.streams.values()),
        "overflow": sorted(n for n, s in tr.streams.items() if s.overflow),
        "ingested": [asdict(i) for i in ingested],
        "finalizer_errors": list(finalizer_errors),
    }


class EpisodeTrace:
    """Opens an episode's eval stream and finalizes its trace; layers add finalizers."""

    def __init__(self, finalizers: Iterable[Finalizer] = (), *, redactor: Redactor | None = None,
                 kernel_uid: int | None = None, max_bytes: int = DEFAULT_MAX_BYTES,
                 max_blob_bytes: int = MAX_INGEST_BLOBS):
        self.finalizers: list[Finalizer] = list(finalizers)
        self.redactor = redactor or DEFAULT
        self.kernel_uid = kernel_uid
        self.max_bytes = max_bytes
        self.max_blob_bytes = max_blob_bytes

    @staticmethod
    def open(episode_dir, episode_id: str, **kw) -> FileTracer:
        """The episode's root tracer, src "eval", writing <episode_dir>/trace."""
        return FileTracer(Path(episode_dir) / TRACE_DIR, "eval", episode_id=episode_id, **kw)

    def register_finalizer(self, fn: Finalizer) -> Finalizer:
        self.finalizers.append(fn)
        return fn

    def finalize(self, episode_dir, *, ingest: Iterable[tuple] = ()) -> dict:
        """Ingest the given (path, src, trusted) streams, run the finalizers, write index.json;
        the index is written even when building it fails (it then carries "error")."""
        episode_dir = Path(episode_dir)
        trace_dir = episode_dir / TRACE_DIR
        os.makedirs(trace_dir, exist_ok=True)
        ctx = FinalizeContext(episode_dir=episode_dir, trace_dir=trace_dir,
                              redactor=self.redactor, kernel_uid=self.kernel_uid,
                              max_bytes=self.max_bytes, max_blob_bytes=self.max_blob_bytes)
        for path, src, trusted in ingest:
            ctx.ingest(path, src, bool(trusted))
        errors = []
        for fn in self.finalizers:
            try:
                fn(ctx)
            except Exception as e:  # noqa: BLE001
                errors.append({"finalizer": getattr(fn, "__qualname__", repr(fn)),
                               "error": repr(e)})
        try:
            index = build_index(trace_dir, ctx.ingested, errors)
            write_json_atomic(trace_dir / INDEX, index, indent=1)
        except Exception as e:  # noqa: BLE001
            index = {"v": 1, "error": repr(e), "ingested": [asdict(i) for i in ctx.ingested],
                     "finalizer_errors": errors}
            write_json_atomic(trace_dir / INDEX, index, indent=1)
        return index
