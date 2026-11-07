#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Reading a trace: streams merged in (t, src, seq) order, kind filters."""

from __future__ import annotations

import math
import os
import stat
from dataclasses import dataclass
from pathlib import Path

from amp.contracts.trace import TRACE_DIR, Event
from amp.fs.safe_files import safe_read_bytes


def _finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _well_typed(e: Event) -> bool:
    return (isinstance(e.seq, int) and not isinstance(e.seq, bool) and _finite(e.t)
            and _finite(e.mono) and isinstance(e.kind, str) and isinstance(e.data, dict)
            and isinstance(e.span, (str, type(None))) and isinstance(e.parent, (str, type(None)))
            and isinstance(e.trusted, bool) and isinstance(e.proc, str))


def parse_event(line: bytes | str, *, trusted: bool | None = None) -> Event | None:
    """Event.from_json plus type checks (finite times), or None for a line that is not an event."""
    try:
        ev = Event.from_json(line, trusted=trusted)
    except (ValueError, TypeError, KeyError, AttributeError, RecursionError):
        return None
    return ev if _well_typed(ev) else None


def parse_lines(raw: bytes) -> tuple[list[Event], int, bool]:
    """(events, malformed line count, whether the last line was cut off mid-write)."""
    lines = raw.split(b"\n")
    tail = lines.pop()
    events, malformed, truncated = [], 0, False
    for i, ln in enumerate([*lines, tail]):
        if not ln.strip():
            continue
        ev = parse_event(ln)
        if ev is not None:
            events.append(ev)
        elif i == len(lines):
            truncated = True
        else:
            malformed += 1
    return events, malformed, truncated


def kind_matches(kind: str, prefix: str) -> bool:
    """Dotted-prefix match: "agent.llm" matches "agent.llm" and "agent.llm.response"."""
    if not prefix or prefix.endswith("."):
        return kind.startswith(prefix)
    return kind == prefix or kind.startswith(prefix + ".")


@dataclass
class StreamInfo:
    name: str
    src: str
    events: int
    bytes: int
    malformed: int
    truncated: bool
    untrusted: int
    overflow: bool
    refused: str | None = None


@dataclass
class Trace:
    dir: Path
    events: list[Event]
    streams: dict[str, StreamInfo]

    def by_kind(self, prefix: str) -> list[Event]:
        return [e for e in self.events if kind_matches(e.kind, prefix)]


def trace_dir_of(path) -> Path:
    """An episode dir or a trace dir -> the trace dir."""
    p = Path(path)
    return p / TRACE_DIR if p.name != TRACE_DIR and (p / TRACE_DIR).is_dir() else p


def read_stream(path, root) -> tuple[list[Event], StreamInfo]:
    """One stream file, read without following links; an unreadable one is recorded as refused."""
    name = os.path.basename(path)
    try:
        raw = safe_read_bytes(path, root)
    except OSError as e:
        return [], StreamInfo(name=name, src="", events=0, bytes=0, malformed=0, truncated=False,
                              untrusted=0, overflow=False, refused=str(e))
    events, malformed, truncated = parse_lines(raw)
    info = StreamInfo(name=name, src=events[0].src if events else "",
                      events=len(events), bytes=len(raw), malformed=malformed,
                      truncated=truncated, untrusted=sum(not e.trusted for e in events),
                      overflow=any(e.kind == "trace.overflow" for e in events))
    return events, info


def stream_files(tdir: Path) -> list[str]:
    """The *.jsonl names in the trace dir (links too, to be refused)."""
    try:
        with os.scandir(tdir) as it:
            names = [e.name for e in it
                     if not stat.S_ISDIR(e.stat(follow_symlinks=False).st_mode)]
    except OSError:
        return []
    return sorted(n for n in names if n.endswith(".jsonl"))


def load(path) -> Trace:
    """Every <stream>.jsonl directly in the trace dir, merged."""
    tdir = trace_dir_of(path)
    events: list[Event] = []
    streams: dict[str, StreamInfo] = {}
    for name in stream_files(tdir):
        evs, info = read_stream(tdir / name, tdir)
        events.extend(evs)
        streams[name] = info
    events.sort(key=lambda e: (e.t, e.src, e.seq))
    return Trace(dir=tdir, events=events, streams=streams)
