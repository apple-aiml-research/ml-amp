#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Replay from the trace: rebuild the exact request body of any turn, or send its context to another
model."""

from __future__ import annotations

import base64
import hashlib
import json
from pathlib import Path

from amp.agents.llm import converter
from amp.agents.recording import BLOB_KEY
from amp.contracts.trace import TRACE_DIR, Event, stream_name
from amp.trace.blobs import read_blob
from amp.trace.redact import REDACTED


def unblobbed(obj, read_blob):
    """The inverse of blobbed; read_blob(ref dict) -> bytes."""
    if isinstance(obj, list):
        return [unblobbed(x, read_blob) for x in obj]
    if not isinstance(obj, dict):
        return obj
    src = obj.get("source")
    if obj.get("type") == "image" and isinstance(src, dict) and BLOB_KEY in src:
        b = src[BLOB_KEY]
        raw = read_blob(b)
        data = base64.b64encode(raw).decode() if b["b64"] else raw.decode()
        new_src = {}
        for k, v in src.items():
            if k == BLOB_KEY:
                new_src["data"] = data
            else:
                new_src[k] = v
        return {**obj, "source": new_src}
    return {k: unblobbed(v, read_blob) for k, v in obj.items()}


class ReplayMismatch(ValueError):
    """A rebuilt body whose sha256 differs from the one recorded when it was sent."""


class Request(dict):
    """A request body as sent; .context is the Anthropic-shaped {system, messages, tools}."""

    context: dict


class TraceView:
    """The agent stream and blobs of one trace: a MemoryTracer, an episode dir or a trace dir."""

    def __init__(self, trace):
        if hasattr(trace, "events") and hasattr(trace, "blobs"):
            self.events = [e for e in trace.events if e.src == "agent"]
            self._mem, self._dir = trace.blobs, None
            return
        d = Path(trace)
        if (d / TRACE_DIR).is_dir():
            d = d / TRACE_DIR
        self._mem, self._dir = None, d
        lines = (d / stream_name("agent")).read_text().splitlines()
        self.events = [Event.from_json(x) for x in lines if x.strip()]

    def blob(self, ref: str) -> bytes:
        if self._mem is not None:
            return self._mem[ref.rsplit("/", 1)[-1].split(".", 1)[0]]
        return read_blob(self._dir, ref)

    def kinds(self, kind: str) -> list[Event]:
        return [e for e in self.events if e.kind == kind]


def requests(trace) -> list[dict]:
    """Every agent.llm.request in order, each resolved to its full context and verified body."""
    out = list(iter_requests(trace))
    for r in out:
        if r["mismatch"]:
            raise ReplayMismatch(r["mismatch"])
    return out


def iter_requests(trace):
    """requests() unchecked and lazily: each record's "mismatch" says why its body is not the
    one sent, or is None."""
    view = trace if isinstance(trace, TraceView) else TraceView(trace)
    system = tools = None
    messages: list[dict] = []
    begin = view.kinds("agent.run.begin")
    model = ((begin[0].data.get("profile") or {}).get("model") or {}) if begin else {}
    if not model.get("supports_images", True):
        raise ValueError("replay rebuilds bodies of models that take images only")
    for e in view.kinds("agent.llm.request"):
        d = e.data
        if "system" in d:
            system = json.loads(view.blob(d["system"]["ref"]))
        if "tools" in d:
            tools = json.loads(view.blob(d["tools"]["ref"]))
        messages = messages[:d["base"]] + unblobbed(d["messages"],
                                                     lambda b: view.blob(b["ref"]))
        if len(messages) != d["n_messages"]:
            raise ValueError(f"turn {d['turn']}: rebuilt {len(messages)} messages, "
                             f"recorded {d['n_messages']}")
        ctx = {"system": system or [], "messages": messages, "tools": tools or []}
        convert, _ = converter(d["provider"])
        conv = convert(ctx["system"], json.loads(json.dumps(messages)), ctx["tools"])
        body = Request((k, d["params"][k] if k in d["params"] else conv[k]) for k in d["keys"])
        payload = json.dumps(body).encode()
        mismatch = None
        if hashlib.sha256(payload).hexdigest() != d["body_sha256"]:
            why = (f"; it holds {REDACTED!r}: a registered secret was redacted inside that turn, "
                   "so its exact body is not on disk" if REDACTED.encode() in payload else "")
            mismatch = (f"turn {d['turn']} attempt {d['attempt']}: the rebuilt body's sha256 is "
                        f"not the one recorded{why}")
        body.context = json.loads(json.dumps(ctx))
        yield {"turn": d["turn"], "attempt": d["attempt"], "body": body,
               "body_sha256": d["body_sha256"], "mismatch": mismatch}
