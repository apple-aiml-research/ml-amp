#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""How a conversation is written into the trace and read back: images become blobs, each
request records only the messages that changed since the previous one."""

from __future__ import annotations

import base64
import binascii
import hashlib
import json

BLOB_KEY = "data_blob"


def image_blob(data: str, media_type: str, tracer) -> dict:
    """A base64 image as a blob reference; the decoded bytes when the base64 is canonical."""
    raw = b""
    try:
        raw = base64.b64decode(data, validate=True)
        canonical = base64.b64encode(raw).decode() == data
    except (binascii.Error, ValueError):
        canonical = False
    if canonical:
        ref = tracer.blob(raw, mime=media_type or "application/octet-stream", role="image")
    else:
        ref = tracer.blob(data.encode(), mime="application/octet-stream", role="image_b64")
    return {"ref": ref.ref, "sha256": hashlib.sha256(raw if canonical else data.encode())
            .hexdigest(), "b64": canonical, "mime": ref.mime, "size": ref.size}


def blobbed(obj, tracer, refs: list | None = None):
    """A deep copy of obj with every image's base64 data replaced by a blob reference."""
    if isinstance(obj, list):
        return [blobbed(x, tracer, refs) for x in obj]
    if not isinstance(obj, dict):
        return obj
    if obj.get("type") == "image" and isinstance(obj.get("source"), dict) and \
            isinstance(obj["source"].get("data"), str):
        src = {k: v for k, v in obj["source"].items() if k != "data"}
        b = image_blob(obj["source"]["data"], src.get("media_type", ""), tracer)
        src[BLOB_KEY] = b
        if refs is not None:
            refs.append(b)
        return {**{k: v for k, v in obj.items() if k != "source"}, "source": src}
    return {k: blobbed(v, tracer, refs) for k, v in obj.items()}


def canonical(obj) -> str:
    return json.dumps(obj, sort_keys=True, separators=(",", ":"), default=str)


class RequestLog:
    """What the previous request held, so the next records only its changed suffix."""

    def __init__(self):
        self.sent: list[str] = []
        self.system: str | None = None
        self.tools: str | None = None

    def delta(self, messages: list[dict]) -> tuple[int, list[dict]]:
        """(base, new): the request is the previous one's first `base` messages, then `new`."""
        now = [canonical(m) for m in messages]
        base = 0
        while base < min(len(now), len(self.sent)) and now[base] == self.sent[base]:
            base += 1
        self.sent = now
        return base, messages[base:]

    def changed(self, what: str, value) -> bool:
        c = canonical(value)
        if getattr(self, what) == c:
            return False
        setattr(self, what, c)
        return True
