#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""TLF1 binary frames: b"TLF1", a uint32-LE header length, a JSON header, then blobs."""

from __future__ import annotations

import json
import struct

import numpy as np

MAGIC = b"TLF1"
CONTENT_TYPE = "application/x-teleop-frame"
_PREFIX = struct.Struct("<4sI")


class FrameError(ValueError):
    """A TLF1 frame that cannot be parsed."""


def decode(data: bytes) -> tuple[dict, dict]:
    """Parse a TLF1 frame into (header, blobs); raw blobs become numpy arrays, others bytes."""
    mv = memoryview(data).cast("B")
    if len(mv) < _PREFIX.size:
        raise FrameError(f"frame too short: {len(mv)} bytes")
    magic, head_len = _PREFIX.unpack(mv[: _PREFIX.size])
    if magic != MAGIC:
        raise FrameError(f"bad magic {bytes(magic)!r}")
    start = _PREFIX.size + head_len
    if start > len(mv):
        raise FrameError(f"header length {head_len} exceeds frame size {len(mv)}")
    header = json.loads(bytes(mv[_PREFIX.size : start]).decode("utf-8"))
    if not isinstance(header, dict):
        raise FrameError("header is not a JSON object")
    payload = mv[start:]
    blobs: dict = {}
    for name, e in (header.get("blobs") or {}).items():
        off, length = int(e["offset"]), int(e["length"])
        if off < 0 or length < 0 or off + length > len(payload):
            raise FrameError(f"blob {name!r} lies outside the payload")
        chunk = payload[off : off + length]
        if e.get("encoding", "bytes") != "raw":
            blobs[name] = bytes(chunk)
            continue
        dtype = np.dtype(e["dtype"]).newbyteorder("<")
        shape = tuple(int(s) for s in e["shape"])
        if int(np.prod(shape, dtype=np.int64)) * dtype.itemsize != length:
            raise FrameError(f"blob {name!r}: {shape} {dtype.name} does not match {length} bytes")
        arr = np.frombuffer(chunk, dtype=dtype).reshape(shape)
        blobs[name] = arr.astype(dtype.newbyteorder("="))
    return header, blobs


def encode(header: dict, blobs: dict | None = None, encodings: dict | None = None) -> bytes:
    """Build a TLF1 frame; numpy blobs are stored raw, bytes with the given encoding."""
    meta, parts, offset = {}, [], 0
    for name, blob in (blobs or {}).items():
        if isinstance(blob, np.ndarray):
            arr = np.ascontiguousarray(blob)
            arr = arr.astype(arr.dtype.newbyteorder("<"))
            data = arr.tobytes()
            entry = {"encoding": "raw", "dtype": arr.dtype.name, "shape": list(arr.shape)}
        else:
            data = bytes(blob)
            entry = {"encoding": (encodings or {}).get(name, "bytes")}
        meta[name] = {"offset": offset, "length": len(data), **entry}
        parts.append(data)
        offset += len(data)
    head = json.dumps({**header, "blobs": meta}, separators=(",", ":")).encode()
    return b"".join([_PREFIX.pack(MAGIC, len(head)), head, *parts])
