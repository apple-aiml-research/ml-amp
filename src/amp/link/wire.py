#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""TLF1 binary frame: magic, a length-prefixed JSON header, then raw or encoded blobs."""

from __future__ import annotations

import json
import math
import struct
from typing import Any

import numpy as np

from amp.contracts.robot import LINK_CONTENT_TYPE, LINK_PROTOCOL_VERSION

MAGIC = b"TLF1"
CONTENT_TYPE = LINK_CONTENT_TYPE
PROTOCOL_VERSION = LINK_PROTOCOL_VERSION
GRADE = "grade"

_PREFIX = struct.Struct("<4sI")

Blob = np.ndarray | bytes


class FrameError(ValueError):
    """A TLF1 frame that cannot be parsed."""


def jsonable(obj: Any) -> Any:
    """Numpy scalars/arrays and tuples as plain JSON types; NaN and infinity become null."""
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        if obj.dtype.kind == "f" and not np.isfinite(obj).all():
            return np.where(np.isfinite(obj), obj, None).tolist()
        return obj.tolist()
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        f = float(obj)
        return f if math.isfinite(f) else None
    return obj


def _raw_bytes(arr: np.ndarray) -> tuple[bytes, dict]:
    arr = np.ascontiguousarray(arr)
    little = arr.dtype.newbyteorder("<")
    if arr.dtype != little:
        arr = arr.astype(little)
    return arr.tobytes(order="C"), {"dtype": arr.dtype.name, "shape": list(arr.shape)}


def pack_frame(
    header: dict,
    blobs: dict[str, np.ndarray | bytes | bytearray | memoryview] | None = None,
    encodings: dict[str, str] | None = None,
) -> bytes:
    """Serialize header and blobs into one TLF1 frame; arrays go raw, bytes keep their encoding."""
    encodings = encodings or {}
    meta: dict[str, dict] = {}
    parts: list[bytes] = []
    offset = 0
    for name, blob in (blobs or {}).items():
        if isinstance(blob, np.ndarray):
            data, info = _raw_bytes(blob)
            entry = {"encoding": "raw", **info}
        elif isinstance(blob, (bytes, bytearray, memoryview)):
            data = bytes(blob)
            entry = {"encoding": encodings.get(name, "bytes")}
        else:
            raise TypeError(f"blob {name!r}: expected ndarray or bytes, got {type(blob).__name__}")
        meta[name] = {"offset": offset, "length": len(data), **entry}
        parts.append(data)
        offset += len(data)
    head = jsonable(dict(header))
    head["blobs"] = meta
    head_bytes = json.dumps(head, separators=(",", ":")).encode("utf-8")
    return b"".join([_PREFIX.pack(MAGIC, len(head_bytes)), head_bytes, *parts])


def unpack_frame(data: bytes | bytearray | memoryview) -> tuple[dict, dict[str, Blob]]:
    """(header, blobs) of a TLF1 frame; raw blobs come back as writeable native-order arrays."""
    mv = memoryview(data).cast("B")
    if len(mv) < _PREFIX.size:
        raise FrameError(f"frame too short: {len(mv)} bytes")
    magic, head_len = _PREFIX.unpack(mv[: _PREFIX.size])
    if magic != MAGIC:
        raise FrameError(f"bad magic {bytes(magic)!r}, expected {MAGIC!r}")
    start = _PREFIX.size + head_len
    if start > len(mv):
        raise FrameError(f"header length {head_len} exceeds frame size {len(mv)}")
    try:
        header = json.loads(bytes(mv[_PREFIX.size : start]).decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise FrameError(f"header is not UTF-8 JSON: {exc}") from exc
    if not isinstance(header, dict):
        raise FrameError("header is not a JSON object")
    meta = header.get("blobs") or {}
    if not isinstance(meta, dict):
        raise FrameError("header['blobs'] is not an object")
    payload = mv[start:]
    return header, {name: _blob(name, entry, payload) for name, entry in meta.items()}


def _blob(name: str, entry: Any, payload: memoryview) -> Blob:
    try:
        off, length = int(entry["offset"]), int(entry["length"])
    except (KeyError, TypeError, ValueError) as exc:
        raise FrameError(f"blob {name!r}: bad offset/length") from exc
    if off < 0 or length < 0 or off + length > len(payload):
        raise FrameError(
            f"blob {name!r}: [{off}, {off + length}) outside payload of {len(payload)} bytes"
        )
    chunk = payload[off : off + length]
    if entry.get("encoding", "bytes") != "raw":
        return bytes(chunk)
    try:
        dtype = np.dtype(entry["dtype"]).newbyteorder("<")
        shape = tuple(int(s) for s in entry["shape"])
    except (KeyError, TypeError) as exc:
        raise FrameError(f"blob {name!r}: raw blob needs dtype and shape") from exc
    expected = int(np.prod(shape, dtype=np.int64)) * dtype.itemsize
    if expected != length:
        raise FrameError(
            f"blob {name!r}: {shape} {dtype.name} needs {expected} bytes, got {length}")
    arr = np.frombuffer(chunk, dtype=dtype).reshape(shape)
    return arr.astype(dtype.newbyteorder("="), copy=True)
