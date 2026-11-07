#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Content-addressed blobs: <trace>/blobs/<sha256>.<ext>."""

from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
import re
import sys
from pathlib import Path

from amp.contracts.trace import BlobRef
from amp.fs.atomic import write_bytes_atomic
from amp.fs.safe_files import safe_read_bytes
from amp.trace.redact import Redactor

BLOB_DIR = "blobs"
GZIP_SUFFIX = ".gz"
EXT_MIME = {
    "bin": "application/octet-stream", "txt": "text/plain", "json": "application/json",
    "npy": "application/x-npy", "png": "image/png", "jpg": "image/jpeg", "html": "text/html",
}
MIME_EXT = {m: e for e, m in EXT_MIME.items()} | {"image/jpg": "jpg"}
REF_RE = re.compile(r"^blobs/([0-9a-f]{64})\.([a-z0-9]{1,8})(\.gz)?$")


def ext_for(mime: str) -> str:
    """The file extension for a mime type (text/* defaults to txt, the rest to bin)."""
    base = mime.split(";")[0].strip().lower()
    return MIME_EXT.get(base) or ("txt" if base.startswith("text/") else "bin")


def is_text_mime(mime: str) -> bool:
    base = mime.split(";")[0].strip().lower()
    return base.startswith("text/") or base == "application/json" or base.endswith("+json")


def encode(payload, mime: str | None, redactor: Redactor) -> tuple[bytes, str, str]:
    """payload -> (bytes, mime, ext); text and JSON are redacted on the way."""
    if isinstance(payload, (bytes, bytearray, memoryview)):
        data, mime = bytes(payload), mime or "application/octet-stream"
        if is_text_mime(mime):
            data = redactor.text(data.decode("utf-8", "replace")).encode()
        return data, mime, ext_for(mime)
    if isinstance(payload, str):
        mime = mime or "text/plain"
        return redactor.text(payload).encode(), mime, ext_for(mime)
    if isinstance(payload, (dict, list)):
        data = json.dumps(redactor.value(payload), separators=(",", ":"), default=str).encode()
        return data, "application/json", "json"
    np = sys.modules.get("numpy")
    if np is not None and isinstance(payload, np.ndarray):
        buf = io.BytesIO()
        np.save(buf, payload, allow_pickle=False)
        return buf.getvalue(), "application/x-npy", "npy"
    pil = sys.modules.get("PIL.Image")
    if pil is not None and isinstance(payload, pil.Image):
        buf = io.BytesIO()
        payload.save(buf, "PNG")
        return buf.getvalue(), "image/png", "png"
    raise TypeError(f"cannot store a {type(payload).__name__} as a blob")


class BlobStore:
    """Writes blobs once each into <trace_dir>/blobs, atomically."""

    def __init__(self, trace_dir, redactor: Redactor):
        self.dir = Path(trace_dir) / BLOB_DIR
        self.redactor = redactor

    def put(self, payload, *, mime: str | None = None, role: str = "",
            compress: bool = False) -> BlobRef:
        """Store payload; compress gzips it (deterministically, so the hash still dedups)."""
        data, mime, ext = encode(payload, mime, self.redactor)
        if compress:
            data, ext = gzip.compress(data, mtime=0), ext + GZIP_SUFFIX
        return self.put_bytes(data, mime=mime, ext=ext, role=role)

    def put_bytes(self, data: bytes, *, mime: str, ext: str, role: str = "") -> BlobRef:
        """Store already-encoded bytes under their sha256."""
        name = f"{hashlib.sha256(data).hexdigest()}.{ext}"
        path = self.dir / name
        try:
            exists = path.stat().st_size == len(data)
        except OSError:
            exists = False
        if not exists:
            os.makedirs(self.dir, exist_ok=True)
            write_bytes_atomic(path, data)
        return BlobRef(ref=f"{BLOB_DIR}/{name}", mime=mime, size=len(data), role=role)


def read_blob(trace_dir, ref: str) -> bytes:
    """A blob's content, gunzipped if its ref ends in .gz; ValueError for a malformed ref."""
    if not REF_RE.match(ref):
        raise ValueError(f"not a blob ref: {ref!r}")
    data = safe_read_bytes(Path(trace_dir) / ref, trace_dir)
    return gzip.decompress(data) if ref.endswith(GZIP_SUFFIX) else data
