#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Atomic file writes: a temp file in the same directory, fsync, then os.replace."""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path

FILE_MODE = 0o644


def write_bytes_atomic(path, data: bytes, *, mode: int = FILE_MODE) -> None:
    """Replace path with data; readers see the old file or the new one, never a partial one."""
    path = Path(path)
    fd, tmp = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as f:
            os.fchmod(f.fileno(), mode)
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def write_json_atomic(path, obj, *, mode: int = FILE_MODE, indent: int | None = None) -> None:
    """Write obj as strict JSON (no NaN) via write_bytes_atomic."""
    write_bytes_atomic(path, json.dumps(obj, allow_nan=False, indent=indent).encode(), mode=mode)
