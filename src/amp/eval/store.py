#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Unit records in a run root, and the NEVER_UPLOAD and TOKEN_FIELDS filters on episode files."""

from __future__ import annotations

import fnmatch
import json
import math
import os
from collections.abc import Sequence
from pathlib import Path

from amp.contracts.eval import (
    DELIVERED_STATUSES,
    NEVER_UPLOAD,
    RETRYABLE_STATUSES,
    TOKEN_FIELDS,
    UNIT_STATUSES,
    outcome,
)
from amp.fs.atomic import write_bytes_atomic
from amp.fs.safe_files import safe_read_bytes

from .layout import unit_rel


def excluded(rel: str, patterns: Sequence[str] = NEVER_UPLOAD) -> bool:
    """`dir/` excludes that top-level dir; anything else is an fnmatch on the whole rel path."""
    rel = rel.replace(os.sep, "/")
    for p in patterns:
        if p.endswith("/") and (rel == p[:-1] or rel.startswith(p)):
            return True
        if not p.endswith("/") and fnmatch.fnmatchcase(rel, p):
            return True
    return False


def strip_tokens(obj):
    """A copy of obj without any TOKEN_FIELDS key, at any depth."""
    if isinstance(obj, dict):
        return {k: strip_tokens(v) for k, v in obj.items() if k not in TOKEN_FIELDS}
    if isinstance(obj, list | tuple):
        return [strip_tokens(v) for v in obj]
    return obj


def jsonable(obj):
    """obj with non-finite floats as None, so it is strict JSON."""
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list | tuple):
        return [jsonable(v) for v in obj]
    return obj


def dump_json(obj) -> bytes:
    return (json.dumps(jsonable(obj), indent=1, sort_keys=True, default=str,
                       allow_nan=False) + "\n").encode()


def write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_bytes_atomic(path, dump_json(obj))


def parse_record(raw: bytes | None) -> dict | None:
    """A unit record from its bytes, or None when absent or not a record."""
    if raw is None:
        return None
    try:
        rec = json.loads(raw)
    except ValueError:
        return None
    return rec if isinstance(rec, dict) and rec.get("status") in UNIT_STATUSES else None


def is_final_record(rec: dict) -> bool:
    """A record no other attempt may replace: delivered, scored or moved, and not retryable."""
    status = rec.get("status")
    if status in DELIVERED_STATUSES:
        return True
    if status in RETRYABLE_STATUSES:
        return False
    return rec.get("void") is True or outcome(rec) is not None or bool(rec.get("attempted"))


def read_record(local_root: Path, task: str, episode: int) -> dict | None:
    """The unit record in the run root, or None."""
    try:
        raw = safe_read_bytes(Path(local_root) / unit_rel(task, episode), Path(local_root))
    except OSError:
        raw = None
    return parse_record(raw)
