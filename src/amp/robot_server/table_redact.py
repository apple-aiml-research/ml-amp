#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""hide_table_height: withhold the table's height from every agent reply."""

from __future__ import annotations

import re

DROPPED_KEYS = frozenset({"table_z", "min_clearance_m"})
_NUM = r"-?\d+(?:\.\d+)?\s?(?:mm|cm|m)"
_SUBS = (
    (re.compile(rf"(?:only )?{_NUM} (above|below) the table"), r"\1 the table"),
    (re.compile(rf"within {_NUM} of the table"), "near the table"),
    (re.compile(rf"\s?\(lowest {_NUM}(?:, at most {_NUM})?\)"), ""),
    (re.compile(r"table_z -?\d+(?:\.\d+)? estimated"), "table estimated"),
    (re.compile(r";?\s?table_z = -?\d+(?:\.\d+)?(?:e-?\d+)?"), ""),
)


def redact_text(s: str) -> str:
    if "table" not in s:
        return s
    for rx, sub in _SUBS:
        s = rx.sub(sub, s)
    return s


def redact(body):
    """A copy of a JSON-able reply body with the table's height removed."""
    if isinstance(body, dict):
        table = body.get("obstacle") == "table"
        return {k: redact(v) for k, v in body.items()
                if k not in DROPPED_KEYS and not (table and k == "clearance_m")}
    if isinstance(body, (list, tuple)):
        return [redact(v) for v in body]
    if isinstance(body, str):
        return redact_text(body)
    return body
