#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A backend's fault records: one JSON line each, appended to the file FAULT_FILE_ENV names."""

from __future__ import annotations

import json
import os

FAULT_FILE_ENV = "AMP_BACKEND_FAULT_FILE"


def report(record: dict, environ=None) -> bool:
    """Append record to the file FAULT_FILE_ENV names; False when none is set. Never raises."""
    path = (os.environ if environ is None else environ).get(FAULT_FILE_ENV)
    if not path:
        return False
    try:
        line = json.dumps(record, sort_keys=True, default=str) + "\n"
        fd = os.open(path, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
        try:
            os.write(fd, line.encode())
        finally:
            os.close(fd)
    except (OSError, TypeError, ValueError):
        return False
    return True
