#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The error every gateway layer raises to answer a request with a status and a JSON body."""

from __future__ import annotations


class HTTPError(Exception):
    def __init__(self, status: int, message: str, extra: dict | None = None):
        super().__init__(message)
        self.status = status
        self.extra = extra or {}

    def body(self) -> dict:
        return {"ok": False, **self.extra, "message": str(self)}
