#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The server's error types; api/ maps them to HTTP (contracts.server docstring)."""

from __future__ import annotations


class SessionConflict(Exception):
    """A request the current state cannot satisfy (HTTP 409); body is the reply."""

    def __init__(self, message, body=None):
        super().__init__(message)
        self.body = body if body is not None else {"ok": False, "message": message}


class NotFound(LookupError):
    """A task, camera or file that does not exist (HTTP 404)."""


class Forbidden(PermissionError):
    """A path that would leave its folder, or a call the harness does not allow (HTTP 403)."""


class AssistError(RuntimeError):
    """A planner raised: HTTP 500, never 400 (which means a bad body)."""


class HTTPError(Exception):
    """An explicit status with a message, for routing and framing errors."""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
