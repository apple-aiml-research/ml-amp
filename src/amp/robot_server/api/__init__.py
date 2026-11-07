#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The HTTP API as handlers: each parses its request, calls a service and returns a Reply."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Request:
    method: str
    path: str
    params: dict
    query: dict
    body: dict | None = None
    raw: bytes = b""
    headers: dict = field(default_factory=dict)


@dataclass
class Reply:
    status: int
    body: object
    content_type: str = "application/json"
    headers: dict = field(default_factory=dict)


def ok(body) -> Reply:
    return Reply(200, body)


def binary(data: bytes, content_type: str, **headers) -> Reply:
    return Reply(200, data, content_type, {"Cache-Control": "no-store", **headers})
