#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Where the agent process runs: a Linux box."""

from __future__ import annotations

from amp.contracts.model import RuntimeEnv


def detect_runtime(environ=None) -> RuntimeEnv:
    """Linux, the only platform AMP runs on; environ is accepted for callers that pass one."""
    return RuntimeEnv(where="linux")
