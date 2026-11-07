#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Kernel sandboxes: bwrap (Linux namespaces), a token-filtering relay, and a fail-closed self-test."""

from .core import (
    ENV_ALLOW,
    HIDDEN,
    Sandbox,
    SandboxError,
    ensure_bwrap,
    filter_env,
    open_sandbox,
    resolve_backend,
)
from .relay import Relay

__all__ = ["ENV_ALLOW", "HIDDEN", "Relay", "Sandbox", "SandboxError", "ensure_bwrap",
           "filter_env", "open_sandbox", "require_ok", "resolve_backend", "self_test"]


def self_test(*a, **kw):
    """amp.kernel.sandbox.selftest.self_test, imported lazily: it carries the probe source."""
    from .selftest import self_test as f

    return f(*a, **kw)


def require_ok(report):
    """amp.kernel.sandbox.selftest.require_ok."""
    from .selftest import require_ok as f

    return f(report)
