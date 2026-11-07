#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Model credentials: which environment variables carry them, and the trace redactor's list."""

from __future__ import annotations

import os
import re

from amp.agents.llm.base import LLMError
from amp.agents.modelconfig import MODELS_FILE_ENV, key_envs

MODEL_ENV_PREFIXES = ("ANTHROPIC_", "OPENAI_", "GEMINI_")
HEADERS_ENV = re.compile(r"^AMP_[A-Z0-9_]+_HEADERS$")
CA_FILE_ENV = "SSL_CERT_FILE"


class ProviderKeyError(LLMError):
    """A model's API key variable is unset; the message names the variable, never a value."""


def _model_names(env) -> set[str]:
    keys = set(key_envs(env))
    return {k for k in env if k.startswith(MODEL_ENV_PREFIXES) or k == MODELS_FILE_ENV
            or HEADERS_ENV.match(k) or k in keys}


def runner_env_keep(environ=None) -> tuple[str, ...]:
    """The names in environ (default os.environ) the agent runner inherits: the vendors' model
    variables, $AMP_MODELS_FILE, AMP_*_HEADERS, the models file's key variables, SSL_CERT_FILE."""
    env = os.environ if environ is None else environ
    names = _model_names(env)
    if CA_FILE_ENV in env:
        names.add(CA_FILE_ENV)
    return tuple(sorted(names))


def register_secret(value: str | None) -> bool:
    """Tell the trace redactor (and so agent.log's filter) about a secret; True if there was one."""
    if not value:
        return False
    from amp.trace.redact import register_secret as reg

    reg(value)
    return True
