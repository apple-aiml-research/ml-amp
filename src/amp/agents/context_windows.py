#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Each model's input-token limit by model_id, for display only."""

from __future__ import annotations

from amp.contracts.model import CONTEXT_WINDOWS as FROZEN

ADDED: dict[str, int] = {
    "claude-opus-5": 1_000_000,
    "claude-opus-4-5-20251101": 200_000,
    "claude-sonnet-5-5": 1_000_000,
    "claude-haiku-5-5": 1_000_000,
    "Qwen/Qwen3.6-35B-A3B": 262_144,
    "Qwen/Qwen3.8-27B": 262_144,
    "zai-org/GLM-5.3-Flash": 1_048_576,
    "deepseek-ai/DeepSeek-V4.1-Flash-1017": 1_048_576,
    "kimi-k3": 1_048_576,
}
CONTEXT_WINDOWS: dict[str, int] = {**FROZEN, **ADDED}


def context_window(model_id: str | None) -> int | None:
    """A model's input-token limit, None when it is not known (never a guess)."""
    return CONTEXT_WINDOWS.get(model_id or "")
