#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""LLM clients. Each speaks the Anthropic Messages shape to the loop and returns a base.Turn."""

from amp.agents.llm.base import ContextOverflow, LLMError, Observer, Turn


def converter(provider: str):
    """(convert, conversation keys) for a provider: what replay needs to rebuild a body."""
    from amp.agents.llm import anthropic, gemini, openai_chat

    return {"anthropic": (anthropic.convert, anthropic.CONVERSATION_KEYS),
            "gemini": (gemini.convert, gemini.CONVERSATION_KEYS),
            "openai-chat": (openai_chat.convert, openai_chat.CONVERSATION_KEYS)}[provider]


__all__ = ["ContextOverflow", "LLMError", "Observer", "Turn", "converter"]
