#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""build_llm: an AgentProfile to the client the agent loop calls."""

from __future__ import annotations

import os
from typing import Any

from amp.agents import modelconfig
from amp.agents.context_windows import context_window
from amp.agents.credentials import ProviderKeyError, register_secret
from amp.agents.llm.base import SHARED, TRANSIENT_ATTEMPTS_ENV
from amp.agents.models import DEGENERATE_RESAMPLES, FALLBACK_SOURCE, MAX_EFFORT
from amp.contracts.model import AgentProfile, LLMClient, RuntimeEnv

SECRET_HEADER = ("authorization", "key", "token", "secret", "cookie")


def per_token(price_per_mtok: dict | None) -> dict | None:
    """USD per 1M tokens to USD per token; a tier's threshold stays in tokens."""
    if price_per_mtok is None:
        return None
    out = {k: v / 1e6 for k, v in price_per_mtok.items() if k not in ("threshold", "above")}
    if "above" in price_per_mtok:
        out["threshold"] = price_per_mtok["threshold"]
        out["above"] = per_token(price_per_mtok["above"])
    return out


def build_llm(profile: AgentProfile, runtime: RuntimeEnv | None = None, *,
              environ=None) -> LLMClient:
    """The client for profile.model.provider; every one returns amp.agents.llm.base.Turn."""
    env = os.environ if environ is None else environ
    m = profile.model
    kw: dict[str, Any] = {}
    if profile.max_tokens is not None:
        kw["max_tokens"] = profile.max_tokens
    reasoning = profile.reasoning or m.default_reasoning
    budget = env.get(TRANSIENT_ATTEMPTS_ENV)
    if budget:
        kw["transient_attempts"] = None if budget == SHARED else int(budget)
    r = modelconfig.resolve(m, env, endpoint=profile.resolved_endpoint)
    if r.key_required and not r.api_key:
        raise ProviderKeyError(f"{m.key}: set {r.api_key_env} (the {r.wire} API key)")
    register_secret(r.api_key)
    for k, v in r.headers.items():
        if any(s in k.lower() for s in SECRET_HEADER):
            register_secret(v)
    kw.update(api_key=r.api_key, api_key_header=r.api_key_header, extra_headers=r.headers,
              cost_header=r.cost_header)
    price = per_token(r.price_per_mtok)
    if m.provider == "openai-chat":
        from amp.agents.llm.openai_chat import OpenAIChatClient

        if reasoning is not None:
            kw["thinking"] = reasoning
        llm = OpenAIChatClient(r.served_model, base_url=r.base_url,
                               context_tokens=r.context_tokens, price=price,
                               max_effort=MAX_EFFORT.get(m.model_id),
                               degenerate_resamples=DEGENERATE_RESAMPLES.get(m.model_id),
                               family=m.model_id, **kw)
    elif m.provider == "anthropic":
        from amp.agents.llm.anthropic import AnthropicClient

        llm = AnthropicClient(reasoning, model=r.served_model, base_url=r.base_url, price=price,
                              **kw)
    elif m.provider == "gemini":
        from amp.agents.llm.gemini import GeminiClient

        llm = GeminiClient(r.served_model, thinking=reasoning or "off", base_url=r.base_url,
                           price=price, **kw)
    else:
        raise ValueError(f"{m.key}: no client for provider {m.provider!r}")
    llm.context_window = context_window(m.model_id) or llm.context_window
    llm.model_config = r.describe()
    if r.estimated:
        llm.computed_source = FALLBACK_SOURCE
    return llm


def describe_llm(llm) -> dict:
    """Where the client sends and how it authenticates, without secrets."""
    out = {"provider": llm.provider}
    cfg = getattr(llm, "model_config", None)
    if cfg:
        out["model_config"] = cfg
    url = getattr(llm, "url", None)
    if url is not None and hasattr(url, "geturl"):
        out["endpoint"] = url._replace(query="").geturl()
    return out
