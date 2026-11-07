#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The model registry: every model the agent loop can drive, by key or "<provider>:<model_id>"."""

from __future__ import annotations

import dataclasses

from amp.agents import modelconfig
from amp.agents.llm import anthropic, gemini, openai_chat
from amp.contracts.model import PROVIDERS, VISIONS, AgentProfile, ModelSpec

OPUS_55_PRICE = {"input": 4.0, "output": 20.0, "cache_read": 0.2, "cache_write": 5.0}
OPUS_5_PRICE = {"input": 5.0, "output": 25.0, "cache_read": 0.5, "cache_write": 6.25}
OPUS_45_PRICE = {"input": 5.0, "output": 25.0, "cache_read": 0.5, "cache_write": 6.25}
SONNET_55_PRICE = {"input": 2.0, "output": 10.0, "cache_read": 0.1, "cache_write": 2.5}
SONNET_5_PRICE = {"input": 2.0, "output": 10.0, "cache_read": 0.2, "cache_write": 2.5}
HAIKU_55_PRICE = {"input": 0.1, "output": 0.5, "cache_read": 0.01, "cache_write": 0.125}
GEMINI_4A_PRICE = {"input": 4.0, "output": 20.0, "cache_read": 0.2}
QWEN_CONTEXT = 262144
LONG_CONTEXT = 1_048_576
MAX_EFFORT = {"Qwen/Qwen3.8-Flash-Next": "xhigh", "Qwen/Qwen3.8-27B": "xhigh"}
DEGENERATE_RESAMPLES = {"Qwen/Qwen3.6-35B-A3B": 3, "Qwen/Qwen3.8-Flash-Next": 3,
                        "Qwen/Qwen3.8-27B": 3}
ALWAYS_THINKS = ("Thinking cannot be off: low, high and max are sent as reasoning_effort on every "
                 "request")
FALLBACK_PRICE = modelconfig.FALLBACK_PRICE
FALLBACK_SOURCE = modelconfig.FALLBACK_SOURCE

MODELS: dict[str, ModelSpec] = {m.key: m for m in (
    ModelSpec(
        key="opus-5.5", label="Opus 5.5",
        provider="anthropic", model_id=anthropic.MODEL,
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.EFFORTS,
        default_reasoning="high", price_per_mtok=OPUS_55_PRICE,
        notes="Thinking cannot be off: always adaptive thinking plus output_config.effort."),
    ModelSpec(
        key="opus-5", label="Opus 5",
        provider="anthropic", model_id="claude-opus-5",
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.EFFORTS,
        default_reasoning="high", price_per_mtok=OPUS_5_PRICE,
        notes="Sent the Opus 5.5 body (adaptive thinking + effort)."),
    ModelSpec(
        key="opus-4.5", label="Opus 4.5",
        provider="anthropic", model_id=anthropic.OPUS_45,
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.OPUS_45_EFFORTS,
        default_reasoning="high", price_per_mtok=OPUS_45_PRICE,
        notes="Extended thinking (budget max_tokens - 1, interleaved) plus output_config.effort; "
              "no adaptive thinking and no block_binding."),
    ModelSpec(
        key="sonnet-5.5", label="Sonnet 5.5",
        provider="anthropic", model_id="claude-sonnet-5-5",
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.EFFORTS,
        default_reasoning="high", price_per_mtok=SONNET_55_PRICE,
        notes="Sent the Opus 5.5 body (adaptive thinking + effort)."),
    ModelSpec(
        key="sonnet-5", label="Sonnet 5",
        provider="anthropic", model_id="claude-sonnet-5",
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.EFFORTS,
        default_reasoning="high", price_per_mtok=SONNET_5_PRICE,
        notes="Sent the Opus 5.5 body (adaptive thinking + effort)."),
    ModelSpec(
        key="haiku-5.5", label="Haiku 5.5",
        provider="anthropic", model_id="claude-haiku-5-5",
        context_tokens=None, reasoning_kind="effort", reasoning_levels=anthropic.EFFORTS,
        default_reasoning="high", price_per_mtok=HAIKU_55_PRICE,
        notes="Sent the Opus 5.5 body (adaptive thinking + effort)."),
    ModelSpec(
        key="gemini-4-argon", label="Gemini 4 Argon",
        provider="gemini", model_id="gemini-4-argon",
        context_tokens=gemini.CONTEXT_TOKENS, reasoning_kind="thinking",
        reasoning_levels=gemini.THINKING, default_reasoning="high",
        price_per_mtok=GEMINI_4A_PRICE,
        notes="Thinking cannot be off: off and low both send thinkingLevel LOW; medium MEDIUM; "
              "high and max HIGH. Implicit caching, no cache-write charge."),
    ModelSpec(
        key="qwen3.8-flash-next", label="Qwen3.8 Flash Next",
        provider="openai-chat", model_id="Qwen/Qwen3.8-Flash-Next",
        context_tokens=262144, reasoning_kind="thinking",
        reasoning_levels=openai_chat.THINKING_LOW, default_reasoning="off", price_per_mtok=None,
        notes="Self-hosted (sglang or vLLM). low: reasoning_effort low; on: the template's "
              "default, xhigh; max: reasoning_effort xhigh (its maximum). A 64000-token "
              "completion at every level."),
    ModelSpec(
        key="qwen3.6-35b-a3b", label="Qwen3.6 35B-A3B",
        provider="openai-chat", model_id="Qwen/Qwen3.6-35B-A3B",
        context_tokens=QWEN_CONTEXT, reasoning_kind="thinking",
        reasoning_levels=openai_chat.THINKING, default_reasoning="off", price_per_mtok=None,
        notes="Self-hosted (sglang or vLLM). max: thinking on, a 64000-token completion like on "
              "and off; no effort control."),
    ModelSpec(
        key="qwen3.8-27b", label="Qwen3.8 27B",
        provider="openai-chat", model_id="Qwen/Qwen3.8-27B",
        context_tokens=QWEN_CONTEXT, reasoning_kind="thinking",
        reasoning_levels=openai_chat.THINKING_LOW, default_reasoning="off", price_per_mtok=None,
        notes="Self-hosted (sglang or vLLM). low: reasoning_effort low; on: the template's "
              "default, xhigh; max: reasoning_effort xhigh (its maximum; the template refuses "
              "high). A 64000-token completion at every level."),
    ModelSpec(
        key="kimi-k3", label="Kimi K3",
        provider="openai-chat", model_id=openai_chat.KIMI_K3,
        context_tokens=LONG_CONTEXT, reasoning_kind="effort",
        reasoning_levels=openai_chat.EFFORTS, default_reasoning="high", price_per_mtok=None,
        notes=f"Self-hosted (sglang or vLLM). {ALWAYS_THINKS}. Turns replay with their "
              "reasoning_content and tool_calls unchanged. A 131072-token completion."),
    ModelSpec(
        key="glm-5.3-flash", label="GLM 5.3 Flash",
        provider="openai-chat", model_id=openai_chat.GLM_53_FLASH,
        context_tokens=LONG_CONTEXT, reasoning_kind="effort",
        reasoning_levels=openai_chat.EFFORTS, default_reasoning="high", price_per_mtok=None,
        notes=f"Self-hosted (sglang or vLLM). {ALWAYS_THINKS}. Preserved thinking "
              "(clear_thinking false): turns replay with their reasoning_content and tool_calls. "
              "A 131072-token completion."),
    ModelSpec(
        key="deepseek-v4.1-flash", label="DeepSeek V4.1 Flash",
        provider="openai-chat", model_id=openai_chat.DEEPSEEK_V41_FLASH,
        context_tokens=LONG_CONTEXT, reasoning_kind="effort",
        reasoning_levels=openai_chat.EFFORTS, default_reasoning="high", price_per_mtok=None,
        notes=f"Self-hosted (sglang or vLLM). {ALWAYS_THINKS}, with chat_template_kwargs "
              "thinking true (the server defaults to chat mode). Turns replay with their "
              "reasoning_content and tool_calls unchanged. A 262144-token completion."),
)}

_ADHOC_BASE = {"anthropic": "opus-5.5", "gemini": "gemini-4-argon",
               "openai-chat": "qwen3.8-flash-next"}


def _adhoc(key: str, provider: str, model_id: str, why: str) -> ModelSpec:
    base = MODELS[_ADHOC_BASE[provider]]
    note = f"{why}; reasoning levels assumed from {base.key}"
    if provider in FALLBACK_PRICE:
        note += "; unpriced unless the models file prices it (else the dearest list rate)"
    ctx = base.context_tokens if provider == "gemini" else None
    return dataclasses.replace(base, key=key, model_id=model_id, label="", endpoint="",
                               price_per_mtok=None, context_tokens=ctx, notes=note)


def get_model(key: str, environ=None) -> ModelSpec:
    """A registry entry, a key the models file defines, or an ad hoc "<provider>:<model_id>"."""
    if key in MODELS:
        return MODELS[key]
    provider, sep, model_id = key.partition(":")
    if not sep or not model_id:
        f = modelconfig.models_file(environ)
        entry = f.models.get(key) if f else None
        if entry is not None:
            wire = f.servers[entry.server].wire
            return _adhoc(key, wire, entry.served_model or key, f"from {f.path}")
        raise KeyError(f"unknown model {key!r}: one of {sorted(MODELS)}, a key of the models "
                       f"file, or '<provider>:<model_id>' with provider in {PROVIDERS}")
    if provider not in PROVIDERS:
        raise KeyError(f"{key!r}: provider {provider!r} not in {PROVIDERS}")
    for m in MODELS.values():
        if m.provider == provider and m.model_id == model_id:
            return m
    return _adhoc(key, provider, model_id, "ad hoc")


def profile(model: str | ModelSpec, reasoning: str | None = None, **overrides) -> AgentProfile:
    """An AgentProfile for `model` (reasoning None: its default)."""
    spec = model if isinstance(model, ModelSpec) else get_model(model)
    return AgentProfile(model=spec, reasoning=reasoning or spec.default_reasoning, **overrides)


DEFAULT_VDM_MODEL = "opus-5.5"


def vision_fields(vision: str | None, vdm_model: str | None = None) -> dict[str, str]:
    """AgentProfile's vision fields for a CLI's --vision / --vdm-model; ValueError when the
    vision model is unknown or takes no images."""
    vision = vision or "image"
    if vision == "image":
        if vdm_model:
            raise ValueError("--vdm-model needs --vision vdm_strict")
        return {}
    if vision not in VISIONS:
        raise ValueError(f"vision {vision!r} not in {', '.join(VISIONS)}")
    spec = get_model(vdm_model or DEFAULT_VDM_MODEL)
    if not spec.supports_images:
        raise ValueError(f"vdm model {spec.key} takes no images")
    return {"vision": vision, "vdm_model": spec.key}


def vdm_profile(prof: AgentProfile) -> AgentProfile:
    """The vision model's own profile: its registry defaults."""
    spec = get_model(prof.vdm_model)
    return profile(spec)


STRICT_REFUSED = ("api/frame/",)
PERCEPTION_ROUTES = ("api/agent/object_points/", "api/agent/segment", "api/agent/define_object",
                     "api/agent/forget_object", "api/agent/reperceive", "api/scene", "api/assist/")
STRICT_DROPPED = ("objects", "perception_notes", "perception_seq", "held")
STRICT_SERVER_ENV = {"drop_depth": "the rgb_only overlay",
                     "hide_table_height": "amp1_basic or amp1_assistive",
                     "hide_calibration": "the no_calibration overlay"}
STRICT_HARNESS = "amp1_basic_noperc_rgb_only_nocal"
_TRUE = ("1", "true", "yes", "on")


def strict_harness_problem(spec, policy: dict) -> str | None:
    """Why harness `spec` (with its routes.json `policy`) cannot run vision vdm_strict, or None."""
    name = getattr(spec, "name", "?")
    drops = policy.get("response_drop") or {}
    if spec.server.obstacles != "table":
        return (f"{name}: its collision checks test against the perceived scene (obstacles "
                f"{spec.server.obstacles!r}), which would reveal the objects; vdm_strict needs a "
                f"harness without perception, such as {STRICT_HARNESS}: a harness with the "
                "no_perception overlay")
    for rule in policy.get("rules") or []:
        route = str(rule.get("path") or rule.get("key") or rule.get("pattern") or "")
        if route.startswith(PERCEPTION_ROUTES):
            return f"{name}: it serves the perception route {route}; vdm_strict needs none"
    dropped = set(drops.get("api/agent/observation") or ())
    missing = [k for k in STRICT_DROPPED if k not in dropped]
    if missing:
        return (f"{name}: its observations keep {', '.join(missing)}; vdm_strict needs them "
                "dropped (as the no_perception overlay does)")
    env = spec.server.env
    unset = [f"{k} ({w})" for k, w in STRICT_SERVER_ENV.items()
             if str(env.get(k, "")).lower() not in _TRUE]
    if unset:
        return (f"{name}: vdm_strict needs depth, the table height and camera calibration withheld "
                f"from every harness configuration; missing: {', '.join(unset)}; use {STRICT_HARNESS}")
    return None


def strict_problem(harness: str, registry=None) -> str | None:
    """strict_harness_problem for a registered harness (amp.harnesses.registry by default)."""
    import json

    if registry is None:
        from amp.harnesses.registry import DEFAULT as registry
    spec = registry.load(harness)
    return strict_harness_problem(spec, json.loads(registry.path(spec, spec.route_policy)
                                                   .read_text()))
