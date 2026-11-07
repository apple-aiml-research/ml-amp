#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The agent side: which model, its reasoning setting, the prompt template, how it sees, limits."""

from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from typing import Any, Protocol

PROVIDERS = ("anthropic", "gemini", "openai-chat")
REASONING_KINDS = ("effort", "thinking", "none")
VISIONS = ("image", "vdm_strict")

CONTEXT_WINDOWS: dict[str, int] = {
    "claude-opus-5-5": 1_000_000,
    "gemini-4-argon": 1_048_576,
    "Qwen/Qwen3.8-Flash-Next": 262_144,
}


@dataclass(frozen=True)
class ModelSpec:
    """One callable model: what to send where. Registry: amp.agents.models."""

    key: str
    provider: str
    model_id: str
    endpoint: str = ""
    context_tokens: int | None = None
    reasoning_kind: str = "none"
    reasoning_levels: tuple[str, ...] = ()
    default_reasoning: str | None = None
    price_per_mtok: dict[str, float] | None = None
    supports_images: bool = True
    notes: str = ""
    label: str = ""

    def __post_init__(self):
        if self.provider not in PROVIDERS:
            raise ValueError(f"{self.key}: provider {self.provider!r} not in {PROVIDERS}")
        if self.reasoning_kind not in REASONING_KINDS:
            raise ValueError(f"{self.key}: reasoning_kind {self.reasoning_kind!r}")
        object.__setattr__(self, "reasoning_levels", tuple(self.reasoning_levels))


@dataclass(frozen=True)
class AgentProfile:
    """Everything about the model side of one harness configuration."""

    model: ModelSpec
    reasoning: str | None = None
    prompt: str = "amp1"
    max_turns: int = 600
    wall_s: float = 21600.0
    max_cost_usd: float = 200.0
    max_tokens: int | None = None
    endpoint: str = ""
    extra: dict[str, Any] = field(default_factory=dict)
    vision: str = "image"
    vdm_model: str = ""

    def __post_init__(self):
        if self.vision not in VISIONS:
            raise ValueError(f"vision {self.vision!r} not in {VISIONS}")
        if (self.vision != "image") != bool(self.vdm_model):
            raise ValueError("vdm_model is set exactly when vision is not 'image'")
        m = self.model
        if self.reasoning is None:
            return
        if m.reasoning_kind == "none":
            raise ValueError(f"{m.key} takes no reasoning setting, got {self.reasoning!r}")
        if m.reasoning_levels and self.reasoning not in m.reasoning_levels:
            raise ValueError(f"{m.key}: reasoning {self.reasoning!r} not in {m.reasoning_levels}")

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        if d["vision"] == "image":
            del d["vision"], d["vdm_model"]
        return d

    @property
    def vision_label(self) -> str:
        """"" for an image harness configuration, else <vision>-<vdm_model>: the tail of the harness configuration id."""
        return f"{self.vision}-{self.vdm_model}" if self.vision != "image" else ""

    @property
    def label(self) -> str:
        return f"{self.model.key}/{self.reasoning}" if self.reasoning else self.model.key

    @property
    def resolved_endpoint(self) -> str:
        return self.endpoint or self.model.endpoint

    @property
    def endpoint_class(self) -> str:
        """The concurrency-cap key: one per endpoint URL, else per provider and model."""
        ep = self.resolved_endpoint
        if ep:
            return f"url:{ep}"
        return f"{self.model.provider}:{self.model.model_id}"


class LLMClient(Protocol):
    """What the agent loop calls."""

    model: str
    context_tokens: int | None
    thinking: str | None

    def create(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> Any: ...


@dataclass(frozen=True)
class RuntimeEnv:
    """Where the agent process runs."""

    where: str
