#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Anthropic Messages client: streamed calls with adaptive thinking and an effort."""

from __future__ import annotations

import copy
import json
import re

from amp.agents.context_windows import context_window
from amp.agents.llm.base import (
    ContextOverflow,
    HTTPClient,
    Retry,
    Turn,
    iter_sse,
    parse_ratelimit,
    per_token_cost,
)

BASE_URL = "https://api.anthropic.com"
PATH = "/v1/messages"
MODEL = "claude-opus-5-5"
KEY_HEADER = "x-api-key"
ANTHROPIC_VERSION = "2023-06-01"
ANTHROPIC_BETA = "thinking-binding-controls-2026-08-01"
EFFORTS = ("low", "medium", "high", "xhigh", "max")
OPUS_45 = "claude-opus-4-5-20251101"
OPUS_45_BETA = "effort-2025-11-24,interleaved-thinking-2025-05-14"
OPUS_45_EFFORTS = ("low", "medium", "high")
OPUS_45_RE = re.compile(r"(?:^|[./])claude-opus-4-5(?:$|[-@:])")
PRICE = {"input": 4e-6, "output": 20e-6, "cache_read": 0.2e-6, "cache_write": 5e-6}
CACHE = {"type": "ephemeral"}
CONVERSATION_KEYS = ("system", "messages", "tools")
OVERFLOW = re.compile(r"prompt is too long:?\s*(\d+)\s*tokens|exceed context limit:?\s*(\d+)",
                      re.IGNORECASE)


def is_opus_45(model: str) -> bool:
    """True when model names Claude Opus 4.5, which gets the extended-thinking body."""
    return OPUS_45_RE.search(model or "") is not None


def _strip_cache(blocks):
    for b in blocks:
        if isinstance(b, dict):
            b.pop("cache_control", None)


def with_cache_control(system: list[dict], messages: list[dict], tools: list[dict]):
    """Copies with breakpoints on system[0], the last tool, and the last block of the newest
    user turn and of the user turn before it (4 at most)."""
    system, messages, tools = copy.deepcopy(system), copy.deepcopy(messages), copy.deepcopy(tools)
    _strip_cache(system)
    _strip_cache(tools)
    for m in messages:
        if isinstance(m.get("content"), list):
            _strip_cache(m["content"])
    if system:
        system[0]["cache_control"] = dict(CACHE)
    if tools:
        tools[-1]["cache_control"] = dict(CACHE)
    users = [m for m in messages if m.get("role") == "user"]
    for m in users[-2:][::-1][: 4 - bool(system) - bool(tools)]:
        if isinstance(m["content"], str):
            m["content"] = [{"type": "text", "text": m["content"]}]
        if m["content"]:
            m["content"][-1]["cache_control"] = dict(CACHE)
    return system, messages, tools


def convert(system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
    """The conversation keys of a request body (replay rebuilds bodies from these)."""
    system, messages, tools = with_cache_control(system, messages, tools)
    out = {"system": system, "messages": messages}
    if tools:
        out["tools"] = tools
    return out


def _merge(dst: dict, src: dict) -> dict:
    for k, v in (src or {}).items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        elif v is not None or k not in dst:
            dst[k] = v
    return dst


class Stream:
    """Assembles a message from Anthropic SSE events; text, thinking and signatures are the
    concatenation of their deltas, exactly as sent."""

    def __init__(self):
        self.message: dict = {}
        self.blocks: dict[int, dict] = {}
        self.json_parts: dict[int, list[str]] = {}
        self.usage: dict = {}
        self.stop_reason = None
        self.done = False

    def feed(self, event: str, data: dict) -> None:
        kind = data.get("type") or event
        if kind == "message_start":
            self.message = data.get("message") or {}
            _merge(self.usage, self.message.get("usage") or {})
        elif kind == "content_block_start":
            blk = dict(data.get("content_block") or {})
            if blk.get("type") in ("tool_use", "server_tool_use"):
                blk["input"] = {}
                self.json_parts[data["index"]] = []
            self.blocks[data["index"]] = blk
        elif kind == "content_block_delta":
            blk = self.blocks.setdefault(data["index"], {})
            d = data.get("delta") or {}
            t = d.get("type")
            if t == "text_delta":
                blk["text"] = blk.get("text", "") + d.get("text", "")
            elif t == "thinking_delta":
                blk["thinking"] = blk.get("thinking", "") + d.get("thinking", "")
            elif t == "signature_delta":
                blk["signature"] = blk.get("signature", "") + d.get("signature", "")
            elif t == "input_json_delta":
                self.json_parts.setdefault(data["index"], []).append(d.get("partial_json", ""))
            elif t == "citations_delta":
                blk.setdefault("citations", []).append(d.get("citation"))
        elif kind == "content_block_stop":
            i = data["index"]
            if i in self.json_parts:
                raw = "".join(self.json_parts.pop(i))
                try:
                    self.blocks[i]["input"] = json.loads(raw) if raw.strip() else {}
                except ValueError:
                    self.blocks[i]["input"] = {}
                    self.blocks[i]["_partial_json"] = raw
        elif kind == "message_delta":
            d = data.get("delta") or {}
            if d.get("stop_reason"):
                self.stop_reason = d["stop_reason"]
            _merge(self.usage, data.get("usage") or {})
        elif kind == "message_stop":
            self.done = True
        elif kind == "error":
            err = data.get("error") or {}
            raise Retry(f"stream error: {err.get('type')}: {err.get('message')}",
                        status=529 if err.get("type") == "overloaded_error" else None, body=data)

    def native(self) -> list[dict]:
        return [copy.deepcopy(self.blocks[i]) for i in sorted(self.blocks)]

    def content(self) -> list[dict]:
        out = []
        for i in sorted(self.blocks):
            blk = {k: v for k, v in self.blocks[i].items() if v is not None}
            if "_partial_json" in blk:
                blk.pop("_partial_json")
                blk["input"] = {}
            if blk.get("type") == "thinking":
                blk.setdefault("thinking", "")
            if blk.get("type") == "text":
                blk.setdefault("text", "")
                if not blk.get("citations"):
                    blk.pop("citations", None)
            out.append(blk)
        return out


class AnthropicClient(HTTPClient):
    """Claude over the Messages API."""

    provider = "anthropic"

    def __init__(self, effort: str, *, model: str = MODEL, max_tokens: int = 64000,
                 base_url: str = BASE_URL, price: dict | None = None, **kw):
        efforts = OPUS_45_EFFORTS if is_opus_45(model) else EFFORTS
        if effort not in efforts:
            raise ValueError(f"effort must be one of {efforts}")
        kw.setdefault("api_key_header", KEY_HEADER)
        super().__init__(base_url.rstrip("/") + PATH, **kw)
        self.effort, self.model, self.max_tokens = effort, model, max_tokens
        if is_opus_45(model) and max_tokens <= 1024:
            raise ValueError(f"{model}: max_tokens must exceed 1024, its thinking budget is max_tokens - 1")
        self.price = PRICE if price is None else price
        self.context_tokens: int | None = None
        self.context_window = context_window(model)
        self.thinking: str | None = None

    @staticmethod
    def prompt_tokens(usage: dict) -> int | None:
        """The whole prompt: input_tokens leaves out the cached prefix, read or written."""
        u = usage or {}
        if not isinstance(u.get("input_tokens"), int):
            return None
        return sum(u.get(k) or 0 for k in ("input_tokens", "cache_read_input_tokens",
                                            "cache_creation_input_tokens"))

    def params(self) -> dict:
        if is_opus_45(self.model):
            return {"model": self.model, "max_tokens": self.max_tokens, "stream": True,
                    "thinking": {"type": "enabled", "budget_tokens": self.max_tokens - 1},
                    "output_config": {"effort": self.effort}}
        return {"model": self.model, "max_tokens": self.max_tokens, "stream": True,
                "thinking": {"type": "adaptive", "display": "summarized",
                             "block_binding": {"prefix_mismatch_behavior": "drop_block"}},
                "output_config": {"effort": self.effort}}

    def build(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
        return {**self.params(), **convert(system, messages, tools)}

    def headers(self, force_token: bool = False) -> dict:
        return self.request_headers({"anthropic-version": ANTHROPIC_VERSION,
                                     "anthropic-beta": OPUS_45_BETA if is_opus_45(self.model)
                                     else ANTHROPIC_BETA,
                                     "content-type": "application/json",
                                     "accept": "text/event-stream"})

    def http_error(self, status, raw, msg, headers):
        if status in (400, 413):
            m = OVERFLOW.search(raw)
            if m:
                return ContextOverflow(msg, status, raw, prompt_tokens=int(m.group(1) or m.group(2)))
        return super().http_error(status, raw, msg, headers)

    def parse(self, resp, headers: dict, rid: str, body: dict) -> Turn:
        st = Stream()
        resp.partial_usage = st.usage
        for event, data in iter_sse(resp):
            try:
                payload = json.loads(data)
            except ValueError:
                continue
            if isinstance(payload, dict):
                st.feed(event or "", payload)
            if st.done:
                break
        if not st.done:
            raise Retry("stream ended before message_stop")
        cost, source = self.header_cost(headers), "header"
        if cost is None:
            cost, source = self.computed(st.usage, per_token_cost(st.usage, self.price))
        content, native = st.content(), st.native()
        return Turn(content=content, stop_reason=st.stop_reason or "", usage=st.usage,
                    cost_usd=cost, request_id=rid, latency_s=0.0, attempts=0,
                    ratelimit=parse_ratelimit(headers), cost_source=source,
                    message_id=st.message.get("id", ""),
                    native=None if native == content else native)
