#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""An OpenAI-compatible chat client (self-hosted sglang or vLLM), speaking Anthropic-shaped turns:
tool calls are parsed from Qwen XML or hermes JSON, reasoning_content becomes thinking."""

from __future__ import annotations

import ast
import base64
import json
import math
import re
import struct
import time
import uuid

from amp.agents.context_windows import context_window
from amp.agents.llm.base import (
    ContextOverflow,
    HTTPClient,
    LLMError,
    Retry,
    Turn,
    iter_sse,
    per_token_cost,
    retry_after,
)

BASE_URL = "http://127.0.0.1:30000/v1"
PATH = "/chat/completions"
KEY_HEADER = "authorization"

THINKING = ("off", "on", "max")
THINKING_LOW = ("off", "low", "on", "max")
EFFORTS = ("low", "high", "max")
EFFORT_MAP = {lvl: lvl for lvl in EFFORTS}
THINKING_BUDGET = 131072
KIMI_K3 = "kimi-k3"
GLM_53_FLASH = "zai-org/GLM-5.3-Flash"
DEEPSEEK_V41_FLASH = "deepseek-ai/DeepSeek-V4.1-Flash-1017"
DEEPSEEK_BUDGET = 262144
FAMILY: dict[str, dict] = {
    KIMI_K3: {"efforts": EFFORT_MAP, "native_history": True, "max_tokens": THINKING_BUDGET},
    GLM_53_FLASH: {"efforts": EFFORT_MAP, "native_history": True,
                   "max_tokens": THINKING_BUDGET,
                   "chat_template_kwargs": {"clear_thinking": False}},
    DEEPSEEK_V41_FLASH: {"efforts": EFFORT_MAP, "native_history": True,
                         "max_tokens": DEEPSEEK_BUDGET,
                         "chat_template_kwargs": {"thinking": True}},
}
NATIVE = "_native"
DEFAULT_CONTEXT = 262144
DEFAULT_MAX_TOKENS = 64000
CONTEXT_MARGIN = 256
MIN_COMPLETION = 4096
OVERFLOW_RE = re.compile(r"maximum context length of (\d+) tokens.*?(\d+) tokens from the input",
                         re.DOTALL)
INPUT_TOO_LONG_RE = re.compile(r"The input \((\d+) tokens\) is longer than the model's context "
                               r"length")
TOOL_CALL_RE = re.compile(r"<tool_call>(.*?)</tool_call>", re.DOTALL)
FUNCTION_RE = re.compile(r"<function=([^>\s]+)\s*>(.*?)(?:</function>|\Z)", re.DOTALL)
PARAM_RE = re.compile(r"<parameter=([^>\s]+)\s*>(.*?)(?:</parameter>|(?=<parameter=)|\Z)",
                      re.DOTALL)
MALFORMED = "_malformed_tool_call"
ERROR_PREFIX = "[the tool call failed]\n"
UNKNOWN_IMAGE_TOKENS = 1024
MIN_IMAGE_SIDE = 32
MAX_IMAGE_ASPECT = 150.0
CONVERSATION_KEYS = ("messages", "tools")
BANG = "!"
BANG_LEAD = 32
BANG_RUN = 256
DEGENERATE = "degenerate"
LEAD_RULE, RUN_RULE = f"lead{BANG_LEAD}", f"run{BANG_RUN}"


def image_size(b64: str) -> tuple[int, int] | None:
    """(width, height) from a base64 PNG or JPEG header, else None."""
    try:
        head = base64.b64decode(b64[:32])
        if head[:8] == b"\x89PNG\r\n\x1a\n":
            return struct.unpack(">II", head[16:24])
        if head[:2] != b"\xff\xd8":
            return None
        n = min(len(b64), 1 << 18) // 4 * 4
        data = base64.b64decode(b64[:n])
        i = 2
        while i + 9 < len(data):
            if data[i] != 0xFF:
                i += 1
                continue
            m = data[i + 1]
            if m == 0xFF:
                i += 1
                continue
            if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
                i += 2
                continue
            if 0xC0 <= m <= 0xCF and m not in (0xC4, 0xC8, 0xCC):
                h, w = struct.unpack(">HH", data[i + 5:i + 9])
                return w, h
            i += 2 + struct.unpack(">H", data[i + 2:i + 4])[0]
    except (ValueError, struct.error):
        return None
    return None


def _coerce(raw: str, schema: dict | None):
    types = (schema or {}).get("type")
    types = types if isinstance(types, list) else [types]
    if schema is None or "string" in types or types == [None]:
        return raw
    try:
        return json.loads(raw)
    except ValueError:
        pass
    try:
        return ast.literal_eval(raw)
    except (ValueError, SyntaxError, TypeError, MemoryError, RecursionError):
        return raw


def _strip_nl(s: str) -> str:
    if s.startswith("\r\n"):
        s = s[2:]
    elif s.startswith("\n"):
        s = s[1:]
    if s.endswith("\r\n"):
        s = s[:-2]
    elif s.endswith("\n"):
        s = s[:-1]
    return s


def _call_id() -> str:
    return "call_" + uuid.uuid4().hex[:12]


def parse_call(inner: str, schemas: dict) -> dict:
    """One tool_use block from the inside of a <tool_call>: Qwen XML, else hermes JSON."""
    fm = FUNCTION_RE.search(inner)
    if fm:
        name = fm.group(1).strip()
        props = (schemas.get(name) or {}).get("properties") or {}
        inp = {}
        for pm in PARAM_RE.finditer(fm.group(2)):
            key = pm.group(1).strip()
            inp[key] = _coerce(_strip_nl(pm.group(2)), props.get(key))
        return {"type": "tool_use", "id": _call_id(), "name": name, "input": inp}
    try:
        obj = json.loads(inner.strip())
        args = obj.get("arguments", obj.get("parameters", {}))
        if isinstance(args, str):
            args = json.loads(args) if args.strip() else {}
        if isinstance(obj.get("name"), str) and isinstance(args, dict):
            return {"type": "tool_use", "id": _call_id(), "name": obj["name"], "input": args}
    except (ValueError, AttributeError):
        pass
    return {"type": "tool_use", "id": _call_id(), "name": MALFORMED, "input": {},
            "_raw_call": inner}


def parse_content(text: str, schemas: dict) -> tuple[str, list[dict], bool]:
    """(visible text, tool_use blocks, whether the last call was left unclosed)."""
    calls, pieces, pos = [], [], 0
    for m in TOOL_CALL_RE.finditer(text):
        pieces.append(text[pos:m.start()])
        calls.append(parse_call(m.group(1), schemas))
        pos = m.end()
    rest = text[pos:]
    unclosed = False
    k = rest.find("<tool_call>")
    if k >= 0:
        pieces.append(rest[:k])
        blk = parse_call(rest[k + len("<tool_call>"):], schemas)
        blk["input"] = {}
        blk.pop("_raw_call", None)
        calls.append(blk)
        unclosed = True
    else:
        pieces.append(rest)
    return "".join(pieces).strip(), calls, unclosed


def render_call(name: str, inp: dict) -> str:
    """A tool call in the Qwen XML format the models emit."""
    parts = ["<tool_call>", f"<function={name}>"]
    for k, v in (inp or {}).items():
        val = v if isinstance(v, str) else json.dumps(v, ensure_ascii=False)
        parts += [f"<parameter={k}>", val, "</parameter>"]
    parts += ["</function>", "</tool_call>"]
    return "\n".join(parts)


def _text_of(blocks) -> str:
    if isinstance(blocks, str):
        return blocks
    return "\n\n".join(b.get("text", "") for b in blocks or [] if b.get("type") == "text")


def _image_refused(b64: str) -> str | None:
    """Why the Qwen-VL processor would 400 the whole request on this image, else None."""
    size = image_size(b64)
    if not size or min(size) <= 0:
        return None
    w, h = size
    if min(w, h) < MIN_IMAGE_SIDE or max(w, h) / min(w, h) > MAX_IMAGE_ASPECT:
        return (f"[image not sent: {w}x{h} px is outside the shapes this model accepts (sides of "
                f"{MIN_IMAGE_SIDE} px or more, aspect ratio up to {MAX_IMAGE_ASPECT:.0f})]")
    return None


def _parts(blocks) -> list[dict]:
    if isinstance(blocks, str):
        return [{"type": "text", "text": blocks}]
    out = []
    for b in blocks or []:
        t = b.get("type")
        if t == "text":
            out.append({"type": "text", "text": b.get("text", "")})
        elif t == "image":
            src = b.get("source") or {}
            why = _image_refused(src.get("data", ""))
            if why:
                out.append({"type": "text", "text": why})
                continue
            url = f"data:{src.get('media_type', 'image/png')};base64,{src.get('data', '')}"
            out.append({"type": "image_url", "image_url": {"url": url}})
    return out


def assistant_message(content) -> dict:
    """An OpenAI assistant message replaying our own Turn.content (tool calls as Qwen XML)."""
    if isinstance(content, str):
        return {"role": "assistant", "content": content}
    thinking = "\n\n".join(b.get("thinking", "") for b in content if b.get("type") == "thinking")
    text = "\n\n".join(b.get("text", "") for b in content if b.get("type") == "text")
    raw = next((b for b in content if "_raw_content" in b), None)
    calls = [b for b in content if b.get("type") == "tool_use"]
    if raw is not None and raw.get("_raw_text", "") == text and all(
            c.get("_rendered") == render_call(c.get("name"), c.get("input")) for c in calls):
        body = raw["_raw_content"]
    else:
        body = "\n\n".join([text] * bool(text) + [
            c.get("_raw_call_text") or render_call(c.get("name"), c.get("input"))
            if c.get("name") == MALFORMED else render_call(c.get("name"), c.get("input"))
            for c in calls])
    msg = {"role": "assistant", "content": body.strip()}
    if thinking.strip():
        msg["reasoning_content"] = thinking.strip()
    return msg


def _arguments(call: dict) -> str:
    """The call's arguments as the server sent them, unless they no longer match its input."""
    raw = call.get("_arguments")
    try:
        if isinstance(raw, str) and json.loads(raw) == call.get("input"):
            return raw
    except ValueError:
        pass
    return json.dumps(call.get("input") or {}, ensure_ascii=False)


def native_message(content: list[dict]) -> dict:
    """An assistant message as the server returned it (content, reasoning_content, tool_calls);
    a block the context window has since edited is sent as edited."""
    rec = next(b[NATIVE] for b in content if NATIVE in b)
    thinking = "\n\n".join(b.get("thinking", "") for b in content if b.get("type") == "thinking")
    text = "\n\n".join(b.get("text", "") for b in content if b.get("type") == "text")
    calls = [b for b in content if b.get("type") == "tool_use"]
    raw_reasoning = rec.get("reasoning_content")
    reasoning = raw_reasoning if raw_reasoning is not None and \
        raw_reasoning.strip() == thinking else thinking
    raw_ok = rec.get("text") == text and all("_arguments" in c for c in calls)
    msg = {"role": "assistant", "content": rec.get("content", text) if raw_ok else text}
    if reasoning:
        msg["reasoning_content"] = reasoning
    if calls:
        msg["tool_calls"] = [{"id": c.get("id"), "type": "function",
                              "function": {"name": c.get("name"), "arguments": _arguments(c)}}
                             for c in calls]
    return msg


def to_openai(system: list[dict], messages: list[dict], tools: list[dict]):
    """(messages, tools) in OpenAI chat form from the Anthropic-shaped request."""
    out = []
    sys_text = _text_of(system)
    if sys_text:
        out.append({"role": "system", "content": sys_text})
    for m in messages:
        if m.get("role") == "assistant":
            c = m.get("content")
            if isinstance(c, list) and any(NATIVE in b for b in c):
                out.append(native_message(c))
            else:
                out.append(assistant_message(c))
            continue
        content = m.get("content")
        if isinstance(content, str):
            out.append({"role": "user", "content": content})
            continue
        pending: list[dict] = []
        for b in content or []:
            if b.get("type") == "tool_result":
                if pending:
                    out.append({"role": "user", "content": pending})
                    pending = []
                parts = _parts(b.get("content")) or [{"type": "text", "text": "(no output)"}]
                if b.get("is_error"):
                    if parts[0]["type"] == "text":
                        parts[0] = {"type": "text", "text": ERROR_PREFIX + parts[0]["text"]}
                    else:
                        parts.insert(0, {"type": "text", "text": ERROR_PREFIX.strip()})
                out.append({"role": "tool", "tool_call_id": b.get("tool_use_id"),
                            "content": parts})
            else:
                pending += _parts([b])
        if pending:
            out.append({"role": "user", "content": pending})
    oa_tools = [{"type": "function",
                 "function": {"name": t["name"], "description": t.get("description", ""),
                              "parameters": t.get("input_schema") or {"type": "object"}}}
                for t in tools or []]
    return out, oa_tools


def convert(system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
    """The conversation keys of a request body (replay rebuilds bodies from these)."""
    msgs, oa_tools = to_openai(system, messages, tools)
    out: dict = {"messages": msgs}
    if oa_tools:
        out["tools"] = oa_tools
    return out


def usage(u: dict | None, max_tokens: int, finish_reason) -> dict:
    """Anthropic-shaped usage from an OpenAI usage chunk."""
    u = u or {}
    details = u.get("prompt_tokens_details") or {}
    return {"input_tokens": u.get("prompt_tokens") or 0,
            "output_tokens": u.get("completion_tokens") or 0,
            "cache_read_input_tokens": details.get("cached_tokens") or 0,
            "cache_creation_input_tokens": 0,
            "output_tokens_details": {"thinking_tokens": u.get("reasoning_tokens") or 0},
            "max_tokens": max_tokens, "finish_reason": finish_reason, "raw": u}


class BangGuard:
    """Flags a stream whose first `lead` characters or any `run` in a row are all "!"."""

    def __init__(self, lead: int = BANG_LEAD, run: int = BANG_RUN):
        self.lead, self.run = lead, run
        self.chars: dict[str, int] = {}
        self.runs: dict[str, int] = {}
        self.clean: dict[str, bool] = {}

    def feed(self, channel: str, text: str) -> tuple[str, str] | None:
        """(rule, channel) once this delta makes the stream degenerate, else None."""
        n, run = self.chars.get(channel, 0), self.runs.get(channel, 0)
        bang_led = self.clean.get(channel, True)
        why = None
        for c in text:
            n += 1
            run = run + 1 if c == BANG else 0
            bang_led = bang_led and c == BANG
            if why is None and bang_led and n == self.lead:
                why = (LEAD_RULE, channel)
            elif why is None and run == self.run:
                why = (RUN_RULE, channel)
        self.chars[channel], self.runs[channel], self.clean[channel] = n, run, bang_led
        return why


class SSE:
    """Assembles a chat completion from OpenAI SSE chunks; a guard sets degenerate."""

    def __init__(self, guard: BangGuard | None = None):
        self.id = ""
        self.created = None
        self.content: list[str] = []
        self.reasoning: list[str] = []
        self.tool_calls: dict[int, dict] = {}
        self.finish_reason = None
        self.usage: dict | None = None
        self.done = False
        self.guard = guard
        self.degenerate: tuple[str, str] | None = None
        self.deltas = 0

    def feed(self, data: dict) -> None:
        if data.get("object") == "error" or "error" in data:
            err = data.get("error") if isinstance(data.get("error"), dict) else data
            raise Retry(f"stream error: {err.get('type')}: {err.get('message')}",
                        status=err.get("code") if isinstance(err.get("code"), int) else None,
                        body=data)
        self.id = self.id or data.get("id") or ""
        self.created = self.created if self.created is not None else data.get("created")
        if data.get("usage"):
            self.usage = data["usage"]
        for ch in data.get("choices") or []:
            d = ch.get("delta") or {}
            if d.get("content"):
                self.content.append(d["content"])
            r = d.get("reasoning_content") or d.get("reasoning")
            if r:
                self.reasoning.append(r)
            if d.get("content") or r:
                self.deltas += 1
                if self.guard is not None and self.degenerate is None:
                    self.degenerate = (self.guard.feed("reasoning", r or "")
                                       or self.guard.feed("content", d.get("content") or ""))
            for tc in d.get("tool_calls") or []:
                slot = self.tool_calls.setdefault(tc.get("index", len(self.tool_calls)),
                                                  {"id": "", "name": "", "arguments": ""})
                slot["id"] = tc.get("id") or slot["id"]
                fn = tc.get("function") or {}
                slot["name"] += fn.get("name") or ""
                slot["arguments"] += fn.get("arguments") or ""
            if ch.get("finish_reason"):
                self.finish_reason = ch["finish_reason"]


class OpenAIChatClient(HTTPClient):
    """One streamed chat completion per call; degenerate_resamples bounds all-"!" resamples."""

    provider = "openai-chat"
    retry_status = frozenset({401, 408, 409, 429, 500, 502, 503, 504})
    refresh_status = frozenset({401})

    def __init__(self, model: str, *, base_url: str = BASE_URL, thinking: str = "off",
                 max_tokens: int | None = None, context_tokens: int | None = None,
                 max_effort: str | None = None, price: dict | None = None,
                 read_timeout_s: float = 900.0, degenerate_resamples: int | None = None,
                 efforts: dict[str, str] | None = None, native_history: bool | None = None,
                 chat_template_kwargs: dict | None = None, family: str | None = None, **kw):
        fam = FAMILY.get(family or model, {})
        efforts = fam.get("efforts") if efforts is None else efforts
        native_history = fam.get("native_history", False) if native_history is None \
            else native_history
        if chat_template_kwargs is None:
            chat_template_kwargs = fam.get("chat_template_kwargs")
        if max_tokens is None:
            max_tokens = fam.get("max_tokens")
        levels = tuple(efforts) if efforts else THINKING_LOW
        if thinking not in levels:
            raise ValueError(f"thinking must be one of {levels}")
        if not base_url:
            raise ValueError(f"no base_url for {model!r}")
        kw.setdefault("api_key_header", KEY_HEADER)
        super().__init__(base_url.rstrip("/") + PATH, read_timeout_s=read_timeout_s, **kw)
        self.model, self.thinking, self.max_effort = model, thinking, max_effort
        self.efforts = dict(efforts) if efforts else None
        self.native_history = native_history
        self.fixed_kwargs = dict(chat_template_kwargs or {})
        self.context_tokens: int | None = context_tokens or DEFAULT_CONTEXT
        self.context_window = context_window(model)
        self.max_tokens = max_tokens if max_tokens is not None else DEFAULT_MAX_TOKENS
        self.price = price
        self.base_url = base_url.rstrip("/")
        if degenerate_resamples is not None:
            self.resamples[DEGENERATE] = degenerate_resamples
        self._schemas: dict = {}

    def attempt_cost(self, usage: dict | None) -> float:
        if not usage or not self.price:
            return 0.0
        return self.computed(usage, per_token_cost(usage, self.price, inclusive=True), True)[0]

    @staticmethod
    def prompt_tokens(usage: dict) -> int | None:
        """prompt_tokens (input_tokens here), which already holds its cached_tokens."""
        n = (usage or {}).get("input_tokens")
        return n if isinstance(n, int) else None

    @staticmethod
    def estimate_image_tokens(w: int, h: int) -> int:
        return math.ceil(w / 32) * math.ceil(h / 32)

    def estimate_prompt_tokens(self, messages: list[dict], tools: list[dict]) -> int:
        """chars/3 for text plus the Qwen-VL patch count for each image."""
        chars, img = len(json.dumps(tools)), 0
        for m in messages:
            chars += 16
            c = m.get("content")
            if isinstance(c, str):
                chars += len(c)
            for p in c if isinstance(c, list) else []:
                if p.get("type") == "text":
                    chars += len(p.get("text", ""))
                elif p.get("type") == "image_url":
                    data = p["image_url"]["url"].partition(",")[2]
                    wh = image_size(data)
                    img += self.estimate_image_tokens(*wh) if wh else UNKNOWN_IMAGE_TOKENS
            chars += len(m.get("reasoning_content") or "")
            if m.get("tool_calls"):
                chars += len(json.dumps(m["tool_calls"]))
        return chars // 3 + img

    def plan_max_tokens(self, prompt_tokens: int) -> int:
        remaining = self.context_tokens - prompt_tokens - CONTEXT_MARGIN
        cap = self.max_tokens if self.max_tokens is not None else remaining
        return max(MIN_COMPLETION, min(cap, remaining))

    def template_kwargs(self) -> dict:
        if self.efforts is not None:
            return dict(self.fixed_kwargs)
        if self.thinking == "off":
            return {"enable_thinking": False}
        if self.thinking == "low":
            return {"enable_thinking": True, "reasoning_effort": "low"}
        if self.thinking == "max":
            kw = {"enable_thinking": True}
            if self.max_effort:
                kw["reasoning_effort"] = self.max_effort
            return kw
        return {}

    def build(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
        self._schemas = {t["name"]: t.get("input_schema") or {} for t in tools or []}
        conv = convert(system, messages, tools)
        body = {"model": self.model, "messages": conv["messages"],
                "max_tokens": self.plan_max_tokens(
                    self.estimate_prompt_tokens(conv["messages"], conv.get("tools", []))),
                "stream": True, "stream_options": {"include_usage": True}}
        if "tools" in conv:
            body["tools"] = conv["tools"]
        if self.efforts is not None:
            body["reasoning_effort"] = self.efforts[self.thinking]
        kw = self.template_kwargs()
        if kw:
            body["chat_template_kwargs"] = kw
        return body

    def headers(self, force_token: bool = False) -> dict:
        return self.request_headers({"content-type": "application/json",
                                     "accept": "text/event-stream"})

    def http_error(self, status, raw, msg, headers):
        if status == 400:
            m = OVERFLOW_RE.search(raw)
            if m:
                return ContextOverflow(msg, 400, raw, prompt_tokens=int(m.group(2)))
            m = INPUT_TOO_LONG_RE.search(raw)
            if m:
                return ContextOverflow(msg, 400, raw, prompt_tokens=int(m.group(1)))
        if status in self.retry_status or status >= 500:
            return Retry(msg, status, retry_after(headers.get("retry-after")), raw)
        return LLMError(msg, status, raw)

    def on_overflow(self, e: ContextOverflow, body: dict) -> bool:
        fit = self.context_tokens - (e.prompt_tokens or 0) - CONTEXT_MARGIN
        if e.prompt_tokens is None or fit < MIN_COMPLETION or fit >= body["max_tokens"]:
            return False
        body["max_tokens"] = fit
        return True

    def parse(self, resp, headers: dict, rid: str, body: dict) -> Turn:
        st = SSE(BangGuard() if DEGENERATE in self.resamples else None)
        for _, data in iter_sse(resp):
            if data.strip() == "[DONE]":
                st.done = True
                break
            try:
                payload = json.loads(data)
            except ValueError:
                continue
            if isinstance(payload, dict):
                st.feed(payload)
                if st.degenerate:
                    raise self.degenerate_retry(st, resp, rid, body["max_tokens"])
        if not st.done:
            raise Retry("stream ended before [DONE]")
        turn = self._turn(st, self._schemas, rid, body["max_tokens"])
        cost = self.header_cost(headers)
        if cost is not None:
            turn.cost_usd, turn.cost_source = cost, "header"
        elif self.price:
            turn.cost_usd, turn.cost_source = self.computed(
                turn.usage, per_token_cost(turn.usage, self.price, inclusive=True), True)
        return turn

    def degenerate_retry(self, st: SSE, rec, rid: str, max_tokens: int) -> Retry:
        """The Retry that cuts a degenerate stream; its partial output is never a turn."""
        elapsed = time.monotonic() - rec.t0
        decode_s = max(0.0, elapsed - (rec.ttft_s or 0.0))
        tokens = (st.usage or {}).get("completion_tokens") or st.deltas
        saved = decode_s / max(1, tokens) * max(0, max_tokens - tokens)
        left = self.remaining()
        if left is not None:
            saved = min(saved, max(0.0, left))
        chars = sum(map(len, st.reasoning)) + sum(map(len, st.content))
        rule, channel = st.degenerate
        info = {"rule": rule, "channel": channel, "response_id": st.id or rid,
                "created": st.created, "endpoint": self.base_url,
                "chars_received": chars, "tokens_received": tokens,
                "elapsed_s": round(elapsed, 3), "saved_s": round(saved, 1)}
        return Retry(f"degenerate output: rule {rule} in {channel} after {chars} chars, request "
                     f"{st.id or rid}", DEGENERATE, kind=DEGENERATE, info=info,
                     usage=usage(st.usage, max_tokens, st.finish_reason) if st.usage else None)

    def _turn(self, st: SSE, schemas: dict, rid: str, max_tokens: int) -> Turn:
        raw_content = "".join(st.content)
        raw_reasoning = "".join(st.reasoning)
        reasoning = raw_reasoning.strip()
        text, calls, unclosed = parse_content(raw_content, schemas)
        for i in sorted(st.tool_calls):
            tc = st.tool_calls[i]
            try:
                args = json.loads(tc["arguments"]) if tc["arguments"].strip() else {}
            except ValueError:
                args = {}
            calls.append({"type": "tool_use", "id": tc["id"] or _call_id(), "name": tc["name"],
                          "input": args if isinstance(args, dict) else {}})
            if self.native_history:
                calls[-1]["_arguments"] = tc["arguments"]
        content: list[dict] = []
        if reasoning:
            content.append({"type": "thinking", "thinking": reasoning})
        if text:
            content.append({"type": "text", "text": text})
        for c in calls:
            if c.get("name") == MALFORMED:
                c["_raw_call_text"] = "<tool_call>" + c.pop("_raw_call", "") + "</tool_call>"
            c["_rendered"] = render_call(c.get("name"), c.get("input"))
        content += calls
        if self.native_history and content:
            rec = {"text": text}
            if raw_content != text:
                rec["content"] = raw_content
            if raw_reasoning != reasoning:
                rec["reasoning_content"] = raw_reasoning
            content[0][NATIVE] = rec
        elif content and not st.tool_calls:
            anchor = next((b for b in content if b["type"] != "thinking"), None)
            if anchor is not None:
                anchor["_raw_content"], anchor["_raw_text"] = raw_content.strip(), text
        if st.finish_reason == "length" or unclosed:
            stop = "max_tokens"
        elif calls:
            stop = "tool_use"
        else:
            stop = "end_turn"
        native = {"reasoning_content": raw_reasoning, "content": raw_content,
                  "tool_calls": [st.tool_calls[i] for i in sorted(st.tool_calls)],
                  "finish_reason": st.finish_reason}
        return Turn(content=content, stop_reason=stop,
                    usage=usage(st.usage, max_tokens, st.finish_reason), cost_usd=0.0,
                    request_id=st.id or rid, latency_s=0.0, attempts=0, ratelimit={},
                    cost_source="self_hosted", message_id=st.id, native=native)
