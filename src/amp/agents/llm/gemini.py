#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Gemini generateContent client, speaking the loop's Anthropic-shaped turns."""

from __future__ import annotations

import copy
import http.client
import json
import random
import re
import time
import uuid

from amp.agents.context_windows import context_window
from amp.agents.llm.base import (
    TRANSIENT_MARK,
    Aborted,
    BudgetExhausted,
    ContextOverflow,
    HTTPClient,
    LLMError,
    Retry,
    Turn,
    budget_exhausted,
    log,
    parse_ratelimit,
    per_token_cost,
    retry_after,
)

BASE_URL = "https://generativelanguage.googleapis.com/v1beta"
MODEL = "gemini-4-argon"
KEY_HEADER = "x-goog-api-key"
CONTEXT_TOKENS = 1_048_576
THINKING = ("off", "low", "medium", "high", "max")
LEVEL = {"off": "LOW", "low": "LOW", "medium": "MEDIUM", "high": "HIGH", "max": "HIGH"}
MEDIA = {"low": "MEDIA_RESOLUTION_LOW", "medium": "MEDIA_RESOLUTION_MEDIUM",
         "high": "MEDIA_RESOLUTION_HIGH"}
IMAGE_TOKENS = {"low": 264, "medium": 527, "high": 1100}
PRICE = {"input": 4e-6, "output": 20e-6, "cache_read": 0.2e-6}
SKIP_SIGNATURE = "skip_thought_signature_validator"
STASH = "_gemini_part"
SCHEMA_KEYS = {"type", "format", "title", "description", "nullable", "enum", "items",
               "properties", "required", "minItems", "maxItems", "minLength", "maxLength",
               "minimum", "maximum", "minProperties", "maxProperties", "pattern", "default",
               "anyOf", "propertyOrdering", "example"}
OVERFLOW = re.compile(r"input token count.*exceeds the maximum|exceeds the maximum number of "
                      r"tokens|context (window|length)", re.IGNORECASE)
RETRY_FINISH = {"MALFORMED_FUNCTION_CALL", "UNEXPECTED_TOOL_CALL", "OTHER", "RECITATION",
                "FINISH_REASON_UNSPECIFIED", "TOO_MANY_TOOL_CALLS"}
SAFETY_FINISH = {"SAFETY", "BLOCKLIST", "PROHIBITED_CONTENT", "SPII", "IMAGE_SAFETY",
                 "IMAGE_PROHIBITED_CONTENT", "LANGUAGE"}
QUOTA_STATUS = frozenset({429, 529})
QUOTA_BODY = re.compile(r"RESOURCE_EXHAUSTED")
OVERLOADED = re.compile(r"overloaded", re.IGNORECASE)
ERROR_STATUS = {
    "INVALID_ARGUMENT": 400, "FAILED_PRECONDITION": 400, "OUT_OF_RANGE": 400,
    "UNAUTHENTICATED": 401, "PERMISSION_DENIED": 403, "NOT_FOUND": 404, "ABORTED": 409,
    "ALREADY_EXISTS": 409, "RESOURCE_EXHAUSTED": 429, "CANCELLED": 499, "UNKNOWN": 500,
    "INTERNAL": 500, "DATA_LOSS": 500, "UNIMPLEMENTED": 501, "UNAVAILABLE": 503,
    "DEADLINE_EXCEEDED": 504, "RATE_LIMIT_ERROR": 429, "OVERLOADED_ERROR": 529, "API_ERROR": 500}
NAP_S = 60.0
CONVERSATION_KEYS = ("contents", "systemInstruction", "tools")


def gemini_schema(s):
    if isinstance(s, list):
        return [gemini_schema(x) for x in s]
    if not isinstance(s, dict):
        return s
    out = {}
    for k, v in s.items():
        if k not in SCHEMA_KEYS:
            continue
        if k == "properties" and isinstance(v, dict):
            out[k] = {name: gemini_schema(sub) for name, sub in v.items()}
        elif k in ("items", "anyOf"):
            out[k] = gemini_schema(v)
        else:
            out[k] = v
    return out


def gemini_tools(tools: list[dict]) -> list[dict]:
    decls = []
    for t in tools or []:
        d = {"name": t["name"], "description": t.get("description", "")}
        if t.get("input_schema"):
            d["parameters"] = gemini_schema(t["input_schema"])
        decls.append(d)
    return [{"functionDeclarations": decls}] if decls else []


def _image_part(b: dict) -> dict | None:
    src = b.get("source") or {}
    if src.get("type") != "base64" or not src.get("data"):
        return None
    return {"inlineData": {"mimeType": src.get("media_type", "image/png"), "data": src["data"]}}


def _blocks(content) -> list[dict]:
    if isinstance(content, str):
        return [{"type": "text", "text": content}]
    return [b for b in content or [] if isinstance(b, dict)]


def _model_parts(blocks: list[dict]) -> tuple[list[dict], dict]:
    """Rebuild a model turn; returns (parts, {tool_use_id: (name, gemini call id or None)})."""
    parts, calls = [], {}
    for b in blocks:
        raw = b.get(STASH) if isinstance(b.get(STASH), dict) else {}
        sig = raw.get("thoughtSignature")
        t = b.get("type")
        if t == "text":
            if not b.get("text") and not sig:
                continue
            p = {"text": b.get("text", "")}
        elif t == "tool_use":
            fc = {"name": b.get("name", ""), "args": b.get("input") or {}}
            gid = (raw.get("functionCall") or {}).get("id")
            if gid:
                fc["id"] = gid
            calls[b.get("id")] = (fc["name"], gid)
            p = {"functionCall": fc}
        else:
            continue
        if sig:
            p["thoughtSignature"] = sig
        parts.append(p)
    fcs = [p for p in parts if "functionCall" in p]
    if fcs and not any("thoughtSignature" in p for p in fcs):
        fcs[0]["thoughtSignature"] = SKIP_SIGNATURE
    return parts, calls


def _tool_text(content) -> tuple[str, list[dict]]:
    texts, images = [], []
    for c in _blocks(content):
        if c.get("type") == "text":
            texts.append(c.get("text", ""))
        elif c.get("type") == "image":
            ip = _image_part(c)
            if ip:
                images.append(ip)
    return "\n".join(texts), images


def gemini_contents(messages: list[dict]) -> list[dict]:
    """Anthropic messages -> Gemini contents."""
    out, calls = [], {}
    for m in messages:
        blocks = _blocks(m.get("content"))
        if m.get("role") == "assistant":
            parts, c = _model_parts(blocks)
            calls.update(c)
            if parts:
                out.append({"role": "model", "parts": parts})
            continue
        frs, rest = [], []
        for b in blocks:
            t = b.get("type")
            if t == "tool_result":
                name, gid = calls.get(b.get("tool_use_id"), ("unknown", None))
                text, images = _tool_text(b.get("content"))
                fr = {"name": name,
                      "response": {"error" if b.get("is_error") else "output": text}}
                if gid:
                    fr["id"] = gid
                if images:
                    fr["parts"] = images
                frs.append({"functionResponse": fr})
            elif t == "text":
                if b.get("text"):
                    rest.append({"text": b["text"]})
            elif t == "image":
                ip = _image_part(b)
                if ip:
                    rest.append(ip)
        if frs:
            out.append({"role": "user", "parts": frs})
        if rest:
            out.append({"role": "user", "parts": rest})
    return out


def convert(system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
    """The conversation keys of a request body (replay rebuilds bodies from these)."""
    out: dict = {"contents": gemini_contents(messages)}
    sys_text = "\n\n".join(b.get("text", "") for b in system or [] if b.get("text"))
    if sys_text:
        out["systemInstruction"] = {"parts": [{"text": sys_text}]}
    t = gemini_tools(tools)
    if t:
        out["tools"] = t
    return out


def parse_candidate(data: dict) -> tuple[list[dict], str, str]:
    """(Anthropic-shaped content, stop_reason, finishReason) from a generateContent reply."""
    cand = (data.get("candidates") or [{}])[0]
    finish = cand.get("finishReason") or ""
    content = []
    for p in (cand.get("content") or {}).get("parts") or []:
        if "functionCall" in p:
            fc = p["functionCall"]
            blk = {"type": "tool_use", "id": fc.get("id") or "toolu_" + uuid.uuid4().hex[:24],
                   "name": fc.get("name", ""), "input": fc.get("args") or {}}
        elif p.get("thought"):
            blk = {"type": "thinking", "thinking": p.get("text", "")}
        elif "text" in p:
            if not p["text"] and not p.get("thoughtSignature"):
                continue
            blk = {"type": "text", "text": p["text"]}
        else:
            continue
        stash = {"thoughtSignature": p["thoughtSignature"]} if p.get("thoughtSignature") else {}
        if "functionCall" in p and p["functionCall"].get("id"):
            stash["functionCall"] = {"id": p["functionCall"]["id"]}
        if stash:
            blk[STASH] = stash
        content.append(blk)
    if any(b["type"] == "tool_use" for b in content):
        stop = "tool_use"
    elif finish == "MAX_TOKENS":
        stop = "max_tokens"
    else:
        stop = "end_turn"
    return content, stop, finish


def map_usage(um: dict, finish: str = "") -> dict:
    thoughts = um.get("thoughtsTokenCount") or 0
    return {"input_tokens": um.get("promptTokenCount") or 0,
            "output_tokens": (um.get("candidatesTokenCount") or 0) + thoughts,
            "cache_read_input_tokens": um.get("cachedContentTokenCount") or 0,
            "cache_creation_input_tokens": 0,
            "output_tokens_details": {"thinking_tokens": thoughts},
            "finish_reason": finish, "raw": um}


def cost_from_usage(usage: dict, price: dict | None = None) -> float:
    """USD of Gemini usage, whose input_tokens already holds the cache read."""
    return per_token_cost(usage, PRICE if price is None else price, inclusive=True)


def _error_message(raw: str) -> str:
    try:
        d = json.loads(raw)
    except ValueError:
        return raw
    err = d.get("error") if isinstance(d, dict) else None
    if isinstance(err, dict) and err.get("message"):
        return str(err["message"])
    return str(d.get("message", raw)) if isinstance(d, dict) else raw


def error_status(err) -> int | None:
    """The HTTP status an error object carries (a 4xx/5xx code, else a named status), or None."""
    if not isinstance(err, dict):
        return None
    code = err.get("code")
    if isinstance(code, str) and code.isdigit():
        code = int(code)
    if isinstance(code, int) and not isinstance(code, bool) and 400 <= code <= 599:
        return code
    for k in ("status", "type", "code"):
        v = err.get(k)
        if isinstance(v, str) and v.upper() in ERROR_STATUS:
            return ERROR_STATUS[v.upper()]
    return None


def malformed(data: dict) -> str | None:
    """Why a 200 reply's JSON object has the wrong nested types for parsing, else None."""
    def bad(v, *types):
        return v is not None and (isinstance(v, bool) or not isinstance(v, types))

    for k, types in (("usageMetadata", dict), ("promptFeedback", dict), ("responseId", str)):
        if bad(data.get(k), types):
            return k
    for k, v in (data.get("usageMetadata") or {}).items():
        if k.endswith("TokenCount") and bad(v, int, float):
            return f"usageMetadata.{k}"
    block = (data.get("promptFeedback") or {}).get("blockReason")
    if bad(block, str):
        return "promptFeedback.blockReason"
    cands = data.get("candidates")
    if not cands:
        return None
    if not isinstance(cands, list) or not isinstance(cands[0], dict):
        return "candidates"
    cand = cands[0]
    if bad(cand.get("finishReason"), str):
        return "finishReason"
    if bad(cand.get("content"), dict):
        return "content"
    parts = (cand.get("content") or {}).get("parts")
    if bad(parts, list) or not all(isinstance(p, dict) for p in parts or []):
        return "parts"
    for p in parts or []:
        fc = p.get("functionCall")
        if "functionCall" in p and (not isinstance(fc, dict) or bad(fc.get("name"), str)
                                    or bad(fc.get("id"), str) or bad(fc.get("args"), dict)):
            return "functionCall"
        if bad(p.get("text"), str) or bad(p.get("thoughtSignature"), str):
            return "part text"
    return None


class QuotaRetry(Retry):
    """A quota or overload failure: retried on the quota budget, not the generic one."""

    def __init__(self, message, status=None, retry_after=None, body=None):
        super().__init__(message, status, retry_after, body)


class PromptBlocked(Retry):
    """promptFeedback.blockReason with no candidates; final once its resample budget is spent."""

    def __init__(self, message, block_reason, body=None, usage=None, *, final=False):
        super().__init__(message, 200, None, body, usage)
        self.block_reason, self.final = block_reason, final


class GeminiClient(HTTPClient):
    """Gemini over generateContent, non-streamed, with separate retry budgets per failure kind."""

    provider = "gemini"

    def __init__(self, model: str = MODEL, *, thinking: str = "off", max_tokens: int = 65536,
                 base_url: str = BASE_URL, media_resolution: str = "high",
                 max_malformed_retries: int = 3, max_safety_retries: int = 2,
                 max_prompt_block_retries: int = 3, price: dict | None = None,
                 read_timeout_s: float = 900.0,
                 quota_max_attempts: int = 40, quota_backoff_s: float = 15.0,
                 quota_backoff_cap_s: float = 240.0, quota_wait_s: float = 3600.0, **kw):
        if thinking not in THINKING:
            raise ValueError(f"thinking must be one of {THINKING}")
        if media_resolution not in MEDIA:
            raise ValueError(f"media_resolution must be one of {tuple(MEDIA)}")
        kw.setdefault("api_key_header", KEY_HEADER)
        super().__init__(f"{base_url.rstrip('/')}/models/{model}:generateContent",
                         read_timeout_s=read_timeout_s, **kw)
        self.model, self.thinking, self.max_tokens = model, thinking, max_tokens
        self.context_tokens: int | None = CONTEXT_TOKENS
        self.context_window = context_window(model)
        self.media_resolution = media_resolution
        self.price = PRICE if price is None else price
        self.max_malformed_retries = max_malformed_retries
        self.max_safety_retries = max_safety_retries
        self.max_prompt_block_retries = max_prompt_block_retries
        self.quota_max_attempts, self.quota_wait_s = quota_max_attempts, quota_wait_s
        self.quota_backoff_s, self.quota_backoff_cap_s = quota_backoff_s, quota_backoff_cap_s
        self.successes = 0
        self._malformed = 0
        self._safety = 0
        self._prompt_block = 0

    def estimate_image_tokens(self, w: int, h: int) -> int:
        return IMAGE_TOKENS[self.media_resolution]

    def attempt_cost(self, usage: dict | None) -> float:
        return self.computed(usage, cost_from_usage(usage, self.price), True)[0] if usage else 0.0

    @staticmethod
    def prompt_tokens(usage: dict) -> int | None:
        """promptTokenCount (input_tokens here), which already holds cachedContentTokenCount."""
        n = (usage or {}).get("input_tokens")
        return n if isinstance(n, int) else None

    def build(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
        conv = convert(system, messages, tools)
        body = {"contents": conv["contents"],
                "generationConfig": {
                    "maxOutputTokens": self.max_tokens,
                    "mediaResolution": MEDIA[self.media_resolution],
                    "thinkingConfig": {"thinkingLevel": LEVEL[self.thinking],
                                       "includeThoughts": True}}}
        for k in ("systemInstruction", "tools"):
            if k in conv:
                body[k] = conv[k]
        return body

    def headers(self, force_token: bool = False) -> dict:
        return self.request_headers({"content-type": "application/json",
                                     "accept": "application/json"})

    def http_error(self, status, raw, msg, headers):
        ra = retry_after(headers.get("retry-after"))
        if status in self.refresh_status:
            return Retry(msg, status, ra, raw)
        retryable = status in self.retry_status or status >= 500
        if (status in QUOTA_STATUS or QUOTA_BODY.search(raw)
                or (retryable and OVERLOADED.search(raw))):
            return QuotaRetry(msg, status, ra, raw)
        if retryable:
            return Retry(msg, status, ra, raw)
        if status == 400 and OVERFLOW.search(_error_message(raw)):
            return ContextOverflow(msg, 400, raw)
        return LLMError(msg, status, raw)

    def quota_delay(self, n: int, e: Retry) -> float:
        """Retry-After, else full jitter over [backoff/2, min(cap, backoff * 2^(n-1))]."""
        if e.retry_after is not None:
            return max(0.0, min(float(e.retry_after), 600.0))
        hi = min(self.quota_backoff_cap_s, self.quota_backoff_s * 2 ** (n - 1))
        return random.uniform(self.quota_backoff_s / 2, hi)

    def plan(self, kind: str, n: int, e: Retry, quota_waited: float) -> tuple[float, str | None]:
        """(wait before the next attempt, why to give up instead or None)."""
        if kind == "finish":
            return 1.0, None
        if kind == "prompt_block":
            return (0.0, "prompt block resamples spent") if e.final else (1.0, None)
        if kind == "transient":
            return ((0.0, f"out of attempts ({n} {TRANSIENT_MARK}s)") if n >= self.transient_attempts
                    else (self.delay(n, e), None))
        if kind != "quota":
            return (0.0, "out of attempts") if n >= self.max_attempts else (self.delay(n, e), None)
        cap = self.quota_wait_s
        if n >= self.quota_max_attempts:
            return 0.0, f"out of quota attempts ({n})"
        if quota_waited >= cap:
            return 0.0, f"quota wait cap {cap:.0f}s spent over {n} attempts"
        return min(self.quota_delay(n, e), cap - quota_waited), None

    def nap(self, s: float) -> None:
        """Sleep s in NAP_S slices, checking the caller's limits between them."""
        while s > 0:
            step = min(s, NAP_S)
            self.sleep(step)
            s -= step
            self.check()

    def create(self, *, system, messages, tools) -> Turn:
        self._malformed = self._safety = self._prompt_block = 0
        body = self.build(system=system, messages=messages, tools=tools)
        errors: list[dict] = []
        try:
            return self._create(body, errors)
        except Aborted as e:
            self.mark_aborted(e, errors)
            raise

    def _create(self, body: dict, errors: list[dict]) -> Turn:
        t0 = time.monotonic()
        force_token = False
        sent: bytes | None = None
        attempt = 0
        used = {"generic": 0, "transient": 0, "quota": 0, "finish": 0, "prompt_block": 0}
        quota_waited = 0.0
        while True:
            self.check()
            attempt += 1
            payload = json.dumps(body).encode()
            if payload != sent:
                self.observer.request(body, payload, attempt)
                sent = payload
            ta = time.monotonic()
            try:
                try:
                    turn = self.once(payload, body, force_token, attempt)
                except (OSError, http.client.HTTPException) as e:
                    raise Retry(f"request: {type(e).__name__}: {e}",
                                timeout=isinstance(e, TimeoutError)) from None
            except BudgetExhausted as e:
                self.refused(e, attempt, errors)
                raise
            except ContextOverflow as e:
                e.attempts = attempt
                errors.append({"attempt": attempt, "status": e.status, "error": str(e)[:500],
                               "retry_in_s": None, "retry_kind": "overflow"})
                self.observer.retry(errors[-1])
                raise
            except Retry as e:
                self.never_connected(e, attempt, errors)
                kind = ("quota" if isinstance(e, QuotaRetry)
                        else "prompt_block" if isinstance(e, PromptBlocked)
                        else "finish" if e.status == "finish"
                        else "transient" if self.endpoint_fault(e)
                        and self.transient_attempts is not None else "generic")
                used[kind] += 1
                transient = self.transient_attempts is not None and kind in ("transient", "quota")
                force_token = e.status in self.refresh_status
                wait, why = self.plan(kind, used[kind], e, quota_waited)
                left = self.remaining()
                if left is not None:
                    wait = min(wait, max(0.0, left))
                err = {"attempt": attempt, "status": e.status, "error": str(e)[:500],
                       "retry_in_s": None if why else round(wait, 1),
                       "elapsed_s": round(time.monotonic() - ta, 3),
                       "cost_usd": self.failed_cost(e), "usage": e.usage,
                       "retry_kind": kind, "kind_attempt": used[kind], "transient": transient}
                if isinstance(e, QuotaRetry):
                    err["quota_waited_s"] = round(quota_waited, 1)
                if isinstance(e, PromptBlocked):
                    err["block_reason"] = e.block_reason
                if e.retry_after is not None:
                    err["retry_after_s"] = e.retry_after
                if why:
                    err["gave_up"] = why
                errors.append(err)
                (log.error if why else log.warning)(
                    "%s attempt %d failed after %.1fs: %s", self.provider, attempt,
                    time.monotonic() - ta, err)
                self.observer.retry(err)
                if why:
                    msg = str(e) if isinstance(e, PromptBlocked) else f"{why}: {e}"
                    if transient and TRANSIENT_MARK not in msg:
                        msg = f"{msg} ({TRANSIENT_MARK})"
                    raise LLMError(msg, e.status, e.body, attempt,
                                   timeout=e.timeout and not self._cut,
                                   transient=transient) from None
                if kind == "quota":
                    quota_waited += wait
                self.nap(wait)
                continue
            self.successes += 1
            self.measured(turn, t0, ta, attempt, errors)
            log.info("%s attempt %d ok %.1fs request %s stop %s", self.provider, attempt,
                     turn.latency_s, turn.request_id, turn.stop_reason)
            return turn

    def parse(self, resp, headers: dict, rid: str, body: dict) -> Turn:
        raw = resp.read().decode(errors="replace")
        try:
            data = json.loads(raw)
        except (ValueError, RecursionError):
            raise Retry(f"bad JSON in 200 reply request {rid}: {raw[:200]}") from None
        try:
            return self.parse_data(data, raw, headers, rid)
        except (AttributeError, TypeError, KeyError, IndexError, ValueError) as e:
            raise Retry(f"malformed 200 reply request {rid}: {type(e).__name__}: {e}: "
                        f"{raw[:200]}", body=raw) from None

    def parse_data(self, data, raw: str, headers: dict, rid: str) -> Turn:
        if not isinstance(data, dict):
            raise Retry(f"non-object JSON ({type(data).__name__}) in 200 reply request {rid}: "
                        f"{raw[:200]}", body=raw)
        cands = data.get("candidates")
        status = None if cands else error_status(data.get("error"))
        msg = f"HTTP 200 with error body ({status}) request {rid}: {raw[:400]}"
        if status is not None and (refusal := budget_exhausted(status, raw, msg)) is not None:
            raise refusal
        if status is not None and self.transient_attempts is not None:
            raise self.http_error(status, raw, msg, headers)
        if (why := malformed(data)) is not None:
            raise Retry(f"malformed 200 reply ({why}) request {rid}: {raw[:200]}", body=raw)
        if not cands:
            um = data.get("usageMetadata")
            usage = map_usage(um) if um else None
            block = (data.get("promptFeedback") or {}).get("blockReason")
            if block:
                final = self._prompt_block >= self.max_prompt_block_retries
                if not final:
                    self._prompt_block += 1
                raise PromptBlocked(f"prompt blocked ({block}) request {rid}", block, raw, usage,
                                    final=final)
            raise Retry(f"reply without candidates request {rid}: {raw[:300]}", body=raw,
                        usage=usage, transient=False)
        content, stop, finish = parse_candidate(data)
        usable = any(b["type"] == "tool_use" or (b["type"] == "text" and b.get("text"))
                     for b in content)
        if not usable and self.resample(finish):
            raise Retry(f"unusable reply, finishReason {finish} request {rid}", "finish",
                        body=raw, usage=map_usage(data.get("usageMetadata") or {}, finish))
        usage = map_usage(data.get("usageMetadata") or {}, finish)
        cost, source = self.header_cost(headers), "header"
        if cost is None:
            cost, source = self.computed(usage, cost_from_usage(usage, self.price), True)
        parts = copy.deepcopy(((data["candidates"][0].get("content") or {}).get("parts")) or [])
        return Turn(content=content, stop_reason=stop, usage=usage, cost_usd=cost,
                    request_id=rid, latency_s=0.0, attempts=0,
                    ratelimit=parse_ratelimit(headers), cost_source=source,
                    message_id=data.get("responseId", ""), native={"parts": parts})

    def resample(self, finish: str) -> bool:
        """Whether a reply with no text or call and this finishReason is sampled again."""
        if finish in RETRY_FINISH and self._malformed < self.max_malformed_retries:
            self._malformed += 1
            return True
        if finish in SAFETY_FINISH and self._safety < self.max_safety_retries:
            self._safety += 1
            return True
        return False
