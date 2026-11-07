#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What every LLM client shares: the Turn, errors, the observer hook and the retrying HTTP call."""

from __future__ import annotations

import email.utils
import http.client
import json
import logging
import os
import random
import re
import ssl
import time
import urllib.parse
from dataclasses import dataclass, field
from typing import Any

log = logging.getLogger("amp.agents.llm")

CA_FILE_ENV = "SSL_CERT_FILE"
TRANSIENT_ATTEMPTS = 32
TRANSIENT_ATTEMPTS_ENV = "AMP_LLM_TRANSIENT_ATTEMPTS"
REFUSED_ATTEMPTS = 3
SHARED = "shared"
TRANSIENT_STATUS = frozenset({408, 429, 529})
TRANSIENT_MARK = "transient endpoint failure"
BUDGET_MARK = "provider budget exhausted"
BUDGET_STATUS = frozenset({402, 403, 429})
BUDGET_BODY = re.compile(r"\bbudget\b|\bspend(?:ing)?\b|\busage limit\b|\binsufficient_quota\b",
                         re.IGNORECASE)
BILLING_BODY = re.compile(r"\bbilling\b|\bcredits?\b", re.IGNORECASE)
CREDIT_400 = re.compile(r"\bcredit balance is too low\b", re.IGNORECASE)


class LLMError(RuntimeError):
    """A call that failed for good; timeout: out of attempts, the last on a socket timeout."""

    def __init__(self, message, status=None, body=None, attempts=0, timeout=False,
                 transient=False):
        super().__init__(message)
        self.status, self.body, self.attempts = status, body, attempts
        self.timeout, self.transient = timeout, transient


class ContextOverflow(LLMError):
    """The prompt does not fit the model's context window; prompt_tokens as the server counted."""

    def __init__(self, message, status=400, body=None, attempts=0, prompt_tokens=None):
        super().__init__(message, status, body, attempts)
        self.prompt_tokens = prompt_tokens


class Aborted(LLMError):
    """The caller's limit (wall clock or cost) passed during the call or a retry wait."""

    deadline = False


class BudgetExhausted(LLMError):
    """The endpoint refused the call for its key's spend budget: never retried."""


def budget_exhausted(status, raw: str | None, msg: str) -> BudgetExhausted | None:
    """BudgetExhausted when a reply's status and body say its token's budget is spent."""
    if not raw:
        return None
    hit = (status in BUDGET_STATUS and BUDGET_BODY.search(raw)) \
        or (status in (402, 403) and BILLING_BODY.search(raw)) \
        or (status == 400 and CREDIT_400.search(raw))
    if not hit:
        return None
    return BudgetExhausted(f"{BUDGET_MARK}: {msg}", status, raw)


class Retry(Exception):
    """One attempt failed in a way worth trying again (kind: its own resample budget)."""

    def __init__(self, message, status=None, retry_after=None, body=None, usage=None,
                 timeout=False, kind=None, info=None, transient=None):
        super().__init__(message)
        self.status, self.retry_after, self.body, self.usage = status, retry_after, body, usage
        self.timeout, self.kind, self.info = timeout, kind, info or {}
        self.transient = transient
        self.cost: float | None = None


def is_transient(e: Retry) -> bool:
    """A failure of the endpoint, not of the request: retried on TRANSIENT_ATTEMPTS."""
    if e.kind:
        return False
    if e.transient is not None:
        return e.transient
    s = e.status
    return s is None or (isinstance(s, int) and (s in TRANSIENT_STATUS or s >= 500))


@dataclass
class Turn:
    """One model reply."""

    content: list[dict]
    stop_reason: str
    usage: dict
    cost_usd: float | None
    request_id: str
    latency_s: float
    attempts: int
    ratelimit: dict
    cost_source: str = "header"
    message_id: str = ""
    errors: list[dict] = field(default_factory=list)
    native: Any = None
    raw: bytes = b""
    ttft_s: float | None = None
    retry_wait_s: float = 0.0
    billed_cost_usd: float | None = None


class Observer:
    """What a client reports while it works; the agent loop turns these into trace events."""

    def request(self, body: dict, payload: bytes, attempt: int) -> None:
        pass

    def retry(self, err: dict) -> None:
        pass


def ssl_context(environ=None) -> ssl.SSLContext:
    """The system trust store, or the bundle $SSL_CERT_FILE names."""
    env = os.environ if environ is None else environ
    return ssl.create_default_context(cafile=env.get(CA_FILE_ENV) or None)


def parse_ratelimit(headers) -> dict:
    """{"<kind>": n} from Ratelimit-Remaining-<Kind> headers."""
    out = {}
    for k, v in headers.items():
        lk = k.lower()
        if lk.startswith("ratelimit-remaining-"):
            try:
                out[lk[len("ratelimit-remaining-"):]] = int(v)
            except ValueError:
                out[lk[len("ratelimit-remaining-"):]] = v
    if not out and headers.get("ratelimit-remaining"):
        for part in headers["ratelimit-remaining"].split(","):
            fields = dict(f.strip().split("=", 1) for f in part.split(";")[1:] if "=" in f)
            if "kind" in fields:
                try:
                    out[fields["kind"]] = int(part.split(";")[0])
                except ValueError:
                    pass
    return out


def retry_after(value) -> float | None:
    """Retry-After as seconds: a number or an HTTP date."""
    if value is None:
        return None
    try:
        return float(value)
    except ValueError:
        pass
    try:
        dt = email.utils.parsedate_to_datetime(value)
    except (TypeError, ValueError):
        return None
    return max(0.0, dt.timestamp() - time.time()) if dt else None


def header_cost(headers: dict, name: str | None) -> float | None:
    """USD from the response header `name` (lower case), None when unset or unreadable."""
    if not name:
        return None
    try:
        v = headers.get(name.lower())
        return float(v) if v is not None else None
    except ValueError:
        return None


def per_token_cost(usage: dict, price: dict | None, inclusive: bool = False) -> float:
    """USD from Anthropic-shaped usage and a per-token price (input, output, cache_read/write)."""
    p = price or {}
    read = usage.get("cache_read_input_tokens") or 0
    write = usage.get("cache_creation_input_tokens") or 0
    prompt = (usage.get("input_tokens") or 0) + (0 if inclusive else read + write)
    if p.get("above") and prompt > p.get("threshold", float("inf")):
        p = {**p, **p["above"]}
    plain = max(0, (usage.get("input_tokens") or 0) - (read if inclusive else 0))
    return (p.get("input", 0.0) * plain
            + p.get("output", 0.0) * (usage.get("output_tokens") or 0)
            + p.get("cache_read", 0.0) * read + p.get("cache_write", 0.0) * write)


class Recorder:
    """Wraps a response so every byte read through it is kept, the first byte is timed, and the
    caller's limit is checked on every line."""

    def __init__(self, resp, t0: float, check=None):
        self.resp, self.t0, self.check = resp, t0, check
        self.buf = bytearray()
        self.ttft_s: float | None = None
        self.partial_usage: dict | None = None

    def _note(self, data: bytes) -> bytes:
        if self.check is not None:
            self.check()
        if data and self.ttft_s is None:
            self.ttft_s = round(time.monotonic() - self.t0, 3)
        self.buf += data
        return data

    def readline(self) -> bytes:
        return self._note(self.resp.readline())

    def read(self) -> bytes:
        return self._note(self.resp.read())


def iter_sse(resp):
    """(event, data) for each server-sent event, data the joined data lines."""
    event, data = None, []
    while True:
        line = resp.readline()
        if not line:
            break
        line = line.decode("utf-8").rstrip("\r\n")
        if not line:
            if data:
                yield event, "\n".join(data)
            event, data = None, []
            continue
        if line.startswith(":"):
            continue
        name, _, val = line.partition(":")
        val = val.removeprefix(" ")
        if name == "event":
            event = val
        elif name == "data":
            data.append(val)
    if data:
        yield event, "\n".join(data)


class HTTPClient:
    """One POST per attempt with retries; subclasses build the body and parse the reply."""

    provider = ""
    computed_source = "table"
    retry_status = frozenset({408, 409, 429, 500, 502, 503, 504, 529})
    refresh_status = frozenset({401})

    def __init__(self, url: str, *, api_key: str | None = None, api_key_header: str | None = None,
                 extra_headers: dict | None = None, cost_header: str | None = None,
                 max_attempts: int = 12, backoff_s: float = 5.0,
                 backoff_cap_s: float = 120.0, read_timeout_s: float = 600.0, sleep=time.sleep,
                 observer: Observer | None = None,
                 transient_attempts: int | None = TRANSIENT_ATTEMPTS):
        self.deadline: float | None = None
        self.should_abort = None
        self.url = urllib.parse.urlsplit(url)
        self.api_key, self.api_key_header = api_key or None, api_key_header
        self.extra_headers = dict(extra_headers or {})
        self.cost_header = cost_header or None
        self.max_attempts = max_attempts
        self.transient_attempts = transient_attempts
        self.backoff_s, self.backoff_cap_s = backoff_s, backoff_cap_s
        self.read_timeout_s = read_timeout_s
        self.sleep = sleep
        self.observer = observer or Observer()
        self.resamples: dict[str, int] = {}
        self._ssl: ssl.SSLContext | None = None
        self._cut = False
        self._connected = False

    def build(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> dict:
        raise NotImplementedError

    def headers(self, force_token: bool = False) -> dict:
        raise NotImplementedError

    def request_headers(self, fixed: dict) -> dict:
        """The key header, then the wire's fixed headers, then extra_headers (which win)."""
        h: dict = {}
        if self.api_key and self.api_key_header:
            name = self.api_key_header
            h[name] = f"Bearer {self.api_key}" if name.lower() == "authorization" else self.api_key
        h.update(fixed)
        lower = {k.lower(): k for k in h}
        for k, v in self.extra_headers.items():
            h.pop(lower.get(k.lower(), k), None)
            h[k] = v
        return h

    def header_cost(self, headers: dict) -> float | None:
        return header_cost(headers, self.cost_header)

    def parse(self, resp, headers: dict, rid: str, body: dict) -> Turn:
        raise NotImplementedError

    def http_error(self, status: int, raw: str, msg: str, headers: dict) -> Exception:
        """The exception for a non-200 reply: Retry, ContextOverflow or LLMError."""
        if status in self.retry_status or status >= 500 or status in self.refresh_status:
            return Retry(msg, status, retry_after(headers.get("retry-after")), raw)
        return LLMError(msg, status, raw)

    def computed(self, usage: dict, table_cost: float, inclusive: bool = False) -> tuple[float, str]:
        """(USD, cost_source) of usage with no cost header: the price table's, under
        computed_source."""
        return round(table_cost, 6), self.computed_source

    def attempt_cost(self, usage: dict | None) -> float:
        """USD of a failed attempt's reported usage (0 when none was reported)."""
        price = getattr(self, "price", None)
        if not usage or not price:
            return 0.0
        return self.computed(usage, per_token_cost(usage, price))[0]

    def failed_cost(self, e: Retry) -> float:
        """A failed attempt's cost: its cost header, else its reported usage priced."""
        return round(e.cost, 6) if e.cost is not None else self.attempt_cost(e.usage)

    def check(self) -> None:
        """Aborted once the deadline has passed or should_abort() gives a reason."""
        if self.deadline is not None and time.monotonic() >= self.deadline:
            e = Aborted("wall-clock deadline passed during the call")
            e.deadline = True
            raise e
        why = self.should_abort() if self.should_abort else None
        if why:
            raise Aborted(why)

    def remaining(self) -> float | None:
        return None if self.deadline is None else self.deadline - time.monotonic()

    def on_overflow(self, e: ContextOverflow, body: dict) -> bool:
        """Adjust body to fit after an overflow and return True to retry at once."""
        return False

    def delay(self, attempt: int, e: Retry) -> float:
        if e.retry_after is not None:
            return max(0.0, min(float(e.retry_after), 600.0))
        if e.status in self.refresh_status or e.status in ("finish", "degenerate"):
            return 1.0
        d = min(self.backoff_cap_s, self.backoff_s * 2 ** min(attempt - 1, 30))
        return min(self.backoff_cap_s, d * random.uniform(0.75, 1.25))

    def endpoint_fault(self, e: Retry) -> bool:
        """is_transient, except a socket wait the caller's deadline cut short: that is no fault."""
        return is_transient(e) and not (e.timeout and self._cut)

    def refused(self, e: BudgetExhausted, attempt: int, errors: list[dict]) -> None:
        """Record a budget refusal on the trace's retry log; the caller re-raises it."""
        e.attempts = attempt
        errors.append({"attempt": attempt, "status": e.status, "error": str(e)[:500],
                       "retry_in_s": None, "retry_kind": "budget", "transient": False})
        log.error("%s attempt %d refused: %s", self.provider, attempt, errors[-1])
        self.observer.retry(errors[-1])

    def never_connected(self, e: Retry, attempt: int, errors: list[dict]) -> None:
        """LLMError once REFUSED_ATTEMPTS connects were refused and none ever got through."""
        if not (getattr(e, "refused", False) and not self._connected
                and attempt >= REFUSED_ATTEMPTS):
            return
        errors.append({"attempt": attempt, "status": None, "error": str(e)[:500],
                       "retry_in_s": None, "transient": False})
        log.error("%s attempt %d refused, never connected: %s", self.provider, attempt, errors[-1])
        self.observer.retry(errors[-1])
        raise LLMError(f"connection refused {attempt} times, never connected to "
                       f"{self.url.scheme}://{self.url.netloc}: {e}", None, None, attempt)

    def mark_aborted(self, e: Aborted, errors: list[dict]) -> None:
        """Name a deadline that passed while the call was retrying transient failures."""
        n = sum(1 for err in errors if err.get("transient"))
        if e.deadline and n and not e.transient and self.transient_attempts is not None:
            e.transient, e.attempts = True, len(errors) + 1
            e.args = (f"{e} after {n} {TRANSIENT_MARK}(s)",)

    def measured(self, turn: Turn, t0: float, ta: float, attempt: int, errors: list[dict]) -> Turn:
        """The final attempt's latency; the retries' wait and cost beside it."""
        now = time.monotonic()
        turn.latency_s, turn.retry_wait_s = round(now - ta, 3), round(ta - t0, 3)
        failed = sum(err.get("cost_usd") or 0.0 for err in errors)
        turn.billed_cost_usd = round((turn.cost_usd or 0.0) + failed, 6)
        turn.attempts, turn.errors = attempt, errors
        return turn

    def ssl_context(self) -> ssl.SSLContext:
        if self._ssl is None:
            self._ssl = ssl_context()
        return self._ssl

    def connect(self):
        host, port = self.url.hostname or "", self.url.port
        left = self.remaining()
        timeout = self.read_timeout_s if left is None else max(1.0, min(self.read_timeout_s, left))
        self._cut = timeout < self.read_timeout_s
        if self.url.scheme == "https":
            return http.client.HTTPSConnection(host, port, context=self.ssl_context(),  # nosemgrep: python.lang.security.audit.httpsconnection-detected.httpsconnection-detected
                                               timeout=timeout)
        return http.client.HTTPConnection(host, port, timeout=timeout)

    def create(self, *, system: list[dict], messages: list[dict], tools: list[dict]) -> Turn:
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
        attempt = charged = flaky = 0
        resampled: dict[str, int] = {}
        while True:
            self.check()
            attempt += 1
            payload = json.dumps(body).encode()
            if payload != sent:
                self.observer.request(body, payload, attempt)
                sent = payload
            ta = time.monotonic()
            try:
                turn = self.once(payload, body, force_token, attempt)
            except BudgetExhausted as e:
                self.refused(e, attempt, errors)
                raise
            except ContextOverflow as e:
                e.attempts = attempt
                retry = self.on_overflow(e, body)
                err = {"attempt": attempt, "status": e.status, "error": str(e)[:500],
                       "retry_in_s": 0.0 if retry else None}
                if retry:
                    err["max_tokens"] = body.get("max_tokens")
                errors.append(err)
                self.observer.retry(err)
                if not retry:
                    raise
                continue
            except Retry as e:
                self.never_connected(e, attempt, errors)
                force_token = e.status in self.refresh_status
                wait = self.delay(attempt, e)
                left = self.remaining()
                if left is not None:
                    wait = min(wait, max(0.0, left))
                transient = self.endpoint_fault(e) and self.transient_attempts is not None
                if e.kind in self.resamples:
                    resampled[e.kind] = resampled.get(e.kind, 0) + 1
                    last = resampled[e.kind] > self.resamples[e.kind]
                elif transient:
                    flaky += 1
                    last = flaky >= self.transient_attempts
                else:
                    charged += 1
                    last = charged >= self.max_attempts
                err = {"attempt": attempt, "status": e.status, "error": str(e)[:500],
                       "retry_in_s": None if last else round(wait, 1),
                       "elapsed_s": round(time.monotonic() - ta, 3),
                       "cost_usd": self.failed_cost(e), "usage": e.usage, "transient": transient}
                if e.kind:
                    err.update(retry_kind=e.kind, **e.info)
                errors.append(err)
                log.warning("%s attempt %d failed after %.1fs: %s", self.provider, attempt,
                            time.monotonic() - ta, err)
                self.observer.retry(err)
                if last and e.kind in self.resamples:
                    raise LLMError(f"{e.kind} after {self.resamples[e.kind]} resamples: {e}",
                                   e.status, e.body, attempt) from None
                if last and transient:
                    raise LLMError(f"out of attempts ({flaky} {TRANSIENT_MARK}s): {e}", e.status,
                                   e.body, attempt, timeout=e.timeout and not self._cut,
                                   transient=True) from None
                if last:
                    raise LLMError(f"out of attempts: {e}", e.status, e.body, attempt,
                                   timeout=e.timeout and not self._cut) from None
                self.sleep(wait)
                self.check()
                continue
            self.measured(turn, t0, ta, attempt, errors)
            log.info("%s attempt %d ok %.1fs request %s stop %s", self.provider, attempt,
                     turn.latency_s, turn.request_id, turn.stop_reason)
            return turn

    def once(self, payload: bytes, body: dict, force_token: bool, attempt: int) -> Turn:
        path = self.url.path + (("?" + self.url.query) if self.url.query else "")
        headers = self.headers(force_token)
        conn = self.connect()
        t0 = time.monotonic()
        resp = None
        try:
            try:
                conn.request("POST", path, payload, headers)
                resp = conn.getresponse()
            except ConnectionRefusedError as e:
                r = Retry(f"connect: {type(e).__name__}: {e}")
                r.refused = True
                raise r from None
            except (OSError, http.client.HTTPException) as e:
                raise Retry(f"connect: {type(e).__name__}: {e}",
                            timeout=isinstance(e, TimeoutError)) from None
            self._connected = True
            hdrs = {k.lower(): v for k, v in resp.getheaders()}
            rid = hdrs.get("request-id") or hdrs.get("x-request-id") or ""
            rec = Recorder(resp, t0, self.check)
            if resp.status != 200:
                try:
                    raw = rec.read().decode(errors="replace")
                except (OSError, http.client.HTTPException) as e:
                    raise Retry(f"HTTP {resp.status}, body unread: {e}", resp.status) from None
                msg = f"HTTP {resp.status} request {rid}: {raw[:400]}"
                if (refusal := budget_exhausted(resp.status, raw, msg)) is not None:
                    raise refusal
                err = self.http_error(resp.status, raw, msg, hdrs)
                if isinstance(err, Retry) and err.cost is None:
                    err.cost = self.header_cost(hdrs)
                raise err
            try:
                turn = self.parse(rec, hdrs, rid, body)
            except Retry as e:
                e.usage = e.usage or rec.partial_usage
                if e.cost is None:
                    e.cost = self.header_cost(hdrs)
                raise
            except (TimeoutError, OSError, http.client.HTTPException, UnicodeDecodeError) as e:
                r = Retry(f"stream read: {type(e).__name__}: {e}", usage=rec.partial_usage,
                          timeout=isinstance(e, TimeoutError))
                r.cost = self.header_cost(hdrs)
                raise r from None
            turn.raw, turn.ttft_s = bytes(rec.buf), rec.ttft_s
            return turn
        finally:
            if resp is not None:
                resp.close()
            conn.close()
