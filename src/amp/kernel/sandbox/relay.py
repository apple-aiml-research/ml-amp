#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The token-filtering relay: the sandbox's only route to the gateway."""

from __future__ import annotations

import http.client
import http.server
import json
import os
import re
import secrets
import socketserver
import tempfile
import threading
import time
import urllib.parse
from dataclasses import dataclass
from pathlib import Path

from amp.contracts.trace import TRACE_HEADER, NullTracer, Tracer
from amp.trace.propagate import header_value

METHODS = frozenset({"GET", "POST", "HEAD"})
HOP_BY_HOP = frozenset({"connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
                        "proxy-connection", "te", "trailer", "transfer-encoding", "upgrade"})
REQUEST_DROP = HOP_BY_HOP | {"host", "content-length", "expect", "x-harness-key", "cookie",
                             "authorization", "forwarded", "x-forwarded-for",
                             "x-forwarded-host", "x-forwarded-proto", "x-real-ip",
                             TRACE_HEADER.lower()}
REPLY_DROP = HOP_BY_HOP | {"server", "date", "content-length", "set-cookie"}
MAX_BODY = 16 << 20
CHUNK = 64 << 10
SPOOL_MEM = 1 << 20
UPSTREAM_TIMEOUT_S = 3600.0
CONN_TIMEOUT_S = 30.0
MAX_CONNECTIONS = 32
SAMPLES = 20
_BAD_ESCAPES = ("%2f", "%5c", "%00", "%2e")
_TOKEN_SEG = re.compile(r"/s/[^/]*/?")


def safe_rest(rest: str) -> bool:
    """True when rest is a plain relative path the gateway cannot resolve outside the token."""
    if not rest or "\\" in rest or any(e in rest.lower() for e in _BAD_ESCAPES):
        return False
    return all(seg not in ("", ".", "..") for seg in urllib.parse.unquote(rest).split("/"))


@dataclass(frozen=True)
class Route:
    """What the relay makes of one request target: the upstream rest, or why it is refused."""

    shown: str
    rest: str | None = None
    query: str = ""
    refused: str | None = None


def route(method: str, target: str, relay_base: str, body_len: int) -> Route:
    """Classify a request: forwarded under relay_base, or refused with a reason."""
    t = urllib.parse.urlsplit(target)
    prefix = "/" + relay_base
    inside = t.path.startswith(prefix)
    shown = t.path[len(prefix):] if inside else \
        "(outside) " + _TOKEN_SEG.sub("/s/<token>/", t.path)[:200]
    if body_len < 0 or body_len > MAX_BODY:
        return Route(shown, refused="request body too large")
    if method not in METHODS:
        return Route(shown, refused=f"method {method}")
    if not inside or t.scheme or t.netloc:
        return Route(shown, refused="only this episode's routes")
    rest = t.path[len(prefix):]
    if not safe_rest(rest):
        return Route(shown, refused="unsafe path")
    return Route(shown, rest=rest, query=t.query)


def _end_to_end(items, drop):
    items = list(items)
    named = {t.strip().lower() for k, v in items if k.lower() == "connection"
             for t in v.split(",")}
    return [(k, v) for k, v in items if k.lower() not in drop and k.lower() not in named]


class _Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    timeout = CONN_TIMEOUT_S
    server_version = "amp-relay/1"
    sys_version = ""

    def log_message(self, format, *args):
        pass

    def address_string(self):
        return "sandbox"

    def _any(self):
        self.server.relay.handle(self, self.command)

    do_GET = do_POST = do_HEAD = do_PUT = do_DELETE = do_PATCH = _any
    do_OPTIONS = do_CONNECT = do_TRACE = _any


_BUSY_BODY = b'{"ok": false, "message": "the relay is at its connection cap"}'
_BUSY = (b"HTTP/1.0 503 Service Unavailable\r\nContent-Type: application/json\r\n"
         b"Content-Length: %d\r\nConnection: close\r\n\r\n%s" % (len(_BUSY_BODY), _BUSY_BODY))


class _ClientGone(Exception):
    """The sandbox side stopped sending or receiving mid-request."""


def _read(h, n: int) -> bytes:
    try:
        chunk = h.rfile.read(n)
    except OSError as e:
        raise _ClientGone(type(e).__name__) from e
    if not chunk:
        raise _ClientGone("the request body ended early")
    return chunk


class _Bounded:
    """At most MAX_CONNECTIONS handler threads; a connection past the cap gets a 503 inline."""

    daemon_threads = True
    block_on_close = False
    relay: Relay

    def process_request(self, request, client_address):
        if not self.relay.slots.acquire(blocking=False):
            try:
                request.settimeout(1.0)
                request.sendall(_BUSY)
            except OSError:
                pass
            self.relay.count("busy")
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, client_address)
        except BaseException:
            self.relay.slots.release()
            raise

    def process_request_thread(self, request, client_address):
        try:
            super().process_request_thread(request, client_address)
        finally:
            self.relay.slots.release()


class _TCPServer(_Bounded, http.server.ThreadingHTTPServer):
    allow_reuse_address = False


class _UnixServer(_Bounded, socketserver.ThreadingUnixStreamServer):
    pass


class Relay:
    """One episode's relay on a loopback TCP port or a Unix socket; start() then close()."""

    def __init__(self, gateway: str, token: str, base: str | None = None, *,
                 unix_path: str | None = None, host: str = "127.0.0.1", port: int = 0,
                 log_path: str | Path | None = None, timeout_s: float = UPSTREAM_TIMEOUT_S,
                 tracer: Tracer | None = None, max_connections: int = MAX_CONNECTIONS):
        u = urllib.parse.urlsplit(gateway)
        if u.scheme != "http" or not u.hostname:
            raise ValueError(f"relay upstream must be http://host:port, got {gateway!r}")
        self.upstream = (u.hostname, u.port or 80)
        self.upstream_base = "/" + (base or f"s/{token}/").strip("/") + "/"
        self.relay_token = secrets.token_urlsafe(18)
        self.relay_base = f"s/{self.relay_token}/"
        self.unix_path = unix_path
        self._bind = (host, int(port))
        self.timeout_s = float(timeout_s)
        self.log_path = None if log_path is None else Path(log_path)
        self.tracer: Tracer = tracer or NullTracer("kernel")
        self._server = None
        self._lock = threading.Lock()
        self.slots = threading.BoundedSemaphore(int(max_connections))
        self.stats = {"forwarded": 0, "blocked": 0, "upstream_errors": 0, "busy": 0}
        self.blocked_samples: list[str] = []

    @property
    def port(self) -> int | None:
        return None if self._server is None or self.unix_path else self._server.server_address[1]

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}"

    def start(self) -> Relay:
        if self.unix_path:
            if os.path.exists(self.unix_path):
                os.unlink(self.unix_path)
            self._server = _UnixServer(self.unix_path, _Handler)
            os.chmod(self.unix_path, 0o600)
        else:
            self._server = _TCPServer(self._bind, _Handler)
        self._server.relay = self
        threading.Thread(target=self._server.serve_forever, kwargs={"poll_interval": 0.2},
                         name="sandbox-relay", daemon=True).start()
        return self

    def close(self) -> None:
        s, self._server = self._server, None
        if s is not None:
            s.shutdown()
            s.server_close()
        if self.unix_path:
            try:
                os.unlink(self.unix_path)
            except OSError:
                pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    def count(self, stat: str) -> None:
        with self._lock:
            self.stats[stat] += 1

    def summary(self) -> dict:
        with self._lock:
            return {**self.stats, "blocked_samples": list(self.blocked_samples)}

    def handle(self, h, method: str) -> None:
        tracer, t0 = self.tracer, time.monotonic()
        try:
            n = int(h.headers.get("Content-Length") or 0)
        except ValueError:
            n = -1
        r = route(method, h.path, self.relay_base, n)
        call = {"method": method, "path": r.shown[:300], "req_bytes": max(n, 0)}
        try:
            if r.refused is None:
                with tempfile.SpooledTemporaryFile(max_size=SPOOL_MEM) as body:
                    left = n
                    while left:
                        left -= body.write(_read(h, min(CHUNK, left)))
                    body.seek(0)
                    status, size = self._forward(h, method, r, n, body, tracer)
            else:
                left = n if 0 < n <= MAX_BODY else 0
                while left:
                    left -= len(_read(h, min(CHUNK, left)))
                status, size = self._refuse(h, method, r)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, _ClientGone):
            h.close_connection = True
            status, size = None, 0
        call.update(status=status, bytes=size, latency_s=round(time.monotonic() - t0, 6),
                    blocked=r.refused is not None)
        if r.refused is not None:
            call["reason"] = r.refused
        ok = r.refused is None and status is not None and status < 500
        tracer.event("sdk.call", call, lvl="info" if ok else "warn")

    def _note(self, method: str, shown: str, status, blocked: bool) -> None:
        with self._lock:
            self.stats["blocked" if blocked else "forwarded"] += 1
            if blocked and len(self.blocked_samples) < SAMPLES:
                self.blocked_samples.append(f"{method} {shown}"[:200])
        if self.log_path is not None and (blocked or status == 502):
            row = {"t": round(time.time(), 3), "method": method, "path": shown[:300],
                   "status": status, "blocked": blocked}
            try:
                with open(self.log_path, "a") as f:
                    f.write(json.dumps(row) + "\n")
            except OSError:
                pass

    def _reply(self, h, status: int, body: dict, method: str) -> int:
        data = json.dumps(body).encode()
        h.send_response(status)
        h.send_header("Content-Type", "application/json")
        h.send_header("Content-Length", str(len(data)))
        h.end_headers()
        if method != "HEAD":
            h.wfile.write(data)
        return len(data)

    def _refuse(self, h, method: str, r: Route) -> tuple[int, int]:
        self._note(method, r.shown, 403, True)
        return 403, self._reply(h, 403, {"ok": False, "message": "not available in the "
                                         f"sandbox: {r.refused}"}, method)

    def _forward(self, h, method: str, r: Route, n: int, body, tracer: Tracer
                 ) -> tuple[int, int]:
        """Send the spooled body (n bytes) up and stream the reply down, CHUNK by CHUNK."""
        upstream = self.upstream_base + r.rest + (f"?{r.query}" if r.query else "")
        conn = http.client.HTTPConnection(*self.upstream, timeout=self.timeout_s)
        try:
            try:
                conn.putrequest(method, upstream, skip_accept_encoding=True)
                for k, v in _end_to_end(h.headers.items(), REQUEST_DROP):
                    conn.putheader(k, v)
                ctx = header_value(tracer)
                if ctx:
                    conn.putheader(TRACE_HEADER, ctx)
                if n or method == "POST":
                    conn.putheader("Content-Length", str(n))
                conn.endheaders()
                while chunk := body.read(CHUNK):
                    conn.send(chunk)
                resp = conn.getresponse()
            except (OSError, http.client.HTTPException) as e:
                with self._lock:
                    self.stats["upstream_errors"] += 1
                self._note(method, r.rest, 502, False)
                return 502, self._reply(h, 502, {"ok": False, "message": "the gateway is not "
                                                 f"answering ({type(e).__name__})"}, method)
            self._note(method, r.rest, resp.status, False)
            h.send_response(resp.status, resp.reason)
            for k, v in _end_to_end(resp.getheaders(), REPLY_DROP):
                h.send_header(k, v)
            if resp.length is not None:
                h.send_header("Content-Length", str(resp.length))
            h.end_headers()
            size = 0
            while method != "HEAD":
                chunk = resp.read(CHUNK)
                if not chunk:
                    break
                h.wfile.write(chunk)
                size += len(chunk)
            return resp.status, size
        finally:
            conn.close()


def episode_view(info: dict, relay: Relay, gateway_url: str) -> dict:
    """The episode file the sandboxed SDK opens: the relay's URL and token, nothing else secret."""
    keep = {k: info[k] for k in ("episode_id", "instruction") if k in info}
    return {**keep, "gateway": gateway_url, "token": relay.relay_token, "base": relay.relay_base}
