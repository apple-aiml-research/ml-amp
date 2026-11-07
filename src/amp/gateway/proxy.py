#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Forwarding one agent request to its slot's robot server: streamed, or buffered and filtered."""

from __future__ import annotations

import http.client
import json
import math
import select
import socket
import threading
from dataclasses import dataclass, field
from urllib.parse import unquote

from amp.gateway.errors import HTTPError
from amp.gateway.policy import drop_counts

CONNECT_TIMEOUT_S = 5.0
PROXY_TIMEOUT_S = 600.0
MAX_BODY = 64 * 1024 * 1024
STREAM_CHUNK = 64 * 1024
WATCH_S = 0.2
KEY_HEADER = "X-Harness-Key"
HOP_BY_HOP = frozenset({"connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
                        "te", "trailer", "transfer-encoding", "upgrade"})
REQUEST_DROP = HOP_BY_HOP | {"host", "content-length", "expect"}
FORWARD_DROP = REQUEST_DROP | {KEY_HEADER.lower(), "cookie", "authorization"}
REPLY_DROP = HOP_BY_HOP | {"server", "date"}
QUIET_ERRORS = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError)


def unsafe(rest: str) -> bool:
    """True when a path could escape its route after decoding: empty, dot or slash segments."""
    for s in rest.split("/"):
        d = unquote(s)
        if d in ("", ".", "..") or "/" in d or "\\" in d:
            return True
    return False


def end_to_end(items, drop) -> list[tuple[str, str]]:
    """Header pairs minus hop-by-hop ones, including any the Connection header names."""
    items = list(items)
    named = set()
    for k, v in items:
        if k.lower() == "connection":
            named.update(t.strip().lower() for t in v.split(",") if t.strip())
    return [(k, v) for k, v in items if k.lower() not in drop and k.lower() not in named]


def read_body(handler) -> bytes:
    """The request body by Content-Length; chunked bodies are refused."""
    handler.body_consumed = True
    if "chunked" in handler.headers.get("Transfer-Encoding", "").lower():
        handler.close_connection = True
        raise HTTPError(411, "chunked bodies are not supported; send Content-Length")
    try:
        n = int(handler.headers.get("Content-Length") or 0)
    except ValueError:
        handler.close_connection = True
        raise HTTPError(400, "bad Content-Length") from None
    if n < 0:
        handler.close_connection = True
        raise HTTPError(400, "bad Content-Length")
    if n > MAX_BODY:
        handler.close_connection = True
        raise HTTPError(413, f"body of {n} bytes exceeds {MAX_BODY}")
    return handler.rfile.read(n) if n else b""


@dataclass
class Forwarded:
    status: int
    nbytes: int = 0
    dropped: dict = field(default_factory=dict)
    filtered: bool = False


def forward(handler, host: str, port: int, method: str, target: str, body: bytes,
            extra_headers=(), drops=(), stopping: threading.Event | None = None,
            on_reply=None, connect_timeout_s: float = CONNECT_TIMEOUT_S,
            proxy_timeout_s: float = PROXY_TIMEOUT_S) -> Forwarded:
    """Send the request upstream and relay the reply to handler's client; on_reply(status,
    filtered) runs before the client sees the reply, so accounting never lags it."""
    conn = http.client.HTTPConnection(host, port, timeout=connect_timeout_s)
    try:
        try:
            conn.connect()
            conn.sock.settimeout(proxy_timeout_s)
            conn.putrequest(method, target, skip_accept_encoding=True)
            extra = list(extra_headers)
            skip = FORWARD_DROP | {k.lower() for k, _ in extra}
            for k, v in end_to_end(handler.headers.items(), skip) + extra:
                conn.putheader(k, v)
            if body or method not in ("GET", "HEAD"):
                conn.putheader("Content-Length", str(len(body)))
            conn.endheaders(body or None)
            upstream = conn.sock
            resp = conn.getresponse()
        except (OSError, http.client.HTTPException) as e:
            handler.log_error("%s:%s unreachable for %s %s: %s", host, port, method, target, e)
            raise HTTPError(502, "the worker is not answering") from None
        if not drops:
            if on_reply is not None:
                on_reply(resp.status, False)
            return Forwarded(resp.status, _relay(handler, resp, upstream, method, stopping))
        return _relay_filtered(handler, resp, method, drops, on_reply)
    finally:
        conn.close()


def _relay_filtered(handler, resp, method, drops, on_reply) -> Forwarded:
    try:
        raw = resp.read()
    except (OSError, http.client.HTTPException):
        raise HTTPError(502, "the worker's reply was cut short") from None
    out = Forwarded(resp.status)
    ctype = (resp.getheader("Content-Type") or "").split(";")[0].strip().lower()
    obj = _json_or_none(raw) if ctype == "application/json" else None
    if 200 <= resp.status < 300 and obj is None:
        raise HTTPError(502, "the worker's reply to a filtered route is not valid JSON")
    if obj is not None:
        out.dropped = drop_counts(obj, drops)
        out.filtered = 200 <= resp.status < 300
        raw = json.dumps(obj).encode()
    if on_reply is not None:
        on_reply(resp.status, out.filtered)
    handler.send_response(resp.status, resp.reason)
    for k, v in end_to_end(resp.getheaders(), REPLY_DROP | {"content-length"}):
        handler.send_header(k, v)
    handler.send_header("Content-Length", str(len(raw)))
    handler.end_headers()
    if method != "HEAD":
        handler.wfile.write(raw)
    handler.wfile.flush()
    out.nbytes = len(raw)
    return out


def _json_or_none(raw: bytes):
    if not raw:
        return None
    try:
        return json.loads(raw)
    except ValueError:
        return None


def _relay(handler, resp, upstream, method, stopping) -> int:
    """Stream the reply as it arrives; returns the body bytes relayed."""
    headers = end_to_end(resp.getheaders(), REPLY_DROP)
    no_body = method == "HEAD" or resp.status in (204, 304) or 100 <= resp.status < 200
    stream = resp.getheader("Content-Length") is None and not no_body
    handler.send_response(resp.status, resp.reason)
    for k, v in headers:
        handler.send_header(k, v)
    if stream:
        handler.send_header("Connection", "close")
        handler.close_connection = True
    handler.end_headers()
    handler.wfile.flush()
    if no_body:
        return 0
    done = threading.Event()
    watcher = None
    if stream:
        watcher = threading.Thread(target=_watch_client,
                                   args=(handler.connection, upstream, done, stopping),
                                   name="amp-gateway-watch", daemon=True)
        watcher.start()
    n = 0
    try:
        while True:
            chunk = resp.read1(STREAM_CHUNK)
            if not chunk:
                break
            handler.wfile.write(chunk)
            handler.wfile.flush()
            n += len(chunk)
    except (OSError, http.client.HTTPException):
        handler.close_connection = True
    finally:
        done.set()
        resp.close()
        if watcher is not None:
            watcher.join()
    return n


def _readable(sock, timeout_s: float) -> bool:
    """Whether sock has data or hung up within timeout_s; poll, unlike select, takes fds past 1023."""
    p = select.poll()
    p.register(sock, select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR)
    return bool(p.poll(max(0, math.ceil(timeout_s * 1000))))


def _watch_client(client, upstream, done, stopping):
    """Shut the upstream socket when the client goes away, so a quiet stream's read returns."""
    while not done.is_set():
        gone = stopping is not None and stopping.is_set()
        if not gone:
            try:
                readable = _readable(client, WATCH_S)
            except (OSError, ValueError):
                gone = True
            else:
                if readable:
                    try:
                        gone = client.recv(1, socket.MSG_PEEK) == b""
                    except OSError:
                        gone = True
                    if not gone:
                        done.wait()
                        return
        if gone:
            try:
                upstream.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            return
