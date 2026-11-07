#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The gateway's HTTP server: token-prefixed agent paths (s/<token>/...) and /harness/* routes."""

from __future__ import annotations

import json
import logging
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from amp.contracts.routes import find
from amp.contracts.trace import TRACE_HEADER, parse_context
from amp.gateway.claims import Gateway, MotionRefused
from amp.gateway.errors import HTTPError
from amp.gateway.proxy import QUIET_ERRORS, forward, read_body, unsafe
from amp.gateway.tracing import emit

log = logging.getLogger("amp.gateway.http")
OFFICIAL_ROUTES = ("_official/cell", "_official/model_timeout")


class GatewayServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, gateway: Gateway):
        self.gateway = gateway
        self.serving = False
        super().__init__(addr, GatewayHandler)

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], QUIET_ERRORS):
            return
        super().handle_error(request, client_address)

    def serve_forever(self, poll_interval=0.5):
        self.serving = True
        super().serve_forever(poll_interval)

    def close(self):
        """Stop serving, stop the gateway's polls and close the socket."""
        self.gateway.stopping.set()
        if self.serving:
            self.shutdown()
        self.gateway.stop()
        self.server_close()


class GatewayHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "amp-gateway/1"

    @property
    def gateway(self) -> Gateway:
        return self.server.gateway

    def log_message(self, format, *args):
        pass

    def log_error(self, format, *args):
        log.warning("%s %s", self.address_string(), format % args)

    def do_GET(self):
        self._dispatch("GET")

    def do_HEAD(self):
        self._dispatch("HEAD")

    def do_POST(self):
        self._dispatch("POST")

    def do_PUT(self):
        self._dispatch("PUT")

    def do_DELETE(self):
        self._dispatch("DELETE")

    def _dispatch(self, method):
        path = urlsplit(self.path).path
        self.body_consumed = False
        try:
            if path.startswith("/s/"):
                self._episode_route(method, path)
            elif path.startswith("/harness/"):
                self._harness_api(method, path[len("/harness/"):])
            else:
                raise HTTPError(404, f"no route {method} {path}")
        except QUIET_ERRORS:
            self.close_connection = True
        except HTTPError as e:
            self._settle_body()
            self._json(e.status, e.body(), head=method == "HEAD")
        except (ValueError, KeyError, TypeError) as e:
            self._settle_body()
            self._json(400, {"ok": False, "message": str(e) or type(e).__name__})
        except Exception as e:  # noqa: BLE001
            traceback.print_exc()
            try:
                self._settle_body()
                self._json(500, {"ok": False, "message": f"{type(e).__name__}: {e}"})
            except QUIET_ERRORS:
                self.close_connection = True

    def _settle_body(self):
        """Read an unread request body before an error reply, so a keep-alive connection's next
        request is not parsed out of it; close the connection if it cannot be read."""
        if self.body_consumed or self.close_connection:
            return
        try:
            read_body(self)
        except HTTPError:
            self.close_connection = True

    def _json(self, status, obj, head=False):
        data = json.dumps(obj, allow_nan=False).encode()
        self.send_response(status)
        if self.close_connection:
            self.send_header("Connection", "close")
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if not head:
            self.wfile.write(data)

    def _json_body(self) -> dict:
        raw = read_body(self)
        if not raw.strip():
            return {}
        try:
            body = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            raise ValueError(f"malformed JSON body: {e}") from None
        if not isinstance(body, dict):
            raise TypeError("JSON body must be an object")
        return body

    def _harness_api(self, method, name):
        gw = self.gateway
        routes = {"claim": "POST", "status": "GET", "release": "POST", "official": "POST"}
        if name.startswith("episodes/") and name.count("/") == 1:
            want = "GET"
        elif name in routes:
            want = routes[name]
        else:
            raise HTTPError(404, f"no route harness/{name}")
        if method != want:
            raise HTTPError(405, f"{method} not allowed on harness/{name}")
        gw.check_key(self.headers)
        if name == "claim":
            return self._json(200, gw.claim(self._json_body()))
        if name == "status":
            return self._json(200, gw.status())
        if name == "release":
            return self._json(200, gw.release(self._json_body()))
        if name == "official":
            return self._json(200, gw.official_start(self._json_body()))
        return self._json(200, gw.episode_status(name.split("/", 1)[1]))

    def _episode_route(self, method, path):
        _, _, token, *rest = path.split("/", 3)
        gw = self.gateway
        ep = gw.lookup(token)
        if ep is None:
            raise HTTPError(404, "no such episode")
        rest = rest[0] if rest else ""
        if rest == "_episode":
            if method != "GET":
                raise HTTPError(405, f"{method} not allowed on _episode")
            return self._json(200, gw.episode_json(ep))
        if rest == "_episode/finish":
            if method != "POST":
                raise HTTPError(405, f"{method} not allowed on _episode/finish")
            self._json_body()
            return self._json(200, gw.finish(ep))
        if rest in OFFICIAL_ROUTES and ep.official is not None:
            if method != "POST":
                raise HTTPError(405, f"{method} not allowed on {rest}")
            try:
                gw.check_official_key(ep, self.headers)
            except HTTPError as e:
                gw.blocked(ep, f"{method} {rest} (bad official key)")
                emit(gw.tracer(ep), "gateway.request", {
                    "method": method, "path": rest, "key": rest, "allowed": False,
                    "status": e.status, "official": "bad_key"})
                raise
            body = self._json_body()
            if rest == "_official/cell":
                return self._json(200, gw.official_cell(ep, body))
            return self._json(200, gw.official_model_timeout(ep))
        self._proxy(gw, ep, method, rest)

    def _proxy(self, gw, ep, method, rest):
        t0 = time.monotonic()
        tr = gw.tracer(ep)
        policy = gw.policy_of(ep).policy
        key = None if unsafe(rest) else policy.key(method, rest)
        ev = {"method": method, "path": rest, "key": key, "policy": policy.name}
        if key is None:
            read_body(self)
            gw.blocked(ep, f"{method} {rest}")
            served = not unsafe(rest) and find(method, rest) is not None
            emit(tr, "gateway.request", {**ev, "allowed": False, "served": served,
                                         "status": 403})
            raise HTTPError(403, f"not available in the harness: {rest}")
        if ep.state == "over":
            raise HTTPError(410, "the episode is over", {"end": ep.end})
        if ep.state == "loading":
            raise HTTPError(409, "the episode is still loading", {"state": "loading"})
        slot = gw.by_name[ep.worker]
        body = read_body(self)
        drops = policy.drops(key)
        query = urlsplit(self.path).query
        target = "/" + rest + (f"?{query}" if query else "")
        incoming = self.headers.get(TRACE_HEADER) or ""
        ctx = incoming if parse_context(incoming)[0] == ep.episode_id else tr.context()
        try:
            executes = gw.begin(ep, key)
        except MotionRefused:
            emit(tr, "gateway.request", {**ev, "allowed": False, "official": "outside_cell",
                                         "status": 409})
            raise
        status, nbytes, dropped = 502, 0, {}
        replied, rid = [], None

        def on_reply(st, f):
            replied.append(st)
            gw.done(ep, key, st, f, executes)

        try:
            rid = gw.started(ep, key, executes)
            out = forward(self, slot.host, slot.port, method, target, body,
                          extra_headers=[(TRACE_HEADER, ctx)] if ctx else [], drops=drops,
                          stopping=gw.stopping, proxy_timeout_s=gw.request_timeout_s,
                          on_reply=on_reply)
            status, nbytes, dropped = out.status, out.nbytes, out.dropped
        except HTTPError as e:
            status = e.status
            if isinstance(e, MotionRefused):
                ev["official"] = "outside_cell"
            raise
        finally:
            gw.stopped(ep, rid, bool(replied), executes)
            emit(tr, "gateway.request", {
                **ev, "allowed": True, "status": status, "bytes": nbytes,
                "latency_s": round(time.monotonic() - t0, 4), "executes": executes,
                "filter": list(drops), "dropped": dropped, "body_bytes": len(body)})


def make_server(gateway: Gateway, host: str = "127.0.0.1", port: int = 0) -> GatewayServer:
    """Bind the gateway's HTTP server and start its polls (not yet serving)."""
    try:
        server = GatewayServer((host, port), gateway)
    except BaseException:
        gateway.stop()
        raise
    gateway.start()
    return server


def start_in_thread(gateway: Gateway, host: str = "127.0.0.1", port: int = 0):
    """Serve on a daemon thread; returns (server, port); stop with server.close()."""
    server = make_server(gateway, host, port)
    server.serving = True
    threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.1},
                     name="amp-gateway-http", daemon=True).start()
    return server, server.server_address[1]
