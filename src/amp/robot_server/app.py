#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The HTTP server: contracts.routes.ROUTES mapped to handlers, nothing else."""

from __future__ import annotations

import json
import sys
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

from amp.contracts.routes import GRADE, ROUTES
from amp.contracts.server import ServerConfig
from amp.link import wire
from amp.robot_server.api import Reply, Request, agent, assist, lifecycle, motion, sensors
from amp.robot_server.core import Server
from amp.robot_server.errors import Forbidden, HTTPError, NotFound, SessionConflict
from amp.robot_server.table_redact import redact

MAX_FRAME_BODY = 64 * 1024 * 1024
MAX_JSON_BODY = 8 * 1024 * 1024
FRAME_PATHS = ("sim/step", "sim/scene", "sim/robot")
SOCKET_TIMEOUT_S = 60.0
AGGREGATED = ("GET api/state",)
_QUIET = (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, TimeoutError)


def _link(name, frame=False):
    def handler(srv, req) -> Reply:
        fn = getattr(srv.link, name)
        return Reply(200, fn(*wire.unpack_frame(req.raw)) if frame else fn(req.body))
    return handler


HANDLERS = {
    ("GET", "api/tasks"): lifecycle.tasks,
    ("POST", "api/tasks/select"): lifecycle.select,
    ("GET", "api/state"): lifecycle.state,
    ("POST", "api/episode/release"): lifecycle.release,
    ("POST", "api/episode/publish"): lifecycle.publish,
    ("POST", "api/episode/grade"): lifecycle.grade,
    ("GET", "api/runs/{task}/{k}"): lifecycle.runs,
    ("GET", "api/runs/{run}/{task}/{k}"): lifecycle.runs,
    ("GET", "api/frame/{cam}"): sensors.frame,
    ("GET", "api/frame/{cam}/{seq}"): sensors.frame,
    ("GET", "api/cloud"): sensors.cloud,
    ("GET", "api/cloud/{query*}"): sensors.cloud,
    ("GET", "api/scene"): sensors.scene,
    ("GET", "api/agent/observation"): agent.observation,
    ("POST", "api/agent/wait"): agent.wait,
    ("GET", "api/agent/depth/{cam}"): agent.depth,
    ("GET", "api/agent/depth/{cam}/{seq}"): agent.depth,
    ("GET", "api/agent/object_points/{name}"): agent.object_points,
    ("POST", "api/agent/robot"): agent.robot,
    ("POST", "api/agent/fk"): agent.fk,
    ("POST", "api/agent/ik"): agent.ik,
    ("POST", "api/agent/ik_batch"): agent.ik_batch,
    ("POST", "api/agent/check"): agent.check,
    ("POST", "api/agent/check_batch"): agent.check_batch,
    ("POST", "api/agent/segment"): agent.segment,
    ("POST", "api/agent/define_object"): agent.define_object,
    ("POST", "api/agent/forget_object"): agent.forget_object,
    ("POST", "api/agent/reperceive"): agent.reperceive,
    ("POST", "api/assist/grasps"): assist.grasps,
    ("POST", "api/assist/places"): assist.places,
    ("POST", "api/assist/check"): assist.check,
    ("POST", "api/assist/plan"): assist.plan,
    ("POST", "api/assist/execute"): assist.execute,
    ("POST", "api/assist/object_at"): assist.object_at,
    ("GET", "api/assist/steps"): assist.steps,
    ("POST", "api/assist/steps/add"): assist.steps_add,
    ("POST", "api/assist/steps/update"): assist.steps_update,
    ("POST", "api/assist/steps/delete"): assist.steps_delete,
    ("POST", "api/assist/steps/clear"): assist.steps_clear,
    ("POST", "api/assist/steps/execute"): assist.steps_execute,
    ("POST", "api/pick"): motion.pick,
    ("POST", "api/plan/preview"): motion.preview,
    ("POST", "api/plan/execute"): motion.plan_execute,
    ("POST", "api/stop"): motion.stop,
    ("POST", "api/hold"): motion.hold,
    ("POST", "api/gripper"): motion.gripper,
    ("POST", "api/home"): motion.home,
    ("POST", "api/motion/base"): motion.base,
    **{("POST", f"sim/{n}"): _link(n) for n in ("hello", "catalog", "episode", "poll", "end",
                                                  "next", "saved")},
    ("POST", "sim/step"): _link("step", frame=True),
    ("POST", "sim/scene"): _link("scene", frame=True),
    ("POST", "sim/robot"): _link("robot_description", frame=True),
}
TABLE = [(r, HANDLERS[(r.method, r.path)]) for r in ROUTES]
RESOLVED_GROUPS = ("agent", "assist")
GET_405 = ("api/assist/steps", "api/cloud", "api/agent/observation")
GET_405_PREFIXES = ("api/cloud/", "api/frame/", "api/agent/depth/", "api/agent/object_points/")
SHAPE_404 = (("api/frame/", "expected /api/frame/<cam> or /api/frame/<cam>/<n>"),
             ("api/agent/depth/", "expected /api/frame/<cam> or /api/frame/<cam>/<n>"),
             ("api/runs/", "expected /api/runs/<task>/<episode>"),
             ("api/agent/object_points/", "expected /api/agent/object_points/<id>"))


def find(method, path):
    """(route, handler, params) of a request's decoded path, or None."""
    for r, h in TABLE:
        params = r.match(method, path)
        if params is not None:
            return r, h, params
    return None


def no_route(method, path) -> HTTPError:
    """OLD's 404/405 split for a request that matches no route."""
    if method in ("GET", "HEAD"):
        for prefix, message in SHAPE_404:
            if path.startswith(prefix):
                return HTTPError(404, message)
        posted = find("POST", path) is not None
        return HTTPError(405 if posted else 404, f"no route GET /{path}")
    if method == "POST" and (path in GET_405 or path.startswith(GET_405_PREFIXES)):
        return HTTPError(405, f"POST not allowed on /{path}")
    if method == "POST":
        return HTTPError(404, f"no route POST /{path}")
    return HTTPError(405, f"{method} not allowed on /{path}")


def handler_resolved(reply) -> list[dict]:
    """A stale id a handler resolved itself: in its JSON body, or object_points' headers."""
    if isinstance(reply.body, dict) and isinstance(reply.body.get("resolved"), dict):
        return [reply.body["resolved"]]
    h = reply.headers or {}
    if h.get("X-Resolved-From"):
        return [{"from": h["X-Resolved-From"], "to": h.get("X-Object-Id"),
                 "why": h.get("X-Resolved-Why")}]
    return []


class RobotHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, srv: Server):
        self.srv = srv
        super().__init__(addr, RobotHandler)

    def handle_error(self, request, client_address):
        if isinstance(sys.exc_info()[1], _QUIET):
            return
        super().handle_error(request, client_address)

    def close(self) -> None:
        self.srv.close()
        self.shutdown()
        self.server_close()


class RobotHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "amp-robot-server/1"
    timeout = SOCKET_TIMEOUT_S

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        self._dispatch("GET")

    def do_HEAD(self):
        self._dispatch("HEAD")

    def do_POST(self):
        self._dispatch("POST")

    def _read_body(self, limit: int) -> bytes:
        """The request body; on any refusal the connection closes, so no body is left unread."""
        if "chunked" in self.headers.get("Transfer-Encoding", "").lower():
            self.close_connection = True
            raise HTTPError(411, "chunked bodies are not supported; send Content-Length")
        try:
            n = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            self.close_connection = True
            raise HTTPError(400, "bad Content-Length") from None
        if n < 0 or n > limit:
            self.close_connection = True
            if n < 0:
                raise HTTPError(400, "bad Content-Length")
            raise HTTPError(413, f"body of {n} bytes exceeds {limit}")
        data = self.rfile.read(n) if n else b""
        if len(data) != n:
            self.close_connection = True
            raise HTTPError(400, "truncated body")
        return data

    @staticmethod
    def _json_body(raw: bytes) -> dict:
        if not raw.strip():
            return {}
        try:
            body = json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            raise ValueError(f"malformed JSON body: {e}") from None
        if not isinstance(body, dict):
            raise ValueError("JSON body must be an object")  # noqa: TRY004
        return body

    def _dispatch(self, method):
        srv: Server = self.server.srv
        url = urlsplit(self.path)
        path = "/".join(unquote(seg) for seg in url.path.strip("/").split("/"))
        self._hide_table = srv.settings.hide_table_height and not path.startswith("sim/")
        t0 = time.monotonic()
        route, status, resolved = None, 500, []
        try:
            raw = self._read_body(MAX_FRAME_BODY if path in FRAME_PATHS else MAX_JSON_BODY) \
                if method == "POST" or self.headers.get("Content-Length") else b""
            got = find("GET" if method == "HEAD" else method, path)
            if got is None:
                raise no_route(method, path)
            route, handler, params = got
            frame = route.path in FRAME_PATHS
            req = Request(method, path, params, parse_qs(url.query),
                          None if method != "POST" or frame else self._json_body(raw), raw,
                          {k.lower(): v for k, v in self.headers.items()})
            if route.group in RESOLVED_GROUPS and req.body is not None:
                req.body, resolved = srv.perception.resolve_request_ids(req.body)
            reply = handler(srv, req)
            if resolved and isinstance(reply.body, dict):
                reply.body = {**reply.body, "resolved": resolved[0]}
                if len(resolved) > 1:
                    reply.body["resolved_all"] = resolved
            if not resolved:
                resolved = handler_resolved(reply)
            status = reply.status
            self._send(reply, head=method == "HEAD")
        except _QUIET:
            self.close_connection = True
        except Exception as e:  # noqa: BLE001
            status = self._error(e, path)
        finally:
            self._trace(srv, method, path, route, status, time.monotonic() - t0, resolved)

    def _trace(self, srv, method, path, route, status, dt, resolved=()) -> None:
        """Agent and lifecycle requests go to the episode stream; polls are aggregated."""
        name = f"{route.method} {route.path}" if route is not None else f"{method} /{path}"
        if route is not None and (route.group == "link" or name in AGGREGATED):
            srv.traces.count(name, status, dt)
            return
        data = {"route": name, "status": status, "handler_s": round(dt, 6)}
        if resolved:
            data["resolved"] = [{k: r.get(k) for k in ("from", "to", "why")} for r in resolved]
        srv.traces.for_header(self.headers.get("X-AMP-Trace")).event(
            "server.request", data, lvl="info" if status < 500 else "error")

    def _error(self, e, path) -> int:
        if isinstance(e, SessionConflict):
            status, body = 409, e.body
        elif isinstance(e, NotFound):
            status, body = 404, {"ok": False, "message": str(e)}
        elif isinstance(e, Forbidden):
            status, body = 403, {"ok": False, "message": str(e)}
        elif isinstance(e, HTTPError):
            status, body = e.status, {"ok": False, "message": str(e)}
        elif isinstance(e, (ValueError, KeyError, TypeError)):
            msg = str(e) or type(e).__name__
            status, body = 400, {"ok": False, "message": msg}
            if path.startswith("sim/"):
                body["error"] = msg
        else:
            traceback.print_exc()
            status, body = 500, {"ok": False, "message": f"{type(e).__name__}: {e}"}
        if status == 400 and path == GRADE.path:
            body = {**body, "reason": "bad_request"}
        try:
            self._send(Reply(status, body))
        except _QUIET:
            self.close_connection = True
        return status

    def _send(self, reply: Reply, head=False) -> None:
        if isinstance(reply.body, (bytes, bytearray)):
            data, headers = bytes(reply.body), reply.headers
        else:
            body = redact(reply.body) if getattr(self, "_hide_table", False) else reply.body
            data = json.dumps(wire.jsonable(body), allow_nan=False).encode()
            headers = {"Cache-Control": "no-store", **reply.headers}
        self.send_response(reply.status)
        self.send_header("Content-Type", reply.content_type)
        self.send_header("Content-Length", str(len(data)))
        for k, v in headers.items():
            self.send_header(k, v)
        self.end_headers()
        if not head:
            self.wfile.write(data)


def build(cfg: ServerConfig, **kw) -> Server:
    """A Server for cfg."""
    return Server(cfg, **kw)


def make_server(srv: Server, host: str | None = None, port: int | None = None):
    return RobotHTTPServer((host or srv.settings.host,
                            srv.settings.port if port is None else port), srv)
