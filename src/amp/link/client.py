#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Client side of the /sim/* link: stdlib HTTP, idempotent retries, long-poll until an action."""

from __future__ import annotations

import http.client
import json
import math
import threading
import time
import urllib.error
import urllib.request
import uuid
from collections.abc import Callable
from typing import Any

import numpy as np

from amp.contracts.embodiment import EmbodimentSpec, RobotDescription
from amp.contracts.robot import ACTION_DIM
from amp.contracts.trace import NullTracer, Tracer
from amp.link.robot import Mesh, robot_payload
from amp.link.wire import CONTENT_TYPE, PROTOCOL_VERSION, jsonable, pack_frame

DEFAULT_URL = "http://127.0.0.1:8765"
WAIT_LOG_EVERY = 60
HELLO_WAIT_S = 120.0
HELLO_RETRY_S = 1.0


class LinkError(RuntimeError):
    """The robot server is unreachable or replied with an error (a 4xx's status and JSON body)."""

    def __init__(self, message: str = "", status: int | None = None, body: dict | None = None):
        super().__init__(message)
        self.status = status
        self.body = body if isinstance(body, dict) else {}


class EpisodeAborted(LinkError):
    """The server answered a step or poll with abort: end the episode unscored."""

    def __init__(self, reason: str = ""):
        super().__init__(f"episode aborted by the robot server: {reason or 'no reason given'}")
        self.reason = reason


def _json_object(detail: str) -> dict | None:
    try:
        body = json.loads(detail)
    except ValueError:
        return None
    return body if isinstance(body, dict) else None


def _server_message(detail: str) -> str:
    try:
        body = json.loads(detail)
    except ValueError:
        return detail[:500]
    if isinstance(body, dict) and (body.get("error") or body.get("message")):
        return str(body.get("error") or body.get("message"))
    return detail[:500]


def _no_proxy_opener() -> urllib.request.OpenerDirector:
    return urllib.request.build_opener(urllib.request.ProxyHandler({}))


class LinkClient:
    """Speaks every /sim/* request of the link protocol, benchmark mode's included."""


    def __init__(
        self,
        base_url: str = DEFAULT_URL,
        *,
        timeout_s: float = 30.0,
        max_retries: int = 3,
        client: str = "robolab",
        client_id: str | None = None,
        tracer: Tracer | None = None,
        opener: Any = None,
        sleep: Callable[[float], None] = time.sleep,
        embodiment: EmbodimentSpec | None = None,
        robot: tuple[RobotDescription, dict[str, Mesh]] | None = None,
        on_wait: Callable[[], None] | None = None,
        hello_wait_s: float = HELLO_WAIT_S,
        features: tuple[str, ...] = (),
        on_grade: Callable[[Any], dict | None] | None = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.timeout_s = float(timeout_s)
        self.max_retries = max(int(max_retries), 0)
        self.client = client
        self.client_id = client_id or uuid.uuid4().hex
        self.server_info: dict | None = None
        self.finish: dict | None = None
        self.annotated: bool = False
        self.select_arm: str | None = None
        self.tracer = tracer or NullTracer(f"robot/{client}")
        self._kind = f"robot.{client}.link"
        self._opener = opener or _no_proxy_opener()
        self._sleep = sleep
        self.hello_wait_s = float(hello_wait_s)
        self._lock = threading.Lock()
        self._request_id = 0
        self.embodiment = embodiment
        self.robot = robot
        self.action_dim = ACTION_DIM if embodiment is None else embodiment.action_dim
        self.on_wait = on_wait
        self.features = tuple(features)
        self.on_grade = on_grade
        self.hello_required = False

    def next_request_id(self) -> int:
        with self._lock:
            self._request_id += 1
            return self._request_id

    def _post(
        self,
        path: str,
        body: bytes,
        content_type: str,
        request_id: int,
        *,
        timeout_s: float | None = None,
        max_retries: int | None = None,
    ) -> dict:
        url = self.base_url + path
        timeout = self.timeout_s if timeout_s is None else float(timeout_s)
        retries = self.max_retries if max_retries is None else max(int(max_retries), 0)
        last: Exception | None = None
        for attempt in range(retries + 1):
            if attempt:
                self._sleep(min(0.2 * 2 ** (attempt - 1), 2.0))
            req = urllib.request.Request(
                url, data=body, method="POST", headers={"Content-Type": content_type}
            )
            try:
                with self._opener.open(req, timeout=timeout) as resp:
                    raw = resp.read()
            except urllib.error.HTTPError as exc:
                detail = exc.read()[:2000].decode("utf-8", "replace")
                if exc.code < 500:
                    raise LinkError(f"robot server {url}: HTTP {exc.code}: "
                                    f"{_server_message(detail)}", exc.code,
                                    _json_object(detail)) from exc
                last = exc
                self._retrying(url, attempt, f"HTTP {exc.code}: {detail[:500]}")
                continue
            except (urllib.error.URLError, OSError, http.client.HTTPException) as exc:
                last = exc
                self._retrying(url, attempt, str(exc))
                continue
            return self._reply(url, raw, request_id)
        raise LinkError(f"robot server {url} unreachable after {retries + 1} attempts: {last}")

    def _retrying(self, url: str, attempt: int, error: str) -> None:
        self.tracer.event(f"{self._kind}.retry", {"url": url, "attempt": attempt + 1,
                                                "error": error}, lvl="warn")

    @staticmethod
    def _reply(url: str, raw: bytes, request_id: int) -> dict:
        try:
            reply = json.loads(raw)
        except ValueError as exc:
            raise LinkError(f"robot server {url}: non-JSON reply {raw[:200]!r}") from exc
        if not isinstance(reply, dict):
            raise LinkError(f"robot server {url}: reply is not an object: {reply!r}")
        rid = reply.get("request_id")
        if rid is not None and rid != request_id:
            raise LinkError(
                f"robot server {url}: reply for request_id {rid}, expected {request_id}")
        return reply

    def _json_body(self, rid: int, body: dict) -> bytes:
        body = {"request_id": rid, "client_id": self.client_id, **body}
        return json.dumps(jsonable(body)).encode()

    def _post_json(self, path: str, body: dict, **post_kwargs) -> dict:
        rid = self.next_request_id()
        reply = self._post(path, self._json_body(rid, body), "application/json", rid, **post_kwargs)
        if "error" in reply:
            raise LinkError(f"robot server {self.base_url}{path}: {reply['error']}")
        return reply

    def hello(self, *, takeover: bool = False) -> dict:
        """Claim the sim link, waiting out a dead owner (takeover=True evicts any); send the robot."""
        body: dict = {"client": self.client, "protocol": PROTOCOL_VERSION,
                      "action_dim": self.action_dim}
        if self.embodiment is not None:
            body["embodiment"] = self.embodiment.to_dict()
        if takeover:
            body["takeover"] = True
        if self.features:
            body["features"] = list(self.features)
        waited = 0.0
        while True:
            try:
                reply = self._post_json("/sim/hello", body)
                break
            except LinkError as exc:
                seen, left = exc.body.get("owner_seen"), exc.body.get("retry_after_s")
                ok = (isinstance(left, int | float) and not isinstance(left, bool)
                      and math.isfinite(left) and left >= 0)
                if (exc.status != 409 or not isinstance(seen, str) or not ok
                        or waited >= self.hello_wait_s or waited + left > self.hello_wait_s):
                    raise
                body["owner_seen"] = seen
                self.tracer.event(f"{self._kind}.hello_wait", {"waited_s": waited,
                                                               "retry_after_s": left}, lvl="warn")
                delay = min(HELLO_RETRY_S, max(float(left), 0.05))
                self._sleep(delay)
                waited += delay
        if reply.get("protocol", PROTOCOL_VERSION) != PROTOCOL_VERSION:
            raise LinkError(f"robot server {self.base_url} speaks protocol "
                            f"{reply.get('protocol')}, this client {PROTOCOL_VERSION}")
        self.server_info = reply
        self.hello_required = False
        if self.robot is not None:
            self.send_robot(*self.robot)
        return reply

    def send_robot(self, description: RobotDescription, meshes: dict[str, Mesh]) -> dict:
        """POST /sim/robot: the robot description and its link meshes; raises if refused."""
        header, blobs = robot_payload(description, meshes)
        rid = self.next_request_id()
        frame = pack_frame({**header, "request_id": rid, "client_id": self.client_id}, blobs)
        reply = self._post("/sim/robot", frame, CONTENT_TYPE, rid)
        if "error" in reply or reply.get("ok") is False:
            raise LinkError(
                f"robot server {self.base_url}/sim/robot: {reply.get('error') or reply}")
        return reply

    def begin_episode(
        self,
        episode: int,
        *,
        task: str,
        instruction: str,
        max_steps: int | None,
        control_dt: float,
        job_id: int | None = None,
    ) -> dict:
        body = {"episode": episode, "task": task, "instruction": instruction,
                "max_steps": max_steps, "control_dt": control_dt}
        if job_id is not None:
            body["job_id"] = int(job_id)
        return self._post_json("/sim/episode", body)

    def send_catalog(
        self, *, policy_label: str, output_dir: str, episodes_per_task: int, tasks: list[dict]
    ) -> dict:
        """POST /sim/catalog: the benchmark's task list and results folder."""
        reply = self._post_json("/sim/catalog", {
            "policy_label": policy_label, "output_dir": output_dir,
            "episodes_per_task": int(episodes_per_task), "tasks": tasks})
        self._note_hello_required(reply)
        return reply

    def next_job(self, loaded: str | None, *, failed: dict | None = None) -> dict | None:
        """Long-poll /sim/next once; the job dict, or None when the server timed out."""
        body: dict = {"loaded": loaded}
        if failed is not None:
            body["failed"] = failed
        reply = self._post_json("/sim/next", body)
        job = reply.get("job")
        if job is not None and not isinstance(job, dict):
            raise LinkError(f"robot server {self.base_url}/sim/next: bad job {job!r}")
        self._note_hello_required(reply)
        return job

    def _note_hello_required(self, reply: dict) -> None:
        """Record that the server has no hello on record; the caller decides when to send it."""
        if reply.get("hello_required") is True:
            self.hello_required = True

    def report_saved(self, *, job_id: int | None, task: str, episode: int, record: dict) -> dict:
        """POST /sim/saved with the episode_results.jsonl record RoboLab appended."""
        return self._post_json("/sim/saved", {"job_id": job_id, "task": task,
                                              "episode": int(episode), "record": record})

    def _action(self, reply: dict) -> np.ndarray | None:
        """The (action_dim,) action of a step/poll reply, or None for wait."""
        url = self.base_url
        if "error" in reply:
            raise LinkError(f"robot server {url}: {reply['error']}")
        if reply.get("abort") is not None:
            abort = reply["abort"]
            raise EpisodeAborted(str(abort.get("reason", "")) if isinstance(abort, dict) else "")
        if reply.get("action") is not None:
            action = np.asarray(reply["action"], dtype=np.float32).reshape(-1)
            if action.shape != (self.action_dim,) or not np.all(np.isfinite(action)):
                raise LinkError(f"robot server {url}: bad action {reply['action']!r}")
            finish = reply.get("finish")
            self.finish = dict(finish) if isinstance(finish, dict) else None
            sel = reply.get("select_arm")
            self.annotated = "select_arm" in reply
            self.select_arm = sel if isinstance(sel, str) else None
            return action
        if reply.get("wait"):
            return None
        raise LinkError(f"robot server {url}: reply has no action, wait or error: {reply}")

    def step(
        self, header: dict, blobs: dict | None = None, encodings: dict | None = None
    ) -> np.ndarray:
        """Send one observation; block (long-polling) until the server dispenses an action."""
        rid = self.next_request_id()
        frame = pack_frame(
            {**header, "request_id": rid, "client_id": self.client_id}, blobs, encodings
        )
        reply = self._post("/sim/step", frame, CONTENT_TYPE, rid)
        action = self._action(reply)
        poll = {"episode": header.get("episode"), "step": header.get("step")}
        polls = 0
        while action is None:
            graded = self._graded(reply)
            if graded is None and self.on_wait is not None:
                self.on_wait()
            polls += 1
            if polls % WAIT_LOG_EVERY == 0:
                self.tracer.event(f"{self._kind}.waiting", {**poll, "polls": polls})
            rid = self.next_request_id()
            body = poll if graded is None else {**poll, "graded": graded}
            reply = self._post("/sim/poll", self._json_body(rid, body), "application/json", rid)
            action = self._action(reply)
        return action

    def _graded(self, reply: dict) -> dict | None:
        """A wait reply's grade request, graded by on_grade: the next poll's "graded", else None."""
        grade = reply.get("grade")
        if not isinstance(grade, dict):
            return None
        cell = grade.get("cell")
        if self.on_grade is None:
            return {"cell": cell, "error": "this backend does not grade"}
        return {**(self.on_grade(cell) or {}), "cell": cell}

    def end_episode(
        self,
        episode: int,
        *,
        reason: str = "reset",
        result: dict | None = None,
        timeout_s: float | None = None,
        max_retries: int | None = None,
    ) -> dict:
        return self._post_json("/sim/end", {"episode": episode, "reason": reason, "result": result},
                               timeout_s=timeout_s, max_retries=max_retries)
