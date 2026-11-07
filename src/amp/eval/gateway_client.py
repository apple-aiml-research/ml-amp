#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The episode runner's client of the local gateway: its keyed runner routes and the episode routes."""

from __future__ import annotations

import http.client
import json
from collections.abc import Callable
from urllib.parse import quote, urlsplit

KEY_HEADER = "X-Harness-Key"
TIMEOUT_S = 30.0
LOAD_TIMEOUT = "not ready after the load timeout"
ENDED_LOADING = "the episode ended while loading: "


class GatewayClient:
    """(status, JSON dict) per call; raises OSError when the gateway cannot be reached."""

    def __init__(self, url: str, key_fn: Callable[[], str], *, timeout_s: float = TIMEOUT_S):
        u = urlsplit(url)
        if u.scheme != "http" or not u.hostname:
            raise ValueError(f"gateway URL must be http://host:port, got {url!r}")
        self.url = url.rstrip("/")
        self.host, self.port = u.hostname, u.port or 80
        self._key_fn = key_fn
        self.timeout_s = timeout_s

    def __repr__(self) -> str:
        return f"GatewayClient({self.url!r})"

    def request(self, method: str, path: str, body: dict | None = None, *, keyed: bool = False,
                timeout: float | None = None) -> tuple[int, dict]:
        conn = http.client.HTTPConnection(self.host, self.port, timeout=timeout or self.timeout_s)
        try:
            headers = {}
            data = None
            if body is not None:
                data = json.dumps(body).encode()
                headers["Content-Type"] = "application/json"
            if keyed:
                headers[KEY_HEADER] = self._key_fn()
            try:
                conn.request(method, "/" + path.lstrip("/"), body=data, headers=headers)
                resp = conn.getresponse()
                raw = resp.read()
            except http.client.HTTPException as e:
                raise OSError(f"gateway {self.url}: {type(e).__name__}: {e}") from None
            return resp.status, _json(raw)
        finally:
            conn.close()

    def claim(self, *, agent: str, harness: str, task: str, episode: int,
              slot: str | None = None,
              run_id: str | None = None, task_set: str | None = None,
              instruction_type: str = "default",
              official: dict | None = None, claim_id: str | None = None) -> tuple[int, dict]:
        """run_id scopes saved episodes per run; task_set may let one run hold two of a task."""
        body = {"agent": agent, "harness": harness, "task": task, "episode": int(episode)}
        if slot is not None:
            body["slot"] = slot
        if run_id:
            body["run_id"] = run_id
        if task_set:
            body["task_set"] = task_set
        if instruction_type != "default":
            body["instruction_type"] = instruction_type
        if official is not None:
            body["official"] = dict(official)
        if claim_id:
            body["claim_id"] = claim_id
        return self.request("POST", "harness/claim", body, keyed=True)

    def official_start(self, token: str) -> tuple[int, dict]:
        """Start the official episode's trial clock: the agent starts now."""
        return self.request("POST", "harness/official", {"token": token, "op": "start"},
                            keyed=True)

    def release(self, token: str, discard: bool = False) -> tuple[int, dict]:
        """End the episode: published if the robot moved, else (or with discard) discarded."""
        body = {"token": token, **({"discard": True} if discard else {})}
        return self.request("POST", "harness/release", body, keyed=True)

    def finish(self, token: str) -> tuple[int, dict]:
        """Publish the episode as the agent would: scored whether or not the robot moved."""
        return self.request("POST", f"s/{quote(token, safe='')}/_episode/finish", {})

    def status(self) -> tuple[int, dict]:
        return self.request("GET", "harness/status", keyed=True)

    def episode(self, episode_id: str, *, timeout: float | None = None) -> tuple[int, dict]:
        """{"over": false, "live": {...executes...}} or {"over": true, "row": ledger row}."""
        return self.request("GET", f"harness/episodes/{quote(episode_id, safe='')}", keyed=True,
                            timeout=timeout)

    def agent_episode(self, token: str) -> tuple[int, dict]:
        """The agent's own view: state, idle_s, wall_s, end."""
        return self.request("GET", f"s/{quote(token, safe='')}/_episode")


def _json(raw: bytes) -> dict:
    try:
        d = json.loads(raw or b"{}")
    except ValueError:
        return {}
    return d if isinstance(d, dict) else {}
