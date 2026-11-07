#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The loop's own gateway client: episode status, the observation, frames, finish, keep-alive."""

from __future__ import annotations

import json
import time
import urllib.error
import urllib.request
from pathlib import Path

from amp.contracts.eval import OFFICIAL_KEY_HEADER

END_RETRIES = 3
KEEPALIVE_BODY = {"until": "frame", "after_seq": 999999999, "timeout_s": 0.1}
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


class GatewayError(RuntimeError):
    def __init__(self, message, status=None, path=None, body=None):
        super().__init__(message)
        self.status, self.path, self.body = status, path, body


class EpisodeOver(GatewayError):
    """The episode has ended (HTTP 410)."""


class EpisodeClient:
    """HTTP under the episode's gateway base: <gateway>/s/<token>/."""

    def __init__(self, base_url: str, timeout_s: float = 120.0):
        self.base_url = base_url.rstrip("/") + "/"
        self.timeout_s = timeout_s

    @classmethod
    def from_file(cls, path: str | Path) -> EpisodeClient:
        info = json.loads(Path(path).read_text())
        base = info.get("base") or f"s/{info['token']}/"
        return cls(info["gateway"].rstrip("/") + "/" + base.lstrip("/"))

    def request(self, method: str, path: str, body=None, *, binary=False, timeout=None,
                retries=None, headers=None):
        url = self.base_url + path.lstrip("/")
        data = None if body is None else json.dumps(body).encode()
        headers = {**({"Content-Type": "application/json"} if data is not None else {}),
                   **(headers or {})}
        n_try = (3 if method == "GET" else 1) if retries is None else retries
        last = None
        for attempt in range(n_try):
            req = urllib.request.Request(url, data=data, headers=headers, method=method)
            try:
                with _OPENER.open(req, timeout=timeout or self.timeout_s) as r:
                    raw = r.read()
                    if binary:
                        return raw
                    return json.loads(raw) if raw else {}
            except urllib.error.HTTPError as e:
                raw = e.read()
                try:
                    parsed = json.loads(raw)
                    msg = parsed.get("message") or raw.decode()[:300]
                except (ValueError, AttributeError):
                    parsed, msg = None, raw.decode(errors="replace")[:300] or e.reason
                raise (EpisodeOver if e.code == 410 else GatewayError)(
                    str(msg), e.code, path, parsed if isinstance(parsed, dict) else None) from None
            except (urllib.error.URLError, ConnectionError, TimeoutError) as e:
                last = e
                if attempt + 1 < n_try:
                    time.sleep(1.0 + attempt)
        raise GatewayError(f"cannot reach the gateway for {path}: {type(last).__name__}", None,
                           path)

    def status(self) -> dict:
        """{"state": loading|ready|over, "end", "idle_s", ...}."""
        return self.request("GET", "_episode")

    def observe(self) -> dict:
        return self.request("GET", "api/agent/observation")

    def frame(self, cam: str) -> bytes:
        """The camera's latest RGB frame as the gateway encodes it (JPEG)."""
        return self.request("GET", f"api/frame/{cam}", binary=True)

    def finish(self) -> dict:
        return self.request("POST", "_episode/finish", {})

    def keepalive(self) -> dict:
        return self.request("POST", "api/agent/wait", dict(KEEPALIVE_BODY), timeout=30)

    def official_begin(self, key: str) -> dict:
        """{"cell", "interrupt_in_s"[, "official_timeout"]}; 409 once the attempt has ended."""
        return self.request("POST", "_official/cell", {"op": "begin"},
                            headers={OFFICIAL_KEY_HEADER: key})

    def official_end(self, key: str, cell: int, interrupted: bool) -> dict:
        """Idempotent by cell number, so it is retried on a network failure."""
        return self.request("POST", "_official/cell",
                            {"op": "end", "cell": cell, "interrupted": interrupted},
                            headers={OFFICIAL_KEY_HEADER: key}, retries=END_RETRIES)

    def official_model_timeout(self, key: str) -> dict:
        return self.request("POST", "_official/model_timeout", {},
                            headers={OFFICIAL_KEY_HEADER: key}, retries=END_RETRIES)

    def copy(self) -> EpisodeClient:
        return EpisodeClient(self.base_url, self.timeout_s)
