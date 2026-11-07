#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The route-policy engine: load a harness's routes.json, key a request, filter a reply."""

from __future__ import annotations

import json
import re
import threading
from dataclasses import dataclass
from pathlib import Path

from amp.contracts.harness import ROUTE_POLICY_FORMAT, RoutePolicy, ServerSpec
from amp.fs.harnesses import ensure_harness

ROUTES_FILE = "routes.json"
MANIFEST_FILE = "manifest.json"
HARNESS_NAME = re.compile(r"[a-z][a-z0-9_]*")


def policy_from_dict(d) -> RoutePolicy:
    """A RoutePolicy, with every malformed input reported as ValueError."""
    if not isinstance(d, dict) or d.get("format") != ROUTE_POLICY_FORMAT:
        got = d.get("format") if isinstance(d, dict) else type(d).__name__
        raise ValueError(f"route policy format {got!r} != {ROUTE_POLICY_FORMAT}")
    if not isinstance(d.get("name"), str) or not d["name"]:
        raise ValueError("route policy needs a name")
    rules = d.get("rules")
    if not isinstance(rules, list) or not rules:
        raise ValueError("route policy has no rules")
    for i, r in enumerate(rules):
        if not isinstance(r, dict):
            raise ValueError(f"route policy rule {i} is not an object")  # noqa: TRY004
        m = r.get("methods")
        if not isinstance(m, list) or not m or not all(isinstance(x, str) for x in m):
            raise ValueError(f"route policy rule {i}: methods must be a list of strings")
        if r.get("pattern"):
            try:
                re.compile(r["pattern"])
            except (re.error, TypeError) as e:
                raise ValueError(f"route policy rule {i}: bad pattern {r['pattern']!r}: {e}") \
                    from None
    ex = d.get("executes", [])
    if not isinstance(ex, list) or not all(isinstance(x, str) for x in ex):
        raise ValueError("route policy executes must be a list of strings")
    rd = d.get("response_drop", {})
    if isinstance(rd, dict) and any(
            isinstance(f, str) and (not f or "" in f.split("."))
            for v in rd.values() if isinstance(v, list) for f in v):
        raise ValueError("response_drop field names must be non-empty dotted names")
    try:
        return RoutePolicy.from_dict(d)
    except (KeyError, TypeError) as e:
        raise ValueError(f"bad route policy: {e!r}") from None


def load_policy(path: str | Path) -> RoutePolicy:
    return policy_from_dict(json.loads(Path(path).read_text()))


def drop_counts(obj, fields) -> dict[str, int]:
    """Delete each dotted field from a JSON value in place; through a list, the rest of the name
    applies to each element."""
    out = {}
    for f in fields:
        n = _drop(obj, f.split("."))
        if n:
            out[f] = n
    return out


def _drop(obj, parts) -> int:
    if isinstance(obj, list):
        return sum(_drop(x, parts) for x in obj)
    if not isinstance(obj, dict) or parts[0] not in obj:
        return 0
    if len(parts) == 1:
        del obj[parts[0]]
        return 1
    return _drop(obj[parts[0]], parts[1:])


@dataclass(frozen=True)
class HarnessPolicy:
    """What the gateway applies to one harness's episodes."""

    name: str
    policy: RoutePolicy
    server: ServerSpec
    source: Path


class PolicyStore:
    """harness -> HarnessPolicy, from <harness_dir>/<name>/routes.json and manifest.json."""

    def __init__(self, harness_dir: str | Path = "", default_dir: str | Path | None = None):
        self.root = Path(harness_dir) if harness_dir else Path(default_dir) if default_dir else None
        self._cache: dict[str, tuple[tuple, HarnessPolicy]] = {}
        self._lock = threading.Lock()

    def routes_path(self, name: str) -> Path:
        """Where name's routes.json is; KeyError if nowhere."""
        if not isinstance(name, str) or not HARNESS_NAME.fullmatch(name) or self.root is None:
            raise KeyError(name)
        return self.root / name / ROUTES_FILE

    def get(self, name: str) -> HarnessPolicy:
        """The policy; KeyError if there is no such harness, ValueError if it is bad."""
        routes = self.routes_path(name)
        manifest = routes.parent / MANIFEST_FILE
        if not routes.is_file():
            ensure_harness(routes.parent.parent, name)
        try:
            stamp = (str(routes), _stamp(routes, required=True), _stamp(manifest))
        except FileNotFoundError:
            raise KeyError(name) from None
        key = name
        with self._lock:
            hit = self._cache.get(key)
            if hit is not None and hit[0] == stamp:
                return hit[1]
        server = ServerSpec()
        if stamp[2] is not None:
            server = ServerSpec(**(json.loads(manifest.read_text()).get("server") or {}))
        hp = HarnessPolicy(name, load_policy(routes), server, routes)
        with self._lock:
            self._cache[key] = (stamp, hp)
        return hp

    def loaded(self) -> list[str]:
        with self._lock:
            return sorted(self._cache)


def _stamp(path: Path, required: bool = False):
    try:
        st = path.stat()
    except FileNotFoundError:
        if required:
            raise
        return None
    return (st.st_mtime_ns, st.st_size)
