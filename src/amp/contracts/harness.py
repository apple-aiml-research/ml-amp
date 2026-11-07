#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A robot harness: what the model may call and how."""

from __future__ import annotations

import dataclasses
import re
from collections.abc import Mapping
from dataclasses import dataclass, field

from amp.contracts.embodiment import DROID, accepted

ROUTE_POLICY_FORMAT = "amp-route-policy-1"
_GROUP = re.compile(r"\{(\d+)\}")
_HARNESS_NAME = re.compile(r"[a-z][a-z0-9_]*")


@dataclass(frozen=True)
class RouteRule:
    """One whitelist entry: `path` matches exactly, `pattern` with re.fullmatch; see RoutePolicy."""

    methods: tuple[str, ...]
    path: str = ""
    pattern: str = ""
    key: str = ""

    def __post_init__(self):
        if isinstance(self.methods, str) or not self.methods:
            raise ValueError(f"route rule methods must be a non-empty list, not {self.methods!r}")
        object.__setattr__(self, "methods", tuple(m.upper() for m in self.methods))
        if bool(self.path) == bool(self.pattern):
            raise ValueError("a route rule has exactly one of path or pattern")
        if self.pattern:
            groups = re.compile(self.pattern).groups
            if not self.key:
                raise ValueError(f"pattern rule {self.pattern!r} needs a key")
            used = [int(g) for g in _GROUP.findall(self.key)]
            if any(g < 1 or g > groups for g in used):
                raise ValueError(f"key {self.key!r} names a group {self.pattern!r} lacks")

    def match(self, method: str, rest: str) -> str | None:
        if method.upper() not in self.methods:
            return None
        if self.path:
            return (self.key or self.path) if rest == self.path else None
        m = re.fullmatch(self.pattern, rest)
        if m is None:
            return None
        return _GROUP.sub(lambda g: m.group(int(g.group(1))) or "", self.key)


@dataclass(frozen=True)
class RoutePolicy:
    """A harness's gateway whitelist: the first matching rule wins, no match is 403."""

    name: str
    rules: tuple[RouteRule, ...]
    executes: frozenset[str] = frozenset()
    response_drop: tuple[tuple[str, tuple[str, ...]], ...] = ()

    def key(self, method: str, rest: str) -> str | None:
        for r in self.rules:
            k = r.match(method, rest)
            if k is not None:
                return k
        return None

    def drops(self, key: str) -> tuple[str, ...]:
        return dict(self.response_drop).get(key, ())

    @classmethod
    def from_dict(cls, d: dict) -> RoutePolicy:
        fmt = d.get("format")
        if fmt != ROUTE_POLICY_FORMAT:
            raise ValueError(f"route policy format {fmt!r} != {ROUTE_POLICY_FORMAT}")
        rules = tuple(RouteRule(methods=r["methods"], path=r.get("path", ""),
                                pattern=r.get("pattern", ""), key=r.get("key", ""))
                      for r in d["rules"])
        rd = d.get("response_drop", {})
        if not isinstance(rd, dict) or not all(
                isinstance(k, str) and isinstance(v, list) and v and all(
                    isinstance(f, str) and f and "" not in f.split(".") for f in v)
                for k, v in rd.items()):
            raise ValueError("response_drop maps a ledger key to a non-empty list of field names")
        return cls(name=d["name"], rules=rules, executes=frozenset(d.get("executes", ())),
                   response_drop=tuple((k, tuple(v)) for k, v in rd.items()))


BOOTSTRAP_NAMES = ("show", "np", "PIL", "Image", "READY_TIMEOUT_S", "EPISODE_FILE")

INFO_MODES = ("standard",)
CAMERA_SETS = ("droid",)
OBSTACLE_MODES = ("scene", "table")


@dataclass(frozen=True)
class ServerSpec:
    """How a harness wants the robot server configured."""

    kind: str = "teleop"
    info_mode: str = "standard"
    cameras: str = "droid"
    obstacles: str = "scene"
    env: Mapping[str, str] = field(default_factory=dict)
    embodiments: tuple[str, ...] = (DROID,)

    def __post_init__(self):
        object.__setattr__(self, "embodiments", accepted(self.embodiments))
        if self.info_mode not in INFO_MODES:
            raise ValueError(f"info_mode {self.info_mode!r} not in {INFO_MODES}")
        if self.cameras not in CAMERA_SETS:
            raise ValueError(f"cameras {self.cameras!r} not in {CAMERA_SETS}")
        if self.obstacles not in OBSTACLE_MODES:
            raise ValueError(f"obstacles {self.obstacles!r} not in {OBSTACLE_MODES}")
        object.__setattr__(self, "env", dict(self.env))

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        if self.embodiments == (DROID,):
            del d["embodiments"]
        return d


@dataclass(frozen=True)
class HarnessSpec:
    """One harness, from amp/harnesses/<name>/manifest.json. Paths are relative to that dir."""

    name: str
    version: str
    description: str
    route_policy: str
    sdk_package: str
    sdk_exclude: tuple[str, ...] = ()
    bootstrap: str = ""
    preloaded: tuple[str, ...] = ()
    reference_docs: tuple[str, ...] = ()
    server: ServerSpec = ServerSpec()
    tool_mode: str = "code"
    trace_kinds: Mapping[str, str] = field(default_factory=dict)
    viewer: str | None = None

    def __post_init__(self):
        if not _HARNESS_NAME.fullmatch(self.name):
            raise ValueError(f"harness name {self.name!r} must match {_HARNESS_NAME.pattern}")
        for f in ("sdk_exclude", "preloaded", "reference_docs"):
            object.__setattr__(self, f, tuple(getattr(self, f)))
        object.__setattr__(self, "trace_kinds", dict(self.trace_kinds))
        prefix = f"harness.{self.name}."
        bad = [k for k in self.trace_kinds if not k.startswith(prefix)]
        if bad:
            raise ValueError(f"{self.name}: trace kinds must start with {prefix!r}: {bad}")

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d["server"] = self.server.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> HarnessSpec:
        d = dict(d)
        d["server"] = ServerSpec(**d.get("server", {}))
        return cls(**d)
