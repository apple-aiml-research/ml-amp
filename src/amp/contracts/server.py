#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot API server (amp.robot_server): its route table, process config, CLI."""

from __future__ import annotations

import json
import re
from dataclasses import asdict, dataclass, field
from typing import Any

from amp.contracts.embodiment import DROID, accepted
from amp.contracts.harness import CAMERA_SETS, INFO_MODES, OBSTACLE_MODES

ROUTE_GROUPS = ("lifecycle", "link", "sensors", "agent", "assist", "motion")
EPISODE_STATES = ("idle", "loading", "ready", "running", "ended")
PLANNERS = ("scene",)
_PARAM = re.compile(r"\{([a-z_]+)(\*?)\}")
_SEGMENT = r"(?!\.\.?(?:/|$))[A-Za-z0-9_][A-Za-z0-9_.=&,-]*"


@dataclass(frozen=True)
class Route:
    """One served route. {name} matches one path segment, a trailing {name*} the rest."""

    method: str
    path: str
    group: str
    executes: bool = False
    summary: str = ""

    def __post_init__(self):
        if self.method not in ("GET", "POST"):
            raise ValueError(f"{self.path}: method {self.method!r}")
        if self.group not in ROUTE_GROUPS:
            raise ValueError(f"{self.path}: group {self.group!r} not in {ROUTE_GROUPS}")
        if self.path.startswith("/"):
            raise ValueError(f"{self.path}: route paths carry no leading slash")

    def regex(self) -> str:
        out, pos = [], 0
        for m in _PARAM.finditer(self.path):
            out.append(re.escape(self.path[pos:m.start()]))
            seg = _SEGMENT if not m.group(2) else rf"{_SEGMENT}(?:/{_SEGMENT})*"
            out.append(f"(?P<{m.group(1)}>{seg})")
            pos = m.end()
        out.append(re.escape(self.path[pos:]))
        return "".join(out)

    def match(self, method: str, path: str) -> dict[str, str] | None:
        if method != self.method:
            return None
        m = re.fullmatch(self.regex(), path)
        return m.groupdict() if m else None


@dataclass(frozen=True)
class ServerConfig:
    """One robot-server process."""

    port: int
    state_dir: str
    harness: str = ""
    results_dir: str = ""
    cameras: str = "droid"
    info_mode: str = "standard"
    obstacles: str = "scene"
    planner: str = "scene"
    depth_stride: int = 4
    idle_timeout_s: float = 1800.0
    load_timeout_s: float = 900.0
    episode_timeout_s: float = 21600.0
    host: str = "127.0.0.1"
    extra: dict[str, Any] = field(default_factory=dict)
    embodiments: tuple[str, ...] = (DROID,)

    def __post_init__(self):
        object.__setattr__(self, "embodiments", accepted(self.embodiments))
        if self.cameras not in CAMERA_SETS:
            raise ValueError(f"cameras {self.cameras!r}")
        if self.info_mode not in INFO_MODES:
            raise ValueError(f"info_mode {self.info_mode!r}")
        if self.obstacles not in OBSTACLE_MODES:
            raise ValueError(f"obstacles {self.obstacles!r}")
        if self.planner not in PLANNERS:
            raise ValueError(f"planner {self.planner!r}")

    def to_json(self) -> str:
        d = asdict(self)
        if self.embodiments == (DROID,):
            del d["embodiments"]
        return json.dumps(d, sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> ServerConfig:
        return cls(**json.loads(text))
