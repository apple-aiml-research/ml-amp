#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Settings: a ServerConfig parsed and validated once; nothing else reads configuration."""

from __future__ import annotations

import math
import socket
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.server import ServerConfig
from amp.planning.options import PROPOSAL_MODES

MAX_DURATION_S = 60.0
POLICY_LABEL = "Human Teleop"
DEFAULT_DURATION_S = 5.0
SPEED_SCALE = 0.8
SETTLE_S = 0.3
POLL_TIMEOUT_S = 1.0
OWNER_WINDOW_S = 10.0
SAVE_WINDOW_S = 60.0
PERCEPTION_INTERVAL_S = 0.5


def _finite(value, name, lo=None, hi=None) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number") from None
    if not math.isfinite(v) or (lo is not None and v < lo) or (hi is not None and v > hi):
        raise ValueError(f"{name} must be in [{lo}, {hi}], got {value!r}")
    return v


@dataclass(frozen=True)
class Tracking:
    """The executor's lead compensation and held-object guard (OLD's track_* config)."""

    lead: bool = True
    handoff: bool = True
    abort_mm: float = 200.0
    abort_rad: float = 0.5
    release_mm: float = 30.0


@dataclass(frozen=True)
class Settings:
    """Everything the server's components read from configuration."""

    port: int
    host: str
    state_dir: Path
    results_dir: str
    harness: str
    info_mode: str
    obstacles: str
    cameras: str
    planner: str
    worker: str
    public_url: str
    perception_floor: bool = True
    idle_timeout_s: float = 1800.0
    load_timeout_s: float = 900.0
    episode_timeout_s: float = 21600.0
    tracking: Tracking = field(default_factory=Tracking)
    keep_camera_roles: tuple[str, ...] = ()
    drop_depth: bool = False
    hide_calibration: bool = False
    hide_table_height: bool = False
    table_obstacle: bool = True
    gripper_reading: str = "measured"
    task_notes: bool = False
    assist_proposals: str = "v1"
    held_tracking: str = "plans"

    @property
    def measured_gripper(self) -> bool:
        """Whether an agent reply may carry the gripper's measured reading."""
        return self.gripper_reading == GRIPPER_READINGS[0]


EXTRA_KEYS = ("track_lead", "track_handoff", "track_abort_mm", "track_abort_rad",
              "track_release_mm", "mobile_base", "keep_camera_roles", "drop_depth",
              "perception_floor", "observation_embodiment", "hide_table_height", "table_obstacle",
              "hide_calibration", "gripper_reading", "task_notes", "assist_place_variants",
              "assist_proposals", "held_tracking")
FIXED = {"mobile_base": "optional", "observation_embodiment": True,
         "assist_place_variants": "flip"}
_TRUE, _FALSE = ("1", "true", "yes", "on"), ("0", "false", "no", "off", "")
HELD_TRACKING_MODES = ("plans", "gripper")


def _bool(value, name) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in _TRUE + _FALSE:
        return value.strip().lower() in _TRUE
    raise ValueError(f"{name} must be true or false, got {value!r}")


def _check_fixed(extra) -> None:
    """ValueError when a FIXED key asks for a behaviour other than the server's one."""
    for k, want in FIXED.items():
        if k not in extra:
            continue
        v = extra[k]
        got = _bool(v, k) if isinstance(want, bool) else \
            (v.strip().lower() if isinstance(v, str) else v)
        if got != want:
            raise ValueError(f"{k} must be {str(want).lower()}, the only mode, got {v!r}")


GRIPPER_READINGS = ("measured", "command")


def _gripper_reading(value) -> str:
    """gripper_reading as one of GRIPPER_READINGS."""
    v = str(value or GRIPPER_READINGS[0]).strip().lower()
    if v not in GRIPPER_READINGS:
        raise ValueError(f"gripper_reading must be one of {list(GRIPPER_READINGS)}, got {value!r}")
    return v


CAMERA_ROLES = ("scene", "wrist")


def _roles(value) -> tuple[str, ...]:
    """The keep_camera_roles roles, each one of CAMERA_ROLES; () keeps every camera."""
    roles = [r.strip() for r in (value.split(",") if isinstance(value, str) else value or ())]
    roles = [r for r in roles if r]
    unknown = [r for r in roles if r not in CAMERA_ROLES]
    if unknown:
        raise ValueError(f"keep_camera_roles {unknown} not among the camera roles {list(CAMERA_ROLES)}")
    return tuple(roles)


def _proposals(value) -> str:
    """assist_proposals as one of PROPOSAL_MODES."""
    mode = value.strip().lower() if isinstance(value, str) else value
    if mode not in PROPOSAL_MODES:
        raise ValueError(f"assist_proposals must be one of {list(PROPOSAL_MODES)}, got {value!r}")
    return mode


def _held_tracking(value) -> str:
    """held_tracking as one of HELD_TRACKING_MODES."""
    mode = value.strip().lower() if isinstance(value, str) else value
    if mode not in HELD_TRACKING_MODES:
        raise ValueError(f"held_tracking must be one of {list(HELD_TRACKING_MODES)}, "
                         f"got {value!r}")
    return mode


def from_config(cfg: ServerConfig) -> Settings:
    """Settings of a ServerConfig; ValueError on an unknown or out-of-range extra entry."""
    extra = dict(cfg.extra or {})
    unknown = set(extra) - set(EXTRA_KEYS)
    if unknown:
        raise ValueError(f"unknown ServerConfig.extra keys {sorted(unknown)} (from the "
                         f"harness's server env?); accepted: {', '.join(EXTRA_KEYS)}")
    _check_fixed(extra)
    tracking = Tracking(
        lead=_bool(extra.get("track_lead", True), "track_lead"),
        handoff=_bool(extra.get("track_handoff", True), "track_handoff"),
        abort_mm=_finite(extra.get("track_abort_mm", 200.0), "track_abort_mm", 0.0, 10000.0),
        abort_rad=_finite(extra.get("track_abort_rad", 0.5), "track_abort_rad", 0.0, 10.0),
        release_mm=_finite(extra.get("track_release_mm", 30.0), "track_release_mm", 0.0,
                           10000.0))
    return Settings(
        port=int(cfg.port), host=cfg.host, state_dir=Path(cfg.state_dir),
        results_dir=cfg.results_dir, harness=cfg.harness,
        info_mode=cfg.info_mode, obstacles=cfg.obstacles,
        cameras=cfg.cameras, planner=cfg.planner,
        worker=f"{socket.gethostname()}:{cfg.port}", public_url=f"http://127.0.0.1:{cfg.port}",
        perception_floor=_bool(extra.get("perception_floor", True), "perception_floor"),
        idle_timeout_s=float(cfg.idle_timeout_s), load_timeout_s=float(cfg.load_timeout_s),
        episode_timeout_s=float(cfg.episode_timeout_s), tracking=tracking,
        keep_camera_roles=_roles(extra.get("keep_camera_roles", "")),
        drop_depth=_bool(extra.get("drop_depth", False), "drop_depth"),
        hide_table_height=_bool(extra.get("hide_table_height", False), "hide_table_height"),
        table_obstacle=_bool(extra.get("table_obstacle", True), "table_obstacle"),
        hide_calibration=_bool(extra.get("hide_calibration", False), "hide_calibration"),
        gripper_reading=_gripper_reading(extra.get("gripper_reading", "")),
        task_notes=_bool(extra.get("task_notes", False), "task_notes"),
        assist_proposals=_proposals(extra.get("assist_proposals", "v1")),
        held_tracking=_held_tracking(extra.get("held_tracking", "plans")),
    )
