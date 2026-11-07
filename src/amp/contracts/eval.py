#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Episodes: the protocol, outcome statuses, requests and results, the episode directory, slots."""

from __future__ import annotations

import dataclasses
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

from amp.contracts import CONTRACT_VERSION
from amp.contracts.model import AgentProfile, ModelSpec

SANDBOX_BACKENDS = ("auto", "bwrap")
INSTRUCTION_TYPES = ("default", "vague", "specific")


@dataclass
class SandboxReport:
    """A sandbox self-test; ok only when every isolation check held."""

    backend: str
    ok: bool
    checks: dict[str, bool]
    details: dict[str, str] = field(default_factory=dict)
    platform: str = ""


@dataclass(frozen=True)
class EvalProtocol:
    """How every episode of a job runs; the sole owner of the sandbox choice and the timeouts."""

    zero_shot: bool = True
    idle_timeout_s: float = 1800.0
    episode_timeout_s: float = 21600.0
    load_timeout_s: float = 900.0
    sandbox: str = "auto"
    instruction_type: str = "default"
    official: bool = False

    def __post_init__(self):
        if self.sandbox not in SANDBOX_BACKENDS:
            raise ValueError(f"sandbox {self.sandbox!r} not in {SANDBOX_BACKENDS}")
        if self.instruction_type not in INSTRUCTION_TYPES:
            raise ValueError(f"instruction_type {self.instruction_type!r} not in {INSTRUCTION_TYPES}")
        if not isinstance(self.official, bool):
            raise ValueError(f"official {self.official!r} is not a bool")  # noqa: TRY004

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        if d["instruction_type"] == "default":
            del d["instruction_type"]
        if not d["official"]:
            del d["official"]
        return d

    @classmethod
    def from_dict(cls, d: dict | None) -> EvalProtocol:
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in (d or {}).items() if k in names})


UNIT_STATUSES = {
    "ok": "scored (success or failure) and delivered",
    "resumed": "already delivered by an earlier attempt; nothing run",
    "infra_error": "failed for an infra cause before the robot moved (an agent end the gateway "
                   "never scored included), retried; except void, which is terminal and unscored",
    "attempt_error": "failed after the robot moved, or scored after the agent ended it itself "
                     "before (a crash, no report, its context overflowing, its idle or "
                     "wall-clock limit); final, never run again",
    "lost": "the robot moved but no score was recorded (e.g. its worker lost the job); final, never run again",
    "canceled": "its job was canceled before or while it ran",
    "error": "deterministic preamble failure; never retried",
}
RETRYABLE_STATUSES = frozenset({"infra_error"})


def is_retryable(rec: dict) -> bool:
    """A unit record that may run again: infra_error, except a void one."""
    return rec.get("status") in RETRYABLE_STATUSES and rec.get("void") is not True


DELIVERED_STATUSES = frozenset({"ok", "resumed"})

CAPX_TRIAL_TIMEOUT_S = 1000
CAPX_MAX_TRIAL_ATTEMPTS = 3
OFFICIAL_KEY_HEADER = "X-AMP-Official-Key"
OFFICIAL_KEY_ENV = "AMP_OFFICIAL_KEY"

_SAFE = re.compile(r"[^A-Za-z0-9_.-]+")


@dataclass(frozen=True)
class Arm:
    """One cell of a study: a harness driven by an agent profile."""

    harness: str
    agent: AgentProfile

    @property
    def id(self) -> str:
        """<harness>__<model>[-<reasoning>]__<prompt>, then __<vision>-<vdm_model> for a vision
        other than image."""
        a = self.agent
        m = a.model.key + (f"-{a.reasoning}" if a.reasoning else "")
        tail = f"__{a.vision_label}" if a.vision_label else ""
        return _SAFE.sub("_", f"{self.harness}__{m}__{a.prompt}{tail}")

    def to_dict(self) -> dict:
        return {**dataclasses.asdict(self), "agent": self.agent.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> Arm:
        a = dict(d["agent"])
        a["model"] = ModelSpec(**a["model"])
        return cls(harness=d["harness"], agent=AgentProfile(**a))


@dataclass(frozen=True)
class EpisodeRequest:
    """One unit as the launcher hands it to the episode runner; local paths are the runtime's."""

    job_id: str
    arm: Arm
    task: str
    episode: int
    protocol: EvalProtocol
    code_ref: str
    contract: int = CONTRACT_VERSION
    task_set: str = "benchmark"
    official_attempt: int = 0

    def __post_init__(self):
        a = self.official_attempt
        if not isinstance(a, int) or isinstance(a, bool) or a not in (0, 1):
            raise ValueError(f"official_attempt {a!r} is not 0 or 1")

    @property
    def run_id(self) -> str:
        return f"{self.job_id}__{self.arm.id}"

    def to_dict(self) -> dict:
        d = {**dataclasses.asdict(self), "arm": self.arm.to_dict(),
             "protocol": self.protocol.to_dict()}
        if not d["official_attempt"]:
            del d["official_attempt"]
        return d

    @classmethod
    def from_dict(cls, d: dict) -> EpisodeRequest:
        d = dict(d)
        if d.get("contract", CONTRACT_VERSION) != CONTRACT_VERSION:
            raise ValueError(f"episode request contract {d.get('contract')} != {CONTRACT_VERSION}")
        d["arm"] = Arm.from_dict(d["arm"])
        d["protocol"] = EvalProtocol.from_dict(d.get("protocol"))
        return cls(**d)


@dataclass
class EpisodeResult:
    job_id: str
    arm_id: str
    task: str
    episode: int
    status: str
    attempted: bool = False
    void: bool = False
    episode_id: str | None = None
    success: bool | None = None
    score: float | None = None
    end: str | None = None
    end_reason: str | None = None
    cost_usd: float | None = None
    tokens: dict[str, int] | None = None
    turns: int | None = None
    wall_s: float | None = None
    timings: dict[str, float] = field(default_factory=dict)
    node: str | None = None
    slot: str | None = None
    sandbox: dict | None = None
    artifact_uri: str | None = None
    uploaded: bool | None = None
    error: str | None = None
    official: dict | None = None

    def __post_init__(self):
        if self.status not in UNIT_STATUSES:
            raise ValueError(f"status {self.status!r} not in {sorted(UNIT_STATUSES)}")

    def to_record(self) -> dict:
        """The unit record: this result plus `run` (= episode), the key the viewer's census uses."""
        rec = dataclasses.asdict(self)
        if rec["official"] is None:
            del rec["official"]
        rec["run"] = self.episode
        return rec

    @classmethod
    def from_record(cls, rec: dict) -> EpisodeResult:
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in rec.items() if k in names})


TOKEN_KEYS = ("input_tokens", "output_tokens", "thinking_tokens", "cache_read_tokens",
              "cache_write_tokens")

RUN_CONFIG_FILE = "run_config.json"

LIVE_PREFIX = "_live_"
EPISODE_FILE = "episode.json"
AGENT_CONFIG_FILE = "agent_config.json"
NEVER_UPLOAD = ("episode.json", "sdk/", "tmp/", "kernel/*.sock", "kernel/sandbox_episode.json",
                "kernel_cwd/meta.json", "kernel_cwd/episode.json")
TOKEN_FIELDS = ("token", "base")


@dataclass(frozen=True)
class AgentRunConfig:
    """<episode>/agent_config.json for `python -m amp.agents.run <episode_dir>`, which exits 0
    whenever its loop ended and non-zero only on a crash."""

    agent: AgentProfile
    harness: str
    kernel_python: str
    protocol: EvalProtocol = EvalProtocol()
    contract: int = CONTRACT_VERSION

    def to_dict(self) -> dict:
        return {**dataclasses.asdict(self), "agent": self.agent.to_dict(),
                "protocol": self.protocol.to_dict()}

    @classmethod
    def from_dict(cls, d: dict) -> AgentRunConfig:
        if d.get("contract") != CONTRACT_VERSION:
            raise ValueError(f"agent config contract {d.get('contract')} != {CONTRACT_VERSION}")
        a = dict(d["agent"])
        a["model"] = ModelSpec(**a["model"])
        return cls(agent=AgentProfile(**a), harness=d["harness"], kernel_python=d["kernel_python"],
                   protocol=EvalProtocol.from_dict(d.get("protocol")), contract=d["contract"])

    def write(self, episode_dir: str | Path) -> Path:
        p = Path(episode_dir) / AGENT_CONFIG_FILE
        p.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n")
        return p

    @classmethod
    def read(cls, episode_dir: str | Path) -> AgentRunConfig:
        return cls.from_dict(json.loads((Path(episode_dir) / AGENT_CONFIG_FILE).read_text()))


@dataclass(frozen=True)
class NodeRuntime:
    """The run's local interpreters, folders and ports; the interpreters default to this one."""

    agent_python: str = field(default_factory=lambda: sys.executable)
    kernel_python: str = field(default_factory=lambda: sys.executable)
    runs_dir: str = "/tmp/amp/runs"  # nosec B108
    state_dir: str = "/tmp/amp/node"  # nosec B108
    server_python: str = field(default_factory=lambda: sys.executable)
    gateway_port: int = 8710
    server_base_port: int = 9600


class Slot(Protocol):
    """One slot as the episode runner sees it: its robot server, behind the run's gateway."""

    slot_id: str
    node_id: str
    gateway_url: str

    def operator_key(self) -> str: ...

    def prepare(self, code_ref: str, harness: str) -> None:
        """Make this slot's robot server run at code_ref with that harness's server spec."""
        ...

    def backend(self) -> Any:
        """The slot's amp.contracts.robot.RobotBackend, for ingest and artifacts."""
        ...

    def health(self) -> dict: ...


AGENTIC_PREFIX = "agentic"


def unit_name(task: str, episode: int) -> str:
    return f"{task}__{int(episode):04d}"


MOVED_FAILURE_STATUSES = frozenset({"attempt_error", "lost"})


def outcome(rec: dict) -> bool | None:
    """The evaluated outcome: the score, else False for a final failure, else None (unscored)."""
    s = rec.get("success")
    if isinstance(s, bool):
        return s
    return False if rec.get("status") in MOVED_FAILURE_STATUSES else None
