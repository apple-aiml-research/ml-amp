#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The agent gateway: one per run, in front of its slot servers, serving every harness."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field


@dataclass(frozen=True)
class GatewayConfig:
    port: int
    state_dir: str
    key_file: str
    slots: dict[str, str] = field(default_factory=dict)
    harness_dir: str = ""
    host: str = "127.0.0.1"
    claim_timeout_s: float = 900.0
    request_timeout_s: float = 600.0

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)

    @classmethod
    def from_json(cls, text: str) -> GatewayConfig:
        return cls(**json.loads(text))
