#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One local RoboLab slot: its GPU, robot server and folders, with exactly what run_unit reads."""

from __future__ import annotations

import http.client
import json
import threading
import time
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from amp.fs.atomic import write_bytes_atomic

from .procs import ProcManager, ProcSpec, RestartPolicy
from .slots import own_src

SERVER_UP_S = 120.0
STATE_TIMEOUT_S = 5.0


class SlotError(RuntimeError):
    """The slot's robot server could not be prepared."""


def get_state(server_url: str, timeout_s: float = STATE_TIMEOUT_S) -> dict | None:
    """The robot server's api/state, or None when it does not answer 200."""
    if not server_url.lower().startswith(("http://", "https://")):
        return None
    try:
        with urllib.request.urlopen(f"{server_url}/api/state", timeout=timeout_s) as r:  # nosec B310  # nosemgrep: python.lang.security.audit.dynamic-urllib-use-detected.dynamic-urllib-use-detected
            return json.loads(r.read()) if r.status == 200 else None
    except (OSError, ValueError, http.client.HTTPException):
        return None


def server_config(harness: str, *, port: int, state_dir: str, results_dir: str) -> dict:
    """ServerConfig fields for harness's ServerSpec, as AMP's slot builds them."""
    from amp.contracts.server import ServerConfig
    from amp.harnesses import registry

    spec = registry.load(harness).server
    cfg = ServerConfig(port=port, state_dir=state_dir, harness=harness, results_dir=results_dir,
                       cameras=spec.cameras, info_mode=spec.info_mode, obstacles=spec.obstacles,
                       extra=dict(spec.env), embodiments=spec.embodiments)
    return json.loads(cfg.to_json())


@dataclass
class RoboLabSlot:
    """Slot i on one GPU: run_unit's Slot protocol plus the paths eval.slots.slot_paths reads."""

    slot_id: str
    gpu_uuid: str
    gpu_index: int
    gateway_url: str
    server_port: int
    dir: Path
    results_dir: Path
    gateway_state_dir: Path
    log_dir: Path
    node_id: str
    key_fn: Callable[[], str]
    robot_backend: Any
    procs: ProcManager | None = None
    server_python: str = ""
    warm: tuple[str, str] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    @property
    def server_url(self) -> str:
        return f"http://127.0.0.1:{self.server_port}"

    @property
    def server_state_dir(self) -> Path:
        return self.dir / "server"

    def operator_key(self) -> str:
        return self.key_fn()

    def backend(self):
        return self.robot_backend

    def health(self) -> dict:
        st = get_state(self.server_url)
        return {"slot": self.slot_id, "gpu": self.gpu_uuid, "server": st is not None,
                "warm": list(self.warm) if self.warm else None}

    def output_dir(self, run_id: str) -> Path:
        """RoboLab's --output-folder-name for one harness configuration's units on this slot (absolute)."""
        return self.results_dir / run_id

    def server_argv(self, cfg_path: Path) -> tuple[str, ...]:
        return (self.server_python, "-m", "amp.robot_server", "--config", str(cfg_path))

    def prepare(self, code_ref: str, harness: str) -> None:
        """Run this slot's robot server with harness's ServerSpec; restart only on a change."""
        with self._lock:
            want = (code_ref, harness)
            proc = self.procs.get(self.slot_id) if self.procs and self._has_proc() else None
            if want == self.warm and proc is not None and proc.alive():
                return
            cfg = server_config(harness, port=self.server_port,
                                state_dir=str(self.server_state_dir),
                                results_dir=str(self.results_dir))
            path = self.dir / "server.json"
            path.parent.mkdir(parents=True, exist_ok=True)
            self.results_dir.mkdir(parents=True, exist_ok=True)
            write_bytes_atomic(path, (json.dumps(cfg, sort_keys=True) + "\n").encode())
            spec = ProcSpec(self.slot_id, self.server_argv(path),
                            str(self.log_dir / f"server_{self.slot_id}.log"),
                            env={"PYTHONPATH": str(own_src()), "PYTHONUNBUFFERED": "1"},
                            cwd=str(self.dir), restart=RestartPolicy(max_failures=3))
            if self.procs is None:
                raise SlotError(f"{self.slot_id}: no process manager")
            if proc is None:
                proc = self.procs.add(spec)
                proc.start()
            else:
                proc.restart(spec)
            self.warm = None
            deadline = time.monotonic() + SERVER_UP_S
            while get_state(self.server_url) is None:
                if time.monotonic() > deadline:
                    raise SlotError(f"{self.slot_id}: robot server did not answer within "
                                    f"{SERVER_UP_S:g}s (log {spec.log})")
                time.sleep(0.5)
            self.warm = want

    def _has_proc(self) -> bool:
        try:
            self.procs.get(self.slot_id)
            return True
        except KeyError:
            return False
