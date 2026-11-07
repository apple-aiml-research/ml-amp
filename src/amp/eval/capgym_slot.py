#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One local CaP-Gym slot: a robot server and a CaP-X client process for one task set on one GPU."""

from __future__ import annotations

import fcntl
import http.client
import json
import os
import random
import secrets
import shutil
import socket
import stat
import threading
import time
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass, field
from fnmatch import fnmatch
from pathlib import Path
from typing import Any

from amp.contracts.robot import SlotSpec
from amp.fs.atomic import write_bytes_atomic

from . import envs
from .procs import ProcManager, ProcSpec, RestartPolicy
from .robolab_slot import get_state, server_config
from .slots import own_src

SERVER_UP_S = 120.0
CATALOG_S = 900.0
RELINK_S = 60.0
FRESH = ("TMPDIR", "XDG_RUNTIME_DIR")
SANDBOX = "sandbox"


class SlotError(RuntimeError):
    """The slot's robot server or CaP-X client could not be prepared."""


def _bindable(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket() as s:
        try:
            s.bind((host, port))
        except OSError:
            return False
    return True


def _lock_dir() -> Path:
    """/tmp/amp-ports-<uid>: mine, 0700, not a symlink."""
    d = Path("/tmp") / f"amp-ports-{os.getuid()}"  # nosec B108
    d.mkdir(mode=0o700, exist_ok=True)
    st = d.lstat()
    if not stat.S_ISDIR(st.st_mode) or st.st_uid != os.getuid():
        raise OSError(f"{d} is not a directory of mine")
    return d


class PortBlock:
    """n consecutive loopback ports reserved across processes by flocks held until release()."""

    def __init__(self, base: int, n: int):
        self.base, self.n = base, n
        self._files: list = []

    def acquire(self) -> bool:
        """Take every lock and check the ports are bindable; False, holding none, if not."""
        if self._files:
            return True
        d = _lock_dir()
        files = []
        try:
            for port in range(self.base, self.base + self.n):
                fd = os.open(d / str(port), os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW, 0o600)
                files.append(os.fdopen(fd, "w"))
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                if not _bindable(port):
                    raise OSError(port)
        except OSError:
            for f in files:
                f.close()
            return False
        self._files = files
        return True

    def release(self) -> None:
        for f in self._files:
            f.close()
        self._files = []


def reserve_port_block(n: int, *, lo: int = 20000, hi: int = 32000) -> PortBlock:
    """n consecutive ports free on loopback and reserved against every other run on this
    machine, below Linux's ephemeral range (32768+) where a client's source port could take one."""
    for _ in range(500):
        block = PortBlock(random.randrange(lo, hi - n), n)
        if block.acquire():
            return block
    raise OSError(f"no block of {n} free ports in {lo}..{hi}")


def prepare_sandbox(slot_dir: str | Path, env: dict[str, str], *, name: str = SANDBOX) -> None:
    """Make every env path under <slot_dir>/<name> a directory, its FRESH ones new and empty."""
    root = Path(os.path.abspath(slot_dir)) / name
    dirs = {p for v in env.values() if os.path.isabs(v)
            and (p := Path(os.path.normpath(v))).is_relative_to(root) and p != root}
    for d in sorted(dirs):
        for p in (root, *(root.joinpath(*d.relative_to(root).parts[:i + 1])
                          for i in range(len(d.relative_to(root).parts)))):
            if p.is_symlink():
                p.unlink()
    for key in FRESH:
        fresh = env.get(key)
        if not fresh or (fresh := Path(os.path.normpath(fresh))) not in dirs:
            continue
        for old in fresh.parent.glob(f".{fresh.name}.old-*"):
            shutil.rmtree(old, ignore_errors=True)
        if fresh.exists() or fresh.is_symlink():
            aside = fresh.with_name(f".{fresh.name}.old-{secrets.token_hex(4)}")
            os.replace(fresh, aside)
            shutil.rmtree(aside, ignore_errors=True)
    for d in sorted(dirs):
        d.mkdir(parents=True, exist_ok=True)
    runtime = env.get("XDG_RUNTIME_DIR")
    if runtime and Path(os.path.normpath(runtime)) in dirs:
        os.chmod(runtime, 0o700)  # nosemgrep: python.lang.security.audit.insecure-file-permissions.insecure-file-permissions


def catalog(server_url: str, timeout_s: float = 30.0) -> frozenset[str] | None:
    """The task names of the server's api/tasks; None when it has no list."""
    try:
        with urllib.request.urlopen(f"{server_url}/api/tasks", timeout=timeout_s) as r:  # nosec B310  # nosemgrep: python.lang.security.audit.dynamic-urllib-use-detected.dynamic-urllib-use-detected
            body = json.loads(r.read()) if r.status == 200 else None
    except (OSError, ValueError, http.client.HTTPException):
        return None
    tasks = body.get("tasks") if isinstance(body, dict) else None
    if not isinstance(tasks, list):
        return None
    return frozenset(t.get("name") for t in tasks if isinstance(t, dict))


def has_catalog(st: dict | None) -> bool:
    return isinstance((st or {}).get("benchmark"), dict)


@dataclass
class CapgymSlot:
    """run_unit's Slot for one task set on one GPU, plus the paths eval.slots.slot_paths reads."""

    slot_id: str
    gpu_index: int
    gpu_uuid: str
    gateway_url: str
    server_port: int
    dir: Path
    gateway_state_dir: Path
    log_dir: Path
    node_id: str
    key_fn: Callable[[], str]
    robot_backend: Any
    procs: ProcManager
    task_set: str
    egl_device: int | None = None
    server_python: str = ""
    config_fn: Callable[..., dict] = server_config
    up_s: float = SERVER_UP_S
    catalog_s: float = CATALOG_S
    relink_s: float = RELINK_S
    poll_s: float = 0.5
    client_label = "the CaP-X client"
    warm: tuple[str, str] | None = None
    client_mode: tuple[str, str] | None = None
    _catalog: frozenset[str] | None = None
    _lock: threading.RLock = field(default_factory=threading.RLock, repr=False)

    @property
    def server_url(self) -> str:
        return f"http://127.0.0.1:{self.server_port}"

    @property
    def server_state_dir(self) -> Path:
        return self.dir / "server"

    @property
    def results_dir(self) -> Path:
        return self.dir / "results"

    @property
    def server_name(self) -> str:
        return f"{self.slot_id}.server"

    @property
    def client_name(self) -> str:
        return f"{self.slot_id}.client"

    def operator_key(self) -> str:
        return self.key_fn()

    def backend(self):
        return self.robot_backend

    def serves(self, task_set: str) -> bool:
        return task_set == self.task_set

    def gpu_mode(self, task_set: str | None = None) -> str:
        from amp.robots.capgym.families import family

        return family(task_set or self.task_set).gpu

    def health(self) -> dict:
        st = get_state(self.server_url)
        return {"slot": self.slot_id, "gpu": self.gpu_uuid, "task_set": self.task_set,
                "server": st is not None, "catalog": has_catalog(st),
                "warm": list(self.warm) if self.warm else None}

    def spec(self, info_mode: str = "standard", cameras: str = "droid") -> SlotSpec:
        env = {}
        if self.gpu_mode() == "cuda":
            env = {"CUDA_VISIBLE_DEVICES": str(self.gpu_index), "CUDA_DEVICE_ORDER": "PCI_BUS_ID"}
        return SlotSpec(slot_id=self.slot_id, gpu=self.gpu_index, index=0,
                        server_port=self.server_port, cameras=cameras, info_mode=info_mode,
                        results_dir=str(self.results_dir), env=env, task_set=self.task_set)

    def client_env(self, spec: SlotSpec) -> dict[str, str]:
        """The client's whole environment: this process's minus what the backend unsets, plus its own."""
        unset = getattr(self.robot_backend, "env_unset", None)
        drop = tuple(unset(spec)) if callable(unset) else ()
        base = {k: v for k, v in envs.node_env().items()
                if not any(fnmatch(k, pat) for pat in drop)}
        env = {**base, "PYTHONPATH": str(own_src()), "PYTHONUNBUFFERED": "1",
               **self.robot_backend.env(spec)}
        if self.gpu_mode() == "egl":
            if self.egl_device is None:
                raise SlotError(f"{self.slot_id}: GPU {self.gpu_uuid} has no EGL device")
            env["MUJOCO_EGL_DEVICE_ID"] = str(self.egl_device)
            env["CUDA_VISIBLE_DEVICES"] = ""
        return env

    def client_spec(self, spec: SlotSpec) -> ProcSpec:
        env = self.client_env(spec)
        own = getattr(self.robot_backend, "cwd", None)
        cwd = Path((own(spec) if callable(own) else None) or self.dir)
        cwd.mkdir(parents=True, exist_ok=True)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        prepare_sandbox(self.dir, env)
        wipe = getattr(self.robot_backend, "wipe_on_exit", None)
        wiped = tuple(wipe(spec)) if callable(wipe) else ()
        return ProcSpec(self.client_name, tuple(self.robot_backend.command(spec, self.server_url)),
                        str(self.log_dir / f"{self.client_name}.log"), env=env, cwd=str(cwd),
                        grace_s=30.0, restart=RestartPolicy(max_failures=3), wipe_on_exit=wiped,
                        wipe_root=str(self.dir) if wiped else "")

    def server_spec(self, harness: str) -> tuple[ProcSpec, dict]:
        cfg = self.config_fn(harness, port=self.server_port, state_dir=str(self.server_state_dir),
                             results_dir=str(self.results_dir))
        path = self.dir / "server.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        write_bytes_atomic(path, (json.dumps(cfg, sort_keys=True) + "\n").encode())
        fresh = {"TMPDIR": str(self.dir / "server_sandbox" / "tmp"),
                 "XDG_RUNTIME_DIR": str(self.dir / "server_sandbox" / "run")}
        prepare_sandbox(self.dir, fresh, name="server_sandbox")
        env = {**envs.node_env(), **fresh, "PYTHONPATH": str(own_src()), "PYTHONUNBUFFERED": "1"}
        argv = (self.server_python, "-m", "amp.robot_server", "--config", str(path))
        return ProcSpec(self.server_name, argv, str(self.log_dir / f"{self.server_name}.log"),
                        env=env, cwd=str(self.dir), restart=RestartPolicy(max_failures=3)), cfg

    def _proc(self, name: str):
        try:
            return self.procs.get(name)
        except KeyError:
            return None

    def _run(self, spec: ProcSpec, *, restart: bool) -> None:
        p = self._proc(spec.name)
        if p is None:
            self.procs.add(spec).start()
        elif restart or not p.alive():
            p.restart(spec)

    def _wait(self, cond: Callable[[], bool], timeout_s: float, what: str) -> None:
        deadline = time.monotonic() + timeout_s
        while not cond():
            for name in (self.server_name, self.client_name):
                p = self._proc(name)
                if p is not None and p.state == "failed":
                    raise SlotError(f"{self.slot_id}: {name} failed while waiting for {what}: "
                                    f"{p.failed_reason}; log {p.spec.log}")
            if time.monotonic() > deadline:
                raise SlotError(f"{self.slot_id}: {what} not reached in {timeout_s:g}s; logs in "
                                f"{self.log_dir}")
            time.sleep(self.poll_s)

    def _linked(self) -> bool:
        return has_catalog(get_state(self.server_url))

    def prepare(self, code_ref: str, harness: str) -> None:
        """Run the server with harness's ServerSpec and the client in its info mode and cameras;
        each restarts only when what it runs changes or it is not running."""
        with self._lock:
            from amp.harnesses import registry

            hs = registry.load(harness).server
            mode = (hs.info_mode, hs.cameras)
            want = (code_ref, harness)
            server, client = self._proc(self.server_name), self._proc(self.client_name)
            server_up = server is not None and server.alive()
            client_up = client is not None and client.alive()
            if want == self.warm and server_up and client_up and mode == self.client_mode \
                    and self._linked():
                return
            restart_server = want != self.warm or not server_up
            restart_client = mode != self.client_mode or not client_up
            self.warm, self._catalog = None, None
            if restart_server:
                spec, _ = self.server_spec(harness)
                self._run(spec, restart=True)
                self._wait(lambda: get_state(self.server_url) is not None, self.up_s,
                           "the robot server answering")
            if restart_client:
                self._run(self.client_spec(self.spec(*mode)), restart=True)
                self.client_mode = mode
                self._wait(self._linked, self.catalog_s, f"{self.client_label}'s task catalog")
            else:
                try:
                    self._wait(self._linked, self.relink_s, f"{self.client_label} re-linking")
                except SlotError:
                    self._run(self.client_spec(self.spec(*mode)), restart=True)
                    self._wait(self._linked, self.catalog_s, f"{self.client_label}'s task catalog")
            self.warm = want

    def ensure_task(self, task_set: str, task: str, official: bool = False) -> bool:
        """False (never moved) once task is in this slot's library; SlotError otherwise."""
        if task_set != self.task_set:
            raise SlotError(f"{self.slot_id}: serves task set {self.task_set!r}, not {task_set!r}")
        if self._catalog is None and self._linked():
            self._catalog = catalog(self.server_url)
        if self._catalog is not None and task not in self._catalog:
            raise SlotError(f"{self.slot_id}: task {task!r} is not in the {task_set!r} library "
                            f"its client registered ({len(self._catalog)} tasks)")
        return False

    def switch_to(self, task_set: str) -> None:
        """Serve task_set from the next unit on: the client and the server stop until prepare."""
        with self._lock:
            if task_set == self.task_set:
                return
            for name in (self.client_name, self.server_name):
                p = self._proc(name)
                if p is not None:
                    p.stop()
            self.task_set, self.warm, self.client_mode, self._catalog = task_set, None, None, None

    def stop(self) -> None:
        for name in (self.client_name, self.server_name):
            p = self._proc(name)
            if p is not None:
                p.close()
