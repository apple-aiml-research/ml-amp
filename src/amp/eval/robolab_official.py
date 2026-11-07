#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp eval robolab: RoboLab-120 through RoboLab's official runner on local GPUs, one run.py
process per (harness configuration, task).

    amp eval robolab --harness amp1_basic,amp1_assistive --prompt amp1 \\
        --model opus-5.5 --task-set benchmark [--task T ...] --num-runs 10 \\
        --gpus auto|0,1 --output runs/<name>

One gateway is shared; each slot has its own GPU, robot server and episode driver. The driver
serves the RoboLab client's begin requests over a Unix socket by running run_unit for that
(task, episode). An infra fault ends run.py; the launcher sets the partial run aside and starts
the task again with the same output folder, so RoboLab's own resume reruns that episode.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import secrets
import shutil
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.eval import (
    Arm,
    EpisodeRequest,
    EpisodeResult,
    EvalProtocol,
    NodeRuntime,
    is_retryable,
    unit_name,
)
from amp.contracts.trace import NullTracer

from . import envs, gpus, robolab_merge, vision
from .layout import RunLayout
from .procs import ProcManager, ProcSpec, RestartPolicy, terminate
from .robolab_slot import RoboLabSlot, get_state
from .slots import own_src
from .store import parse_record, write_json
from .study import Study, UnitRef

BEGIN_TIMEOUT_S = 840.0
MAX_RESTARTS = 3
KIT_START_LOCK_ENV = "AMP_KIT_START_LOCK"
GATEWAY_UP_S = 60.0
POLL_S = 0.5
ROBOLAB_ROOT_ENV = "AMP_ROBOLAB_ROOT"
ROBOLAB_PYTHON_ENV = "AMP_ROBOLAB_PYTHON"
METADATA = Path("robolab") / "tasks" / "_metadata"


def log(msg: str) -> None:
    print(f"[amp eval robolab {time.strftime('%H:%M:%S')}] {msg}", file=sys.stderr, flush=True)


@dataclass(frozen=True)
class Unit:
    """One harness configuration on one task: a single official run.py process with --num-runs N."""

    arm: Arm
    task: str
    est_s: float = 0.0

    @property
    def key(self) -> tuple[str, str]:
        return self.arm.id, self.task


def order_units(arms: Sequence[Arm], tasks: Sequence[str], est: dict[str, float]) -> list[Unit]:
    """Longest task first; each task's harness configurations back to back, the leading one alternating."""
    ranked = sorted(dict.fromkeys(tasks), key=lambda t: (-float(est.get(t, 0.0)), t))
    out = []
    for i, t in enumerate(ranked):
        order = list(arms) if i % 2 == 0 else list(reversed(arms))
        out += [Unit(a, t, float(est.get(t, 0.0))) for a in order]
    return out


class UnitQueue:
    """Units in order; a slot takes the first unit unpinned or pinned to it."""

    def __init__(self, units: Iterable[Unit], pins: dict[tuple[str, str], str] | None = None):
        self._units = list(units)
        self._pins = dict(pins or {})
        self._lock = threading.Lock()

    def take(self, slot_id: str) -> Unit | None:
        with self._lock:
            for i, u in enumerate(self._units):
                if self._pins.get(u.key, slot_id) == slot_id:
                    return self._units.pop(i)
            return None

    def __len__(self) -> int:
        return len(self._units)


def robot_server_job(server_url: str, task: str, episode: int, run_id: str) -> str | None:
    """The job id the slot's robot server holds for (task, episode) in run_id, pending or active."""
    st = get_state(server_url)
    bench = (st or {}).get("benchmark") or {}
    for job in (bench.get("pending"), bench.get("job")):
        if not isinstance(job, dict) or job.get("quit"):
            continue
        if job.get("task") == task and job.get("episode") == episode \
                and job.get("run") in (None, "", run_id) and job.get("job_id") is not None:
            return str(job["job_id"])
    return None


class Driver:
    """The slot's episode driver: begin {task, episode} -> run_unit in a thread, replying once
    the slot's robot server holds the claimed job."""

    def __init__(self, slot, *, socket_path: Path, study: Study, runtime: NodeRuntime,
                 run_root: Path, num_runs: int, tracer=None,
                 begin_timeout_s: float = BEGIN_TIMEOUT_S, poll_s: float = POLL_S):
        self.slot, self.path, self.study, self.runtime = slot, Path(socket_path), study, runtime
        self.run_root, self.num_runs = Path(run_root), int(num_runs)
        self.tracer = tracer or NullTracer("eval")
        self.begin_timeout_s, self.poll_s = begin_timeout_s, poll_s
        self.unit: Unit | None = None
        self.begun = threading.Event()
        self.results: dict[tuple[str, str, int], EpisodeResult] = {}
        self.replies: list[dict] = []
        self._threads: list[threading.Thread] = []
        self._aborts: list[threading.Event] = []
        self._lock = threading.Lock()
        self._claim_lock = threading.Lock()
        self._server: socketserver.ThreadingUnixStreamServer | None = None

    def set_unit(self, unit: Unit | None) -> None:
        with self._lock:
            self.unit = unit
            self.begun.clear()

    def request(self, task: str, episode: int) -> EpisodeRequest:
        assert self.unit is not None
        return self.study.request(UnitRef(self.study.job_id, self.unit.arm.id, task, episode))

    def handle(self, msg) -> dict:
        """One protocol request; every failure is a {"ok": false, "error"} reply."""
        if not isinstance(msg, dict):
            return {"ok": False, "error": "a request is one JSON object"}
        op = msg.get("op")
        if op == "ping":
            return {"ok": True}
        if op != "begin":
            return {"ok": False, "error": f"unknown op {op!r}"}
        try:
            return self.begin(msg.get("task"), msg.get("episode"), msg.get("instruction"))
        except Exception as e:  # noqa: BLE001
            return {"ok": False, "error": f"driver: {type(e).__name__}: {e}"}

    def begin(self, task, episode, instruction=None) -> dict:
        unit = self.unit
        if unit is None:
            return {"ok": False, "error": "no unit is running on this slot"}
        if task != unit.task:
            return {"ok": False, "error": f"task {task!r} is not this slot's unit {unit.task!r}"}
        if not isinstance(episode, int) or isinstance(episode, bool) \
                or not 0 <= episode < self.num_runs:
            return {"ok": False, "error": f"episode {episode!r} is not in 0..{self.num_runs - 1}"}
        self.begun.set()
        with self._claim_lock:
            req = self.request(task, episode)
            before = robot_server_job(self.slot.server_url, task, episode, req.run_id)
            claimed = threading.Event()
            done = threading.Event()
            abort = threading.Event()
            box: dict = {}

            def progress(p: dict) -> None:
                if p.get("phase") == "claim":
                    claimed.set()

            def work() -> None:
                try:
                    box["result"] = self.run(req, abort, progress)
                finally:
                    done.set()

            t = threading.Thread(target=work, name=f"amp-unit-{self.slot.slot_id}-{episode}",
                                 daemon=True)
            with self._lock:
                self._threads.append(t)
                self._aborts.append(abort)
            t.start()
            self.tracer.event("eval.driver.begin", {"slot": self.slot.slot_id, "task": task,
                                                    "episode": episode,
                                                    "instruction": instruction})
            reply = self._await_job(req, before, claimed, done, abort, box)
            self.replies.append({"task": task, "episode": episode, **reply})
            return reply

    def _await_job(self, req, before, claimed, done, abort, box) -> dict:
        deadline = time.monotonic() + self.begin_timeout_s
        while True:
            if claimed.is_set():
                job = robot_server_job(self.slot.server_url, req.task, req.episode,
                                       req.run_id)
                if job is not None and job != before:
                    return {"ok": True, "job_id": job}
            if done.is_set():
                res = box.get("result")
                why = f"{res.status}: {res.error}" if res is not None else "no result"
                return {"ok": False, "error": f"run_unit ended before the job was queued: {why}"}
            if time.monotonic() > deadline:
                abort.set()
                return {"ok": False, "error": f"no job queued within {self.begin_timeout_s:g}s"}
            time.sleep(self.poll_s)

    def run(self, req: EpisodeRequest, abort: threading.Event, progress) -> EpisodeResult:
        from .episode import run_unit

        try:
            res = run_unit(req, self.slot, runtime=self.runtime, run_dir=self.run_root / req.arm.id,
                           tracer=self.tracer, abort=abort, abort_status=lambda: "infra_error",
                           progress=progress)
        except Exception as e:  # noqa: BLE001
            res = EpisodeResult(job_id=req.job_id, arm_id=req.arm.id, task=req.task,
                                episode=req.episode, status="infra_error",
                                error=f"{type(e).__name__}: {e}")
        self.results[(req.arm.id, req.task, req.episode)] = res
        return res

    def wait_idle(self, timeout_s: float | None = None) -> bool:
        """Join every run_unit thread; True when none is left."""
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._lock:
            threads = list(self._threads)
        for t in threads:
            t.join(None if deadline is None else max(0.0, deadline - time.monotonic()))
        with self._lock:
            self._threads = [t for t in self._threads if t.is_alive()]
            return not self._threads

    def abort_all(self) -> None:
        with self._lock:
            for a in self._aborts:
                a.set()

    def start(self) -> None:
        driver = self

        class Handler(socketserver.StreamRequestHandler):
            def handle(self):
                line = self.rfile.readline()
                try:
                    msg = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, ValueError):
                    msg = None
                reply = driver.handle(msg)
                with contextlib.suppress(OSError):
                    self.wfile.write((json.dumps(reply) + "\n").encode())

        self.path.parent.mkdir(parents=True, exist_ok=True)
        with contextlib.suppress(FileNotFoundError):
            self.path.unlink()
        self._server = socketserver.ThreadingUnixStreamServer(str(self.path), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, name=f"amp-driver-{self.slot.slot_id}",
                         daemon=True).start()

    def close(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        with contextlib.suppress(FileNotFoundError):
            self.path.unlink()


def robolab_rows(output_dir: Path, task: str) -> dict[int, dict]:
    """{episode: row} RoboLab wrote for task in this output folder."""
    out = {}
    for r in _records().load_records(str(output_dir / robolab_merge.RESULTS)):
        if robolab_merge.row_task(r) == task and isinstance(r.get("episode"), int):
            out[r["episode"]] = r
    return out


def _port_in_use(port: int) -> bool:
    """True when something already accepts connections on loopback:port."""
    with socket.socket() as c:
        c.settimeout(1.0)
        return c.connect_ex(("127.0.0.1", port)) == 0


def _records():
    from amp.robots.robolab import records

    return records


@dataclass
class Launcher:
    """Slots x units: the shared gateway, a robot server and driver per slot, the run.py queue."""

    study: Study
    backend: object
    tasks: list[str]
    num_runs: int
    output: Path
    gpu_list: list[gpus.Gpu]
    slots_per_gpu: int
    est: dict[str, float] = field(default_factory=dict)
    gateway_port: int = 8710
    server_base_port: int = 9600
    python: str = sys.executable
    max_restarts: int = MAX_RESTARTS
    slots: list[RoboLabSlot] = field(default_factory=list)
    drivers: dict[str, Driver] = field(default_factory=dict)
    failures: list[str] = field(default_factory=list)
    spawns: list[dict] = field(default_factory=list)

    def __post_init__(self):
        self.output = Path(self.output).resolve()
        self.node = self.output / "_node"
        self.logs = self.node / "logs"
        self.procs = ProcManager(token=secrets.token_hex(12))
        self._key = secrets.token_urlsafe(32)
        self._sock_dir = Path(tempfile.mkdtemp(prefix="amp-drv-"))
        self._stopping = threading.Event()
        self.runtime = NodeRuntime(agent_python=self.python, kernel_python=self.python,
                                   runs_dir=str(self.output), state_dir=str(self.node),
                                   server_python=self.python,
                                   gateway_port=self.gateway_port,
                                   server_base_port=self.server_base_port)

    @property
    def gateway_url(self) -> str:
        return f"http://127.0.0.1:{self.gateway_port}"

    def run_id(self, arm: Arm) -> str:
        return f"{self.study.job_id}__{arm.id}"

    def build_slots(self) -> None:
        i = 0
        for g in self.gpu_list:
            for _ in range(self.slots_per_gpu):
                sid = f"slot{i}"
                s = RoboLabSlot(
                    slot_id=sid, gpu_uuid=g.uuid, gpu_index=g.index, gateway_url=self.gateway_url,
                    server_port=self.server_base_port + i, dir=self.node / "slots" / sid,
                    results_dir=self.output / "robolab" / sid,
                    gateway_state_dir=self.node / "gateway" / "state", log_dir=self.logs,
                    node_id=socket.gethostname(), key_fn=lambda: self._key,
                    robot_backend=self.backend, procs=self.procs, server_python=self.python)
                self.slots.append(s)
                self.drivers[sid] = Driver(
                    s, socket_path=self._sock_dir / f"{sid}.sock", study=self.study,
                    runtime=self.runtime, run_root=self.output, num_runs=self.num_runs)
                i += 1

    def check_ports(self) -> None:
        """Fail at once, naming the flag, when another run on this host holds a port we need."""
        if _port_in_use(self.gateway_port):
            raise RuntimeError(f"port {self.gateway_port} is in use (another run on this host?); "
                               f"pass --gateway-port")
        for s in self.slots:
            if _port_in_use(s.server_port):
                raise RuntimeError(f"port {s.server_port} (robot server of {s.slot_id}) is in use "
                                   f"(another run on this host?); pass --server-base-port")

    def start_gateway(self) -> None:
        state = self.node / "gateway"
        (state / "state").mkdir(parents=True, exist_ok=True)
        key_file = state / "operator.key"
        fd = os.open(key_file, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(self._key)
        cfg = {"port": self.gateway_port, "state_dir": str(state / "state"),
               "key_file": str(key_file), "slots": {s.slot_id: s.server_url for s in self.slots},
               "harness_dir": str(own_src() / "amp" / "harnesses"),
               "host": "127.0.0.1", "claim_timeout_s": 900.0, "request_timeout_s": 600.0}
        path = state / "config.json"
        path.write_text(json.dumps(cfg, sort_keys=True) + "\n")
        argv = (self.python, "-m", "amp.gateway", "--config", str(path), "--poll-s", "1.0")
        spec = ProcSpec("gateway", argv, str(self.logs / "gateway.log"),
                        env={**envs.node_env(), "PYTHONPATH": str(own_src()),
                             "PYTHONUNBUFFERED": "1"},
                        cwd=str(state), restart=RestartPolicy(max_failures=3))
        self.logs.mkdir(parents=True, exist_ok=True)
        self.procs.add(spec).start()
        deadline = time.monotonic() + GATEWAY_UP_S
        while not self._gateway_up():
            if time.monotonic() > deadline:
                raise RuntimeError(f"the gateway did not answer on port {self.gateway_port} in "
                                   f"{GATEWAY_UP_S:g}s (log {spec.log})")
            time.sleep(0.5)

    def _gateway_up(self) -> bool:
        from .gateway_client import GatewayClient

        with contextlib.suppress(Exception):
            code, _ = GatewayClient(self.gateway_url, lambda: self._key).status()
            return code == 200
        return False

    def pins(self) -> dict[tuple[str, str], str]:
        """Units already started on a slot: their rows are in that slot's folder."""
        out = {}
        for arm in self.study.arms:
            for s in self.slots_on_disk():
                for t in self.tasks:
                    if (self.output / "robolab" / s / self.run_id(arm) / t).is_dir():
                        out[(arm.id, t)] = s
        return out

    def slots_on_disk(self) -> list[str]:
        root = self.output / "robolab"
        return sorted(p.name for p in root.glob("slot*") if p.is_dir()) if root.is_dir() else []

    def unit_done(self, slot: RoboLabSlot, unit: Unit) -> bool:
        rows = robolab_rows(slot.output_dir(self.run_id(unit.arm)), unit.task)
        return set(rows) >= set(range(self.num_runs))

    def missing(self, slot: RoboLabSlot, unit: Unit) -> list[int]:
        rows = robolab_rows(slot.output_dir(self.run_id(unit.arm)), unit.task)
        return [k for k in range(self.num_runs) if k not in rows]

    def check_wording(self) -> None:
        """RuntimeError if this --output holds RoboLab rows of another wording than the study's:
        RoboLab's resume would keep them as done."""
        want = self.study.protocol.instruction_type
        for arm in self.study.arms:
            for s in self.slots_on_disk():
                path = self.output / "robolab" / s / self.run_id(arm) / robolab_merge.RESULTS
                ran = sorted({r.get("instruction_type") or "default"
                              for r in _records().load_records(str(path))} - {want})
                if ran:
                    raise RuntimeError(f"{path} holds rows of the {', '.join(ran)} wording, not "
                                       f"{want!r}: give each wording its own --output")

    def run(self) -> int:
        self.build_slots()
        self.check_wording()
        pins = self.pins()
        known = {s.slot_id for s in self.slots}
        orphan = sorted({s for s in pins.values() if s not in known})
        if orphan:
            raise RuntimeError(f"{', '.join(orphan)} of an earlier run hold started tasks: resume "
                               f"with at least {len(self.slots_on_disk())} slots")
        units = order_units(self.study.arms, self.tasks, self.est)
        queue = UnitQueue(units, pins)
        self.write_plan(units, pins)
        workers: list[threading.Thread] = []
        try:
            self.check_ports()
            self.procs.start_monitor()
            self.start_gateway()
            for d in self.drivers.values():
                d.start()
            workers = [threading.Thread(target=self.work, args=(s, queue), name=f"amp-{s.slot_id}",
                                        daemon=True) for s in self.slots]
            for w in workers:
                w.start()
            for w in workers:
                while w.is_alive():
                    w.join(1.0)
        except KeyboardInterrupt:
            log("interrupted: stopping (rerun with the same --output to resume)")
            self._stopping.set()
            for d in self.drivers.values():
                d.abort_all()
            for w in workers:
                w.join(60.0)
            raise
        finally:
            self.close()
        return self.finish()

    def write_plan(self, units: list[Unit], pins: dict) -> None:
        self.output.mkdir(parents=True, exist_ok=True)
        from .finalize import write_run_config

        for arm in self.study.arms:
            with contextlib.suppress(Exception):
                write_run_config(RunLayout(self.output / arm.id), self.study, arm)
        write_json(self.node / "plan.json", {
            "job_id": self.study.job_id, "num_runs": self.num_runs,
            "gpus": [g.uuid for g in self.gpu_list], "slots_per_gpu": self.slots_per_gpu,
            "units": [{"arm": u.arm.id, "task": u.task, "est_s": u.est_s,
                       "slot": pins.get(u.key)} for u in units]})

    def work(self, slot: RoboLabSlot, queue: UnitQueue) -> None:
        while not self._stopping.is_set():
            unit = queue.take(slot.slot_id)
            if unit is None:
                return
            try:
                self.run_unit_on(slot, unit)
            except Exception as e:  # noqa: BLE001
                self.failures.append(f"{unit.arm.id} {unit.task} on {slot.slot_id}: "
                                     f"{type(e).__name__}: {e}")
                log(self.failures[-1])

    def run_unit_on(self, slot: RoboLabSlot, unit: Unit) -> None:
        driver = self.drivers[slot.slot_id]
        out_dir = slot.output_dir(self.run_id(unit.arm))
        if self.unit_done(slot, unit):
            log(f"{slot.slot_id}: {unit.arm.id} {unit.task} already done")
            self.reconcile(slot, unit)
            return
        slot.prepare(self.study.code_ref, unit.arm.harness)
        driver.set_unit(unit)
        try:
            for attempt in range(self.max_restarts + 1):
                if self._stopping.is_set():
                    return
                rc = self.spawn(slot, driver, unit, out_dir, attempt)
                driver.wait_idle()
                left = self.missing(slot, unit)
                if not left:
                    break
                log(f"{slot.slot_id}: {unit.task} run.py exited {rc} with episodes {left} "
                    f"unwritten (attempt {attempt + 1})")
                self.set_aside_partial(out_dir / unit.task, unit, left)
                if attempt == self.max_restarts:
                    self.failures.append(f"{unit.arm.id} {unit.task}: episodes {left} unwritten "
                                         f"after {self.max_restarts} restarts")
        finally:
            driver.set_unit(None)
        self.reconcile(slot, unit)

    def spawn(self, slot: RoboLabSlot, driver: Driver, unit: Unit, out_dir: Path,
              attempt: int) -> int | None:
        out_dir.mkdir(parents=True, exist_ok=True)
        kw = {"task": unit.task, "num_runs": self.num_runs, "output_dir": str(out_dir),
              "server_url": slot.server_url, "driver_socket": str(driver.path)}
        kw["gpu_index"] = slot.gpu_index
        itype = driver.study.protocol.instruction_type
        if itype != "default":
            kw["instruction_type"] = itype
        cmd = self.backend.run_command(**kw)
        env = self.backend.env(gpu_uuid=slot.gpu_uuid, base=envs.node_env())
        lock = self.node / "locks" / f"kit_{slot.gpu_uuid}.lock"
        lock.parent.mkdir(parents=True, exist_ok=True)
        env[KIT_START_LOCK_ENV] = str(lock)
        logf = self.logs / f"run_{slot.slot_id}_{unit.arm.id}_{unit.task}.log"
        logf.parent.mkdir(parents=True, exist_ok=True)
        self.spawns.append({"slot": slot.slot_id, "arm": unit.arm.id, "task": unit.task,
                            "attempt": attempt, "gpu": slot.gpu_uuid, "t": time.time()})
        driver.begun.clear()
        with open(logf, "ab") as out:
            p = subprocess.Popen(cmd, env=env, stdout=out, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL, start_new_session=True)
            log(f"{slot.slot_id}: {unit.arm.id} {unit.task} pid {p.pid} "
                f"(attempt {attempt + 1}, log {logf})")
            while p.poll() is None:
                if self._stopping.is_set():
                    return terminate(p, 30.0)
                time.sleep(POLL_S)
        return p.returncode

    def set_aside_partial(self, task_dir: Path, unit: Unit, episodes: Iterable[int]) -> None:
        """An unwritten episode's partial RoboLab files and its AMP record go aside."""
        layout = RunLayout(self.output / unit.arm.id)
        stamp = time.strftime("%Y%m%dT%H%M%S")
        for k in episodes:
            if task_dir.is_dir():
                moved = _records().set_aside(str(task_dir), k)
                if moved:
                    log(f"set aside {unit.task} run {k}: {moved}")
            rec = layout.unit_path(unit.task, k)
            if rec.is_file():
                rec.rename(rec.with_name(f"{rec.name}.set_aside.{stamp}"))

    def reconcile(self, slot: RoboLabSlot, unit: Unit) -> list[int]:
        """Episodes with a RoboLab row but no final AMP record get one from the row."""
        layout = RunLayout(self.output / unit.arm.id)
        out_dir = slot.output_dir(self.run_id(unit.arm))
        done = []
        for k, row in sorted(robolab_rows(out_dir, unit.task).items()):
            path = layout.unit_path(unit.task, k)
            rec = parse_record(path.read_bytes()) if path.is_file() else None
            if rec is not None and not is_retryable(rec):
                continue
            self.ingest_offline(slot, unit, k, row, rec, out_dir / unit.task)
            done.append(k)
        return done

    def ingest_offline(self, slot, unit: Unit, k: int, row: dict, prior: dict | None,
                       task_dir: Path) -> None:
        from .episode import instruction_mismatch

        layout = RunLayout(self.output / unit.arm.id)
        eid = (prior or {}).get("episode_id")
        success = row.get("success")
        mismatch = instruction_mismatch(row, self.study.protocol.instruction_type)
        status = "error" if mismatch else "ok" if success is not None else "attempt_error"
        res = EpisodeResult(job_id=self.study.job_id, arm_id=unit.arm.id, task=unit.task,
                            episode=k, status=status, attempted=True, episode_id=eid,
                            success=None if success is None else bool(success),
                            score=row.get("score"), end="ended", slot=slot.slot_id,
                            node=slot.node_id,
                            error=mismatch or (None if success is not None
                                               else "RoboLab's row has no verdict"))
        rec = {**res.to_record(), "reconciled": "from RoboLab's row: the ledger row never "
                                                "reached the runner",
               **({"prior_status": prior.get("status")} if prior else {})}
        if eid:
            from amp.contracts.robot import EpisodeRef

            ref = EpisodeRef(episode_id=eid, task=unit.task, episode=k,
                             episode_dir=layout.episodes / layout.final_name(unit.task, eid),
                             results_dir=task_dir, run_id=self.run_id(unit.arm))
            try:
                self.backend.ingest(ref, NullTracer("eval"))
            except Exception as e:  # noqa: BLE001
                rec["ingest_error"] = f"{type(e).__name__}: {e}"
        write_json(layout.unit_path(unit.task, k), rec)
        log(f"reconciled {unit.arm.id} {unit_name(unit.task, k)} from RoboLab's row")

    def finish(self) -> int:
        """Merge each harness configuration's slot folders with RoboLab's tools and check the census."""
        from .finalize import finalize_run

        ok = not self.failures
        merged_root = self.output / "robolab" / "merged"
        for arm in self.study.arms:
            rid = self.run_id(arm)
            sources = [self.output / "robolab" / s / rid for s in self.slots_on_disk()]
            dest = merged_root / rid
            if dest.exists():
                shutil.rmtree(dest)
            dest.parent.mkdir(parents=True, exist_ok=True)
            try:
                got = robolab_merge.merge(sources, dest,
                                          robolab_root=self.backend.robolab_root,
                                          python=self.backend.python, tasks=self.tasks,
                                          num_runs=self.num_runs, log=self.logs / "merge.log")
                log(f"{arm.id}: merged {got['rows']} rows into {dest}")
                if got.get("summary"):
                    print(got["summary"])
            except robolab_merge.MergeError as e:
                ok = False
                self.failures.append(f"{arm.id}: merge: {e}")
                log(self.failures[-1])
        with contextlib.suppress(Exception):
            finalize_run(self.study, self.output)
        for f in self.failures:
            log(f"FAILED: {f}")
        return 0 if ok else 1

    def close(self) -> None:
        for d in self.drivers.values():
            d.abort_all()
            d.wait_idle(60.0)
            d.close()
        self.procs.stop_monitor()
        self.procs.close()
        shutil.rmtree(self._sock_dir, ignore_errors=True)


def task_metadata(robolab_root: str | Path, task_set: str = "benchmark") -> list[str]:
    """The task set's task names, from RoboLab's _metadata/task_metadata.json."""
    rows = json.loads((Path(robolab_root) / METADATA / "task_metadata.json").read_text())
    return sorted(r["task_name"] for r in rows
                  if str(r.get("filename", "")).split("/", 1)[0] == task_set)


def task_estimates(robolab_root: str | Path, tasks: Iterable[str], num_runs: int
                   ) -> dict[str, float]:
    """Seconds per task: RoboLab's measured wall_total_s, else episode_s x num_runs."""
    md = Path(robolab_root) / METADATA
    est: dict[str, float] = {}
    with contextlib.suppress(OSError, ValueError):
        for r in json.loads((md / "task_timing.json").read_text()):
            est[r["task_name"]] = float(r["wall_total_s"])
    with contextlib.suppress(OSError, ValueError):
        for r in json.loads((md / "task_metadata.json").read_text()):
            est.setdefault(r["task_name"], float(r.get("episode_s") or 0) * num_runs)
    return {t: est.get(t, 0.0) for t in tasks}


class ProfileError(ValueError):
    """A harness configuration make_profile refuses; str() is one line saying why."""


def _prompt_names(d: Path) -> set[str] | None:
    return {p.stem for p in d.glob("*.md") if p.stem.lower() != "readme"} if d.is_dir() else None


def make_profile(model: str, reasoning: str | None = None, prompt: str | None = None,
                 extra: dict | None = None, *, prompts: Path | None = None,
                 vision: str | None = None, vdm_model: str | None = None):
    """The harness configuration's AgentProfile; refuses unknown models, reasoning efforts,
    prompts, vision models and bad extras."""
    from amp.agents.models import profile, vision_fields

    names = _prompt_names(prompts or Path(__file__).resolve().parents[1] / "agents" / "prompts")
    if prompt and names is not None and prompt not in names:
        raise ProfileError(f"unknown prompt {prompt!r}: one of {', '.join(sorted(names))}")
    kw = {"prompt": prompt} if prompt else {}
    if extra:
        kw["extra"] = dict(extra)
    try:
        kw.update(vision_fields(vision, vdm_model))
    except (KeyError, ValueError) as e:
        raise ProfileError(str(e).strip("'\"")) from None
    try:
        built = profile(model, reasoning or None, **kw)
    except KeyError as e:
        raise ProfileError(str(e).strip("'\"")) from None
    except ValueError as e:
        raise ProfileError(str(e)) from None
    spec = built.model
    if reasoning and spec.reasoning_kind == "none":
        raise ProfileError(f"model {spec.key} takes no reasoning setting, got {reasoning!r}")
    if reasoning and spec.reasoning_levels and reasoning not in spec.reasoning_levels:
        raise ProfileError(f"model {spec.key}: reasoning {reasoning!r} is not one of "
                           f"{', '.join(spec.reasoning_levels)}")
    return built


def make_backend(robolab_root: str, python: str):
    from amp.robots.robolab.backend import RoboLabBackend

    return RoboLabBackend(robolab_root=robolab_root, python=python)


def robolab_paths(root: str | None = None, python: str | None = None, environ=None
                  ) -> tuple[str, str | None]:
    """(RoboLab root, its venv python): flags, then $AMP_ROBOLAB_*, $ROBOLAB_ROOT, then a clone
    in external/robolab; python defaults to <root>/.venv/bin/python when that exists."""
    env = os.environ if environ is None else environ
    root = root or env.get(ROBOLAB_ROOT_ENV) or env.get("ROBOLAB_ROOT") \
        or str(own_src().parent / "external" / "robolab")
    python = python or env.get(ROBOLAB_PYTHON_ENV)
    if not python:
        venv = Path(root) / ".venv" / "bin" / "python"
        python = str(venv) if venv.exists() else None
    return root, python


def parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="amp eval robolab", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--harness", required=True,
                    help="comma-separated harness ids, one harness configuration each: amp1_basic "
                         "(the basic harness) or amp1_assistive (the assistive harness)")
    ap.add_argument("--prompt", default="amp1",
                    help="the system prompt id (default amp1, the AMP1 system prompt)")
    ap.add_argument("--model", required=True, help="a model key (docs/models.md)")
    ap.add_argument("--reasoning", default=None,
                    help="reasoning effort (default: the model's)")
    vision.add_arguments(ap)
    ap.add_argument("--task-set", default="benchmark",
                    help="RoboLab's task folder (benchmark: the 120 tasks of RoboLab-120)")
    ap.add_argument("--task", action="append", default=[],
                    help="run only these tasks (repeat or comma-separate)")
    ap.add_argument("--num-runs", type=int, default=10,
                    help="episodes per task (RoboLab's --num-runs); the paper runs 10 per task on "
                         "RoboLab-120, 1,200 per model (default 10)")
    ap.add_argument("--gpus", default="auto", help="auto (every NVIDIA GPU that is not busy), or GPU indices or UUIDs")
    ap.add_argument("--slots-per-gpu", default="auto", help="auto or 1 (one Kit per GPU)")
    ap.add_argument("--output", required=True, type=Path, help="the run folder; reuse to resume")
    ap.add_argument("--job-id", default=None, help="default: the --output folder's name")
    ap.add_argument("--sandbox", default="auto", choices=("auto", "bwrap"))
    ap.add_argument("--instruction-type", default="default",
                    choices=("default", "vague", "specific"),
                    help="the instruction condition of every task; the paper runs all three")
    ap.add_argument("--robolab-root", default=None,
                    help=f"the RoboLab checkout (${ROBOLAB_ROOT_ENV}, else external/robolab)")
    ap.add_argument("--robolab-python", default=None,
                    help=f"RoboLab's venv python (${ROBOLAB_PYTHON_ENV}, else <root>/.venv)")
    ap.add_argument("--gateway-port", type=int, default=8710)
    ap.add_argument("--server-base-port", type=int, default=9600)
    ap.add_argument("--max-restarts", type=int, default=MAX_RESTARTS)
    return ap


def build_study(a: argparse.Namespace, tasks: Sequence[str], est: dict[str, float]) -> Study:
    agent = make_profile(a.model, a.reasoning, a.prompt, **vision.profile_kwargs(a))
    arms = tuple(Arm(h.strip(), agent) for h in a.harness.split(",") if h.strip())
    vision.check(arms, a.sandbox)
    proto = EvalProtocol(sandbox=a.sandbox, instruction_type=a.instruction_type, official=True)
    job = a.job_id or Path(a.output).resolve().name
    return Study(arms=arms, corpus=None, tasks=tuple(tasks), episodes=a.num_runs,
                 protocol=proto, job_id=job, code_ref="local",
                 est_s=tuple(sorted(est.items())))


def main(argv: list[str] | None = None) -> int:
    ap = parser()
    a = ap.parse_args(argv)
    if a.num_runs < 1:
        ap.error("--num-runs must be >= 1")
    a.robolab_root, a.robolab_python = robolab_paths(a.robolab_root, a.robolab_python)
    if not a.robolab_python:
        ap.error(f"give --robolab-root and --robolab-python (or ${ROBOLAB_ROOT_ENV} and "
                 f"${ROBOLAB_PYTHON_ENV}); amp doctor checks them")
    try:
        chosen, refused = gpus.select(a.gpus, gpus.query(), warn=log)
        for g, why in refused:
            log(f"refused {g.describe()}: {why}")
        per_gpu = gpus.slots_per_gpu(a.slots_per_gpu, chosen)
    except gpus.GpuError as e:
        ap.error(str(e))
    tasks = [t.strip() for arg in a.task for t in arg.split(",") if t.strip()]
    if not tasks:
        tasks = task_metadata(a.robolab_root, a.task_set)
    est = task_estimates(a.robolab_root, tasks, a.num_runs)
    try:
        study = build_study(a, tasks, est)
    except (KeyError, ValueError) as e:
        ap.error(str(e))
    backend = make_backend(a.robolab_root, a.robolab_python)
    log(f"{len(study.arms)} harness configurations x {len(tasks)} tasks x {a.num_runs} runs on "
        f"{len(chosen)} GPUs x {per_gpu} slots: {', '.join(g.describe() for g in chosen)}")
    launcher = Launcher(study=study, backend=backend, tasks=tasks, num_runs=a.num_runs,
                        output=a.output, gpu_list=chosen, slots_per_gpu=per_gpu, est=est,
                        gateway_port=a.gateway_port, server_base_port=a.server_base_port,
                         max_restarts=a.max_restarts)
    stop_on_signals()
    try:
        return launcher.run()
    except KeyboardInterrupt:
        return 130


def _interrupt(signum, frame):
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    raise KeyboardInterrupt(f"signal {signum}")


def stop_on_signals() -> None:
    """SIGTERM and SIGINT stop the run cleanly; a shell's background job inherits SIGINT ignored,
    and an unhandled SIGTERM would orphan the gateway, robot servers and Kits."""
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, _interrupt)


if __name__ == "__main__":
    sys.exit(main())
