#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp eval capgym: official CaP-Gym units on local GPUs, one CaP-X client process per slot.

    amp eval capgym --harness amp1_basic,amp1_assistive --prompt amp1 \\
        --model opus-5.5 --corpus libero-pro [--task T ...] [--episodes N] \\
        --gpus auto|0,1 --slots-per-gpu auto|N --output runs/<name>

One gateway is shared; each slot has its GPU, a port block, a robot server and a CaP-X client
for one task set. Every unit runs CaP-Gym's official protocol as one attempt on an unbounded trial
clock. Rerunning with the same --output skips the units already final.
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import secrets
import shutil
import socket
import sys
import threading
import time
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.eval import Arm, EpisodeResult, EvalProtocol, NodeRuntime
from amp.trace import FileTracer

from . import egl, envs, gpus, vision
from .capgym_clock import TRIAL_CLOCK
from .capgym_slot import CapgymSlot, reserve_port_block
from .episode import RunnerOptions
from .procs import ProcManager, ProcSpec, RestartPolicy
from .robolab_official import make_profile, stop_on_signals
from .slots import own_src
from .store import write_json
from .study import Study, StudyError

BENCHMARK = "capgym"
GATEWAY_UP_S = 60.0
EGL_AUTO_SLOTS = 3
EGL_MAX_SLOTS = 6
CUDA_MAX_SLOTS = 1
PYTHON_FLAGS = {"libero": ("libero",), "robosuite": ("robosuite", "robosuite_two_arm",
                                                     "robosuite_wipe"), "behavior": ("behavior",)}


def log(msg: str, prog: str = "amp eval capgym") -> None:
    print(f"[{prog} {time.strftime('%H:%M:%S')}] {msg}", file=sys.stderr, flush=True)


def slots_per_gpu(spec: str, mode: str, *, warn=log) -> int:
    """Slots per GPU for --slots-per-gpu auto|N: MuJoCo (egl) families 3 by default, BEHAVIOR 1."""
    top = CUDA_MAX_SLOTS if mode == "cuda" else EGL_MAX_SLOTS
    if spec.strip() in ("", "auto"):
        return 1 if mode == "cuda" else EGL_AUTO_SLOTS
    try:
        n = int(spec)
    except ValueError:
        raise gpus.GpuError(f"--slots-per-gpu {spec!r} is not auto or an integer") from None
    if not 1 <= n <= top:
        raise gpus.GpuError(f"--slots-per-gpu must be 1..{top} for {mode} families")
    if mode == "egl" and n > EGL_AUTO_SLOTS:
        warn(f"warning: {n} MuJoCo slots per GPU; CaP-Gym's runs use {EGL_AUTO_SLOTS}")
    return n


def allocate(waiting: Counter[str], slots: int) -> list[str]:
    """A task set per slot: each set at least one while slots last, the rest by its units."""
    sets = sorted(waiting, key=lambda t: (-waiting[t], t))
    out = sets[:slots]
    while len(out) < slots and sets:
        load = Counter(out)
        out.append(max(sets, key=lambda t: (waiting[t] / (load[t] + 1), -sets.index(t))))
    return out


def task_set_modes(task_sets) -> dict[str, str]:
    from amp.robots.capgym.families import family

    return {ts: family(ts).gpu for ts in task_sets}


@dataclass
class Launcher:
    """The shared gateway, the slots and run_local over them; then the run's summary."""

    study: Study
    backend: object
    output: Path
    gpu_list: list[gpus.Gpu]
    slots_per_gpu: int
    egl_map: dict[str, int] = field(default_factory=dict)
    options: RunnerOptions | None = None
    python: str = sys.executable
    start_services: bool = True
    retries: int | None = None
    slot_factory: object = CapgymSlot
    slots: list = field(default_factory=list)
    results: list[EpisodeResult] = field(default_factory=list)
    prog = "amp eval capgym"

    def log(self, msg: str) -> None:
        log(msg, self.prog)

    def __post_init__(self):
        self.output = Path(self.output).resolve()
        self.node = self.output / "_node"
        self.logs = self.node / "logs"
        self.tracer = FileTracer(self.node / "trace", "eval", episode_id=self.study.job_id)
        self.procs = ProcManager(token=secrets.token_hex(12), tracer=self.tracer)
        self._key = secrets.token_urlsafe(32)
        self.abort = threading.Event()
        self._serving_lock = threading.Lock()
        self._moving: dict[str, str] = {}
        self._ports = None
        self.gateway_port = 0

    @property
    def n_slots(self) -> int:
        """The GPUs' seats, but no more than there are units: an idle slot only costs ports."""
        units = len(self.study.units) * len(self.study.arms)
        return max(1, min(len(self.gpu_list) * self.slots_per_gpu, units))

    @property
    def gateway_url(self) -> str:
        return f"http://127.0.0.1:{self.gateway_port}"

    @property
    def runtime(self) -> NodeRuntime:
        return NodeRuntime(agent_python=self.python, kernel_python=self.python,
                           runs_dir=str(self.output), state_dir=str(self.node),
                           server_python=self.python,
                           gateway_port=self.gateway_port,
                           server_base_port=self.gateway_port + 1)

    def waiting(self) -> Counter[str]:
        return Counter(u.task_set for u in self.study.units)

    def build_slots(self) -> None:
        n = self.n_slots
        self._ports = reserve_port_block(n + 1)
        self.gateway_port = self._ports.base
        seats = [(g, k) for k in range(self.slots_per_gpu) for g in self.gpu_list][:n]
        for i, ((g, k), ts) in enumerate(zip(seats, allocate(self.waiting(), n), strict=True)):
            sid = f"g{g.index}.{k}"
            self.slots.append(self.slot_factory(
                slot_id=sid, gpu_index=g.index, gpu_uuid=g.uuid, gateway_url=self.gateway_url,
                server_port=self.gateway_port + 1 + i, dir=self.node / "slots" / sid,
                gateway_state_dir=self.node / "gateway" / "state", log_dir=self.logs,
                node_id=socket.gethostname(), key_fn=lambda: self._key,
                robot_backend=self.backend, procs=self.procs, task_set=ts,
                egl_device=self.egl_map.get(g.uuid), server_python=self.python))

    def next_task_set(self, slot, waiting: Counter[str]) -> str | None:
        """Where a slot whose task set ran dry goes: the set with the most units per slot."""
        with self._serving_lock:
            serving = Counter(self._moving.get(s.slot_id, s.task_set) for s in self.slots
                              if s is not slot)
            sets = [t for t in waiting if waiting[t] > 0]
            if not sets:
                return None
            to = max(sets, key=lambda t: (waiting[t] / (serving[t] + 1), t))
            if serving[to] and waiting[to] <= serving[to]:
                return None
            self.log(f"{slot.slot_id}: {slot.task_set} is done; moving to {to} "
                f"({waiting[to]} units waiting)")
            self._moving[slot.slot_id] = to
            self._write_slot_task_sets()
            return to

    @property
    def slot_task_sets_path(self) -> Path:
        return self.node / "gateway" / "slot_task_sets.json"

    def slot_task_sets(self) -> dict[str, str]:
        """{slot: the task set it hosts, or is moving to}: what the gateway exempts it for."""
        return {s.slot_id: self._moving.get(s.slot_id, s.task_set) for s in self.slots}

    def _write_slot_task_sets(self) -> None:
        write_json(self.slot_task_sets_path, self.slot_task_sets())

    def gateway_argv(self, config: Path) -> tuple[str, ...]:
        """The gateway's command: every CaP-Gym set may run episodes of one task on several slots."""
        from amp import benchmarks

        argv = (self.python, "-m", "amp.gateway", "--config", str(config), "--poll-s", "1.0",
                "--concurrent-task-sets", ",".join(benchmarks.get(BENCHMARK).task_sets),
                "--slot-task-sets", str(self.slot_task_sets_path))
        floors = benchmarks.gateway_load_timeouts()
        if floors:
            argv += ("--load-timeouts", ",".join(f"{k}={v:g}" for k, v in sorted(floors.items())))
        return argv

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
        with self._serving_lock:
            self._write_slot_task_sets()
        spec = ProcSpec("gateway", self.gateway_argv(path), str(self.logs / "gateway.log"),
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

    def write_plan(self) -> None:
        self.output.mkdir(parents=True, exist_ok=True)
        write_json(self.node / "plan.json", {
            "job_id": self.study.job_id, "corpus": self.study.corpus,
            "episodes": self.study.episodes, "units": len(self.study.units) * len(self.study.arms),
            "trial_clock": TRIAL_CLOCK,
            "gpus": [g.uuid for g in self.gpu_list], "slots_per_gpu": self.slots_per_gpu,
            "busy_gpus": [g.uuid for g in self.gpu_list if g.busy],
            "egl_devices": {u: d for u, d in self.egl_map.items()
                            if u in {g.uuid for g in self.gpu_list}},
            "gateway_port": self.gateway_port,
            "slots": [{"slot": s.slot_id, "gpu": s.gpu_uuid, "server_port": s.server_port,
                       "task_set": s.task_set, "egl_device": s.egl_device} for s in self.slots]})

    def on_result(self, r: EpisodeResult) -> None:
        self.results.append(r)
        verdict = "" if r.success is None else f" success={r.success}"
        self.log(f"{r.arm_id} {r.task} #{r.episode}: {r.status}{verdict}"
            + (f" ({r.error[:160]})" if r.error and r.status != "ok" else ""))

    def run(self) -> int:
        from .finalize import success_summary
        from .local import run_local
        from .provenance import runtime_facts

        self.build_slots()
        self.write_plan()
        kw = {} if self.retries is None else {"retries": self.retries}
        try:
            if self.start_services:
                self.procs.start_monitor()
                self.start_gateway()
            for s in self.slots:
                self.log(f"{s.slot_id}: GPU {s.gpu_index} ({s.gpu_uuid}), {s.task_set}, server "
                    f"{s.server_url}" + (f", EGL device {s.egl_device}"
                                         if s.egl_device is not None else ""))
            run = run_local(self.study, launcher=self, runs_dir=self.output.parent, root=self.output,
                            parallel=self.n_slots, runtime=self.runtime, tracer=self.tracer,
                            options=self.options, abort=self.abort, on_result=self.on_result,
                            provenance=runtime_facts(), **kw)
        except KeyboardInterrupt:
            self.log("interrupted: stopping (rerun with the same --output to resume)")
            self.abort.set()
            raise
        finally:
            self.close()
        resumed = sum(r.status == "resumed" for r in run.results)
        if resumed:
            self.log(f"{resumed} units already had a final record and were skipped")
        summary = success_summary(self.study, self.output)
        for arm, s in summary["arms"].items():
            self.log(f"{arm}: {s['total']['succeeded']}/{s['total']['scored']} succeeded "
                f"({s['total']['units']} units, {s['total']['missing']} without a record)")
        self.log(f"{run.ok}/{len(run.results)} delivered; records in {self.output}")
        return 0 if run.ok == len(run.results) else 1

    def close(self) -> None:
        for s in self.slots:
            with contextlib.suppress(Exception):
                s.stop()
        self.procs.stop_monitor()
        self.procs.close()
        if self._ports is not None:
            self._ports.release()
        with contextlib.suppress(Exception):
            self.tracer.close()


def make_backend(pythons: dict[str, str]):
    from amp.robots.capgym.backend import CapgymBackend

    return CapgymBackend(python=pythons or None)


def family_pythons(a: argparse.Namespace) -> dict[str, str]:
    """{family: interpreter} from --libero-python, --robosuite-python and --behavior-python."""
    out: dict[str, str] = {}
    for flag, fams in PYTHON_FLAGS.items():
        py = getattr(a, f"{flag}_python", None)
        if py:
            out.update({f: py for f in fams})
    return out


def check_pythons(backend, task_sets: Sequence[str]) -> list[str]:
    """What stops a task set's CaP-X client starting here: its venv, LIBERO config or toolchain."""
    from amp.robots.capgym.backend import LIBERO_CONFIG_ENV
    from amp.robots.capgym.families import family, flag
    from amp.robots.capgym.sandbox import TOOLCHAIN_ENV

    problems = []
    for ts in task_sets:
        fam = family(ts)
        try:
            py = backend.python_for(ts)
        except ValueError as e:
            problems.append(f"{ts}: {e}")
            continue
        if not py:
            problems.append(f"{ts}: CaP-X {fam.name} has no interpreter: set "
                            f"${fam.python_env} or {flag(fam)}")
        elif not (os.path.isfile(py) or shutil.which(py)):
            problems.append(f"{ts}: CaP-X {fam.name} interpreter {py} not found (set "
                            f"${fam.python_env} or {flag(fam)})")
        need = {"libero": LIBERO_CONFIG_ENV, "behavior": TOOLCHAIN_ENV}.get(fam.name)
        if need and not os.environ.get(need):
            problems.append(f"{ts}: CaP-X {fam.name} needs ${need} (docs/capgym.md section 1)")
    return problems


def parser() -> argparse.ArgumentParser:
    from amp import benchmarks

    ap = argparse.ArgumentParser(prog="amp eval capgym", description=__doc__,
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
    ap.add_argument("--corpus", default=None,
                    help="default: " + benchmarks.get(BENCHMARK).default_corpus + "; "
                    + benchmarks.listing(BENCHMARK))
    ap.add_argument("--task", action="append", default=[],
                    help="run only these tasks of the corpus (repeat or comma-separate)")
    ap.add_argument("--episodes", type=int, default=None,
                    help="episodes per task (default: the corpus's official count)")
    ap.add_argument("--gpus", default="auto",
                    help="auto (every NVIDIA GPU that is not busy), or GPU indices or UUIDs")
    ap.add_argument("--slots-per-gpu", default="auto",
                    help=f"auto ({EGL_AUTO_SLOTS} for MuJoCo families, 1 for BEHAVIOR) or N")
    ap.add_argument("--output", required=True, type=Path, help="the run folder; reuse to resume")
    ap.add_argument("--job-id", default=None, help="default: the --output folder's name")
    ap.add_argument("--sandbox", default="auto", choices=("auto", "bwrap"))
    ap.add_argument("--idle-timeout-s", type=float, default=1800.0)
    ap.add_argument("--episode-timeout-s", type=float, default=21600.0)
    ap.add_argument("--load-timeout-s", type=float, default=900.0)
    ap.add_argument("--libero-python", default=None,
                    help="the CaP-X LIBERO venv's python (default $AMP_CAPX_LIBERO_PYTHON)")
    ap.add_argument("--robosuite-python", default=None,
                    help="the CaP-X robosuite venv's python (default $AMP_CAPX_ROBOSUITE_PYTHON)")
    ap.add_argument("--behavior-python", default=None,
                    help="the CaP-X BEHAVIOR venv's python (default $AMP_CAPX_BEHAVIOR_PYTHON)")
    ap.add_argument("--max-retries", type=int, default=None,
                    help="re-runs of a unit that ended infra_error (default 2)")
    return ap


def build_study(a: argparse.Namespace, tasks: Sequence[str]) -> Study:
    from amp import benchmarks

    agent = make_profile(a.model, a.reasoning, a.prompt, **vision.profile_kwargs(a))
    arms = tuple(Arm(h.strip(), agent) for h in a.harness.split(",") if h.strip())
    vision.check(arms, a.sandbox)
    proto = EvalProtocol(sandbox=a.sandbox, idle_timeout_s=a.idle_timeout_s,
                         episode_timeout_s=a.episode_timeout_s, load_timeout_s=a.load_timeout_s,
                         official=True)
    _, corpus = benchmarks.resolve(BENCHMARK, a.corpus)
    job = a.job_id or Path(a.output).resolve().name
    return Study(arms=arms, corpus=corpus, tasks=tuple(tasks) or None, episodes=a.episodes,
                 protocol=proto, job_id=job, code_ref="local", benchmark=BENCHMARK)


def egl_map(chosen: Sequence[gpus.Gpu], python: str | None) -> tuple[dict[str, int], str]:
    """The chosen GPUs' EGL devices; the backend reads the same map from AMP_EGL_DEVICE_MAP."""
    got, how, listed = egl.resolve([g.uuid for g in chosen],
                                   devices=lambda: egl.enumerate_devices(python))
    flat = egl.positional(got, listed)
    if flat is not None:
        os.environ[egl.MAP_ENV] = flat
    else:
        os.environ.pop(egl.MAP_ENV, None)
    return got, how


def main(argv: list[str] | None = None) -> int:
    ap = parser()
    a = ap.parse_args(argv)
    if a.episodes is not None and a.episodes < 1:
        ap.error("--episodes must be >= 1")
    tasks = [t.strip() for arg in a.task for t in arg.split(",") if t.strip()]
    try:
        study = build_study(a, tasks)
        units = study.units
    except (KeyError, ValueError, StudyError) as e:
        ap.error(str(e).strip("'\""))
    modes = task_set_modes({u.task_set for u in units})
    if len(set(modes.values())) > 1:
        ap.error("BEHAVIOR (CUDA) and the MuJoCo (EGL) families do not share GPUs: an EGL render "
                 "next to busy CUDA work can stall for seconds; run them separately")
    mode = next(iter(modes.values()))
    backend = make_backend(family_pythons(a))
    problems = check_pythons(backend, sorted(modes))
    if problems:
        ap.error("; ".join(problems))
    try:
        chosen, refused = gpus.select(a.gpus, gpus.query(), warn=log)
        for g, why in refused:
            log(f"refused {g.describe()}: {why}")
        per_gpu = slots_per_gpu(a.slots_per_gpu, mode)
    except gpus.GpuError as e:
        ap.error(str(e))
    devices: dict[str, int] = {}
    if mode == "egl":
        try:
            devices, how = egl_map(chosen, backend.python_for(min(modes)))
        except egl.EglError as e:
            ap.error(str(e))
        log(f"EGL devices ({how}): " + ", ".join(f"GPU {g.index} -> {devices[g.uuid]}"
                                                 for g in chosen))
    n_units = len(units) * len(study.arms)
    log(f"{len(study.arms)} harness configurations x {len(units)} units ({study.corpus}, "
        f"{', '.join(sorted(modes))}) on {len(chosen)} GPUs x {per_gpu} slots, trial clock "
        f"{TRIAL_CLOCK}: {n_units} units")
    launcher = Launcher(study=study, backend=backend, output=a.output, gpu_list=chosen,
                        slots_per_gpu=per_gpu, egl_map=devices, retries=a.max_retries)
    stop_on_signals()
    try:
        return launcher.run()
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
