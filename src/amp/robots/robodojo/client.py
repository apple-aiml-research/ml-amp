#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The RoboDojo launcher: the slot's only link client, driving RoboDojo's own runner per episode.
One episode is one unit (task, seed, layout) and one RoboDojo process (docs/robodojo.md)."""

from __future__ import annotations

import argparse
import atexit
import contextlib
import json
import os
import re
import secrets
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections.abc import Callable, Iterator, Sequence
from dataclasses import asdict
from pathlib import Path

from amp.benchmarks.robodojo import CONTROL_DT, TASK_SET, catalog_tasks, unit_spec
from amp.link.client import DEFAULT_URL, LinkClient, LinkError
from amp.robots import jobs
from amp.robots.base import pythonpath
from amp.robots.capgym import sandbox
from amp.robots.capgym.adapter import Frame
from amp.robots.capgym.client import (
    DEFAULT_POLICY_LABEL,
    CapgymWorker,
    run_episode,
    wire_idle,
)
from amp.robots.robodojo import bridge as B
from amp.robots.robodojo import overlay as O
from amp.robots.robolab import records as R

BENCHMARK = "RoboDojo"
POLICY_NAME = "amp"
ENV_CFG_TYPE = "arx_x5_amp"
CONFIG_NAME = "arx_x5"
RESULT_FILE = "_result.json"
ROBOT_DIR = ("Robots", "x5")
CAMERAS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
CAMERA_KEYS = ("color", "depth", "intrinsic_matrix", "extrinsic_matrix")
LAYOUT_DIR = ("Eval_Layout", BENCHMARK, CONFIG_NAME)
KIT_WRITES = ("data", "logs", "cache")
PORT_ENV = "AMP_ROBODOJO_PORT"
DEFAULT_PORT = 19700
LOAD_TIMEOUT_S = 850.0
STEP_TIMEOUT_S = 300.0
HOLD_TIMEOUT_S = 1800.0
EXIT_TIMEOUT_S = 600.0
POLL_S = 2.0
KILL_GRACE_S = 10.0
KILL_POLL_S = 0.05
KILL_CONFIRM_S = 0.5
LINK_KEEPALIVE_S = 60.0
ATEXIT_GRACE_S = 5.0
SHM_MONITOR_GLOB = "physx_monitor_*"
SHM_DIR = "/dev/shm"
SCORED, ABANDONED = "scored", "abandoned"


class LayoutAbandoned(RuntimeError):
    """RoboDojo abandoned this layout before AMP saw the scene: terminal, and never replayed."""

    void = True


class RoboDojoError(RuntimeError):
    """RoboDojo ran, but not the unit AMP asked for: a retried infra failure, same layout."""


def _log(message: str, *, err: bool = False) -> None:
    print(f"[amp.robots.robodojo] {message}", file=sys.stderr if err else sys.stdout, flush=True)


def layout_ids(assets: Path, seed: int, task: str) -> list[int]:
    """The layout ids of (task, seed): Assets/Eval_Layout/RoboDojo/arx_x5/<seed>/<task>_<i>.json."""
    folder = Path(assets).joinpath(*LAYOUT_DIR, str(int(seed)))
    out = []
    for path in sorted(folder.glob(f"{task}_*.json")) if folder.is_dir() else []:
        tail = path.stem[len(task) + 1:]
        if tail.isdigit():
            out.append(int(tail))
    return sorted(out)


def unit_dir(overlay: Path, task: str, seed: int, additional_info: str, *,
             policy: str = POLICY_NAME) -> Path:
    """Where RoboDojo keeps this (task, seed, info)'s manifests and run dirs."""
    return (Path(overlay) / "eval_result" / BENCHMARK / task / policy / CONFIG_NAME
            / f"{int(seed)}_{additional_info}")


def save_dir(overlay: Path, task: str, seed: int, additional_info: str, run_id: str, *,
             policy: str = POLICY_NAME) -> Path:
    """EvalEnv.save_dir: where _result.json and the episode's mp4s land."""
    return unit_dir(overlay, task, seed, additional_info, policy=policy) / run_id


def resume_manifest(run_id: str, abandoned: Sequence[int]) -> dict:
    """RoboDojo's resume format, restricting its runner to the one layout AMP wants."""
    return {"run_id": run_id, "success_nums": 0, "fail_nums": 0, "unstable_nums": 0,
            "total_score": 0.0, "details": {},
            "abandoned_layout_ids": [int(i) for i in abandoned], "restart_count": 0}


def write_resume(overlay: Path, *, task: str, seed: int, additional_info: str, run_id: str,
                 abandoned: Sequence[int], policy: str = POLICY_NAME) -> Path:
    path = (unit_dir(overlay, task, seed, additional_info, policy=policy)
            / f"_resume_{run_id}.json")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(resume_manifest(run_id, abandoned), indent=2))
    return path


def server_command(xpl_python: str, overlay: Path, *, task: str, port: int, seed: int) -> list[str]:
    """XPolicyLab's own policy server, serving AMP's no-op model."""
    return [xpl_python, "-m", "XPolicyLab.setup_policy_server", "--config_path",
            f"XPolicyLab/policy/{POLICY_NAME}/deploy.yml", "--overrides", f"port={port}",
            "host=localhost", f"bench_name={BENCHMARK}", f"task_name={task}", "ckpt_name=amp",
            f"env_cfg_type={ENV_CFG_TYPE}", f"seed={int(seed)}", f"policy_name={POLICY_NAME}",
            "action_type=joint"]


def eval_command(overlay: Path, *, task: str, gpu: int, port: int, seed: int,
                 additional_info: str) -> list[str]:
    """RoboDojo's unmodified runner, from the overlay root."""
    return ["bash", str(Path(overlay) / "scripts" / "eval_policy.sh"), "--root_dir", str(overlay),
            "--task_name", task, "--env_cfg_type", ENV_CFG_TYPE, "--device_id", str(max(gpu, 0)),
            "--policy_name", POLICY_NAME, "--port", str(port), "--seed", str(int(seed)),
            "--additional_info", additional_info]


def server_env(base: dict[str, str], overlay: Path) -> dict[str, str]:
    """The XPolicyLab server's env: the overlay and its XPolicyLab on PYTHONPATH."""
    parts = [str(overlay), str(Path(overlay) / "XPolicyLab")]
    inherited = [p for p in base.get("PYTHONPATH", "").split(os.pathsep) if p and p not in parts]
    return {**base, "PYTHONPATH": os.pathsep.join(parts + inherited), "PYTHONUNBUFFERED": "1"}


def kit_dir(venv: Path) -> Path | None:
    """Isaac Sim's kit directory in a RoboDojo venv, which is also Kit's portable root."""
    return next(iter(sorted(Path(venv).glob("lib/python*/site-packages/isaacsim/kit"))), None)


def _mirror(root: Path, source: Path, *, skip: Sequence[str]) -> Path:
    """`root`, carrying a symlink to each of `source`'s entries but those named in `skip`."""
    root.mkdir(parents=True, exist_ok=True)
    for entry in sorted(Path(source).iterdir()):
        link = root / entry.name
        if entry.name not in skip and not os.path.lexists(link):
            link.symlink_to(entry)
    return root


def kit_portable_root(slot_dir: Path, kit: Path) -> Path:
    """A per-slot stand-in for Isaac Sim's kit dir: data/logs/cache of its own, the rest the venv's."""
    root = _mirror(Path(slot_dir) / "kit_portable", Path(kit).parent, skip=("kit",))
    portable = _mirror(root / "kit", kit, skip=KIT_WRITES)
    for name in KIT_WRITES:
        (portable / name).mkdir(parents=True, exist_ok=True)
    return portable


def eval_env(base: dict[str, str], *, overlay: Path, run_id: str, bridge: dict[str, str],
             cache_dir: Path, python: str | None = None) -> dict[str, str]:
    """RoboDojo's env (docs/robodojo.md): its venv first on PATH, Vulkan, the run id and AMP's
    bridge; the Omniverse EULA variables are the caller's own."""
    py = Path(python or sys.executable)
    path = [str(py.parent), *[p for p in base.get("PATH", os.defpath).split(os.pathsep)
                              if p and p != str(py.parent)]]
    out = {**base, "PYTHONPATH": pythonpath(inherited=base.get("PYTHONPATH", "")),
           "PATH": os.pathsep.join(path), "VIRTUAL_ENV": str(py.parent.parent),
           "OMNI_CACHE_PATH": str(cache_dir), "TERM": "xterm-256color", "EVAL_NUM": "1",
           "ROBODOJO_RUN_ID": run_id, "PYTHONUNBUFFERED": "1", **bridge}
    for suppressed in ("PYTHONHOME", "ROBODOJO_MAX_BASH_RETRIES", "ROBODOJO_FATAL_RESTART_COUNT"):
        out.pop(suppressed, None)
    kit = kit_dir(Path(py.parent.parent))
    if kit is not None:
        out["CARB_APP_PATH"] = str(kit_portable_root(Path(cache_dir).parent, kit))
    icd = "/etc/vulkan/icd.d/nvidia_icd.json"
    if os.path.exists(icd):
        out["VK_ICD_FILENAMES"] = icd
    return out


def episode_score(entry: dict) -> float:
    """The unit's score in [0, 1] from RoboDojo's details entry, as upstream writes it."""
    score = float(entry.get("score") or 0.0)
    return min(max(score / 100.0 if score > 1.0 else score, 0.0), 1.0)


def instruction_of(obs: dict) -> str:
    """The episode's instruction from a RoboDojo obs: a str, or the first of a list of them."""
    raw = obs.get("instruction")
    if isinstance(raw, str):
        return raw
    if isinstance(raw, (list, tuple)):
        return next((r for r in raw if isinstance(r, str) and r), "")
    return ""


def check_cameras(obs: dict) -> None:
    """RoboDojoError unless the obs carries AMP's rig with depth and camera parameters."""
    vision = obs.get("vision") if isinstance(obs, dict) else None
    if not isinstance(vision, dict) or set(vision) != set(CAMERAS):
        raise RoboDojoError(
            f"RoboDojo rendered {sorted(vision) if isinstance(vision, dict) else vision}, not "
            f"AMP's rig {list(CAMERAS)}: is {ENV_CFG_TYPE}.yml in effect?")
    for name in CAMERAS:
        missing = [k for k in CAMERA_KEYS if vision[name].get(k) is None]
        if missing:
            raise RoboDojoError(
                f"camera {name} carries no {', '.join(missing)}: AMP's camera config "
                f"(depth, intrinsic_matrix, extrinsic_matrix) is not in effect")


_GROUPS: set[int] = set()
_LEADERS: dict[int, subprocess.Popen] = {}


def track(proc: subprocess.Popen | None) -> int | None:
    """Remember a child's own session so the atexit sweep can reach it, and say which it is."""
    if proc is None:
        return None
    try:
        pgid = os.getpgid(proc.pid)
    except OSError:
        return None
    if pgid != os.getpgrp():
        _GROUPS.add(pgid)
        _LEADERS[pgid] = proc
    return pgid


def forget(pgid: int | None) -> None:
    """Take a session off the atexit sweep's list; only ever one nothing is left in."""
    if pgid is not None:
        _GROUPS.discard(pgid)
        _LEADERS.pop(pgid, None)


def group_alive(pgid: int) -> bool:
    """Whether any process is still in `pgid` (a reaped child's group is gone)."""
    try:
        os.killpg(pgid, 0)
    except (OSError, ProcessLookupError):
        return False
    return True


def _signal_group(pgid: int, sig: int) -> bool:
    """Send `sig` to every process in `pgid`; False when there was nothing left to signal."""
    try:
        os.killpg(pgid, sig)
    except (OSError, ProcessLookupError):
        return False
    return True


def _gone(proc: subprocess.Popen | None, pgid: int | None) -> bool:
    """Whether nothing of a child is left: its leader has exited and its group is empty."""
    if proc is not None and proc.poll() is None:
        return False
    return pgid is None or not group_alive(pgid)


def _wait_gone(proc: subprocess.Popen | None, pgid: int | None, deadline: float) -> bool:
    """Poll until nothing of the child is left, or the deadline passes; whether it is gone."""
    while not _gone(proc, pgid):
        if time.monotonic() >= deadline:
            return False
        time.sleep(KILL_POLL_S)
    return True


def kill_groups(grace_s: float = ATEXIT_GRACE_S) -> list[int]:
    """SIGTERM then SIGKILL every session this process started; the ones it had to signal."""
    hit, empty = [], []
    for pgid in sorted(_GROUPS):
        if pgid == os.getpgrp():
            continue
        (hit if _signal_group(pgid, signal.SIGTERM) else empty).append(pgid)
    deadline = time.monotonic() + grace_s
    left = [pgid for pgid in hit if not _wait_gone(_LEADERS.get(pgid), pgid, deadline)]
    for pgid in left:
        _signal_group(pgid, signal.SIGKILL)
    confirm = time.monotonic() + KILL_CONFIRM_S
    stubborn = {pgid for pgid in left if not _wait_gone(_LEADERS.get(pgid), pgid, confirm)}
    for pgid in (set(hit) | set(empty)) - stubborn:
        forget(pgid)
    return hit


def stop_children_at_exit(*, close: Callable[[], None] | None = None) -> None:
    """Turn SIGTERM into an exit and kill RoboDojo's sessions from an atexit hook."""
    atexit.register(kill_groups)
    if close is not None:
        atexit.register(close)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))


def sweep_shm(pids: set[int], shm_dir: str = SHM_DIR) -> list[str]:
    """Remove the PhysX monitor files this launcher's own Kit left in /dev/shm; those it removed."""
    out = []
    for path in sorted(Path(shm_dir).glob(SHM_MONITOR_GLOB)) if pids else []:
        named = {int(part) for part in re.findall(r"\d+", path.name)}
        if not (named & pids):
            continue
        try:
            path.unlink()
        except OSError:
            continue
        out.append(str(path))
    return out


def session_pids(pgid: int | None) -> set[int]:
    """The pids currently in `pgid`, read from /proc; empty where /proc lists none."""
    if not pgid:
        return set()
    out = set()
    for entry in Path("/proc").glob("[0-9]*") if Path("/proc").is_dir() else []:
        try:
            pid = int(entry.name)
            if os.getpgid(pid) == pgid:
                out.add(pid)
        except (OSError, ValueError):
            continue
    return out


def _read_json(path: Path) -> dict | None:
    """The JSON object at `path`, or None when it is absent, truncated or not an object."""
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _pgid_of(proc: subprocess.Popen | None) -> int | None:
    """The process group `proc` leads, or None once its pid is gone or it shares this one's."""
    if proc is None:
        return None
    try:
        pgid = os.getpgid(proc.pid)
    except (OSError, ProcessLookupError):
        return None
    return None if pgid == os.getpgrp() else pgid


def _kill(proc: subprocess.Popen | None, pgid: int | None = None,
          grace_s: float = KILL_GRACE_S) -> bool:
    """SIGTERM then SIGKILL the whole process group; whether nothing of it is left. Never raises."""
    pgid = _pgid_of(proc) if pgid is None else pgid
    pgid = None if pgid == os.getpgrp() else pgid
    for sig in (signal.SIGTERM, signal.SIGKILL):
        if _gone(proc, pgid):
            return True
        if (pgid is None or not _signal_group(pgid, sig)) and proc is not None:
            with contextlib.suppress(OSError):
                proc.send_signal(sig)
        if _wait_gone(proc, pgid, time.monotonic() + grace_s):
            return True
    return _gone(proc, pgid)


def resolve_port(env: dict[str, str], port: int | None = None) -> int:
    """The XPolicyLab server's port: the one asked for, else the slot's block, else the default."""
    if port:
        return int(port)
    given = (env.get(PORT_ENV) or "").strip()
    if given:
        return int(given)
    what = "the RoboDojo XPolicyLab policy server"
    sandbox.require_ports(what)
    why = (env.get(sandbox.PORT_ERROR_ENV) or "").strip()
    if why:
        raise ValueError(f"{what} binds a port, and this slot has none: {why}")
    return DEFAULT_PORT


class RoboDojoAdapter:
    """One RoboDojo unit at a time, behind capgym's SimAdapter protocol."""

    def __init__(self, *, robodojo_root: str | Path, xpl_python: str, overlay_root: str | Path,
                 gpu: int = 0, port: int | None = None, load_timeout_s: float = LOAD_TIMEOUT_S,
                 step_timeout_s: float = STEP_TIMEOUT_S, hold_timeout_s: float = HOLD_TIMEOUT_S,
                 exit_timeout_s: float = EXIT_TIMEOUT_S, env: dict[str, str] | None = None,
                 link_keepalive: Callable[[], None] | None = None,
                 link_keepalive_s: float = LINK_KEEPALIVE_S,
                 log: Callable[..., None] = _log):
        self.robodojo_root = Path(robodojo_root).absolute()
        self.xpl_python, self.overlay_root = xpl_python, Path(overlay_root).absolute()
        self.gpu = gpu
        self.base_env = dict(os.environ if env is None else env)
        self.port = resolve_port(self.base_env, port)
        self.load_timeout_s, self.step_timeout_s = load_timeout_s, step_timeout_s
        self.hold_timeout_s, self.exit_timeout_s = hold_timeout_s, exit_timeout_s
        self.link_keepalive, self.link_keepalive_s = link_keepalive, link_keepalive_s
        self.log = log
        self.loaded: str | None = None
        self.unit: dict | None = None
        self.run_id, self.additional_info = "", ""
        self.layout_outcome, self.abandon_reason = "", ""
        self._overlay: Path | None = None
        self._proc: subprocess.Popen | None = None
        self._server: subprocess.Popen | None = None
        self._listener: B.BridgeServer | None = None
        self._bridge: B.Bridge | None = None
        self._sock_dir: str | None = None
        self._frame: Frame | None = None
        self._observed = 0
        self._instruction = ""
        self._ended = False
        self._robot: tuple | None = None
        self._spec = None
        self._pids: set[int] = set()
        self._runner_pgid: int | None = None
        self._server_pgid: int | None = None
        self._pinged = 0.0


    def robot_description(self):
        """The /sim/robot payload: the X5 chain and its STL meshes, from the checkout's Assets."""
        if self._robot is None:
            from amp.robots.robodojo.description import description

            self._robot = description(str(self.robodojo_root.joinpath("Assets", *ROBOT_DIR)))
        return self._robot

    def embodiment(self):
        """robodojo_dual_x5 naming the sha of the description this launcher actually sends."""
        if self._spec is None:
            from amp.link.robot import description_sha256
            from amp.robots.robodojo.embodiment import robodojo_dual_x5

            desc, meshes = self.robot_description()
            self._spec = robodojo_dual_x5(description_sha256(desc, meshes))
        return self._spec

    def success_checks(self) -> str:
        return "end"

    def tasks(self) -> list[dict]:
        return catalog_tasks()

    def load(self, task: str) -> None:
        self.loaded = task

    def control_dt(self) -> float:
        return CONTROL_DT

    def horizon(self) -> int | None:
        return None if self.unit is None else int(self.unit["step_lim"])

    def gt(self) -> None:
        return None

    def scene(self) -> None:
        return None

    def ended(self) -> bool:
        return self._ended

    def observe(self) -> Frame:
        if self._frame is None:
            raise RuntimeError("no observation yet: reset() before observe()")
        return self._frame

    def instruction(self) -> str:
        """RoboDojo's instruction for the reset episode, from its first observation, as it is."""
        return getattr(self, "_instruction", "") or ""

    def provenance(self) -> dict:
        """The checkout this launcher runs; its commit is the run's robodojo_commit (amp.eval)."""
        return {"task_set": TASK_SET, "robodojo_root": str(self.robodojo_root)}

    def overlay(self) -> Path:
        """The slot's overlay root, built on first use and rebuilt when AMP's policy changes."""
        if self._overlay is None:
            self._overlay = O.build_overlay(self.robodojo_root, self.overlay_root,
                                            amp_policy_dir=O.policy_dir(),
                                            amp_env_cfg_dir=O.env_cfg_dir())
        return self._overlay

    def outputs(self) -> Path | None:
        """RoboDojo's own output dir for the unit that just ran."""
        if self.unit is None or not self.run_id:
            return None
        return save_dir(self.overlay(), self.unit["task"], self.unit["seed"],
                        self.additional_info, self.run_id)


    def reset(self, episode: int) -> None:
        """Start RoboDojo on the unit of episode `episode` and wait for its first observation."""
        if self.loaded is None:
            raise RuntimeError("load(task) before reset()")
        self.close()
        unit = dict(unit_spec(self.loaded, int(episode)))
        self.unit, self._ended, self._observed, self._frame = unit, False, 0, None
        self.layout_outcome, self.abandon_reason, self._instruction = "", "", ""
        task, seed, layout = unit["task"], int(unit["seed"]), int(unit["layout"])
        overlay = self.overlay()
        self.run_id = f"{os.getpid()}_{int(episode)}_{secrets.token_hex(3)}"
        self.additional_info = f"amp_{self.run_id}"
        ids = layout_ids(O.assets_dir(overlay), seed, task)
        if layout not in ids:
            raise RoboDojoError(
                f"{task} seed {seed}: layout {layout} is not among the "
                f"{len(ids)} layouts in {O.assets_dir(overlay).joinpath(*LAYOUT_DIR, str(seed))}")
        write_resume(overlay, task=task, seed=seed, additional_info=self.additional_info,
                     run_id=self.run_id, abandoned=[i for i in ids if i != layout])
        self._sock_dir = B.socket_dir()
        self._listener = B.BridgeServer(os.path.join(self._sock_dir, B.SOCKET_NAME),
                                        B.new_authkey())
        cache = self.overlay_root.parent / "omni_cache"
        cache.mkdir(parents=True, exist_ok=True)
        self._server = subprocess.Popen(
            server_command(self.xpl_python, overlay, task=task, port=self.port, seed=seed),
            cwd=str(overlay), env=server_env(self.base_env, overlay), start_new_session=True)
        self._server_pgid = track(self._server)
        with self.keeping_link_alive():
            env = eval_env(self.base_env, overlay=overlay, run_id=self.run_id,
                           bridge=self._listener.env(), cache_dir=cache)
            self._proc = subprocess.Popen(
                eval_command(overlay, task=task, gpu=self.gpu, port=self.port, seed=seed,
                             additional_info=self.additional_info),
                cwd=str(overlay), start_new_session=True, env=env)
            self._runner_pgid = track(self._proc)
            self.log(f"{task} seed {seed} layout {layout}: run {self.run_id}, port {self.port}")
            deadline = time.monotonic() + self.load_timeout_s
            self._bridge = self._accept(deadline)
            self._take(self._await(deadline, "its first observation"))
        self._pinged = time.monotonic()
        self._note_pids()

    def apply(self, action: Sequence[float]) -> None:
        """Send one action and wait for the next observation or the episode's end."""
        from amp.robots.robodojo.frames import action_to_robodojo

        self._send({"type": "act", "action": action_to_robodojo(action)})
        self._take(self._await(time.monotonic() + self.step_timeout_s, "the next observation"))

    def idle(self) -> None:
        """capgym's wire_idle hook, called at every wait reply while the agent plans."""
        now = time.monotonic()
        if self._bridge is None or now - self._pinged < B.KEEPALIVE_INTERVAL_S:
            return
        self._pinged = now
        try:
            self._bridge.send(B.keepalive_message())
        except B.BridgeClosed:
            pass

    def success(self) -> tuple[bool, float]:
        """RoboDojo's verdict for this unit, from its own _result.json; (False, 0) once abandoned."""
        with self.keeping_link_alive():
            if not self._ended:
                self._hold()
            self._note_pids()
            self._wait_exit()
        outcome, reason, entry = self._outcome()
        self.layout_outcome, self.abandon_reason = outcome, reason
        if outcome == ABANDONED:
            self.log(f"RoboDojo abandoned {self.unit and self.unit['task']} layout "
                     f"{self.unit and self.unit['layout']} ({reason}): the unit is terminal and "
                     f"is never replayed, as upstream refills from its seed queue instead")
            return False, 0.0
        if outcome != SCORED:
            raise RoboDojoError(
                f"RoboDojo wrote no verdict for {self.unit and self.unit['task']} layout "
                f"{self.unit and self.unit['layout']} and did not abandon it (run {self.run_id}, "
                f"rc {self._proc and self._proc.returncode}): it broke before it could judge the "
                f"layout, so AMP retries the unit on the same layout")
        return bool(entry.get("success")), episode_score(entry)

    def close(self) -> None:
        """Kill both children and take away what the unit left; idempotent, never raises."""
        self._note_pids()
        for proc, pgid in ((self._proc, getattr(self, "_runner_pgid", None)),
                           (self._server, getattr(self, "_server_pgid", None))):
            gone = False
            try:
                gone = _kill(proc, pgid)
            except Exception as exc:  # noqa: BLE001
                self.log(f"could not kill {proc}: {exc!r}", err=True)
            if gone:
                forget(pgid)
            elif pgid is not None:
                self.log(f"process group {pgid} outlived close()'s SIGKILL; it stays on the "
                         f"atexit sweep's list", err=True)
        self._proc = self._server = None
        self._runner_pgid = self._server_pgid = None
        for closeable in (self._bridge, self._listener):
            try:
                if closeable is not None:
                    closeable.close()
            except Exception as exc:  # noqa: BLE001
                self.log(f"could not close the bridge: {exc!r}", err=True)
        self._bridge = self._listener = None
        if self._sock_dir:
            shutil.rmtree(self._sock_dir, ignore_errors=True)
            self._sock_dir = None
        self._sweep_shm()
        self._discard_outputs()
        self._frame, self._ended = None, False


    def _note_pids(self) -> None:
        """Remember this unit's session members while they are still alive, for the shm sweep."""
        self._pids |= session_pids(getattr(self, "_runner_pgid", None))

    def _sweep_shm(self) -> None:
        gone = sweep_shm(self._pids)
        self._pids = set()
        if gone:
            self.log(f"removed {len(gone)} PhysX monitor file(s) this unit's Kit left: "
                     f"{', '.join(gone)}")

    def _discard_outputs(self) -> None:
        """Delete the unit's whole eval_result dir once its files are copied out."""
        if self.unit is None or not self.additional_info or self._overlay is None:
            return
        shutil.rmtree(unit_dir(self._overlay, self.unit["task"], self.unit["seed"],
                               self.additional_info), ignore_errors=True)

    @contextlib.contextmanager
    def keeping_link_alive(self) -> Iterator[None]:
        """Touch the link every link_keepalive_s while the caller waits on RoboDojo."""
        ping, every = self.link_keepalive, self.link_keepalive_s
        if ping is None or every <= 0:
            yield
            return
        stop = threading.Event()

        def loop() -> None:
            while not stop.wait(every):
                try:
                    ping()
                except Exception as exc:  # noqa: BLE001
                    self.log(f"the link keepalive stopped: {exc!r}", err=True)
                    return

        thread = threading.Thread(target=loop, name="robodojo-link-keepalive", daemon=True)
        thread.start()
        try:
            yield
        finally:
            stop.set()
            thread.join(timeout=every + 5.0)


    def _exited(self) -> int | None:
        return None if self._proc is None else self._proc.poll()

    def _fail(self, message: str) -> RuntimeError:
        """Why the unit ends: LayoutAbandoned once RoboDojo threw the layout away, else retried."""
        if self._exited() is not None:
            try:
                outcome, reason, _ = self._outcome()
            except RoboDojoError as exc:
                return exc
            if outcome == ABANDONED:
                self.layout_outcome, self.abandon_reason = outcome, reason
                return LayoutAbandoned(f"RoboDojo abandoned this layout ({reason}): {message}")
        return RoboDojoError(message)

    def _outcome(self) -> tuple[str, str, dict]:
        """(layout_outcome, reason, the scored details entry) from RoboDojo's own files."""
        result, manifest = self._result(), self._resume()
        want = int((self.unit or {}).get("layout", -1))
        details = (result or {}).get("details") or {}
        entry = details.get("0", details.get(0))
        if isinstance(entry, dict) and "layout_id" in entry:
            got = int(entry["layout_id"])
            if got != want:
                raise RoboDojoError(f"RoboDojo scored layout {got}, not the unit's {want} "
                                    f"(run {self.run_id})")
            return SCORED, "", entry
        if want in {int(i) for i in (manifest or {}).get("abandoned_layout_ids") or []}:
            return ABANDONED, "abandoned_layout_id", {}
        if result is not None:
            return ABANDONED, "dropped_from_details", {}
        if manifest is None:
            return ABANDONED, "unstable_or_broken_at_reset", {}
        return "", "", {}

    def _accept(self, deadline: float) -> B.Bridge:
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise self._fail(f"RoboDojo did not connect to AMP's bridge within "
                                 f"{self.load_timeout_s:g} s (run {self.run_id})")
            try:
                return self._listener.accept(timeout_s=min(left, POLL_S),
                                             recv_timeout_s=self.step_timeout_s)
            except B.BridgeTimeout:
                rc = self._exited()
                if rc is not None:
                    raise self._fail(
                        f"RoboDojo exited (rc {rc}) before AMP's deploy.py connected: an unstable "
                        f"layout or a start-up failure (run {self.run_id})") from None

    def _await(self, deadline: float, what: str) -> dict:
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                raise self._fail(f"RoboDojo sent no {what} in time (run {self.run_id})")
            try:
                return self._bridge.recv(min(left, POLL_S))
            except B.BridgeTimeout:
                if self._exited() is not None:
                    raise self._fail(f"RoboDojo exited (rc {self._exited()}) before {what} "
                                     f"(run {self.run_id})") from None
            except B.BridgeClosed as exc:
                raise self._fail(f"the bridge closed before {what}: {exc} "
                                 f"(run {self.run_id})") from exc

    def _send(self, message: dict) -> None:
        if self._bridge is None:
            raise self._fail(f"no bridge to RoboDojo (run {self.run_id})")
        try:
            self._bridge.send(message)
        except B.BridgeClosed as exc:
            raise self._fail(f"the bridge closed before AMP could send "
                             f"{message['type']}: {exc} (run {self.run_id})") from exc

    def _take(self, message: dict) -> None:
        """Record what deploy.py sent: a frame, or the end of the episode."""
        from amp.robots.robodojo.frames import frame_from_obs

        if message["type"] == "end":
            self._ended = True
            return
        check_cameras(message["obs"])
        if self._observed == 0:
            self._instruction = instruction_of(message["obs"])
            if not self._instruction.strip():
                raise RoboDojoError(f"RoboDojo's first observation carries no instruction "
                                    f"(run {self.run_id}): the agent would have no goal")
        self._frame = frame_from_obs(message["obs"])
        self._observed += 1

    def _hold(self) -> None:
        """Tell deploy.py to hold until RoboDojo ends the episode (the publish case)."""
        try:
            self._send({"type": "hold"})
            self._take(self._await(time.monotonic() + self.hold_timeout_s, "the episode's end"))
        except RuntimeError as exc:
            self.log(f"hold failed, falling back to {RESULT_FILE}: {exc}", err=True)

    def _wait_exit(self) -> int | None:
        if self._proc is None:
            return None
        try:
            return self._proc.wait(timeout=self.exit_timeout_s)
        except subprocess.TimeoutExpired:
            self.log(f"RoboDojo did not exit within {self.exit_timeout_s:g} s; killing it",
                     err=True)
            _kill(self._proc, self._runner_pgid)
            return self._proc.poll()

    def _result(self) -> dict | None:
        out = self.outputs()
        if out is None:
            return None
        return _read_json(out / RESULT_FILE)

    def _resume(self) -> dict | None:
        """This run's resume manifest, which RoboDojo deletes when its eval loop ends normally."""
        if self.unit is None or not self.run_id or self._overlay is None:
            return None
        return _read_json(unit_dir(self._overlay, self.unit["task"], self.unit["seed"],
                                   self.additional_info) / f"_resume_{self.run_id}.json")


def copy_outputs(src: Path | None, task_dir: str, k: int) -> list[str]:
    """RoboDojo's per-episode files into the episode's task dir; the names it wrote."""
    if src is None or not Path(src).is_dir():
        return []
    out = []
    for path in sorted(Path(src).iterdir()):
        if not path.is_file():
            continue
        if path.name == RESULT_FILE:
            name = f"robodojo_result_{k}.json"
        elif path.suffix == ".mp4":
            name = f"{path.stem}_{k}.mp4"
        else:
            continue
        shutil.copyfile(path, os.path.join(task_dir, name))
        out.append(name)
    return out


class RoboDojoWorker(CapgymWorker):
    """CapgymWorker with RoboDojo's env_cfg, unit provenance and per-episode outputs."""

    def episode_provenance(self) -> dict:
        out = dict(self.provenance())
        unit = getattr(self.adapter, "unit", None)
        if isinstance(unit, dict):
            out["unit"] = {k: unit[k] for k in ("task", "seed", "layout", "step_lim")
                           if k in unit}
        run_id = getattr(self.adapter, "run_id", "")
        if run_id:
            out["run_id"] = run_id
        out["layout_outcome"] = getattr(self.adapter, "layout_outcome", "") or SCORED
        reason = getattr(self.adapter, "abandon_reason", "")
        if reason:
            out["abandon_reason"] = reason
        return out

    def write_env_cfg(self, task_dir: str, task: str, spec: dict) -> None:
        with open(os.path.join(task_dir, "env_cfg.json"), "w") as f:
            json.dump({"robodojo": True, "task": task, "instruction": spec["instruction"],
                       "policy": self.policy_label, "env_cfg_type": ENV_CFG_TYPE,
                       **self.provenance()}, f)

    def run_job(self, job: dict, task: str, k: int, run: str = "") -> dict:
        from amp.robots.capgym.client import capgym_record

        spec = self.tasks[task]
        root = R.run_dir(self.output_dir, run)
        results_file = os.path.join(root, R.RESULTS_FILE)
        task_dir = os.path.join(root, task)
        os.makedirs(task_dir, exist_ok=True)
        fresh = self.loaded != task
        if fresh:
            self.loaded = None
            self.adapter.load(task)
            self.loaded = task
        if fresh or not os.path.exists(os.path.join(task_dir, "env_cfg.json")):
            self.write_env_cfg(task_dir, task, spec)
        if R.find_record(R.load_records(results_file), task, k) and not job.get("replace"):
            self.log(f"{task} episode {k} has a record; treating job {job.get('job_id')} as a redo")
        moved = R.set_aside(task_dir, k)
        t0 = time.monotonic()
        abandoned: LayoutAbandoned | None = None
        try:
            self.adapter.reset(k)
            dt = float(self.adapter.control_dt())
            allowed = self.episode_steps(spec, dt)
            horizon = self.horizon(allowed)
            if horizon is not None:
                self.write_horizon(task_dir, horizon)
            instruction = self.adapter.instruction()
            spec = {**spec, "instruction": instruction}
            res = run_episode(self.link, self.adapter, task=task, instruction=instruction,
                              episode=k, max_steps=allowed, cameras=self.cameras,
                              job_id=job.get("job_id"), depth_stride=self.depth_stride,
                              jpeg_quality=self.jpeg_quality, log=self.log)
        except LayoutAbandoned as exc:
            abandoned, dt, horizon = exc, float(self.adapter.control_dt()), None
            res = {"success": False, "score": 0.0, "steps": 0, "reason": "abandoned",
                   "truncated": False}
            self.log(f"{task} episode {k}: {exc}")
        except BaseException:
            R.discard_attempt(task_dir, k, moved)
            raise
        copied = copy_outputs(getattr(self.adapter, "outputs", lambda: None)(), task_dir, k)
        record = capgym_record(spec, k, res, policy_label=self.policy_label, dt=dt,
                               wall_s=time.monotonic() - t0,
                               provenance=self.episode_provenance(), horizon=horizon)
        if "capx" in record:
            record["robodojo"] = record.pop("capx")
        log = {"schema_version": 2, "dt": dt, "task": task, "env_id": 0, "run": k,
               "success": record["success"], "final_step": record["episode_step"],
               "truncated": record["truncated"], "events": [], "robodojo_files": copied}
        with open(os.path.join(task_dir, f"log_{k}_env0.json"), "w") as f:
            json.dump(log, f)
        with R.results_lock(self.output_dir):
            with open(results_file, "a") as f:
                f.write(json.dumps(record) + "\n")
            R.keep_latest(results_file, task, k)
        if abandoned is not None:
            raise abandoned
        return record


def run_benchmark(link, adapter: RoboDojoAdapter, output_dir: str, *, episodes_per_task: int = 1,
                  policy_label: str = DEFAULT_POLICY_LABEL, retry_s: float = jobs.LINK_RETRY_S,
                  **worker_kw) -> jobs.Stats:
    """Benchmark mode: serve /sim/next jobs over RoboDojo's task set until the server says quit."""
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    for what in R.recover_all(output_dir):
        _log(f"restored {what}: its redo was interrupted before it saved")
    entries = [{k: t[k] for k in jobs.CATALOG_TASK_KEYS} for t in adapter.tasks()]
    worker = RoboDojoWorker(link, adapter, output_dir, tasks={t["name"]: t for t in entries},
                            policy_label=policy_label, adapter_name="robodojo", **worker_kw)
    catalog = jobs.Catalog(policy_label=policy_label, output_dir=output_dir,
                           episodes_per_task=episodes_per_task, tasks=entries)
    wire_idle(link, adapter)
    if adapter.link_keepalive is None:
        body = asdict(catalog)
        adapter.link_keepalive = lambda: link.send_catalog(**body)
    return jobs.serve(link, catalog, worker, retry_s=retry_s, log=_log)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m amp.robots.robodojo", description=__doc__)
    p.add_argument("--server", default=DEFAULT_URL)
    p.add_argument("--benchmark", action="store_true",
                   help="benchmark mode: serve /sim/next jobs over the task set until quit")
    p.add_argument("--output-dir", help="benchmark results folder (RoboLab layout)")
    p.add_argument("--robodojo-root", required=True,
                   help="the read-only RoboDojo checkout the overlay links to")
    p.add_argument("--xpl-python", required=True,
                   help="the interpreter for XPolicyLab's policy server")
    p.add_argument("--overlay-root", help="the slot's overlay root (default: beside --output-dir)")
    p.add_argument("--gpu", type=int, default=0)
    p.add_argument("--port", type=int, default=0, help=f"0 = ${PORT_ENV} or {DEFAULT_PORT}")
    p.add_argument("--episodes-per-task", type=int, default=1)
    p.add_argument("--policy-label", default=DEFAULT_POLICY_LABEL)
    p.add_argument("--load-timeout", type=float, default=LOAD_TIMEOUT_S)
    p.add_argument("--takeover", action="store_true", help="evict a live owner of the sim link")
    return p


def overlay_for(output_dir: str) -> Path:
    """The overlay root beside a slot's results dir (backend.overlay_root's default)."""
    return Path(os.path.abspath(output_dir)).parent / "robodojo_overlay"


def main(argv=None) -> int:
    sandbox.install()
    p = parser()
    args = p.parse_args(argv)
    if args.benchmark and not args.output_dir:
        p.error("--benchmark needs --output-dir")
    overlay_root = args.overlay_root or overlay_for(args.output_dir or os.getcwd())
    try:
        adapter = RoboDojoAdapter(robodojo_root=args.robodojo_root, xpl_python=args.xpl_python,
                                  overlay_root=overlay_root, gpu=args.gpu,
                                  port=args.port or None, load_timeout_s=args.load_timeout)
    except ValueError as exc:
        _log(str(exc), err=True)
        return 2
    stop_children_at_exit(close=adapter.close)
    try:
        link = LinkClient(args.server, client="robodojo", embodiment=adapter.embodiment(),
                          robot=adapter.robot_description())
        link.hello(takeover=args.takeover)
        if not args.benchmark:
            p.error("the RoboDojo launcher runs in benchmark mode only")
        stats = run_benchmark(link, adapter, args.output_dir,
                              episodes_per_task=args.episodes_per_task,
                              policy_label=args.policy_label)
        _log(f"benchmark: {stats.saved} saved, {stats.aborted} aborted, {stats.failed} failed")
    except KeyboardInterrupt:
        _log("interrupted, episode closed", err=True)
        return 130
    except LinkError as exc:
        _log(str(exc), err=True)
        return 1
    finally:
        adapter.close()
    return 0
