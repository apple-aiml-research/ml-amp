#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The capgym robot as a link client: benchmark mode over a SimAdapter's task library."""

from __future__ import annotations

import argparse
import importlib
import itertools
import json
import os
import sys
import time
import traceback
from collections.abc import Callable

import numpy as np

from amp.contracts.embodiment import EmbodimentSpec
from amp.contracts.harness import CAMERA_SETS, INFO_MODES
from amp.contracts.robot import ACTION_DIM
from amp.geometry.cameras import CAMERA_RIGS, WRIST_CAMERA
from amp.link.client import DEFAULT_URL, EpisodeAborted, LinkClient, LinkError
from amp.link.obs import env_error_logged, pack_observation
from amp.link.pointcloud import CameraModel
from amp.robots import jobs
from amp.robots.capgym import affinity, boundary, object_poses, sandbox
from amp.robots.capgym.adapter import (
    SUCCESS_CHECKS,
    TASK_SETS,
    AdapterFactory,
    EnvError,
    Frame,
    SimAdapter,
)
from amp.robots.capgym.families import family
from amp.robots.robolab import records as R

DEFAULT_POLICY_LABEL = "AMP Agent"
ADAPTERS = ("official",)
DEPTH_STRIDE = 2
STEP_LOG_ENV = "AMP_CAPGYM_STEP_LOG"
ENV_ERROR_FRAMES = 5
ENV_ERROR_MESSAGE = 500

def _log(message: str, *, err: bool = False) -> None:
    print(f"[amp.robots.capgym] {message}", file=sys.stderr if err else sys.stdout, flush=True)


def rig(cameras: str, spec: EmbodimentSpec | None = None) -> dict[str, bool]:
    """camera name -> carries depth, for every camera a frame must carry: the declared ones, or
    for DROID the camera set's rig plus the wrist."""
    if spec is not None:
        return {c.name: c.depth for c in spec.cameras}
    return dict.fromkeys([*CAMERA_RIGS[cameras], WRIST_CAMERA], True)


def pack_frame(frame: Frame, *, episode: int, step: int, control_dt: float, max_steps: int | None,
               instruction: str, task: str, cameras: dict[str, bool], dof: int = 7,
               depth_stride: int = DEPTH_STRIDE, jpeg_quality: int = 85,
               spec: EmbodimentSpec | None = None, env_error: str | None = None):
    """(header, blobs, encodings) of an adapter Frame; ValueError if it is not the declared robot."""
    if set(frame.cameras) != set(cameras):
        raise ValueError(f"adapter rendered cameras {sorted(frame.cameras)}, "
                         f"the rig is {sorted(cameras)}")
    q = np.asarray(frame.q, dtype=np.float64).reshape(-1)
    if q.shape != (dof,):
        raise ValueError(f"q has {q.size} values, the embodiment {dof}")
    if spec is None:
        want = set(frame.ee)
    else:
        want = {e.name for e in spec.end_effectors if e.gripper is not None}
    if not frame.ee or set(frame.grippers) != want:
        raise ValueError(f"grippers {sorted(frame.grippers)} must name the end effectors with a "
                         f"gripper, {sorted(want)}")
    cams = {}
    for name, c in frame.cameras.items():
        h, w = np.asarray(c.rgb).shape[:2]
        if c.depth is None and cameras[name]:
            raise ValueError(f"camera {name}: no depth, but it is declared with depth")
        if c.depth is not None and np.asarray(c.depth).shape[:2] != (h, w):
            raise ValueError(f"camera {name}: depth {c.depth.shape} must have the rgb's "
                             f"(H, W) = {(h, w)}")
        depth = c.depth if cameras[name] else None
        cams[name] = (CameraModel(name, w, h, c.K, c.T_root_cam), c.rgb, depth)
    if len(frame.ee) == 1 and frame.grippers:
        (name, (pos, quat)), = frame.ee.items()
        proprio = {"gripper": frame.grippers[name], "ee_pos": pos, "ee_quat": quat}
    else:
        proprio = {"ee": frame.ee, "grippers": frame.grippers}
    if getattr(frame, "T_world_base", None) is not None:
        proprio["T_world_base"] = frame.T_world_base
    return pack_observation(episode=episode, step=step, control_dt=control_dt,
                            max_steps=max_steps, instruction=instruction, task=task, q=q,
                            cameras=cams, depth_stride=depth_stride, jpeg_quality=jpeg_quality,
                            body_contacts=getattr(frame, "body_contacts", None),
                            contacts_unavailable=getattr(frame, "contacts_unavailable", False),
                            env_error=env_error, **proprio)


def env_error(exc: BaseException) -> tuple[str, str]:
    """(the agent's text: type, message, innermost frames; the record's "<type>: <message> @ <frame>")."""
    frames = [f"{os.path.basename(f.filename)}:{f.lineno} in {f.name}"
              for f in traceback.extract_tb(exc.__traceback__)][-ENV_ERROR_FRAMES:]
    head = f"{type(exc).__name__}: {str(exc)[:ENV_ERROR_MESSAGE]}"
    where = frames[-1] if frames else "unknown"
    text = f"{head} (traceback, innermost last: {'; '.join(frames)})" if frames else head
    return text, f"{head} @ {where}"


def run_episode(
    link,
    adapter: SimAdapter,
    *,
    task: str,
    instruction: str,
    episode: int = 0,
    max_steps: int | None = 300,
    cameras: str = "droid",
    job_id: int | None = None,
    depth_stride: int = DEPTH_STRIDE,
    jpeg_quality: int = 85,
    log: Callable[..., None] = _log,
) -> dict:
    """Drive one episode of the loaded, reset task over the link; {success, score, steps, reason,
    truncated, env_errors if the env raised, cells, verdict_from and verdict_requested if graded}. max_steps None:
    no step horizon, so the agent or AMP's wall clock ends it."""

    dt = float(adapter.control_dt())
    spec = adapter.embodiment()
    names = rig(cameras, spec)
    dof, action_dim = (7, ACTION_DIM) if spec is None else (spec.dof, spec.action_dim)
    checks = adapter.success_checks()
    if checks not in SUCCESS_CHECKS:
        raise ValueError(f"adapter success_checks {checks!r} is not one of {SUCCESS_CHECKS}")
    at_end = checks == "end"
    extra = {} if job_id is None else {"job_id": job_id}
    link.begin_episode(episode, task=task, instruction=instruction, max_steps=max_steps,
                       control_dt=dt, **extra)
    success, score, steps, reason, failed, aborted = False, 0.0, 0, "reset", True, False
    truncated = False


    errors, pending, streak = {"count": 0, "last": None}, None, 0

    finish_at = None

    cells: dict = {}
    verdict = None
    requested = None

    def grade(cell):
        refusal = getattr(adapter, "grade_refusal", None)
        why = refusal() if callable(refusal) else None
        if why:
            return {"error": why}
        ok, sc = adapter.success()
        cells[cell] = (bool(ok), float(sc), steps)
        return {"step": steps}

    def published():
        if verdict is None:
            return adapter.success()
        return cells.get(verdict.get("cell"), (False, 0.0, None))[:2]

    if hasattr(link, "on_grade"):
        link.on_grade = grade
    log_path = os.environ.get(STEP_LOG_ENV)
    steplog = open(log_path, "a") if log_path else None  # noqa: SIM115
    try:
        for step in range(max_steps) if max_steps is not None else itertools.count():
            t0 = time.perf_counter()
            frame = adapter.observe()
            t1 = time.perf_counter()
            header, blobs, encodings = pack_frame(
                frame, episode=episode, step=step, control_dt=dt,
                max_steps=max_steps, instruction=instruction, task=task, cameras=names, dof=dof,
                depth_stride=depth_stride,
                jpeg_quality=jpeg_quality, spec=spec, env_error=pending)
            pending = None
            t2 = time.perf_counter()
            action = link.step(header, blobs, encodings)
            t3 = time.perf_counter()
            action = np.asarray(action, dtype=np.float64).reshape(-1)
            if action.shape != (action_dim,) or not np.all(np.isfinite(action)):
                raise ValueError(f"action must be {action_dim} finite numbers, got {action}")
            follow = getattr(adapter, "follow_select", None)
            if callable(follow) and getattr(link, "annotated", False):
                follow(link.select_arm)
            try:
                adapter.apply(action)
                streak = 0
            except EnvError as exc:
                pending, errors["last"] = env_error(exc.cause)
                errors["count"] += 1
                streak += 1
                if env_error_logged(streak):
                    again = f" ({streak} in a row)" if streak > 1 else ""
                    log(f"ERROR env_error at step {step}{again}, the action failed and the "
                        f"episode goes on: {pending}", err=True)
            steps += 1
            if steplog is not None:
                steplog.write(json.dumps({
                    "episode": episode, "step": step, "observe_s": t1 - t0, "pack_s": t2 - t1,
                    "link_s": t3 - t2, "apply_s": time.perf_counter() - t3,
                    "q": np.asarray(frame.q).tolist(), "action": action.tolist()}) + "\n")
            ended = getattr(adapter, "ended", None)
            if callable(ended) and ended():
                (success, score), reason = adapter.success(), "terminated"
                break
            finish = getattr(link, "finish", None)
            if finish is not None:
                if isinstance(finish.get("verdict"), dict):
                    verdict = dict(finish["verdict"])
                if isinstance(finish.get("verdict_requested"), dict):
                    requested = dict(finish["verdict_requested"])
                if at_end or verdict is not None:
                    (success, score), reason = published(), "published"
                    break
                if finish_at is None:
                    finish_at = max(step + 1, 3)
            if finish_at is not None and steps >= finish_at:
                (success, score), reason = published(), "published"
                break
            if not at_end:
                success, score = adapter.success()
                if success:
                    reason = "terminated"
                    break
        else:

            success, score = adapter.success()
            truncated = True
        failed = False
    except EpisodeAborted:
        reason, aborted = "aborted", True
        raise
    except KeyboardInterrupt:
        reason = "closed"
        raise
    finally:
        if hasattr(link, "on_grade"):
            link.on_grade = None
        if steplog is not None:
            steplog.close()

        result = None if aborted or failed else {"success": bool(success), "score": float(score),
                                                 "steps": steps}
        try:
            link.end_episode(episode, reason=reason, result=result)
        except Exception as exc:
            if not failed:
                raise
            log(f"end_episode failed: {exc}", err=True)
    out = {**result, "reason": reason, "truncated": truncated}
    if errors["count"]:
        out["env_errors"] = dict(errors)
    if cells or verdict is not None:
        out["cells"] = [{"cell": c, "success": ok, "score": sc, "step": st}
                        for c, (ok, sc, st) in sorted(cells.items())]
        out["verdict_from"] = "end" if verdict is None else {"cell": verdict.get("cell")}
        if requested is not None:
            out["verdict_requested"] = requested
    return out


def capgym_record(spec: dict, k: int, res: dict, *, policy_label: str, dt: float,
                  wall_s: float = 0.0, provenance: dict | None = None,
                  horizon: dict | None = None, capped: bool = False) -> dict:
    """An episode_results.jsonl record: RoboLab's keys, the adapter's "capx", and the horizon."""
    steps = int(res["steps"])
    extra = {"capx": dict(provenance)} if provenance else {}
    if "cells" in res:
        extra.setdefault("capx", {}).update(cells=[dict(c) for c in res["cells"]],
                                            verdict_from=res["verdict_from"])
        if "verdict_requested" in res:
            extra["capx"]["verdict_requested"] = dict(res["verdict_requested"])
    if horizon is not None:
        extra.update(horizon=dict(horizon),
                     partial=horizon["steps_allowed"] < horizon["official_steps"])
    elif capped:
        extra["partial"] = bool(res["truncated"])
    return {
        "env_name": spec["name"],
        "task_name": spec["name"],
        "run_name": f"{spec['name']}_{k}",
        "run": k,
        "episode": k,
        "env_id": 0,
        "policy": policy_label,
        "instruction": spec["instruction"],
        "attributes": list(spec.get("attributes") or []),
        "success": bool(res["success"]),
        "episode_step": steps,
        "duration": steps * dt if steps else 0,
        "dt": dt,
        "metrics": {},
        "events": {},
        "instruction_type": "default",
        "timing": {"wall_total_s": round(wall_s, 3)},
        "score": float(res["score"]),
        "reason": "success" if res["success"] else "not yet",
        "end_reason": res.get("reason"),
        "truncated": bool(res.get("truncated", False)),
        **extra,
    }


def features(adapter) -> tuple[str, ...]:
    """The hello features the adapter supports: "grade" when its success() steps no physics."""
    return tuple(getattr(adapter, "features", ()))


def wire_idle(link, adapter) -> None:
    """Have the link call adapter.idle() at every wait reply (the server idles: nothing in flight)."""
    if getattr(link, "on_wait", False) is None:
        link.on_wait = getattr(adapter, "idle", None)


OFFICIAL_MAX_ATTEMPTS = 3


def official_attempt(job: dict) -> int | None:
    """The attempt k of an official CaP-Gym job, None for any other: only official mode sends
    official_attempt >= 1."""
    a = job.get("official_attempt")
    if a is None or a == 0:
        return None
    if not isinstance(a, int) or isinstance(a, bool) or not 1 <= a <= OFFICIAL_MAX_ATTEMPTS:
        raise ValueError(f"job {job.get('job_id')}: official_attempt {a!r} is not an int in "
                         f"1..{OFFICIAL_MAX_ATTEMPTS}")
    return a


class CapgymWorker:
    """Runs capgym jobs on one SimAdapter into a RoboLab-shaped results folder."""

    def __init__(self, link, adapter: SimAdapter, output_dir: str, *, tasks: dict[str, dict],
                 policy_label: str, adapter_name: str = "", max_steps: int | None = None,
                 cameras: str = "droid",
                 depth_stride: int = DEPTH_STRIDE, jpeg_quality: int = 85,
                 restart: Callable[[dict], None] | None = None,
                 log: Callable[..., None] = _log):
        self.link, self.adapter, self.output_dir, self.tasks = link, adapter, output_dir, tasks
        self.policy_label, self.adapter_name, self.max_steps = policy_label, adapter_name, max_steps
        self.cameras = cameras
        self.depth_stride, self.jpeg_quality, self.log = depth_stride, jpeg_quality, log


        self.restart = restart
        self.loaded: str | None = None
        self._provenance: dict | None = None

    def provenance(self) -> dict:
        """What the adapter simulates (its CaP-X / LIBERO-PRO commits), if it says; {} if not."""
        if self._provenance is None:
            fn = getattr(self.adapter, "provenance", None)
            try:
                self._provenance = dict(fn()) if callable(fn) else {}
            except Exception as exc:  # noqa: BLE001
                self.log(f"adapter.provenance() failed: {exc!r}", err=True)
                self._provenance = {}
        return self._provenance

    def stop_reason(self) -> str | None:
        """Why this backend must take no more jobs (the adapter disabled its task set), else None."""
        return getattr(self.adapter, "disabled", None)

    def unload(self) -> None:
        self.loaded = None
        try:
            self.adapter.close()
        except Exception as exc:  # noqa: BLE001
            self.log(f"adapter.close() failed: {exc!r}", err=True)


        self.loaded = getattr(self.adapter, "loaded_config", None)

    def episode_steps(self, spec: dict, dt: float) -> int | None:
        """The adapter's own horizon (--max-steps may only shorten it), else --max-steps, else the
        task's episode_length_s in control steps; None when the task has no step horizon."""
        horizon = self.adapter.horizon()
        if horizon is not None:
            return min(int(horizon), int(self.max_steps)) if self.max_steps else int(horizon)
        if self.max_steps:
            return int(self.max_steps)
        if spec.get("episode_length_s") is None:
            return None
        return max(1, round(float(spec["episode_length_s"]) / dt))

    def horizon(self, steps_allowed: int | None) -> dict | None:
        """The reset episode's official horizon beside the one it runs; None without one."""
        official = self.adapter.horizon()
        if official is None:
            return None
        unit = getattr(self.adapter, "unit", None)
        return {"official_max_steps": (unit or {}).get("max_steps"),
                "official_steps": int(official), "max_steps_limit": self.max_steps or None,
                "steps_allowed": None if steps_allowed is None else int(steps_allowed)}

    def episode_provenance(self) -> dict:
        """provenance(), the unit the adapter ran and its reset scene's fingerprint, when it says."""
        out = dict(self.provenance())
        unit = getattr(self.adapter, "unit", None)
        if isinstance(unit, dict):




            out["unit"] = {k: unit[k] for k in ("trial", "seed", "init_state", "suite", "task_id",
                                                "max_steps", "source", "instruction_source",
                                                "bddl_file", "early_termination",
                                                "process_trial_index", "body_contacts",
                                                "reset_hold_check", "reset_index",
                                                "reset_schedule")
                           if k in unit}
        if isinstance(unit, dict) and isinstance(unit.get("contact_lost"), dict):
            out["contact_lost"] = dict(unit["contact_lost"])
        pin = affinity.record()
        if pin is not None:
            out["cpu_affinity"] = pin
        sim_steps = getattr(self.adapter, "sim_steps", None)
        if callable(sim_steps):
            out["sim_steps"] = int(sim_steps())
        fingerprint = getattr(self.adapter, "fingerprint", None)
        if callable(fingerprint):
            try:
                fp = fingerprint()
            except Exception as exc:  # noqa: BLE001
                self.log(f"adapter.fingerprint() failed: {exc!r}", err=True)
                fp = None
            if fp:
                out["fingerprint"] = fp
        if self.adapter_name == "official":

            out["boundary_v"] = boundary.SCHEMA
            out["boundary"] = b = boundary.record()
            if b["ok"] is None:
                self.log(f"ERROR CaP-Gym boundary not checked: {b['error']}", err=True)
            elif not b["ok"]:
                n = b.get("violations_total", len(b["violations"]))
                self.log(f"ERROR CaP-Gym boundary violated, {n} violation(s): "
                         f"{'; '.join(b['violations'])}", err=True)
        return out

    def write_state(self, task_dir: str, k: int) -> None:
        """run_<k>_state.json: the adapter's object-pose snapshots, when it takes them. In the
        results folder only: no frame, route or sandbox bind reaches it."""
        state = getattr(self.adapter, "state_record", None)
        rec = state() if callable(state) else None
        if rec:
            with open(os.path.join(task_dir, object_poses.STATE_FILE.format(k=k)), "w") as f:
                json.dump(rec, f)

    @staticmethod
    def write_horizon(task_dir: str, horizon: dict) -> None:
        """env_cfg.json carries the official max_steps and the --max-steps limit beside it."""
        path = os.path.join(task_dir, "env_cfg.json")
        cfg = {}
        if os.path.exists(path):
            with open(path) as f:
                cfg = json.load(f)
        want = {"official_max_steps": horizon["official_max_steps"],
                "max_steps_limit": horizon["max_steps_limit"],
                "partial": horizon["steps_allowed"] < horizon["official_steps"]}
        if any(cfg.get(key) != v for key, v in want.items()):
            with open(path, "w") as f:
                json.dump({**cfg, **want}, f)

    def run_job(self, job: dict, task: str, k: int, run: str = "") -> dict:
        try:
            return self._run_job(job, task, k, run)
        except BaseException:
            self.close_boundary_window(task, k)
            raise

    def close_boundary_window(self, task: str, k: int) -> None:
        """End the boundary window of a unit that has no row, so the next unit starts clean."""
        if self.adapter_name != "official":
            return
        try:
            b = boundary.record()
        except Exception as exc:  # noqa: BLE001
            self.log(f"boundary record of {task} episode {k} failed: {exc!r}", err=True)
            return
        if b["ok"] is False:
            n = b.get("violations_total", len(b["violations"]))
            self.log(f"ERROR CaP-Gym boundary violated in {task} episode {k}, which ended "
                     f"without a row, {n} violation(s): {'; '.join(b['violations'])}", err=True)

    def _run_job(self, job: dict, task: str, k: int, run: str = "") -> dict:
        spec = self.tasks[task]
        root = R.run_dir(self.output_dir, run)
        results_file = os.path.join(root, R.RESULTS_FILE)
        task_dir = os.path.join(root, task)
        os.makedirs(task_dir, exist_ok=True)
        need = getattr(self.adapter, "needs_fresh_process", None)
        if callable(need) and need(task) and self.restart is not None:
            self.log(f"{task} needs a fresh process (loaded: {self.loaded}); restarting for it")
            self.restart(job)
            raise RuntimeError("the restart hook returned")
        fresh = self.loaded != task
        if fresh:
            self.loaded = None
            self.adapter.load(task)
            self.loaded = task
        elif getattr(self.adapter, "loaded_config", None) is not None and self.adapter.task != task:
            self.adapter.load(task)
        if fresh or not os.path.exists(os.path.join(task_dir, "env_cfg.json")):
            with open(os.path.join(task_dir, "env_cfg.json"), "w") as f:
                json.dump({"capgym": True, "adapter": self.adapter_name, "task": task,
                           "instruction": spec["instruction"], "policy": self.policy_label,
                           **self.provenance()}, f)
        if R.find_record(R.load_records(results_file), task, k) and not job.get("replace"):
            self.log(f"{task} episode {k} has a record; treating job {job.get('job_id')} as a redo")
        moved = R.set_aside(task_dir, k)
        t0 = time.monotonic()
        try:
            if hasattr(self.adapter, "attempt"):
                self.adapter.attempt = official_attempt(job) or 1
            self.adapter.reset(k)
            dt = float(self.adapter.control_dt())
            allowed = self.episode_steps(spec, dt)
            horizon = self.horizon(allowed)
            if horizon is not None:
                self.write_horizon(task_dir, horizon)
            res = run_episode(self.link, self.adapter, task=task, instruction=spec["instruction"],
                              episode=k, max_steps=allowed,
                              cameras=self.cameras, job_id=job.get("job_id"),
                              depth_stride=self.depth_stride,
                              jpeg_quality=self.jpeg_quality, log=self.log)
        except BaseException:
            R.discard_attempt(task_dir, k, moved)
            if (getattr(self.adapter, "loaded_config", None) is None
                    and getattr(self.adapter, "task", task) != task):
                self.loaded = None
            raise
        self.write_state(task_dir, k)
        prov = self.episode_provenance()
        if res.get("env_errors"):
            prov["env_errors"] = res["env_errors"]
        record = capgym_record(spec, k, res, policy_label=self.policy_label, dt=dt,
                               wall_s=time.monotonic() - t0, provenance=prov,
                               horizon=horizon, capped=bool(self.max_steps))
        log = {"schema_version": 2, "dt": dt, "task": task, "env_id": 0, "run": k,
               "success": record["success"], "final_step": record["episode_step"],
               "truncated": record["truncated"], "events": []}
        with open(os.path.join(task_dir, f"log_{k}_env0.json"), "w") as f:
            json.dump(log, f)
        with R.results_lock(self.output_dir):
            with open(results_file, "a") as f:
                f.write(json.dumps(record) + "\n")
            R.keep_latest(results_file, task, k)
        if res.get("env_errors"):

            self.log(f"{task} episode {k} had {res['env_errors']['count']} env error(s); unloading")
            self.unload()
        return record


def run_benchmark(link, adapter: SimAdapter, output_dir: str, *, episodes_per_task: int = 1,
                  policy_label: str = DEFAULT_POLICY_LABEL, retry_s: float = jobs.LINK_RETRY_S,
                  resume: dict | None = None, **worker_kw) -> jobs.Stats:
    """Benchmark mode: serve /sim/next jobs over the adapter's tasks until the server says quit."""
    output_dir = os.path.abspath(output_dir)
    os.makedirs(output_dir, exist_ok=True)
    for what in R.recover_all(output_dir):
        _log(f"restored {what}: its redo was interrupted before it saved")
    entries = [{k: t[k] for k in jobs.CATALOG_TASK_KEYS} for t in adapter.tasks()]
    worker = CapgymWorker(link, adapter, output_dir, tasks={t["name"]: t for t in entries},
                          policy_label=policy_label, **worker_kw)
    catalog = jobs.Catalog(policy_label=policy_label, output_dir=output_dir,
                           episodes_per_task=episodes_per_task, tasks=entries)
    return jobs.serve(link, catalog, worker, retry_s=retry_s, log=_log, resume=resume)


EXIT_DISABLED = 3
RESUME_ENV = "AMP_CAPGYM_RESUME_JOB"
CLIENT_ENV = "AMP_CAPGYM_CLIENT_ID"


def restart_process(job: dict, client_id: str, *, execve=os.execve) -> None:
    """Replace this process by a fresh one that resumes `job` as the same link client, so the
    server sees one backend (and no lost job) while the new process starts with an empty stage
    (BEHAVIOR cannot load a second config). Same pid, so the slot runner sees no exit;
    the fresh dirs are wiped first, as an exit would."""
    sandbox.wipe_fresh()
    env = {**os.environ, RESUME_ENV: json.dumps(job), CLIENT_ENV: client_id}
    argv = [sys.executable, *sys.orig_argv[1:]]
    sys.stdout.flush()
    sys.stderr.flush()
    execve(sys.executable, argv, env)


class AdapterUnavailable(RuntimeError):
    """The requested adapter cannot be built in this Python."""


def make_adapter(name: str, *, task_set: str, cameras: str, seed: int | None) -> SimAdapter:
    """The SimAdapter called `name`; the official one imports its family's module only then."""
    if name != "official":
        raise AdapterUnavailable(f"unknown adapter {name!r}; known: {', '.join(ADAPTERS)}")
    try:
        fam = family(task_set)
    except ValueError as exc:
        raise AdapterUnavailable(str(exc)) from exc
    try:
        mod = importlib.import_module(fam.module)
    except ImportError as exc:
        if getattr(exc, "name", None) == fam.module:
            raise AdapterUnavailable(f"--adapter official needs {fam.module}, which this "
                                     "AMP checkout does not have") from exc
        raise AdapterUnavailable(f"--adapter official: {fam.module} failed to import "
                                 f"({exc}); run it with the CaP-X {fam.name.upper()} python "
                                 f"({sys.executable} is this one)") from exc
    factory: AdapterFactory = mod.make_adapter
    return factory(task_set=task_set, cameras=cameras, seed=seed)


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m amp.robots.capgym", description=__doc__)
    p.add_argument("--server", default=DEFAULT_URL)
    p.add_argument("--adapter", choices=ADAPTERS, default="official")
    p.add_argument("--benchmark", action="store_true",
                   help="benchmark mode (the only one): serve /sim/next jobs until quit")
    p.add_argument("--output-dir", help="benchmark results folder (RoboLab layout)")
    p.add_argument("--episodes-per-task", type=int, default=1)
    p.add_argument("--task-set", choices=TASK_SETS, default=TASK_SETS[0],
                   help="the one task library this process registers")
    p.add_argument("--policy-label", default=DEFAULT_POLICY_LABEL)
    p.add_argument("--cameras", choices=CAMERA_SETS, default="droid",
                   help="DROID adapters only: a declared embodiment names its own cameras")
    p.add_argument("--info", choices=INFO_MODES, default="standard",
                   help="standard sends no ground truth")
    p.add_argument("--max-steps", type=int, default=0,
                   help="steps per episode; 0 = the adapter's horizon, else episode_length_s / "
                        "control_dt, else none (the agent or the wall clock ends it); an adapter "
                        "with its own horizon only takes a shorter one")
    p.add_argument("--seed", type=int, default=0, help="< 0 = the adapter's nominal layout")
    return p


def main(argv=None) -> int:
    sandbox.install()
    p = parser()
    args = p.parse_args(argv)
    if args.adapter == "official":
        boundary.bind(family(args.task_set).name)
    if not args.benchmark or not args.output_dir:
        p.error("give --benchmark and --output-dir")
    try:
        adapter = make_adapter(args.adapter, task_set=args.task_set, cameras=args.cameras,
                               seed=None if args.seed < 0 else args.seed)
    except AdapterUnavailable as exc:
        _log(str(exc), err=True)
        return 2
    spec = adapter.embodiment()
    kw: dict = {"max_steps": args.max_steps or None, "cameras": args.cameras}
    resume = os.environ.pop(RESUME_ENV, None)
    client_id = os.environ.pop(CLIENT_ENV, None)
    try:
        robot = adapter.robot_description() if spec is not None else None
        link = LinkClient(args.server, client="capgym", embodiment=spec, robot=robot,
                          features=features(adapter),
                          **({"client_id": client_id} if client_id else {}))
        wire_idle(link, adapter)
        link.hello()
        if hasattr(adapter, "needs_fresh_process"):
            kw["restart"] = lambda job: restart_process(job, link.client_id)
        stats = run_benchmark(link, adapter, args.output_dir,
                              episodes_per_task=args.episodes_per_task,
                              policy_label=args.policy_label, adapter_name=args.adapter,
                              resume=json.loads(resume) if resume else None, **kw)
        _log(f"benchmark: {stats.saved} saved, {stats.aborted} aborted, "
             f"{stats.failed} failed")
    except jobs.WorkerStopped as exc:
        _log(f"stopped: {exc}", err=True)
        return EXIT_DISABLED
    except KeyboardInterrupt:
        _log("interrupted, episode closed", err=True)
        return 130
    except LinkError as exc:
        _log(str(exc), err=True)
        return 1
    finally:
        try:
            adapter.close()
        except Exception as exc:  # noqa: BLE001
            _log(f"adapter.close() failed: {exc!r}", err=True)
        if sandbox.EXP_PATH_ENV in os.environ:
            sandbox.wipe_fresh()
    return 0
