#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Lifecycle routes: the catalog, directed select, api/state, release, publish, saved runs."""

from __future__ import annotations

import time

from amp.contracts.eval import CAPX_MAX_TRIAL_ATTEMPTS, INSTRUCTION_TYPES
from amp.contracts.harness import INFO_MODES, OBSTACLE_MODES
from amp.geometry import kinematics as kin
from amp.link.wire import jsonable
from amp.robot_server import config as config_mod
from amp.robot_server import embodiment as rt
from amp.robot_server.api import Reply, ok
from amp.robot_server.api.agent import ee_states
from amp.robot_server.episode import SELECT_MODES
from amp.robot_server.errors import NotFound, SessionConflict
from amp.robot_server.observations import tolist
from amp.robot_server.results import check_run_id


def tasks(srv, req) -> Reply:
    with srv.cond:
        srv.episode.need_catalog()
        return ok(srv.results.tasks_json(srv.episode.running_task()))


def select(srv, req) -> Reply:
    """POST api/tasks/select: queue episode k of a task for the backend's next job."""
    body = req.body
    task, mode = body.get("task"), body.get("mode", "new")
    if mode not in SELECT_MODES:
        raise ValueError(f"mode must be 'new', 'redo', 'resume' or 'quit', got {mode!r}")
    if mode in ("redo", "resume"):
        raise ValueError(f"mode {mode!r} is not supported by the robot server; use 'new'")
    abandon = body.get("abandon", False)
    if not isinstance(abandon, bool):
        raise ValueError("abandon must be true or false")  # noqa: TRY004
    with srv.cond:
        cat = srv.episode.need_catalog()
    if mode != "quit" and cat.task(task) is None:
        raise ValueError(f"unknown task {task!r}")
    want = body.get("episode") if mode == "new" else None
    if want is not None and (isinstance(want, bool) or not isinstance(want, int) or want < 0):
        raise ValueError(f"episode must be an integer >= 0, got {want!r}")
    run = check_run_id(body.get("run_id"))
    info, obstacles = body.get("info_mode"), body.get("obstacles")
    if info is not None:
        if info not in INFO_MODES:
            raise ValueError(f"info_mode must be one of {list(INFO_MODES)}, got {info!r}")
        if info != srv.settings.info_mode:
            raise SessionConflict(f"this server runs info mode {srv.settings.info_mode!r}, "
                                  f"not {info!r}")
    if obstacles is not None and obstacles not in OBSTACLE_MODES:
        raise ValueError(f"obstacles must be one of {list(OBSTACLE_MODES)}, got {obstacles!r}")
    itype = body.get("instruction_type") or "default"
    if itype not in INSTRUCTION_TYPES:
        raise ValueError(f"instruction_type must be one of {list(INSTRUCTION_TYPES)}, "
                         f"got {itype!r}")
    attempt = body.get("official_attempt")
    if attempt is not None and (isinstance(attempt, bool) or not isinstance(attempt, int)
                                or not 1 <= attempt <= CAPX_MAX_TRIAL_ATTEMPTS):
        raise ValueError(f"official_attempt must be an integer in 1..{CAPX_MAX_TRIAL_ATTEMPTS}, "
                         f"got {attempt!r}")
    job, message = srv.episode.select(task, mode, abandon, want, obstacles=obstacles, run=run,
                                      instruction_type=itype, official_attempt=attempt or 0)
    srv.traces.for_header(req.headers.get("x-amp-trace"), make_current=True)
    srv.trace_episode("select", job=job.get("job_id"), task=task, episode=job.get("episode"))
    srv.log("info", message)
    reply = {"ok": True, "job_id": job["job_id"], "message": message}
    if job.get("instruction_type"):
        reply["instruction_type"] = job["instruction_type"]
    return ok(reply)


def release(srv, req) -> Reply:
    discard, force = req.body.get("discard"), req.body.get("force")
    if discard is not None and not isinstance(discard, bool):
        raise ValueError("discard must be true or false")
    if force is not None and not isinstance(force, bool):
        raise ValueError("force must be true or false")
    if force and discard is False:
        raise ValueError("force discards the episode: it cannot come with discard false")
    return ok(srv.episode.release(force=bool(force)))


def _cell(v) -> bool:
    return v is None or (isinstance(v, int) and not isinstance(v, bool) and v >= 0)


def publish(srv, req) -> Reply:
    """POST api/episode/publish {"verdict"?: {"cell": n | null}}: score by that cell's grade."""
    verdict = (req.body or {}).get("verdict")
    if verdict is not None and (not isinstance(verdict, dict) or set(verdict) != {"cell"}
                                or not _cell(verdict["cell"])):
        raise ValueError(f'verdict must be {{"cell": n}} with n >= 0 or null, got {verdict!r}')
    return ok(srv.episode.publish(verdict))


def grade(srv, req) -> Reply:
    """POST api/episode/grade {"cell": n}: the backend grades the state as cell n's end."""
    cell = (req.body or {}).get("cell")
    if cell is None or not _cell(cell):
        raise ValueError(f"cell must be an integer >= 0, got {cell!r}")
    return ok(srv.episode.grade(cell))


def runs(srv, req) -> Reply:
    """GET api/runs/<task>/<k> (unscoped) and api/runs/<run>/<task>/<k>."""
    k = req.params["k"]
    if not k.isdigit():
        raise NotFound("expected /api/runs/<task>/<episode>")
    try:
        run = check_run_id(req.params.get("run"))
    except ValueError as exc:
        raise NotFound(str(exc)) from None
    with srv.cond:
        srv.episode.need_catalog()
        out = srv.results.run_json(req.params["task"], int(k), run)
    if out is None:
        raise NotFound(f"unknown task {req.params['task']!r}")
    return ok(out)


def state(srv, req) -> Reply:
    """GET api/state: OLD's state JSON, whose STATE_FIELDS the gateway reads."""
    with srv.cond:
        return ok(state_json(srv))


def _ee_state(srv, ee, s, q) -> dict:
    """One end effector in api/state's robot block."""
    pos = quat = tcp_pos = tcp_quat = None
    if s["T_ee"] is not None:
        pos, quat = kin.pose_parts(s["T_ee"])
        tcp_pos, tcp_quat = kin.pose_parts(s["T_tcp"])
    qa = [float(v) for v in q][s["q_slice"][0]:s["q_slice"][1]]
    return {"ee_pos": tolist(pos), "ee_quat": tolist(quat), "tcp_pos": tolist(tcp_pos),
            "tcp_quat": tolist(tcp_quat), "gripper_cmd": s["gripper_cmd"],
            "gripper_measured": s["gripper_measured"], "gripper_gap_m": s.get("gripper_gap_m"),
            "links": tolist(srv.embodiment.tool(ee).link_points(qa)), "q_slice": s["q_slice"]}


def state_json(srv) -> dict:
    """Under the lock: the full state document."""
    ep, obs_s, world, ex = srv.episode, srv.obs, srv.world, srv.executor
    obs = obs_s.obs
    robot, gt = None, None
    if obs is not None:
        tcp_pos, tcp_quat = kin.pose_parts(obs_s.observed_tcp())
        fingers = srv.embodiment.gripper() is not None
        robot = {"q": tolist(obs.q), "gripper_measured": float(obs.gripper) if fingers else None,
                 "gripper_cmd": ex.gripper_cmd, "ee_pos": tolist(obs.ee_pos),
                 "ee_quat": tolist(obs.ee_quat), "tcp_pos": tolist(tcp_pos),
                 "tcp_quat": tolist(tcp_quat),
                 "links": tolist(obs_s.tool.link_points(obs_s.arm_q(obs.q)))}
        per = ee_states(srv)
        if per is not None:
            robot["end_effectors"] = {e: _ee_state(srv, e, s, obs.q) for e, s in per.items()}
        gt = jsonable(obs.gt)
    ch = ex.chunk
    execution = None
    if ch is not None:
        execution = {"chunk_id": ch.chunk_id, "kind": ch.kind, "index": ch.index,
                     "total": ch.total, "planned_tcp": list(ch.planned_tcp),
                     "actual_tcp": list(ch.actual_tcp), "started_step": ch.start_step}
    s = srv.settings
    lim = rt.limits(srv.embodiment)
    task, episode = ep.on_json()
    me = {"worker": s.worker, "url": s.public_url, "task": task, "episode": episode,
          "mode": ep.mode, "viewers": 0, "age_s": 0.0, "stale": False, "self": True}
    return {
        "server_time": time.time(), "mode": ep.mode, "connected": srv.link.connected(),
        "episode": ep.episode, "task": ep.task, "instruction": ep.instruction,
        "step": ep.step, "sim_time": ep.sim_time, "max_steps": ep.max_steps,
        "control_dt": ep.control_dt, "frame_seq": obs_s.frame_seq,
        "sim_client": srv.link.client_json(), "robot": robot, "tool": obs_s.tool_json(),
        "cameras": obs_s.camera_json(), "execution": execution,
        "history": [dict(h) for h in ex.history], "gt": gt, "info_mode": s.info_mode,
        "perception": srv.perception.state_json(), "result": ep.result,
        "last_result": ep.last_result, "publishing": ep.publishing is not None,
        "limits": {"q_min": tolist(lim["q_min"]), "q_max": tolist(lim["q_max"]),
                   "qd_max": tolist(lim["qd_max"]), "table_z": obs_s.table_z},
        "config": {"default_duration": config_mod.DEFAULT_DURATION_S,
                   "speed_scale": config_mod.SPEED_SCALE,
                   "poll_timeout_s": config_mod.POLL_TIMEOUT_S, "settle_s": config_mod.SETTLE_S},
        "scene": world.scene_json(), "scene_error": world.error,
        "assist": {"busy": srv.planning.busy or srv.planning.replanning,
                   "held": world.held_json(), "plan": srv.planning.state_json(),
                   "planner": srv.planning.planner_json()},
        "benchmark": ep.benchmark_json(),
        "worker": {"name": s.worker, "url": s.public_url},
        "workers": [me] if srv.results.root is not None else None,
        "link": srv.link.liveness(),
        **({"motion": srv.motion()} if srv.link.grades() else {}),
    }
