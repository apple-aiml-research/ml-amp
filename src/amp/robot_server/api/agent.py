#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Agent routes: the observation, waits, depth, kinematics, checks and perception tools."""

from __future__ import annotations

import math
import time

import numpy as np

from amp.geometry import kinematics as kin
from amp.robot_server import embodiment as rt
from amp.robot_server.api import Reply, binary, ok
from amp.robot_server.observations import npy_bytes

WAIT_DEFAULT_S = 30.0
WAIT_MAX_S = 120.0
WAIT_RECHECK_S = 1.0
CALIBRATION = ("K", "K_depth", "T_world_cam", "depth_width", "depth_height")


def _vec(x) -> list:
    return [float(v) for v in np.asarray(x, dtype=np.float64).reshape(-1)]


def ee_states(srv) -> dict | None:
    """Under the lock: each end effector's observed poses, gripper and q slice (None for one)."""
    ex = srv.executor
    if not ex.multi or srv.obs.obs is None:
        return None
    o, emb, lay = srv.obs, srv.embodiment, ex.layout
    obs = o.obs
    out = {}
    for e in lay.ees:
        pose = obs.ee.get(e)
        if pose is None and e == lay.primary:
            pose = (obs.ee_pos, obs.ee_quat)
        T_ee = None if pose is None else kin.make_pose(*pose)
        g = o.gripper_of(e)
        idx = lay.ee_q[e]
        out[e] = {"T_ee": T_ee, "T_tcp": None if T_ee is None else emb.tool(e).tcp_from_ee(T_ee),
                  "gripper_cmd": ex.gripper_cmds.get(e), "gripper_measured": g,
                  "q_slice": [int(idx[0]), int(idx[-1]) + 1]}
        if g is not None:
            out[e]["gripper_gap_m"] = float(emb.gripper(e).gap(g))
    return out


def wrist_cameras(emb) -> dict[str, str]:
    """{ee: name} of each "wrist" camera mounted on a link its ee's arm moves (or carries)."""
    spec, desc = emb.spec, emb.description
    parent = {} if desc is None else {j.child: j.parent for j in desc.joints}
    moved = {}
    for a in spec.arms:
        links = set(emb.arm(a.name).link_names[1:])
        if desc is not None:
            links |= {j.child for j in desc.joints if j.name in a.joints}
        moved[a.name] = links
    out: dict[str, str] = {}
    for cam in spec.cameras:
        if cam.role != "wrist" or cam.mount == "root":
            continue
        chain, link = [], cam.mount
        while link is not None and link not in chain:
            chain.append(link)
            link = parent.get(link)
        on = [e for e in spec.end_effectors if e.name not in out
              and (e.ee_link in chain or moved[e.arm] & set(chain))]
        best = next((e for e in on if e.ee_link in chain), on[0] if on else None)
        if best is not None:
            out[best.name] = cam.name
    return out


def _pose7(T):
    return None if T is None else _vec(kin.pose7(T))


def observation_json(srv) -> dict:
    """Under the lock: the agent's observation, built from a whitelist (never from api/state)."""
    ep, o, world, ex = srv.episode, srv.obs, srv.world, srv.executor
    obs = o.obs
    emb = srv.embodiment
    robot = None
    if obs is not None:
        lim = rt.limits(emb)
        T_ee = kin.make_pose(obs.ee_pos, obs.ee_quat)
        fingers = emb.gripper() is not None and srv.settings.measured_gripper
        robot = {"q": _vec(obs.q), "gripper_cmd": ex.gripper_cmd,
                 "gripper_measured": float(obs.gripper) if fingers else None,
                 "ee_pose": _vec(kin.pose7(T_ee)),
                 "tcp_pose": _vec(kin.pose7(o.tool.tcp_from_ee(T_ee))),
                 "tcp_offset": float(o.tool.tcp_offset), "q_min": _vec(lim["q_min"]),
                 "q_max": _vec(lim["q_max"]), "q_home": _vec(lim["q_home"])}
        if fingers:
            robot["gripper_gap_m"] = float(emb.gripper().gap(float(obs.gripper)))
        T_base = getattr(obs, "T_world_base", None)
        if emb.spec.base is not None and T_base is not None:
            robot["T_world_base"] = np.asarray(T_base, dtype=np.float64).tolist()
        per = ee_states(srv)
        if per is not None:
            wrist = wrist_cameras(emb)
            keep = srv.settings.measured_gripper
            robot["end_effectors"] = {
                e: {"ee_pose": _pose7(s["T_ee"]), "tcp_pose": _pose7(s["T_tcp"]),
                    "gripper_cmd": s["gripper_cmd"],
                    "gripper_measured": s["gripper_measured"] if keep else None,
                    **({"gripper_gap_m": s["gripper_gap_m"]}
                       if keep and "gripper_gap_m" in s else {}),
                    "q_slice": s["q_slice"], "wrist_camera": wrist.get(e)}
                for e, s in per.items()}
    cameras = {}
    for name, cam in o.camera_json().items():
        has = cam["depth_width"] is not None and obs.cameras[name].depth is not None
        cameras[name] = {**cam, **({} if has else {"depth_width": None, "depth_height": None,
                                                   "K_depth": None})}
    if srv.settings.hide_calibration:
        cameras = {name: {k: v for k, v in cam.items() if k not in CALIBRATION}
                   for name, cam in cameras.items()}
    ch = ex.chunk
    execution = None if ch is None else {"chunk_id": ch.chunk_id, "kind": ch.kind,
                                         "index": int(ch.index), "n_steps": int(ch.total)}
    last_chunk = None
    if ex.history:
        h = ex.history[-1]
        last_chunk = {"chunk_id": h["chunk_id"], "kind": h["kind"], "status": h["status"],
                      "n_steps": int(h["n_steps"]), "message": h.get("reason")}
        if "base" in h:
            last_chunk["base"] = h["base"]
        if "dropped" in h:
            last_chunk["dropped"] = h["dropped"]
    held = None
    if world.held is not None:
        held = {"object": world.held["object"], "reason": world.held.get("reason")}
        if ex.multi:
            held["ees"] = list(world.held_ees)
    seq, objects, notes = srv.perception.observation_fields()
    embodiment = rt.observation_json(emb)
    if ex.multi:
        wrist = wrist_cameras(emb)
        for e in embodiment["end_effectors"]:
            e["wrist_camera"] = wrist.get(e["name"])
    return {
        "ok": True, "info_mode": srv.settings.info_mode, "episode": ep.episode,
        "instruction": ep.agent_instruction(), "mode": ep.mode,
        "publishing": ep.publishing is not None, "step": int(ep.step),
        "max_steps": ep.max_steps, "steps_left": ep.steps_left(), "control_dt": ep.control_dt,
        "frame_seq": int(o.frame_seq), "robot": robot, "cameras": cameras,
        "table_z": float(srv.table_z()), "execution": execution, "last_chunk": last_chunk,
        "held": held, "plan_executing": srv.planning.executing(),
        "perception_seq": seq, "objects": objects, "perception_notes": notes,
        "embodiment": embodiment,
    }


def observation(srv, req) -> Reply:
    with srv.cond:
        return ok(observation_json(srv))


def wait(srv, req) -> Reply:
    """POST api/agent/wait: block until the robot is idle or a newer frame arrives."""
    body = req.body
    until = body.get("until", "idle")
    if until not in ("idle", "frame"):
        raise ValueError(f"until must be 'idle' or 'frame', got {until!r}")
    timeout = body.get("timeout_s", WAIT_DEFAULT_S)
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or timeout < 0):
        raise ValueError(f"timeout_s must be a number >= 0, got {timeout!r}")
    after = body.get("after_seq")
    if after is not None and (isinstance(after, bool) or not isinstance(after, int)):
        raise ValueError(f"after_seq must be an integer, got {after!r}")
    deadline = time.monotonic() + min(float(timeout), WAIT_MAX_S)
    with srv.cond:
        after = srv.obs.frame_seq if after is None else after
        while True:
            if until == "idle":
                done = srv.episode.mode != "executing" and not srv.planning.executing()
            else:
                done = srv.obs.frame_seq > after
            left = deadline - time.monotonic()
            if done or left <= 0 or srv.stopping.is_set():
                return ok({"ok": True, "timed_out": not done,
                           "observation": observation_json(srv)})
            srv.cond.wait(min(left, WAIT_RECHECK_S))


def depth(srv, req) -> Reply:
    with srv.cond:
        srv.observations_live()
        arr, seq = srv.obs.depth(req.params["cam"])
    return binary(npy_bytes(np.asarray(arr, dtype=np.float32)), "application/x-npy",
                  **{"X-Frame-Seq": str(seq)})


def object_points(srv, req) -> Reply:
    arr, oid, resolved = srv.perception.object_points(req.params["name"])
    headers = {"X-Object-Id": oid}
    if resolved is not None:
        headers.update({"X-Resolved-From": resolved["from"], "X-Resolved-Why": resolved["why"]})
    return binary(npy_bytes(arr), "application/x-npy", **headers)


def _planning(method):
    def handler(srv, req) -> Reply:
        return ok(getattr(srv.planning, method)(req.body))
    handler.__name__ = method
    return handler


def _perception(method):
    def handler(srv, req) -> Reply:
        return ok(getattr(srv.perception, method)(req.body))
    handler.__name__ = method
    return handler


robot, fk, ik, ik_batch = (_planning(m) for m in ("robot", "fk", "ik", "ik_batch"))
check, check_batch = _planning("check"), _planning("check_batch")
segment, define_object = _perception("segment"), _perception("define_object")
forget_object, reperceive = _perception("forget_object"), _perception("reperceive")
