#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A mobile manipulator's own motions: base moves checked against perceived occupancy, and joint
moves of an arm or a joint group (a torso), the torso's checked against the scene."""

from __future__ import annotations

import dataclasses
import math
import re

import numpy as np

from amp.geometry import kinematics as kin
from amp.geometry.collision.peers import BODY
from amp.geometry.embodiment.description import mesh_arrays, parent_joints
from amp.perception.cloud import fuse_frames, remove_robot, robot_spheres, transform
from amp.perception.occupancy import SELF_HIDDEN_NEAR_M, OccupancyGrid, self_hidden
from amp.planning import checks, trajectory
from amp.planning.multi import PEER_MARGIN_M, PEER_START_TOL_M, SELF_KIND
from amp.robot_server.executor import (
    BASE_SETTLE_MARGIN_STEPS,
    BASE_TRACK_SHARE_FLOOR,
    BaseTrack,
    base_goal,
    planar,
)

FOOTPRINT_MARGIN_M = 0.03
OCCUPANCY_MAX_DEPTH_M = 5.0
ROBOT_INFLATE_M = 0.03
BASE_MAX_M = 3.0
BASE_SETTLE_STEPS = 6
TORSO_STEP_M = 0.01
BASE_BLOCKED = "base_blocked"
MEMORY_STRIDE = 2
SELF_PROBE_Z_M = 0.01
NEVER_SEEN = "unknown cells ahead were never seen"
TURN_UNSEEN = "unknown cells the turn sweeps were never seen"
NO_KNOWN_LOOK = "no camera pose is known to reveal these cells; try another route"
PREFIX_MIN = (0.01, math.radians(1.0))
LOOK_AHEAD = {"capx_b1k_r1pro": {"left": (-0.35, 0.10, 0, 0, 0, 0, 0),
                                 "right": (-0.35, -0.10, 0, 0, 0, 0, 0)}}
TURN_LOOK = {"capx_b1k_r1pro": {"group": "torso", "joint": 3, "value": {+1: 0.8}}}
LOOK_AHEAD_DEPTH_M = 0.50
LOOK_DONE_TOL = 0.10
BASE_SHADOW_M = 0.5


def _num(x, name: str) -> float:
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
        raise ValueError(f"{name} must be a finite number, got {x!r}")
    return float(x)


def parse_delta(body) -> np.ndarray:
    """(dx m, dy m, dyaw rad) of a motion/base body, in the base's own frame."""
    if not isinstance(body, dict):
        raise ValueError("motion/base takes {dx, dy, dyaw}")  # noqa: TRY004
    d = np.array([_num(body.get(k, 0.0), k) for k in ("dx", "dy", "dyaw")])
    if np.abs(d[:2]).max() > BASE_MAX_M or abs(d[2]) > math.pi:
        raise ValueError(f"a base move is |dx|, |dy| <= {BASE_MAX_M} m and |dyaw| <= pi, got "
                         f"{d.tolist()}")
    if not np.any(d):
        raise ValueError("a base move needs a non-zero dx, dy or dyaw")
    return d


def footprint(emb) -> np.ndarray:
    """(4, 2) the base's footprint box in its own frame, grown by FOOTPRINT_MARGIN_M."""
    desc, blobs = emb.description, emb.blobs
    pts = []
    if desc is not None and blobs is not None:
        up = parent_joints(desc)
        poses = emb.link_poses(np.zeros(emb.dof))
        for static in (True, False):
            for lk in desc.links:
                if not lk.meshes or (static and not _fixed_to_root(up, lk.name, desc.root)):
                    continue
                T = poses[lk.name]
                for m in lk.meshes:
                    V = mesh_arrays(blobs, m)[0].astype(np.float64)
                    pts.append(V @ T[:3, :3].T + T[:3, 3])
            if pts:
                break
    if not pts:
        raise ValueError(f"{emb.name}: no meshes to take a base footprint from")
    P = np.vstack(pts)[:, :2]
    lo, hi = P.min(axis=0) - FOOTPRINT_MARGIN_M, P.max(axis=0) + FOOTPRINT_MARGIN_M
    return np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])


def _fixed_to_root(up, link: str, root: str) -> bool:
    while link != root:
        j = up[link]
        if j.type != "fixed":
            return False
        link = j.parent
    return True


def perceived_points(emb, frames, q, T_flange_ee, T_world_base, stride: int = 1) -> np.ndarray:
    """(N, 3) world points of the depth frames, the robot's own removed."""
    xyz, _, _ = fuse_frames(frames, crop=False, max_depth=OCCUPANCY_MAX_DEPTH_M, embodiment=emb,
                            stride=stride)
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz):
        xyz = xyz[remove_robot(xyz, robot_spheres(q, T_flange_ee, embodiment=emb),
                               ROBOT_INFLATE_M)]
    return transform(xyz, T_world_base)


def _grid(emb, frames, q, T_flange_ee, T_world_base, stride=1):
    pts = perceived_points(emb, frames, q, T_flange_ee, T_world_base, stride)
    return OccupancyGrid.from_points(pts, float(np.asarray(T_world_base)[2, 3])), len(pts)


def remember(emb, frames, q, T_flange_ee, T_world_base, memory, stride=MEMORY_STRIDE, *,
             token=None, stamp=None) -> bool:
    """Add these frames (of frame stamp) to episode token's base memory; whether it took them."""
    grid, _ = _grid(emb, frames, q, T_flange_ee, T_world_base, stride)
    return memory.observe(grid, np.asarray(T_world_base, dtype=np.float64)[:2, 3], token=token,
                          stamp=stamp)


def _body_view(emb, frames, q, T_world_base):
    """(cameras, spheres, link names) in the root frame: what self_hidden needs of this robot."""
    cams = [(fr.depth_model.T_world_cam, fr.depth_model.K, fr.depth) for fr in frames.values()
            if fr.depth is not None and fr.depth_model is not None]
    C, R, names = emb.whole_spheres(q)
    return cams, (C, R + ROBOT_INFLATE_M), names


def _to_root(xy, T_world_base, floor_z):
    T = np.asarray(T_world_base, dtype=np.float64)
    P = np.c_[xy, np.full(len(xy), floor_z + SELF_PROBE_Z_M)]
    return (P - T[:3, 3]) @ T[:3, :3]


def _fill_self_hidden(grid, emb, view, fp, T_world_base, floor_z):
    """The grid with unknown cells near the footprint only the body hides taken as free."""
    cams, spheres, _ = view
    res, (ox, oy) = grid.resolution, grid.origin_xy
    lo, hi = fp.min(axis=0), fp.max(axis=0)
    T = np.asarray(T_world_base, dtype=np.float64)
    ii, jj = np.nonzero(~grid.known)
    xy = np.c_[ox + (jj + 0.5) * res, oy + (ii + 0.5) * res]
    local = (xy - T[:2, 3]) @ T[:2, :2]
    near = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    cand = np.linalg.norm(near, axis=1) <= SELF_HIDDEN_NEAR_M
    if not cand.any() or not cams:
        return grid, 0
    hidden = self_hidden(_to_root(xy[cand], T, floor_z), cams, spheres) >= 0
    known = grid.known.copy()
    known[ii[cand][hidden], jj[cand][hidden]] = True
    return OccupancyGrid(grid.origin_xy, res, grid.occupied, known), int(hidden.sum())


def _look_hint(emb, q, turn: str | None = None) -> str | None:
    """The look-ahead call for emb unless its arms already hold that pose, else None."""
    look = LOOK_AHEAD.get(getattr(emb, "name", None))
    if look is None:
        return None
    if q is not None:
        held = emb.split(q)
        if all(np.abs(np.asarray(held[a]) - v).max() < LOOK_DONE_TOL for a, v in look.items()):
            return None
    calls = " and ".join(f'ep.move_joints("{a}", [{", ".join(f"{x:g}" for x in v)}])'
                         for a, v in look.items())
    if turn:
        return ("the floor the turn sweeps is out of the cameras' view: swing both arms forward "
                f"to look ({calls}), then retry the {turn}")
    return ("the floor just ahead is out of the cameras' view: swing both arms forward to look "
            f"({calls}), then retry the move")


def _turn_look_hint(emb, q, sign: int, retry: str) -> str | None:
    """The measured torso turn look for a turn of that sign, once the forward look is held and
    until the torso is turned that way (so at most once), its call exactly as checked; else None."""
    look = TURN_LOOK.get(getattr(emb, "name", None))
    if look is None or q is None or not sign:
        return None
    held = emb.split(q)
    ahead = LOOK_AHEAD.get(emb.name)
    if ahead is not None and not all(np.abs(np.asarray(held[a]) - v).max() < LOOK_DONE_TOL
                                     for a, v in ahead.items()):
        return None
    if sign not in look["value"]:
        return None
    group, j, value = look["group"], look["joint"], float(look["value"][sign])
    now = np.asarray(held[group], dtype=np.float64)
    if abs(now[j] - value) < LOOK_DONE_TOL:
        return None
    want = now.copy()
    want[j] = value
    text = [repr(float(v)) for v in want]
    spec = emb.joint_groups[group]
    printed = np.array([float(t) for t in text])
    if not np.array_equal(printed, want) or (printed < np.asarray(spec.q_min)).any() \
            or (printed > np.asarray(spec.q_max)).any():
        return None
    return (f'turn the torso toward the turn to look (ep.move_joints("{group}", '
            f"[{', '.join(text)}])), then retry the {retry}")


def _is_torso(emb, link) -> bool:
    """Whether link is one a joint group of emb carries (a torso link, the head), not the base."""
    groups = getattr(emb, "joint_groups", None) or {}
    return any(link in emb.moved_by(g) for g in groups)


def _toward_zero(v: float, digits: int = 3) -> tuple[float, str]:
    """v cut toward zero to `digits` significant digits, and the text that parses back to it."""
    if abs(v) < 1e-9:
        return 0.0, "0"
    places = max(0, digits - 1 - math.floor(math.log10(abs(v))))
    text = f"{math.trunc(round(v * 10**places, 6)) / 10**places:.{places}f}"
    while abs(float(text)) > abs(v):
        text = f"{(abs(float(text)) - 10**-places) * math.copysign(1.0, v):.{places}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return float(text), text


def _printed(d) -> tuple[np.ndarray, str]:
    """(the move as its call will be read back, that call): every component no longer than d's."""
    vals, args = [], []
    for k, v in zip(("dx", "dy", "dyaw"), d):
        x, text = _toward_zero(float(v))
        vals.append(x)
        if x:
            args.append(f"{k}={text}")
    return np.array(vals), f"ep.move_base({', '.join(args)})"


def _clear_part(grid, fp, T, delta, why):
    """(the longest leading part of the move the check accepts, as printed, its call), or None:
    the printed values are the ones checked, so the call an agent copies is the accepted move."""
    m = re.search(r"sample (\d+)/(\d+)", why)
    if m is None:
        return None
    k, n = int(m.group(1)), int(m.group(2))
    d0 = np.asarray(delta, dtype=np.float64)
    for kk in range(k - 1, 0, -1):
        d, call = _printed(d0 * kk / n)
        if math.hypot(d[0], d[1]) < PREFIX_MIN[0] and abs(d[2]) < PREFIX_MIN[1]:
            return None
        if grid.swept_free(fp, T, tuple(d))[0]:
            return d, call
    return None


def _onward(grid, fp, T, delta, why) -> str:
    """What a refused move can do next: its clear first part as checked, else NO_KNOWN_LOOK."""
    if (part := _clear_part(grid, fp, T, delta, why)) is not None:
        return f"the first part of the move is clear ({part[1]}); {NO_KNOWN_LOOK} past it"
    return NO_KNOWN_LOOK


def _unknown_hint(grid, why, emb, view, T_world_base, floor_z, fp, q=None, delta=(0, 0, 0),
                  turn=None) -> str:
    """Why the unknown cell a refusal names is unknown, and what can be done that is not refused."""
    T = np.asarray(T_world_base, dtype=np.float64)
    head = TURN_UNSEEN if turn else NEVER_SEEN

    def onward():
        if turn:
            sign = int(np.sign(np.asarray(delta, dtype=np.float64)[2]))
            if (look := _turn_look_hint(emb, q, sign, turn)) is not None:
                return look
        return _onward(grid, fp, T, delta, why)

    m = re.search(r"cell \((-?\d+), (-?\d+)\)", why)
    cams = view[0] if view is not None else []
    if m is None or "off the grid" in why or not cams:
        return f"{head}; {onward()}"
    i, j = int(m.group(1)), int(m.group(2))
    cell = np.array([[grid.origin_xy[0] + (j + 0.5) * grid.resolution,
                      grid.origin_xy[1] + (i + 0.5) * grid.resolution]])
    _, spheres, names = view
    who = int(self_hidden(_to_root(cell, T, floor_z), cams, spheres)[0])
    x, y = ((cell[0] - T[:2, 3]) @ T[:2, :2]).tolist()
    lo, hi = fp.min(axis=0), fp.max(axis=0)
    within = abs(y) <= hi[1] + SELF_HIDDEN_NEAR_M
    span = abs(y) <= hi[1] + LOOK_AHEAD_DEPTH_M if turn else within
    ahead = span and hi[0] < x <= hi[0] + LOOK_AHEAD_DEPTH_M
    behind = within and x < lo[0]
    part = None if who < 0 else "arms" if names[who] in emb.arm_links() else "torso or base"
    if behind and part == "torso or base" and x >= lo[0] - BASE_SHADOW_M \
            and not _is_torso(emb, names[who]):
        return ("unknown cells behind are hidden by the base itself (no camera sees the strip "
                f"right behind it); {onward()}")
    if who < 0:
        look = _look_hint(emb, q, turn) if ahead else None
        return look or f"{head}; {onward()}"
    what = "the turn sweeps" if turn else "ahead"
    return (f"unknown cells {what} are hidden by the robot's own {part} (beyond "
            f"{SELF_HIDDEN_NEAR_M * 100:.0f} cm of the footprint); {onward()}")


def check_base(emb, frames, q, T_flange_ee, T_world_base, delta, memory=None, *, token=None,
               stamp=None):
    """(free, reason, info) of a base move against these frames' occupancy and the memory."""
    Tb = np.asarray(T_world_base, dtype=np.float64)
    floor_z = float(Tb[2, 3])
    grid, n = _grid(emb, frames, q, T_flange_ee, Tb)
    remembered = 0
    if memory is not None:
        memory.observe(grid, Tb[:2, 3], token=token, stamp=stamp)
        merged = memory.merge(grid, Tb[:2, 3], token=token)
        remembered = int(merged.known.sum() - grid.known.sum())
        grid = merged
    fp = footprint(emb)
    view, filled = None, 0
    if emb.whole_body:
        view = _body_view(emb, frames, q, Tb)
        grid, filled = _fill_self_hidden(grid, emb, view, fp, Tb, floor_z)
    ok, why = grid.swept_free(fp, Tb, tuple(delta))
    if not ok and why.startswith("unknown"):
        d = np.asarray(delta, dtype=np.float64)
        rot = bool(d[2]) and (not np.any(d[:2]) or grid.swept_free(fp, Tb, (d[0], d[1], 0.0))[0])
        turn = ("turn" if not np.any(d[:2]) else "move") if rot else None
        why = f"{why}; {_unknown_hint(grid, why, emb, view, Tb, floor_z, fp, q, d, turn)}"
    return bool(ok), why, {"footprint": fp.tolist(), "floor_z": floor_z, "points": n,
                           "remembered_cells": remembered, "self_hidden_cells": filled}


def base_step_max(spec, dt: float, speed_scale: float) -> np.ndarray:
    """A step's largest (dx, dy, dyaw): qd_max scaled, within the base drive's step_clamp."""
    base = spec.base
    lim = np.asarray(base.qd_max, dtype=np.float64) * float(dt) * float(speed_scale)
    gd = next((g for g in spec.drive.groups if g.target == base.name), None)
    if gd is not None and gd.step_clamp:
        lim = np.minimum(lim, np.asarray(gd.step_clamp, dtype=np.float64))
    return lim


def base_track_for(spec, T_world_base, delta, dt, speed_scale):
    """(BaseTrack, planned steps) of a base move: ceil(largest step ratio) commanded steps and
    BASE_SETTLE_STEPS more; the track then holds until settled for at most `max_extra` steps, the
    commanded steps scaled by what a base tracking at BASE_TRACK_SHARE_FLOOR still owes, plus
    BASE_SETTLE_MARGIN_STEPS."""
    step = base_step_max(spec, dt, speed_scale)
    n_cmd = math.ceil(float(np.max(np.abs(delta) / step)))
    max_extra = math.ceil(n_cmd * (1.0 / BASE_TRACK_SHARE_FLOOR - 1.0)) + BASE_SETTLE_MARGIN_STEPS
    return (BaseTrack(base_goal(T_world_base, delta), step, max_extra=max_extra),
            n_cmd + BASE_SETTLE_STEPS)


def base_plan(c, ctx, lay, spec, delta, speed_scale, embodiment, issue=None, T_world_base=None):
    """A composed base-move result: arms, torso and grippers held, the BaseTrack from T_world_base
    (the frame the swept check used; else the context's)."""
    dt = c["dt"]
    T0 = np.asarray(ctx["T_world_base"] if T_world_base is None else T_world_base,
                    dtype=np.float64)
    track, n = base_track_for(spec, T0, delta, dt, speed_scale)
    res = trajectory.plan_hold(c["hold_q"], c["gripper"], c["tool"], n * dt, dt,
                               embodiment=embodiment)
    hold = np.asarray(ctx["hold_q"], dtype=np.float64)
    q_full = np.tile(hold, (len(res.actions), 1))
    goal = track.T_goal
    extra = {**res.extra, "base": {"delta": [float(v) for v in delta],
                                   "T_world_base": T0.tolist(),
                                   "T_world_goal": goal.tolist(), "goal": list(planar(goal)),
                                   "step_max": track.step_max.tolist()}}
    issues = list(res.issues) + ([issue] if issue is not None else [])
    res = dataclasses.replace(
        res, kind="base", ok=res.ok and issue is None, issues=issues, q=q_full,
        start_q=hold.copy(), extra=extra,
        actions=lay.compose_rows(q_full, {e: float(ctx["grippers"].get(e, 0)) for e in lay.ees}))
    res.composed = True
    res.base_track = track
    return res


def blocked(why: str) -> trajectory.Issue:
    return trajectory.Issue(0, 0.0, BASE_BLOCKED, "error", why or "the swept footprint is blocked")


def parse_joints(plan_json, emb, lay) -> tuple[str, str, np.ndarray, float, float | None]:
    """(kind "arm" | "group", its name, q, duration, speed_scale) of a {"joints": ...} plan."""
    j = plan_json.get("joints")
    if not isinstance(j, dict) or not isinstance(j.get("group"), str):
        raise ValueError('a joint plan is {"joints": {"group": <arm or joint group>, "q": [...]},'  # noqa: TRY004
                         ' "duration": s}')
    name = j["group"]
    if name in emb.end_effectors:
        name = emb.ee(name).arm
    if name in lay.arm_q:
        kind, chain = "arm", next(a for a in emb.spec.arms if a.name == name)
    elif name in lay.group_q:
        kind, chain = "group", emb.joint_groups[name]
    else:
        raise ValueError(f"group must be one of {[*lay.arm_q, *lay.group_q]}, got {name!r}")
    q = j.get("q")
    n = len(chain.joints)
    if not isinstance(q, (list, tuple)) or len(q) != n:
        raise ValueError(f"joints.q must be {n} positions for {name}, got {q!r}")
    q = np.array([_num(v, f"joints.q[{i}]") for i, v in enumerate(q)])
    lo, hi = np.asarray(chain.q_min), np.asarray(chain.q_max)
    bad = np.flatnonzero((q < lo) | (q > hi))
    if len(bad):
        i = int(bad[0])
        raise ValueError(f"joints.q[{i}] = {q[i]:.4f} is outside {name}'s limits "
                         f"[{lo[i]:.4f}, {hi[i]:.4f}]")
    d = _num(plan_json.get("duration", 2.0), "duration")
    speed = plan_json.get("speed_scale")
    if speed is not None:
        speed = _num(speed, "speed_scale")
        if not 0.0 < speed <= 1.0:
            raise ValueError(f"speed_scale must be in (0, 1], got {speed}")
    return kind, name, q, d, speed


def _minjerk(q0, q1, n: int) -> np.ndarray:
    tau = (np.arange(n) + 1) / n
    s = tau**3 * (10 - 15 * tau + 6 * tau * tau)
    return q0 + s[:, None] * (q1 - q0)


def group_plan(srv, ctx, lay, name, q_goal, duration, dt, speed_scale, settle_s, primary,
               world_of) -> trajectory.PlanResult:
    """A joint group (torso) moving from the observed q to q_goal, every arm held: composed rows,
    the primary TCP it carries along, and the arms' contacts along it as errors."""
    emb = srv.embodiment
    g = emb.joint_groups[name]
    idx = lay.group_q[name]
    hold = np.asarray(ctx["hold_q"], dtype=np.float64).copy()
    hold[idx] = np.asarray(ctx["q"], dtype=np.float64)[idx]
    n_move = max(1, round(trajectory._check_duration(duration) / dt))
    n_settle = trajectory.settle_step_count(settle_s, dt)
    path = _minjerk(hold[idx], np.asarray(q_goal, dtype=np.float64), n_move)
    path = np.vstack([path, np.repeat(path[-1:], n_settle, axis=0)])
    n = len(path)
    t = (np.arange(n) + 1) * dt
    q_full = np.tile(hold, (n, 1))
    q_full[:, idx] = path
    arm_idx = lay.ee_q[primary]
    q_arm = hold[arm_idx]
    clone = emb.with_groups({name: hold[idx]})
    tool = clone.tool(primary)
    tcp, links = np.empty((n, 7)), []
    for k in range(n):
        clone.set_group_q(name, path[k])
        tcp[k] = kin.pose7(tool.fk_tcp(q_arm))
        links.append(tool.link_points(q_arm))
    clone.set_group_q(name, hold[idx])
    grip = float(ctx["grippers"].get(primary, 0))
    res = trajectory.finish("joints", dt, t, np.tile(q_arm, (n, 1)), tcp, np.full(n, grip), tool,
                            q_arm, grip, check_table=False)
    issues = list(res.issues)
    qd = np.abs(np.diff(np.vstack([hold[idx], path]), axis=0)) / dt
    ratio = qd / (np.asarray(g.qd_max) * speed_scale)
    worst = float(ratio.max(initial=0.0))
    min_duration = n_move * dt * worst
    if worst > 1.0:
        k = int(np.argmax(ratio.max(axis=1)))
        issues.append(trajectory.Issue(k, float(t[k]), "velocity", "error",
                                       f"{name} moves {worst:.2f}x its speed limit (x "
                                       f"{speed_scale:g}); min_duration {min_duration:.2f} s"))
    issues += group_contacts(srv, ctx, lay, name, path, t, world_of)
    issues.sort(key=lambda i: (i.step, i.severity != "error"))
    res = dataclasses.replace(
        res, kind="joints", ok=not any(i.severity == "error" for i in issues), issues=issues,
        q=q_full, start_q=hold.copy(), tcp_fk=tcp, links=np.asarray(links),
        start_tcp=tcp[0] if n else res.start_tcp, min_duration=max(min_duration, 0.0),
        settle_steps=n_settle,
        actions=lay.compose_rows(q_full, {e: float(ctx["grippers"].get(e, 0)) for e in lay.ees}),
        extra={**res.extra, "joints": {"group": name, "q": [float(v) for v in q_goal]}})
    res.composed = True
    res.ee = primary
    return res


def group_contacts(srv, ctx, lay, name, path, t, world_of) -> list:
    """Error issues of a group's path for what it carries, on the server's observed start."""
    q_hold = np.asarray(ctx["hold_q"], dtype=np.float64).copy()
    q_hold[lay.group_q[name]] = np.asarray(ctx["q"], dtype=np.float64)[lay.group_q[name]]
    return group_path_issues(srv.embodiment, name, q_hold, path, t, world_of)


def held_local(w) -> np.ndarray:
    """(n, 3) the points of the object w's arm holds, in its TCP frame; empty when none."""
    cw = w.world
    if cw._att_pts is None:
        return np.empty((0, 3))
    return np.vstack([cw._att_pts, cw._att_hull])


def _upper_points(clone, name, ees, qa, moved, held=None) -> np.ndarray:
    """Centres of every sphere group `name` carries, and the points its hands hold (held_local)."""
    pts = [clone.spheres(e.arm).world_batch(qa[e.arm][None])[0] for e in ees]
    for e in ees:
        P = (held or {}).get(e.name)
        if P is not None and len(P):
            T = clone.tool(e.name).fk_tcp(qa[e.arm])
            pts.append(P @ T[:3, :3].T + T[:3, 3])
    if moved:
        C, _, names = clone.body_world()
        pts.append(C[np.isin(np.asarray(names), list(moved))])
    return np.vstack(pts) if pts else np.empty((0, 3))


def _checked_steps(clone, name, ees, qa, moved, path, held=None) -> list[int]:
    """The steps no carried sphere or held point moves more than TORSO_STEP_M between, the last."""
    keep, last = [], None
    for k in range(len(path)):
        clone.set_group_q(name, path[k])
        P = _upper_points(clone, name, ees, qa, moved, held)
        if last is None or k == len(path) - 1 or np.abs(P - last).max() > TORSO_STEP_M:
            keep.append(k)
            last = P
    return keep


def _scene_contacts(worlds, C, R, names, moving) -> dict:
    """{(link, obstacle): clearance} of body spheres touching the scene's meshes, moving skipped."""
    out: dict = {}
    for w in worlds:
        clr, which = w.world.sphere_clearance(C, R, ignore=moving)
        for i in np.flatnonzero(clr < 0):
            key = (names[i], w.world.obstacle_names[int(which[i])])
            out[key] = min(float(clr[i]), out.get(key, 0.0))
    return out


def _body_part_contacts(w, q) -> dict:
    """{arm link: {body link: clearance}} of the links within clearance of each body part alone."""
    peers = w.world.peers
    out: dict = {}
    for part in peers.body_parts():
        w.world.set_peers(peers.only_body_part(part))
        for (link, ob), clr in checks.pair_contacts(w, q).items():
            if ob == BODY:
                out.setdefault(link, {})[part] = clr
    w.world.set_peers(peers)
    return out


def group_path_issues(emb, name, q_hold, path, t, world_of) -> list:
    """Errors of joint group `name` moving along path with every arm held at q_hold: its carried
    arms (and a held object) against the scene and the body, its own links against the scene,
    and body links it moves against the rest; a start contact may persist, per pair, not deepen."""
    g = emb.joint_groups[name]
    ees = [e for e in emb.spec.end_effectors if e.arm in g.carries]
    if not len(path):
        return []
    qa = emb.split(q_hold)
    q0 = np.asarray(qa[name], dtype=np.float64)
    clone = emb.with_groups({name: q0})
    body = emb.whole_body
    moved = clone.moved_by(name) if body else set()
    worlds = {e.name: world_of(clone, e.name, qa[e.arm]) for e in emb.spec.end_effectors}
    held = {e.name: held_local(worlds[e.name]) for e in ees}
    moving = {worlds[e.name].world.attached_name for e in ees} - {None}
    carried = {e.name: (worlds[e.name].world.attached_name, clone.tool(e.name).fk_tcp(qa[e.arm]))
               for e in ees if worlds[e.name].world.attached_name is not None}
    steps = _checked_steps(clone, name, ees, qa, moved, path, held)
    start, parted, seen, out, shifted, touching = {}, set(), set(), [], {}, set()

    def hit(k, key, clr, kind, message):
        if k < 0:
            start[key] = min(clr, start.get(key, clr))
            return
        live = key in start and key not in parted
        if live and clr >= start[key] - PEER_START_TOL_M:
            return
        if live:
            message += f", {(start[key] - clr) * 1000:.0f} mm deeper than at the start"
        if key not in seen:
            seen.add(key)
            out.append(trajectory.Issue(int(k), float(t[k]), kind, "error", message))

    for k in [-1, *steps]:
        clone.set_group_q(name, q0 if k < 0 else path[k])
        at = "the start" if k < 0 else f"step {k}"
        now = set()
        for e in ees:
            w = worlds[e.name]
            if k >= 0:
                w.world.set_peers(clone.view(e.name).peer_spheres)
                for f, (obj, T0) in carried.items():
                    if f != e.name and obj != w.world.attached_name:
                        T = clone.tool(f).fk_tcp(qa[emb.ee(f).arm]) @ np.linalg.inv(T0)
                        last = shifted.get((e.name, f), np.eye(4))
                        w.world.carry_obstacle(obj, T @ np.linalg.inv(last))
                        shifted[(e.name, f)] = T
            contacts = checks.pair_contacts(w, qa[e.arm])
            by_part = None
            for (link, ob), clr in contacts.items():
                key = (e.name, link, ob)
                if ob == BODY and (k < 0 or (e.name, link) in touching):
                    if by_part is None:
                        by_part = _body_part_contacts(w, qa[e.arm])
                    for part, c in by_part.get(link, {}).items():
                        touching.add((e.name, link))
                        now.add((*key, part))
                        hit(k, (*key, part), c, SELF_KIND, f"{e.name}: {link} hits {BODY}:{part} "
                            f"as {name} moves ({at}, clearance {c * 1000:.0f} mm)")
                    continue
                now.add(key)
                if ob == BODY:
                    hit(k, key, clr, SELF_KIND, f"{e.name}: {link} hits {BODY} as {name} "
                        f"moves ({at}, clearance {clr * 1000:.0f} mm)")
                else:
                    hit(k, key, clr, "collision",
                        f"{e.name}: {link} would hit {ob} as {name} moves ({at})")
        if moved:
            C, R, names = clone.body_world()
            sel = np.isin(np.asarray(names), list(moved))
            hits = _scene_contacts(worlds.values(), C[sel], R[sel], np.asarray(names)[sel],
                                   moving)
            for (link, ob), clr in hits.items():
                key = (name, link, ob)
                now.add(key)
                hit(k, key, clr, "collision", f"{name}: {link} would hit {ob} as {name} moves "
                    f"({at})")
            for (a, b), clr in clone.body_self_clearances().items():
                if clr < PEER_MARGIN_M:
                    key = (name, BODY, a, b)
                    now.add(key)
                    hit(k, key, clr - PEER_MARGIN_M, SELF_KIND, f"{name}: {b} hits {BODY}:{a} "
                        f"as {name} moves ({at}, clearance {clr * 1000:.0f} mm, margin "
                        f"{PEER_MARGIN_M * 1000:.0f} mm)")
        if k >= 0:
            parted |= {key for key in start if key not in now}
    out.sort(key=lambda i: i.step)
    return out
