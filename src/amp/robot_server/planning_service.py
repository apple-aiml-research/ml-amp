#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp.planning behind the server: agent IK and checks, assist plans and steps, manual plans."""

from __future__ import annotations

import dataclasses
import functools
import math
import threading
from collections import OrderedDict

import numpy as np

from amp.geometry import kinematics as kin
from amp.geometry import world as scene_mod
from amp.link.wire import jsonable
from amp.planning import checks, ik, jaw, trajectory
from amp.planning import options as opt
from amp.planning.assist import AssistPlan, StepSpec, held_transform
from amp.planning.grasp import held_geometry
from amp.planning.motion import held_contacts
from amp.planning.multi import (
    SCHEDULES,
    ArmPlan,
    body_path_issues,
    grip_drift,
    peer_path_issues,
    plan_chunk_multi,
)
from amp.planning.planner import (
    PlanContext,
    PlannerUnavailable,
    ScenePlanner,
    feasible_first,
)
from amp.robot_server import embodiment as rt
from amp.robot_server import mobile
from amp.robot_server.config import MAX_DURATION_S, SETTLE_S, SPEED_SCALE, _finite
from amp.robot_server.errors import AssistError, SessionConflict
from amp.robot_server.executor import BASE_UNAVAILABLE

PLAN_CACHE_SIZE = 8
MAX_CANDIDATES = 32
MOVED_SINCE_PLAN = "the robot moved since this plan; plan again"
ASSIST_BUSY = "the assist planner is busy with another request"
PLAN_EXECUTING = "the plan is executing; stop it to edit"
PUBLISHING = "the episode is being published"
NOTHING_HELD = "nothing is held; pick it first (or pass force)"
JOIN_NOTE = "{name} is held by {holders}: the pick ends once the fingers close"
CO_HELD = ("{name} is held by {holders}: move both together (ee_waypoints / move_both) or open "
           "one hand first")
CO_HELD_KIND = "co_held"
GRASP_KINDS = ("pick", "place", "lift", "push")
CO_HELD_IDLE = ("{name} is held by {holders}: every one of them must move in an ee_waypoints plan "
                "or none ({idle} has no waypoints)")
CO_HELD_DRIFT = ("{name} is held by {holders}: the grip must stay rigid, but the hands' relative "
                 "pose drifts by {mm:.1f} mm and {deg:.1f} deg at step {step} (limits "
                 "{tol_mm:g} mm and {tol_deg:g} deg)")
CO_HELD_POS_TOL_M = 0.005
CO_HELD_ANG_TOL_RAD = math.radians(2.0)
CO_HELD_TOL = 1e-3
WORKER_ROUNDS = 16
_DEFAULT = object()


def q7(value, name="q", dof=7) -> np.ndarray:
    """A joint configuration of dof finite radians."""
    if not isinstance(value, (list, tuple)) or len(value) != dof:
        raise ValueError(f"{name} must be {dof} joint positions in radians, got {value!r}")
    out = []
    for v in value:
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f"{name} must be {dof} finite numbers, got {value!r}")
        out.append(float(v))
    return np.array(out)


def int_in(value, name, lo, hi) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not lo <= value <= hi:
        raise ValueError(f"{name} must be an integer in [{lo}, {hi}], got {value!r}")
    return value


def batch(body, key, what) -> list:
    v = body.get(key)
    if not isinstance(v, list) or not 1 <= len(v) <= checks.BATCH_MAX:
        raise ValueError(f"{key} must be a list of 1..{checks.BATCH_MAX} {what}, got "
                         f"{type(v).__name__} of {len(v) if isinstance(v, list) else 0}")
    return v


def vec(x) -> list:
    return [float(v) for v in np.asarray(x, dtype=np.float64).reshape(-1)]


def count(body) -> int:
    n = body.get("max", 8)
    if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= MAX_CANDIDATES:
        raise ValueError(f"max must be an integer in [1, {MAX_CANDIDATES}], got {n!r}")
    return n


def at_of(body):
    at = body.get("at")
    if at is not None and (isinstance(at, bool) or not isinstance(at, int) or at < 0):
        raise ValueError(f"at must be a step count >= 0, got {at!r}")
    return at


def step_id_of(body) -> int:
    sid = body.get("id")
    if isinstance(sid, bool) or not isinstance(sid, int):
        raise ValueError(f"id must be a step id (an integer), got {sid!r}")  # noqa: TRY004
    return sid


def with_notes(reply, cands=None) -> dict:
    """reply plus "notes": the arm-inside notes of its candidates, if any."""
    notes = []
    for c in reply.get("candidates", ()) if cands is None else cands:
        notes += [n for n in c.get("notes") or () if n.startswith("ignored ") and n not in notes]
    if notes:
        reply["notes"] = notes
        if reply.get("message"):
            reply["message"] = f"{reply['message']}; {'; '.join(notes)}"
    elif cands is not None:
        reply.pop("message", None)
    return reply


def _hands(ees) -> str:
    return ", ".join(ees[:-1]) + " and " + ees[-1]


def _first_move(res, until_open=True) -> int | None:
    """The first step where res's arm leaves its start q (until_open: with the gripper closed)."""
    q = np.asarray(res.q, dtype=np.float64)
    q0 = np.asarray(res.start_q, dtype=np.float64).reshape(-1)
    if q.ndim != 2 or not len(q) or q.shape[1] != len(q0):
        return None
    opened = np.flatnonzero(np.asarray(res.gripper, dtype=np.float64)[:len(q)] < 0.5)
    n = int(opened[0]) if len(opened) and until_open else len(q)
    moved = np.flatnonzero(np.abs(q[:n] - q0).max(axis=1, initial=0.0) > CO_HELD_TOL)
    return int(moved[0]) if len(moved) else None


class Snapshot:
    """What one planning call needs, copied under the lock: the context, scene and held state."""

    def __init__(self, base: dict, scene, T_world_tcp, held, ctx: PlanContext, planner_rev,
                 ee=None, others=None):
        self.base, self.scene, self.T_world_tcp, self.held = base, scene, T_world_tcp, held
        self.ctx, self.planner_rev = ctx, planner_rev
        self.seq = base["seq"]
        self.ee = ee
        self.others = others


class Scope:
    """One end effector of a robot with several: its arm's slice of the full q, and its view."""

    def __init__(self, srv, ee: str, peer_q):
        lay = srv.executor.layout
        self.ee, self.arm = ee, lay.ee_arm[ee]
        self.idx = lay.ee_q[ee]
        self.base = srv.embodiment.view(ee)
        with srv.cond:
            name, holders = srv.world.held_name, list(srv.world.held_ees)
        held = {lay.ee_arm[e]: name for e in holders if lay.ee_arm.get(e, self.arm) != self.arm}
        self.view = self.base.with_peers(peer_q, held)

    def part(self, q) -> np.ndarray:
        return np.asarray(q, dtype=np.float64)[self.idx].copy()

    def json(self) -> dict:
        return {"ee": self.ee, "q_slice": [int(self.idx[0]), int(self.idx[-1]) + 1]}


class PlanningService:
    """Under the server lock unless noted: the planner, the plan cache and the multi-step plan."""

    def __init__(self, srv, *, planner: ScenePlanner | None = None):
        self._srv = srv
        self._own_planner = planner is None
        try:
            self.planner = planner if planner is not None else self._make_planner(None)
        except PlannerUnavailable as exc:
            raise ValueError(f"ServerConfig.planner {srv.settings.planner!r}: {exc}") from exc
        self.planner_rev = 0
        self.assist_lock = threading.Lock()
        self.busy = False
        self._plans: OrderedDict = OrderedDict()
        self._next_plan_id = 1
        self.plan: AssistPlan | None = None
        self._base = None
        self._json = None
        self._preview = None
        self._specs = None
        self.rev = 0
        self._next_step_id = 1
        self._pop_due = None
        self.exec = None
        self._worker = None
        self.replanning = False
        self._plan_ee = None
        self._grip_ref = None

    def _make_planner(self, emb) -> ScenePlanner:
        """The assist planner for emb (None = DROID)."""
        return ScenePlanner(planner="builtin", embodiment=emb)

    def emb(self):
        """The embodiment planning calls take: None for DROID, else the server's runtime."""
        emb = self._srv.embodiment
        return None if rt.is_droid(emb) else emb

    def set_embodiment(self) -> None:
        """Under the lock: plan for the server's new robot; plans made for the old one go."""
        if self._own_planner:
            self.planner = self._make_planner(self.emb())
        self.planner_rev += 1
        self.reset()

    def reset(self) -> None:
        """Under the lock, at an episode start."""
        self._plans.clear()
        self.rev += 1
        self.plan = self._base = self._json = self._preview = self._specs = None
        self._pop_due = self.exec = None
        self._next_step_id = 1
        self._plan_ee = None
        self._grip_ref = None

    def spec_count(self) -> int:
        return len(self._specs or ()) if self.plan is not None else 0

    def wanted_ids(self) -> list[str]:
        """Object ids the plan steps name: perception keeps them across renumbering."""
        out = []
        for spec in (self._specs if self.plan is not None else None) or ():
            out += [x for x in (spec.get("object"), spec.get("target")) if isinstance(x, str)]
        return out

    def set_planner(self, name) -> bool:
        """Select the assist planner; True when it changed. 409 when it is unavailable."""
        before = self.planner.planner
        try:
            self.planner.set_planner(name)
        except PlannerUnavailable as exc:
            raise SessionConflict(str(exc)) from exc
        changed = self.planner.planner != before
        if changed:
            with self._srv.cond:
                self.planner_rev += 1
                self.kick()
        return changed

    def planner_json(self) -> dict:
        return self.planner.planner_state()

    def q(self, value, name="q") -> np.ndarray:
        """A full joint configuration of the server's embodiment."""
        return q7(value, name, self._srv.embodiment.dof)

    def scoped(self) -> bool:
        """Whether requests plan for one end effector of several arms or hands."""
        spec = self._srv.embodiment.spec
        return len(spec.arms) > 1 or len(spec.end_effectors) > 1

    def primary(self) -> str:
        return self._srv.executor.layout.primary

    def ee_name(self, ee, *, star=False) -> str | None:
        """A request's "ee": None when absent, "*" (every one) where star, else an ee's name."""
        if ee is None or (star and ee == "*"):
            return ee
        emb = self._srv.embodiment
        if isinstance(ee, str):
            if ee in emb.end_effectors:
                return ee
            on = [e.name for e in emb.spec.end_effectors if e.arm == ee]
            if len(on) == 1:
                return on[0]
        alt = ' or "*"' if star else ""
        raise ValueError(f"ee must be one of {list(emb.end_effectors)}{alt}, got {ee!r}")

    def ee_of(self, body, *, star=False) -> str | None:
        return self.ee_name(body.get("ee") if isinstance(body, dict) else None, star=star)

    def no_gripper(self, ee, what="it cannot grasp") -> str | None:
        """Why ee (None: the primary) cannot do what needs fingers, or None when it has some."""
        name = ee or self.primary()
        if self._srv.embodiment.ee(name).gripper is not None:
            return None
        return f"end effector {name} has no gripper: {what}"

    def refuse_unmodelled_hand(self, ee) -> None:
        """A 409 before any planner call when ee's hand has an axis pair grasp planning lacks."""
        emb = self._srv.embodiment
        hand = emb.ee(ee or self.primary())
        if hand.gripper is None:
            return
        try:
            jaw.canonical_rotation(hand, emb.name)
        except ValueError as exc:
            raise SessionConflict(f"grasp assist is unavailable: {exc}; use move_ee and the "
                                  "gripper") from exc

    def _scope(self, ee, peer_q) -> Scope:
        return Scope(self._srv, ee or self.primary(), peer_q)

    def _observed_q(self) -> np.ndarray:
        with self._srv.cond:
            return np.asarray(self._srv.observations_live().q, dtype=np.float64).copy()

    def _q_full(self, value, name, ee, base) -> tuple[np.ndarray, bool]:
        """(full q, whether value gave every arm): value is dof-long, or arm-long for ee with the
        other arms from base(); None is base()."""
        dof = self._srv.embodiment.dof
        idx = self._srv.executor.layout.ee_q[ee]
        if value is None:
            return base(), False
        if isinstance(value, (list, tuple)) and len(value) == len(idx) != dof:
            q = base()
            q[idx] = q7(value, name, len(idx))
            return q, False
        if isinstance(value, (list, tuple)) and len(value) == dof:
            return q7(value, name, dof), True
        raise ValueError(f"{name} must be {dof} joint positions (every arm) or {len(idx)} "
                         f"({ee}'s arm) in radians, got {value!r}")

    def _fk_scoped(self, body, ee) -> dict:
        srv = self._srv
        q, _ = self._q_full(body.get("q"), "q", ee, self._observed_q)
        emb, lay = srv.embodiment, srv.executor.layout
        with srv.cond:
            tools = {e: srv.obs.tool_copy(e) for e in lay.ees}
        ees = {}
        for e, tool in tools.items():
            qa = q[lay.ee_q[e]]
            ees[e] = {"ee_pose": vec(kin.pose7(tool.fk_ee(qa))),
                      "tcp_pose": vec(kin.pose7(tool.fk_tcp(qa))),
                      "q_slice": self._scope(e, q).json()["q_slice"]}
        sc = self._scope(ee, q)
        return {"ok": True, "q": vec(q), **sc.json(),
                "links": np.asarray(tools[ee].link_points(sc.part(q))).tolist(),
                "ee_pose": ees[ee]["ee_pose"], "tcp_pose": ees[ee]["tcp_pose"],
                "link_poses": {n: vec(kin.pose7(T)) for n, T in emb.link_poses(q).items()},
                "end_effectors": ees}

    def _seed_scoped(self, body, ee):
        """(scope, ee's arm seed, ee's tool) of an ik request."""
        seed, _ = self._q_full(body.get("seed"), "seed", ee, self._observed_q)
        sc = self._scope(ee, seed)
        with self._srv.cond:
            tool = self._srv.obs.tool_copy(sc.ee)
        return sc, sc.part(seed), tool

    def _q_and_tool(self, body, key="q"):
        srv = self._srv
        raw = body.get(key)
        with srv.cond:
            q = self.q(raw, key) if raw is not None else np.asarray(
                srv.observations_live().q, dtype=np.float64).copy()
            return q, srv.obs.tool_copy()

    def _arm_q(self, q):
        with self._srv.cond:
            return self._srv.obs.arm_q(q)

    def fk(self, body) -> dict:
        ee = self.ee_of(body)
        if self.scoped():
            return self._fk_scoped(body, ee or self.primary())
        q, tool = self._q_and_tool(body)
        qa = self._arm_q(q)
        out = {"ok": True, "q": vec(q), "links": np.asarray(tool.link_points(qa)).tolist(),
               "ee_pose": vec(kin.pose7(tool.fk_ee(qa))),
               "tcp_pose": vec(kin.pose7(tool.fk_tcp(qa)))}
        emb = self._srv.embodiment
        if not rt.is_droid(emb):
            out["link_poses"] = {n: vec(kin.pose7(T)) for n, T in emb.link_poses(q).items()}
        return out

    def ik(self, body) -> dict:
        if "tcp" not in body:
            raise ValueError("ik needs 'tcp' [x, y, z, qw, qx, qy, qz]")
        T = scene_mod.pose_from7(body["tcp"], "tcp")
        n_max = int_in(body.get("max", ik.MAX_DEFAULT), "max", 1, ik.MAX_LIMIT)
        ee = self.ee_of(body)
        if self.scoped():
            sc, seed, tool = self._seed_scoped(body, ee or self.primary())
            sols = ik.solve(T, seed, tool, n_max, embodiment=sc.base)
            return {"ok": True, "reachable": ik.reachable(sols),
                    "solutions": [s.to_json() for s in sols], **sc.json()}
        seed, tool = self._q_and_tool(body, "seed")
        sols = ik.solve(T, seed, tool, n_max, embodiment=self.emb())
        return {"ok": True, "reachable": ik.reachable(sols),
                "solutions": [s.to_json() for s in sols]}

    def ik_batch(self, body) -> dict:
        tcps = batch(body, "tcps", "TCP poses [x, y, z, qw, qx, qy, qz]")
        Ts = np.stack([scene_mod.pose_from7(t, f"tcps[{i}]") for i, t in enumerate(tcps)])
        n_max = int_in(body.get("max", 1), "max", 1, ik.MAX_LIMIT)
        ee = self.ee_of(body)
        if self.scoped():
            sc, seed, tool = self._seed_scoped(body, ee or self.primary())
            return {"ok": True, **sc.json(), "results": [
                {"reachable": ik.reachable(sols), "solutions": [s.to_json() for s in sols]}
                for sols in ik.solve_batch(Ts, seed, tool, n_max, embodiment=sc.base)]}
        seed, tool = self._q_and_tool(body, "seed")
        return {"ok": True, "results": [
            {"reachable": ik.reachable(sols), "solutions": [s.to_json() for s in sols]}
            for sols in ik.solve_batch(Ts, seed, tool, n_max, embodiment=self.emb())]}

    def robot(self, body) -> dict:
        """POST api/agent/robot: the collision spheres (the model's fingers are always open)."""
        g = body.get("gripper")
        if g is not None and (isinstance(g, bool) or not isinstance(g, (int, float))
                              or not 0.0 <= g <= 1.0):
            raise ValueError(f"gripper must be a number in [0, 1], got {g!r}")
        ee = self.ee_of(body)
        if self.scoped():
            q, _ = self._q_full(body.get("q"), "q", ee or self.primary(), self._observed_q)
            sc = self._scope(ee, q)
            with self._srv.cond:
                tool = self._srv.obs.tool_copy(sc.ee)
            return {**self._spheres_json(q, g, sc.part(q), tool, sc.arm), **sc.json()}
        q, tool = self._q_and_tool(body)
        qa = self._arm_q(q)
        emb = self._srv.embodiment
        return self._spheres_json(q, g, qa, tool, emb.ee().arm)

    def _spheres_json(self, q, g, qa, tool, arm) -> dict:
        emb = self._srv.embodiment
        if emb.whole_body and len(q) == emb.dof:
            c, r, names = emb.whole_spheres(q)
        else:
            sp = emb.spheres(arm)
            c, r, link = sp.world(qa, tool.T_flange_ee)
            names = [sp.link_names[int(i)] for i in link]
        return {"ok": True, "q": vec(q), "gripper": g,
                "spheres": np.column_stack([c, r]).tolist(),
                "sphere_links": names,
                "links": np.asarray(tool.link_points(qa)).tolist(),
                "tcp_pose": vec(kin.pose7(tool.fk_tcp(qa)))}

    def _check_world(self, body) -> checks.CheckWorld:
        """The agent check world for body's "held" and "ignore", per the episode's obstacles."""
        srv = self._srv
        if srv.obstacles() == "table":
            with srv.cond:
                srv.observations_live()
                q_obs = np.asarray(srv.obs.obs.q, dtype=np.float64).copy()
                table_z = srv.table_z()
                tool = srv.obs.tool_copy()
            return checks.table_world(table_z=table_z, tool=tool, q_obs=q_obs,
                                      margin=self.planner.margin, held=body.get("held"),
                                      ignore=body.get("ignore") or [], embodiment=self.emb(),
                                      table=srv.settings.table_obstacle)
        held = body.get("held")
        if held is not None and not isinstance(held, str):
            raise ValueError(f"held must be an object id or null, got {held!r}")
        srv.perception.ensure()
        with srv.cond:
            obs = srv.observations_live()
            tool = srv.obs.tool_copy()
            q_obs = np.asarray(obs.q, dtype=np.float64).copy()
            T_world_tcp = srv.obs.observed_tcp()
            table_z = srv.table_z()
            sc = srv.world.scene
            scene = sc.copy() if sc is not None else scene_mod.Scene(
                srv.episode.episode or 0, {}, table_z, 0)
            if "held" not in body:
                held = srv.world.held_name
                held = held if held in scene.objects else None
        try:
            return checks.scene_world(
                scene, tool=tool, q_obs=q_obs, T_world_tcp=T_world_tcp,
                margin=self.planner.margin, held=held, ignore=body.get("ignore"),
                arm_inside=self.planner.arm_inside_at,
                embodiment=self.emb(), table=srv.settings.table_obstacle)
        except checks.UnknownObject as exc:
            raise SessionConflict(str(exc)) from None

    def check(self, body) -> dict:
        if body.get("q") is not None and body.get("tcp") is not None:
            raise ValueError("check takes 'q' or 'tcp', not both")
        T = scene_mod.pose_from7(body["tcp"], "tcp") if body.get("tcp") is not None else None
        ee = self.ee_of(body)
        if self.scoped():
            return self._check_scoped(body, ee, T)
        q = self.q(body["q"]) if body.get("q") is not None else None
        return checks.check(self._check_world(body), q=q, T_tcp=T)

    def check_batch(self, body) -> dict:
        ee = self.ee_of(body)
        if self.scoped():
            return self._check_batch_scoped(body, ee)
        qs = np.stack([self.q(q, f"qs[{i}]") for i, q in
                       enumerate(batch(body, "qs", "joint configurations"))])
        return checks.check_batch(self._check_world(body), qs)

    def _check_inputs(self, body) -> dict:
        """What every end effector's check world is built from, copied under the lock."""
        srv = self._srv
        table = srv.obstacles() == "table"
        held = body.get("held")
        if not table:
            if held is not None and not isinstance(held, str):
                raise ValueError(f"held must be an object id or null, got {held!r}")
            srv.perception.ensure()
        with srv.cond:
            obs = srv.observations_live()
            ees = srv.executor.layout.ees
            out = {"table": table, "table_z": srv.table_z(),
                   "q_obs": np.asarray(obs.q, dtype=np.float64).copy(),
                   "hold_q": srv.executor.hold_q(),
                   "tools": {e: srv.obs.tool_copy(e) for e in ees}}
            if not table:
                tcp = {}
                for e in ees:
                    try:
                        tcp[e] = srv.obs.observed_tcp(e)
                    except KeyError:
                        tcp[e] = None
                out["tcp"] = tcp
                sc = srv.world.scene
                out["scene"] = sc.copy() if sc is not None else scene_mod.Scene(
                    srv.episode.episode or 0, {}, out["table_z"], 0)
                out["held_by"] = {e: srv.world.held_by(e) for e in ees}
        return out

    def _ee_world(self, body, inp, sc: Scope, held=_DEFAULT) -> checks.CheckWorld:
        """sc's check world: its arm against the scene, the table and the peers at sc's q."""
        tool, q_obs = inp["tools"][sc.ee], sc.part(inp["q_obs"])
        if inp["table"]:
            return checks.table_world(table_z=inp["table_z"], tool=tool, q_obs=q_obs,
                                      margin=self.planner.margin, held=body.get("held"),
                                      ignore=body.get("ignore") or [], embodiment=sc.view,
                                      table=self._srv.settings.table_obstacle)
        scene = inp["scene"]
        if held is _DEFAULT:
            held = body.get("held") if "held" in body else inp["held_by"][sc.ee]
            if "held" not in body and held not in scene.objects:
                held = None
        T = inp["tcp"][sc.ee]
        arm_inside = functools.partial(self.planner.arm_inside_at, embodiment=sc.base)
        try:
            return checks.scene_world(
                scene, tool=tool, q_obs=q_obs, T_world_tcp=tool.fk_tcp(q_obs) if T is None else T,
                margin=self.planner.margin, held=held, ignore=body.get("ignore"),
                arm_inside=arm_inside, embodiment=sc.view,
                table=self._srv.settings.table_obstacle)
        except checks.UnknownObject as exc:
            raise SessionConflict(str(exc)) from None

    def _every_arm(self, body, inp, q) -> dict:
        """A full q checked arm by arm, each against the other arms at q, and combined."""
        lay = self._srv.executor.layout
        per = {}
        for a in self._srv.embodiment.spec.arms:
            ee = next((e for e in lay.ees if lay.ee_arm[e] == a.name), None)
            if ee is None:
                continue
            sc = self._scope(ee, q)
            held = _DEFAULT
            if "held" in body and not inp["table"]:
                h = body.get("held")
                held = h if ee == lay.primary or inp["held_by"][ee] == h else None
            r = checks.check(self._ee_world(body, inp, sc, held), q=sc.part(q))
            per[ee] = {**{k: v for k, v in r.items() if k not in ("ok", "margin_m")},
                       **sc.json()}
        rs = list(per.values())
        clr = [r["min_clearance_m"] for r in rs if r["min_clearance_m"] is not None]
        inside: dict = {}
        for r in rs:
            for k, v in r["arm_inside"].items():
                inside[k] = max(inside.get(k, v), v)
        return {"margin_m": float(self.planner.margin), "q": vec(q),
                "reachable": all(r["reachable"] for r in rs),
                "in_limits": all(r["in_limits"] for r in rs),
                "singular": any(r["singular"] for r in rs),
                "collision_free": all(r["collision_free"] for r in rs),
                "min_clearance_m": min(clr) if clr else None,
                "contacts": sorted(({**c, "ee": e} for e, r in per.items() for c in r["contacts"]),
                                   key=lambda c: c["clearance_m"]),
                "arm_inside": inside, "end_effectors": per}

    def _whole(self, ee, raw) -> bool:
        """An ee-less dof-long (or absent) q on a robot with several arms: check every arm."""
        dof = self._srv.embodiment.dof
        return ee is None and len(self._srv.embodiment.spec.arms) > 1 and (
            raw is None or (isinstance(raw, (list, tuple)) and len(raw) == dof))

    def _check_scoped(self, body, ee, T) -> dict:
        raw = body.get("q")
        inp = self._check_inputs(body)
        if T is None and self._whole(ee, raw):
            return {"ok": True, **self._every_arm(
                body, inp, inp["q_obs"] if raw is None else self.q(raw))}
        ee = ee or self.primary()
        q, full = self._q_full(raw, "q", ee, lambda: inp["q_obs"].copy())
        sc = self._scope(ee, q if full else inp["hold_q"])
        reply = checks.check(self._ee_world(body, inp, sc), q=None if raw is None else sc.part(q),
                             T_tcp=T)
        return {**reply, **sc.json()}

    def _check_batch_scoped(self, body, ee) -> dict:
        raw = batch(body, "qs", "joint configurations")
        inp = self._check_inputs(body)
        if all(self._whole(ee, v) for v in raw):
            results = [self._every_arm(body, inp, self.q(v, f"qs[{i}]")) for i, v in
                       enumerate(raw)]
            for r in results:
                del r["margin_m"]
            return {"ok": True, "margin_m": float(self.planner.margin), "results": results}
        ee = ee or self.primary()
        parsed = [self._q_full(v, f"qs[{i}]", ee, lambda: inp["q_obs"].copy())
                  for i, v in enumerate(raw)]
        worlds: dict = {}
        results = []
        sc = None
        for q, full in parsed:
            sc = self._scope(ee, q if full else inp["hold_q"])
            key = np.delete(q if full else inp["hold_q"], sc.idx).tobytes()
            if key not in worlds:
                worlds[key] = self._ee_world(body, inp, sc)
            w, qa = worlds[key], sc.part(q)
            results.append({**checks.q_fields(qa, True, w.embodiment),
                            **checks.collisions(w, qa[None])[0]})
        return {"ok": True, "margin_m": float(self.planner.margin), "results": results,
                **sc.json()}

    def snapshot(self, names=(), ee=None) -> Snapshot:
        """Under the lock: the plan start state, scene copy and held object for planning outside;
        on a robot with several end effectors, for ee's arm (None: the primary)."""
        srv = self._srv
        base = srv.executor.plan_context()
        world = srv.world
        if world.scene is None:
            why = srv.perception.no_scene_reason()
            raise SessionConflict(f"no scene for this episode: no perception ({why})")
        for n in names:
            if n is not None and n not in world.scene.objects:
                raise SessionConflict(f"unknown object {n!r}; the scene has "
                                      f"{sorted(world.scene.objects)}")
        scene = world.scene.copy()
        if self.scoped():
            sc = self._scope(ee, base["hold_q"])
            try:
                T_world_tcp = srv.obs.observed_tcp(sc.ee)
            except KeyError as exc:
                raise SessionConflict(str(exc.args[0] if exc.args else exc)) from None
            held = world.held_by(sc.ee)
            others = None
            if world.held_name is not None and held is None:
                others = (world.held_name, list(world.held_ees))
            q = sc.part(base["q"])
            ctx = PlanContext(tool=srv.obs.tool_copy(sc.ee), q=q,
                              gripper=int(base["grippers"].get(sc.ee, 0)), dt=base["dt"],
                              settle_s=SETTLE_S,
                              arm_inside_q=q.copy(),
                              T_world_tcp=T_world_tcp, held=held, embodiment=sc.view,
                              proposals=srv.settings.assist_proposals)
            return Snapshot(base, scene, T_world_tcp, held, ctx, self.planner_rev, sc.ee, others)
        T_world_tcp = srv.obs.observed_tcp()
        held = world.held_name
        ctx = PlanContext(tool=base["tool"], q=base["q"], gripper=base["gripper"],
                          dt=base["dt"], settle_s=SETTLE_S,
                          arm_inside_q=base["q"].copy(),
                          T_world_tcp=T_world_tcp, held=held, embodiment=self.emb(),
                          proposals=srv.settings.assist_proposals)
        return Snapshot(base, scene, T_world_tcp, held, ctx, self.planner_rev)

    def plan_ee(self) -> str | None:
        """Under the lock: the end effector the multi-step plan moves (None: one-ee robots)."""
        return (self._plan_ee or self.primary()) if self.scoped() else None

    def _owns(self, ee) -> bool:
        """Under the lock: whether a request for ee (None: the primary) is the plan's ee's."""
        return not self.scoped() or (ee or self.primary()) == self.plan_ee()

    def stale(self) -> bool:
        """Under the lock: the plan was made from an older observation, scene or planner."""
        if self.plan is None:
            return False
        sc = self._srv.world.scene
        return self._pop_due is not None or self._base != (
            self._srv.obs.frame_seq, sc.version if sc is not None else None, self.planner_rev)

    def _guarded(self, fn, what="assist planner"):
        try:
            return fn()
        except (SessionConflict, AssistError):
            raise
        except PlannerUnavailable as exc:
            raise SessionConflict(str(exc)) from exc
        except Exception as exc:
            self._srv.log("error", f"{what} failed: {exc!r}")
            raise AssistError(f"{what} failed: {type(exc).__name__}: {exc}") from exc

    def _busy(self, on: bool) -> None:
        with self._srv.cond:
            self.busy = on
            self._srv.cond.notify_all()

    def _run(self, names, compute, at=None, ee=None):
        """compute(snapshot, scene, T_world_tcp, ctx, pred) on a snapshot, one at a time; pred is
        the plan's PredictedState before step `at` (None at the observation)."""
        srv = self._srv
        if not self.assist_lock.acquire(blocking=False):
            raise SessionConflict(ASSIST_BUSY)
        busy = False
        try:
            srv.perception.ensure()
            with srv.cond:
                own = self._owns(ee)
                if not own and at:
                    raise SessionConflict(
                        f"at counts steps of the plan, which moves {self.plan_ee()}; plan "
                        f"{ee or self.primary()} from the observation (no at)")
                snap = self.snapshot(names, ee)
                plan, token, stale = self.plan if own else None, srv.episode.token, self.stale()
                self.busy = busy = True

            def run():
                if plan is not None and stale and (at is None or at > 0):
                    try:
                        self._bring_current(plan, snap.scene.copy(), snap.ctx)
                    finally:
                        self._publish(plan, snap, token)
                n = len(plan.steps) if plan is not None else 0
                k = n if at is None else at
                if not 0 <= k <= n:
                    raise SessionConflict(f"at must be in [0, {n}]: the plan has {n} steps")
                scene, ctx, T, pred = snap.scene, snap.ctx, snap.T_world_tcp, None
                if k > 0:
                    pred = plan.state_at(k)
                    scene, ctx = pred.scene(scene), pred.ctx(ctx)
                    T = ctx.tool.fk_tcp(np.asarray(pred.q, dtype=np.float64))
                return compute(snap, scene, T, ctx, pred)

            return self._guarded(run)
        finally:
            if busy:
                self._busy(False)
            self.assist_lock.release()

    def grasps(self, body) -> dict:
        name = opt.name_of(body, "object")
        target, point = opt.target_or_point(body, name, exactly_one=False)
        where = {} if target is None and point is None else {"target": target, "point": point}
        if body.get("near") is not None:
            where["near"] = opt.point3(body.get("near"), "near")
        n, at = count(body), at_of(body)
        ee = self.ee_of(body)
        if (why := self.no_gripper(ee)) is not None:
            return {"ok": False, "object": name, "candidates": [], "message": why}
        self.refuse_unmodelled_hand(ee)

        def compute(_snap, scene, _T, ctx, _pred):
            cands = list(self.planner.grasps(scene, name, ctx, max_candidates=n, **where))[:n]
            shown, message = feasible_first(cands, "grasp", name)
            return with_notes({"ok": True, "object": name, "candidates": shown,
                               "message": message})

        return self._run([name, target], compute, at, ee)

    def places(self, body) -> dict:
        name = opt.name_of(body, "object")
        target, point = opt.target_or_point(body, name, exactly_one=True)
        n, at = count(body), at_of(body)
        ee = self.ee_of(body)
        insert = body.get("insert", False)
        if not isinstance(insert, bool):
            raise ValueError(f"insert must be true or false, got {insert!r}")  # noqa: TRY004
        if (why := self.no_gripper(ee, "it cannot place")) is not None:
            return {"ok": False, "object": name, "target": target,
                    "point": None if point is None else point.tolist(), "candidates": [],
                    "message": why}
        self.refuse_unmodelled_hand(ee)

        def compute(_snap, scene, T, ctx, pred):
            T_tcp_obj = held_transform(name, scene, T, pred)
            cands = list(self.planner.places(
                scene, name, T_tcp_obj, ctx, target=target, point=point, max_candidates=n))[:n]
            shown, message = feasible_first(cands, "placement", name)
            return with_notes({"ok": True, "object": name, "target": target,
                               "point": None if point is None else point.tolist(),
                               "candidates": shown, "message": message})

        return self._run([name, target], compute, at, ee)

    def check_assist(self, body) -> dict:
        """POST api/assist/check: exactly one grasp or release pose evaluated."""
        kind = body.get("kind")
        if kind not in ("grasp", "place"):
            raise ValueError(f"kind must be 'grasp' or 'place', got {kind!r}")
        name = opt.name_of(body, "object")
        T_tcp = scene_mod.pose_from7(body.get("tcp"), "tcp")
        target = point = None
        if kind == "place":
            target, point = opt.target_or_point(body, name, exactly_one=False)
        at = at_of(body)
        ee = self.ee_of(body)
        if (why := self.no_gripper(ee)) is not None:
            return {"ok": False, "candidate": None, "message": why}
        self.refuse_unmodelled_hand(ee)

        def compute(_snap, scene, T, ctx, pred):
            if kind == "grasp":
                cand = self.planner.check_grasp(scene, name, T_tcp, ctx)
            else:
                cand = self.planner.check_place(scene, name, held_transform(name, scene, T, pred),
                                                T_tcp, ctx, target=target, point=point)
            c = jsonable(cand.to_json(0))
            return with_notes({"ok": True, "candidate": c, "message": ""}, [c])

        return self._run([name, target], compute, at, ee)

    def plan_assist(self, body) -> dict:
        """POST api/assist/plan: a collision-free pick, place, lift, release, move or push."""
        srv = self._srv
        if self._intent(body) == "select":
            return self._select_only(body)
        kind = body.get("kind")
        if kind not in opt.ASSIST_KINDS:
            raise ValueError(f"kind must be one of {list(opt.ASSIST_KINDS)}, got {kind!r}")
        name = opt.name_of(body, "object") if kind in ("pick", "place") or \
            body.get("object") is not None else None
        T_tcp = scene_mod.pose_from7(body.get("tcp"), "tcp") if kind in ("pick", "place",
                                                                         "move") else None
        target = point = start = end = None
        if kind == "place":
            target, point = opt.target_or_point(body, name, exactly_one=False)
        if kind == "push":
            start, end = opt.point3(body.get("start"), "start"), opt.point3(body.get("end"), "end")
        force = opt.force(body)
        raw = body.get("options")
        opt.validate(kind, raw)
        request = jsonable(body)
        ee = self.ee_of(body)
        if self._intent(body) == "move" and kind != "lift":
            with srv.cond:
                srv.executor.note_call([ee or self.primary()])
        why, why_kind = (self.no_gripper(ee) if kind != "move" else None), "no_gripper"
        if why is None and kind != "release":
            why, why_kind = self._co_held_why(ee), CO_HELD_KIND
        if why is not None:
            return {"ok": False, "kind": kind, "plan_id": None, "message": why,
                    **({"ee": ee or self.primary()} if self.scoped() else {}),
                    "issues": [{"step": 0, "t": 0.0, "kind": why_kind, "severity": "error",
                                "message": why}]}
        if kind in GRASP_KINDS:
            self.refuse_unmodelled_hand(ee)

        def compute(snap, scene, T, ctx, _pred):
            obj_name = name
            opts = opt.parse(kind, raw, settle_s=SETTLE_S)
            held = snap.held
            if kind in ("place", "lift") and held is None and not force:
                raise SessionConflict(NOTHING_HELD)
            if kind in ("lift", "release", "move") and obj_name is None:
                obj_name = held
            obj = scene.objects.get(obj_name) if obj_name is not None else None
            T_tcp_obj = None if obj is None else np.linalg.inv(T) @ obj.pose
            carried = obj_name if obj_name is not None and obj_name == held else None
            p = self.planner
            joins = kind == "pick" and snap.others is not None and snap.others[0] == obj_name
            if joins:
                opts = dataclasses.replace(opts, stop_after="close")
            if kind == "pick":
                result = p.plan_pick(scene, obj_name, T_tcp, ctx, opts)
            elif kind == "place":
                result = p.plan_place(scene, obj_name, T_tcp_obj, T_tcp, ctx, opts,
                                      target=target, point=point)
            elif kind == "lift":
                result = p.plan_lift(scene, obj_name if obj is not None else None, T_tcp_obj,
                                     ctx, opts)
            elif kind == "release":
                result = p.plan_release(scene, obj_name, ctx, opts)
            elif kind == "move":
                result = p.plan_move(scene, carried, None if carried is None else T_tcp_obj,
                                     T_tcp, ctx, opts)
            else:
                result = p.plan_push(scene, start, end, ctx, opts)
            result.kind = kind
            self._tag(result, snap.ee)
            if joins:
                info = result.extra.setdefault("planner", {})
                note = JOIN_NOTE.format(name=obj_name, holders=", ".join(snap.others[1]))
                info["message"] = "; ".join(m for m in (info.get("message"), note) if m)
            preview = result.to_json()
            preview.setdefault("segments", [])
            preview.setdefault("planner", {})
            geom = None
            if kind == "pick":
                extra = getattr(result, "extra", None) or {}
                used = extra.get("tcp_used") if extra.get("variant") else None
                T_grasp = T_tcp if used is None else scene_mod.pose_from7(used, "tcp_used")
                geom = held_geometry(obj_name, obj, obj.pose, T_grasp)
            entry = {"result": result, "seq": snap.seq, "kind": kind, "object": obj_name,
                     "z0": None if obj is None else float(obj.pose[2, 3]), "request": request,
                     "segments": preview["segments"], "planner": preview["planner"]}
            if kind == "pick" and opts.stop_after == "lift_test":
                entry["rise_m"] = 0.6 * float(opts.lift_test_m or 0.015)
            if kind == "pick" and opts.stop_after == "close":
                entry["closed_only"] = True
            if geom is not None:
                entry["geom"] = geom
            with srv.cond:
                plan_id = self._next_plan_id
                self._next_plan_id += 1
                self._plans[plan_id] = entry
                while len(self._plans) > PLAN_CACHE_SIZE:
                    self._plans.popitem(last=False)
            srv.trace("server.plan", {"plan_id": plan_id, "kind": kind, "object": obj_name,
                                      "ok": bool(result.ok), "n_steps": len(result.actions)})
            preview["plan_id"] = plan_id
            return preview

        return self._run([name, target], compute, 0, ee)

    def execute_assist(self, body) -> dict:
        """POST api/assist/execute: start a cached assist plan made from the held observation."""
        srv = self._srv
        plan_id = body.get("plan_id")
        if isinstance(plan_id, bool) or not isinstance(plan_id, int):
            raise ValueError(f"plan_id must be an integer, got {plan_id!r}")  # noqa: TRY004

        def refuse(message):
            raise SessionConflict(message, {"ok": False, "chunk_id": None, "message": message,
                                            "preview": None})

        with srv.cond:
            if srv.episode.publishing is not None:
                refuse(PUBLISHING)
            entry = self._plans.get(plan_id)
        if entry is None:
            refuse(f"unknown plan_id {plan_id} (expired or from another episode); plan again")

        def make(ctx):
            if ctx["seq"] != entry["seq"]:
                refuse(MOVED_SINCE_PLAN)
            if entry["kind"] != "release":
                why = self._moves_co_held(entry["result"])
                if why is not None:
                    refuse(why)
            return entry["result"]

        assist = {"object": entry["object"], "z0": entry["z0"]}
        if "rise_m" in entry:
            assist["rise_m"] = entry["rise_m"]
        if entry.get("closed_only"):
            assist["closed_only"] = True
        if "geom" in entry:
            assist["geom"] = entry["geom"]
            with srv.cond:
                assist["sig0"] = srv.perception.pre_pick_sig(entry["object"])
        reply = srv.executor.start(entry["kind"], {"plan_id": plan_id,
                                                   "request": entry["request"]},
                                   make, assist=assist,
                                   select=None if entry["kind"] == "lift" else
                                   [self.ee_of(entry["request"]) or self.primary()])
        if reply.get("preview") is not None:
            reply["preview"].setdefault("segments", entry["segments"])
            reply["preview"].setdefault("planner", entry["planner"])
            reply["preview"]["plan_id"] = plan_id
        return reply

    def _new_plan(self, snap, specs) -> AssistPlan:
        objs = [StepSpec.from_json(s) if isinstance(s, dict) else s for s in specs]
        return AssistPlan(self.planner, snap.scene, snap.ctx, objs)

    def _bring_current(self, plan, scene, ctx) -> None:
        """Under the assist lock: drop a completed first step, then re-plan from (scene, ctx)."""
        with self._srv.cond:
            pop, self._pop_due = self._pop_due, None
        if pop is not None and plan.steps and plan.steps[0].spec.id == pop:
            plan.pop_first()
        plan.rebase(scene, ctx)

    def _publish(self, plan, snap, token) -> bool:
        """Cache the plan's JSON, preview and specs for readers that must not touch the plan."""
        srv = self._srv
        j = jsonable(plan.to_json())
        self._mark_steps(j, snap.scene)
        try:
            pv = plan.preview(snap.ctx.dt)
        except ValueError as exc:
            srv.log("warn", f"no combined preview: {exc}")
            pv = None
        pv = None if pv is None else jsonable(pv)
        specs = [jsonable(st.spec.to_json()) for st in plan.steps]
        with srv.cond:
            if token != srv.episode.token:
                return False
            self.rev += 1
            if pv is not None:
                pv["plan_id"] = self.rev
            self.plan, self._json, self._preview = plan, j, pv
            self._plan_ee = snap.ee
            self._base = (snap.seq, snap.scene.version, snap.planner_rev)
            if specs != self._specs:
                self._specs = specs
                srv.episode.plan_changed()
            ids = [s["id"] for s in specs if isinstance(s.get("id"), int)]
            self._next_step_id = max([self._next_step_id, *(i + 1 for i in ids)])
            srv.cond.notify_all()
            return True

    def _mark_steps(self, plan_json, scene) -> None:
        """Say in a step's message when perception is why it cannot be planned."""
        why = self._srv.world.held_reason
        for st in plan_json.get("steps") or ():
            gone = [x for x in (st.get("object"), st.get("target"))
                    if isinstance(x, str) and x not in scene.objects]
            if gone and not st.get("ok"):
                st["message"] = (f"needs re-selection: {', '.join(gone)} "
                                 f"{'is' if len(gone) == 1 else 'are'} no longer perceived")
                st["needs_reselection"] = True
            elif (not st.get("ok") and "holds nothing here" in (st.get("message") or "")
                  and why and why.startswith("not held")):
                st["message"] = f"{st['message']} ({why})"

    def steps_reply(self) -> dict:
        """Under the lock: the plan JSON."""
        steps = ((self._json or {}).get("steps") or []) if self.plan is not None else []
        ex = self.exec
        out = {"ok": True, "version": self.rev,
               "valid": self.plan is not None and bool(steps) and all(s.get("ok")
                                                                      for s in steps),
               "steps": steps, "preview": self._preview if self.plan is not None else None,
               "draft": self._srv.episode.draft.json(), "stale": self.stale(),
               "replanning": self.replanning, "executing": ex is not None,
               "step": None if ex is None else ex["step_id"]}
        if self.scoped():
            out["ee"] = self.plan_ee()
        return out

    def state_json(self) -> dict:
        r = self.steps_reply()
        return {"version": r["version"], "n_steps": len(r["steps"]), "valid": r["valid"],
                "executing": r["executing"], "step": r["step"], "draft": r["draft"],
                "stale": r["stale"], "replanning": r["replanning"]}

    def steps(self) -> dict:
        with self._srv.cond:
            return self.steps_reply()

    def _edit(self, edit, names=(), ee=None) -> dict:
        """Bring the plan current, apply edit(plan) under the assist lock, publish and save; a
        step for another end effector than the plan's starts a new plan, if it has no steps."""
        srv = self._srv
        if not self.assist_lock.acquire(blocking=False):
            raise SessionConflict(ASSIST_BUSY)
        busy = False
        try:
            srv.perception.ensure()
            with srv.cond:
                if srv.episode.mode == "episode_over":
                    raise SessionConflict("the episode is over")
                if self.exec is not None:
                    raise SessionConflict(PLAN_EXECUTING)
                plan = self.plan
                if not self._owns(ee) and ee is not None:
                    if plan is not None and plan.steps:
                        raise SessionConflict(
                            f"the plan moves {self.plan_ee()}; execute or clear its steps "
                            f"before adding steps for {ee}")
                    plan = None
                snap = self.snapshot(names, ee if ee is not None else self.plan_ee())
                token, stale = srv.episode.token, self.stale()
                self.busy = busy = True
            try:
                if plan is None:
                    plan = self._new_plan(snap, ())
                elif stale:
                    self._bring_current(plan, snap.scene, snap.ctx)
                edit(plan)
            except (SessionConflict, AssistError):
                raise
            except (ValueError, LookupError) as exc:
                raise SessionConflict(str(exc)) from exc
            except Exception as exc:
                srv.log("error", f"assist plan failed: {exc!r}")
                raise AssistError(f"assist plan failed: {type(exc).__name__}: {exc}") from exc
            finally:
                if plan is not None:
                    self._publish(plan, snap, token)
        finally:
            if busy:
                self._busy(False)
            self.assist_lock.release()
        with srv.cond:
            srv.episode.save_draft()
            return self.steps_reply()

    @staticmethod
    def _index(plan, sid) -> int:
        for i, st in enumerate(plan.steps):
            if st.spec.id == sid:
                return i
        raise SessionConflict(f"no step {sid} in the plan")

    def steps_add(self, body) -> dict:
        """POST api/assist/steps/add: append a pick or place step anchored from a world TCP."""
        kind = body.get("kind")
        if kind not in ("pick", "place"):
            raise ValueError(f"kind must be 'pick' or 'place', got {kind!r}")
        name = opt.name_of(body, "object")
        T_tcp = scene_mod.pose_from7(body.get("tcp"), "tcp")
        target = point = None
        if kind == "place":
            target, point = opt.target_or_point(body, name, exactly_one=True)
        options = opt.validate(kind, body.get("options"))
        force = opt.force(body)
        ee = self.ee_of(body)
        if (why := self.no_gripper(ee)) is not None:
            raise SessionConflict(why)
        self.refuse_unmodelled_hand(ee)

        def edit(plan):
            if kind == "place" and not force and plan.state_at(len(plan.steps)).held is None:
                raise SessionConflict(NOTHING_HELD)
            anchor = plan.anchor_for(kind, name, T_tcp, len(plan.steps), target=target,
                                     point=point)
            with self._srv.cond:
                sid = self._next_step_id
                self._next_step_id += 1
            plan.add(StepSpec(id=sid, kind=kind, object=name, target=target, point=point,
                              anchor=anchor, options=options))

        return self._edit(edit, [name, target], ee)

    def steps_update(self, body) -> dict:
        """POST api/assist/steps/update: a new TCP, options and, for a place, target or point."""
        sid = step_id_of(body)
        tcp, raw = body.get("tcp"), body.get("options")
        T_tcp = None if tcp is None else scene_mod.pose_from7(tcp, "tcp")
        retarget = "target" in body or "point" in body
        if T_tcp is None and raw is None and not retarget:
            raise ValueError("update needs tcp, options, target or point")

        def edit(plan):
            i = self._index(plan, sid)
            st = plan.steps[i]
            spec = st.spec
            options = None if raw is None else {**spec.options, **opt.validate(spec.kind, raw)}
            target, point = spec.target, spec.point
            if retarget:
                if spec.kind != "place":
                    raise ValueError("only a place step has a target or point")
                target, point = opt.target_or_point(body, spec.object, exactly_one=True)
                with self._srv.cond:
                    sc = self._srv.world.scene
                    if target is not None and target not in sc.objects:
                        raise SessionConflict(f"unknown object {target!r}")
            T = T_tcp if T_tcp is not None else (st.tcp if retarget else None)
            if retarget and T is None:
                raise SessionConflict("the step has no pose to re-anchor; give tcp")
            anchor = None
            if T is not None:
                anchor = plan.anchor_for(spec.kind, spec.object, T, i, target=target,
                                         point=point)
            spec.target, spec.point = target, point
            plan.update(sid, anchor=anchor, options=options)

        return self._edit(edit)

    def steps_delete(self, body) -> dict:
        sid = step_id_of(body)

        def edit(plan):
            self._index(plan, sid)
            plan.delete(sid)

        return self._edit(edit)

    def steps_clear(self, body=None) -> dict:
        srv = self._srv
        with srv.cond:
            no_plan = self.plan is None
            if no_plan:
                if self.exec is not None:
                    raise SessionConflict(PLAN_EXECUTING)
                self.rev += 1
        if not no_plan:
            return self._edit(lambda plan: plan.clear())
        with srv.cond:
            srv.episode.save_draft()
            return self.steps_reply()

    def steps_execute(self, body=None) -> dict:
        """POST api/assist/steps/execute: run every step back to back, starting now."""
        srv = self._srv
        ep = srv.episode

        def refuse(message):
            raise SessionConflict(message, {"ok": False, "chunk_id": None, "message": message,
                                            "preview": None})

        if not self.assist_lock.acquire(blocking=False):
            with srv.cond:
                if self.plan is not None and self.plan.steps:
                    srv.executor.note_call([self.plan_ee()])
            refuse(ASSIST_BUSY)
        busy = False
        try:
            srv.perception.ensure()
            with srv.cond:
                if ep.publishing is not None:
                    refuse(PUBLISHING)
                if ep.abort is not None:
                    refuse("the episode is being abandoned")
                if self.plan is not None and self.plan.steps:
                    srv.executor.note_call([self.plan_ee()])
                if self.exec is not None:
                    refuse("the plan is already executing")
                if ep.mode != "awaiting_plan":
                    refuse(f"cannot start a chunk in mode {ep.mode!r}")
                if srv.executor.stale:
                    refuse("the robot moved since this observation; try again")
                plan, token = self.plan, ep.token
                if plan is None or not plan.steps:
                    refuse("the plan is empty")
                snap = self.snapshot(ee=self.plan_ee())
                stale = self.stale()
                self.busy = busy = True
            try:
                if stale:
                    self._guarded(lambda: self._bring_current(plan, snap.scene, snap.ctx),
                                  "assist plan")
            finally:
                self._publish(plan, snap, token)
            with srv.cond:
                if (token != ep.token or ep.mode != "awaiting_plan" or srv.executor.stale
                        or ep.abort is not None or srv.obs.frame_seq != snap.seq):
                    refuse(MOVED_SINCE_PLAN)
                steps = list(plan.steps)
                if not steps:
                    refuse("the plan is empty")
                for i, st in enumerate(steps):
                    if not st.ok:
                        refuse(f"step {i + 1} ({st.spec.kind} {st.spec.object}) is infeasible: "
                               f"{st.message or 'see its issues'}")
                if steps[0].result is None or not len(steps[0].result.actions):
                    refuse("the first step has no motion")
                if (why := self._step_co_held(steps[0])) is not None:
                    refuse(why)
                self.exec = {"phase": "running", "chunk_id": None, "step_id": None, "done": 0}
                chunk_id = self._start_step(self.exec, steps[0])
                plan_json = self.steps_reply()
        finally:
            if busy:
                self._busy(False)
            self.assist_lock.release()
        return {"ok": True, "chunk_id": chunk_id, "plan": plan_json,
                "preview": jsonable(steps[0].result.to_json()),
                "message": f"executing step 1 of {len(steps)}"}

    def _start_step(self, ex, st) -> int:
        """Under the lock: start plan step st as a pick/place chunk with held tracking."""
        srv = self._srv
        spec, result = st.spec, st.result
        name = spec.object
        z0 = srv.gt_z(name)
        sc = srv.world.scene
        if z0 is None and sc is not None and name in sc.objects:
            z0 = float(sc.objects[name].pose[2, 3])
        plan = jsonable({"assist_step": spec.to_json(), "plan_version": self.rev})
        assist = {"object": name, "z0": z0}
        if spec.kind == "pick":
            base = self.plan.scene if self.plan is not None else sc
            obj = base.objects.get(name) if base is not None else None
            before = st.before.poses.get(name) if st.before is not None else None
            if obj is not None:
                assist["geom"] = held_geometry(name, obj, obj.pose if before is None else before,
                                               st.tcp)
            assist["sig0"] = srv.perception.pre_pick_sig(name)
        chunk_id = srv.executor.begin_chunk(spec.kind, plan, result, assist, step_id=spec.id,
                                            ee=self.plan_ee())
        ex.update(phase="running", chunk_id=chunk_id, step_id=spec.id)
        srv.log("info", f"chunk {chunk_id} ({spec.kind} {name}) started: plan step {spec.id}, "
                f"{len(result.actions)} steps")
        return chunk_id

    def on_chunk_done(self, ch, status) -> None:
        """Under the lock, from the executor: a completed plan step leaves the plan next."""
        if ch.step_id is not None and status == "done":
            self._pop_due = ch.step_id
        ex = self.exec
        if ex is not None and ex.get("chunk_id") == ch.chunk_id:
            if status == "done":
                ex["phase"] = "replan"
                ex["done"] += 1
            else:
                self.exec = None

    def cancel_execution(self):
        """Under the lock: stop a multi-step execution (its steps stay); what was running."""
        ex, self.exec = self.exec, None
        return ex

    def executing(self) -> bool:
        return self.exec is not None

    def holding_turn(self) -> bool:
        """Under the lock: the server, not the agent, acts next although no chunk runs."""
        ex = self.exec
        return ex is not None and ex["phase"] in ("replan", "planning")

    def release_turn(self) -> None:
        srv = self._srv
        ep = srv.episode
        if (ep.mode == "executing" and srv.executor.chunk is None and srv.obs.obs is not None
                and not srv.executor.stale and not self.holding_turn()):
            ep.set_mode("awaiting_plan")
        srv.cond.notify_all()

    def _due_job(self):
        srv = self._srv
        ep = srv.episode
        if (srv.obs.obs is None or srv.executor.stale or srv.executor.chunk is not None
                or ep.mode == "episode_over" or ep.abort is not None
                or ep.publishing is not None):
            return None
        ex = self.exec
        if ex is not None:
            return "continue" if ex["phase"] == "replan" else None
        if ep.mode == "awaiting_plan" and self.stale():
            return "rebase"
        return None

    def kick(self) -> None:
        """Under the lock: start the plan worker if a job is due and it is not running."""
        if self._worker is not None or self._due_job() is None:
            return
        self._worker = threading.Thread(target=self._work, name="amp-plan", daemon=True)
        self._worker.start()

    def _work(self) -> None:
        srv = self._srv
        try:
            for _ in range(WORKER_ROUNDS):
                with srv.cond:
                    job, token = self._due_job(), srv.episode.token
                    if job is None:
                        self._worker = None
                        return
                with self.assist_lock:
                    self._run_job(job, token)
                with srv.cond:
                    srv.episode.save_draft()
            srv.log("warn", "the plan worker kept finding work; pausing it")
        except Exception as exc:  # noqa: BLE001
            srv.log("error", f"the plan worker failed: {exc!r}")
        with srv.cond:
            self._worker = None
            self.replanning = False
            self.exec = None
            self.release_turn()

    def _run_job(self, job, token) -> None:
        """Under the assist lock: continue an execution, or re-base the plan."""
        srv = self._srv
        srv.perception.ensure()
        with srv.cond:
            if token != srv.episode.token or self._due_job() != job:
                return
            ex = self.exec
            try:
                snap = self.snapshot(ee=self.plan_ee())
            except SessionConflict as exc:
                if job == "continue":
                    self.exec = None
                    srv.log("warn", f"plan execution stopped: {exc}")
                self._base = (srv.obs.frame_seq, None, self.planner_rev)
                self.release_turn()
                return
            plan = self.plan
            if ex is not None:
                ex["phase"] = "planning"
            self.replanning = True
        failure = None
        try:
            if plan is None:
                plan = self._new_plan(snap, ())
            else:
                self._bring_current(plan, snap.scene, snap.ctx)
        except Exception as exc:  # noqa: BLE001
            failure = f"{type(exc).__name__}: {exc}"
            srv.log("error", f"planning the assist plan failed: {failure}")
        published = False
        if plan is not None:
            try:
                published = self._publish(plan, snap, token)
            except Exception as exc:  # noqa: BLE001
                failure = failure or f"{type(exc).__name__}: {exc}"
                srv.log("error", f"publishing the assist plan failed: {failure}")
        with srv.cond:
            self.replanning = False
            if token != srv.episode.token:
                return
            if not published:
                self._base = (snap.seq, snap.scene.version, snap.planner_rev)
            if job == "continue":
                self._continue(ex, plan, snap, failure)
            self.release_turn()

    def _continue(self, ex, plan, snap, failure) -> None:
        """Under the lock, after re-planning: start the next step, or hand the turn back."""
        srv = self._srv
        if self.exec is not ex:
            return
        steps = plan.steps
        message = None
        if failure is not None:
            message = f"re-planning failed: {failure}"
        elif not steps:
            self.exec = None
            srv.log("info", f"plan executed: {ex['done']} step(s)")
            return
        elif not steps[0].ok:
            s = steps[0]
            message = (f"step {s.spec.id} ({s.spec.kind} {s.spec.object}) is infeasible from the "
                       f"observed state: {s.message or 'see its issues'}")
        elif (srv.obs.frame_seq != snap.seq or srv.executor.stale
              or srv.episode.mode == "episode_over" or srv.episode.abort is not None):
            message = "the robot moved while the next step was planned"
        elif steps[0].result is None or not len(steps[0].result.actions):
            message = f"step {steps[0].spec.id} has no motion"
        elif (why := self._step_co_held(steps[0])) is not None:
            message = why
        if message is not None:
            self.exec = None
            srv.log("warn", f"plan execution stopped after {ex['done']} step(s): {message}; "
                    f"{len(steps)} step(s) remain in the plan")
            return
        self._start_step(ex, steps[0])

    def _narrow(self, ctx, ee) -> dict:
        """ctx (Executor.plan_context) for one end effector of several: its arm's q and hold_q,
        its gripper, tool and view (the other arms held at hold_q); ctx itself for one ee."""
        if not self.scoped():
            return ctx
        sc = self._scope(ee, ctx["hold_q"])
        with self._srv.cond:
            tool = self._srv.obs.tool_copy(sc.ee)
        return {**ctx, "ee": sc.ee, "q": sc.part(ctx["q"]), "hold_q": sc.part(ctx["hold_q"]),
                "gripper": int(ctx["grippers"].get(sc.ee, 0)), "tool": tool,
                "embodiment": sc.view}

    def _plan_emb(self, ctx):
        return ctx["embodiment"] if "embodiment" in ctx else self.emb()

    @staticmethod
    def _tag(res, ee):
        """Name the end effector a result is for: the executor composes its rows for that ee."""
        if ee is not None:
            res.ee = ee
            res.extra = {**res.extra, "ee": ee}
        return res

    def manual(self, plan_json, ctx) -> trajectory.PlanResult:
        """A waypoint plan from ctx, with the held object's contacts along it as errors."""
        srv = self._srv
        if isinstance(plan_json, dict) and "ee_waypoints" in plan_json:
            return self.manual_multi(plan_json, ctx)
        if self._joints_body(plan_json):
            return self.manual_joints(plan_json, ctx)
        ee = self.ee_of(plan_json)
        plan_json = self._tool_waypoints(plan_json, ee)
        waypoints, interp, speed_scale = trajectory.parse_plan(plan_json)
        table_contact = self._table_contact(plan_json, ee, waypoints)
        c = self._narrow(ctx, ee)
        res = trajectory.plan_chunk(
            c["q"], c["gripper"], c["tool"], waypoints, c["dt"], interp=interp or "minjerk",
            speed_scale=speed_scale if speed_scale is not None else SPEED_SCALE,
            table_z=srv.obs.table_z, check_table=srv.settings.table_obstacle,
            settle_s=SETTLE_S, embodiment=self._plan_emb(c),
            table_contact=table_contact)
        res = self._check_peers(self._check_held_path(res, c), c, ctx)
        return self._check_co_held(self._tag(res, c.get("ee")))

    def _joints_body(self, plan_json) -> bool:
        """A {"joints": ...} plan, which only a robot with joint groups takes; every other robot
        reads the body exactly as it always has."""
        return bool(self._srv.embodiment.joint_groups) and isinstance(plan_json, dict) \
            and "joints" in plan_json

    def manual_joints(self, plan_json, ctx) -> trajectory.PlanResult:
        """{"joints": {"group": arm | joint group, "q"}, "duration"}: a joint-space move of one
        arm (the rest held) or of a joint group such as a torso (every arm held, carried along)."""
        srv = self._srv
        lay = srv.executor.layout
        kind, name, q, d, speed = mobile.parse_joints(plan_json, srv.embodiment, lay)
        speed = SPEED_SCALE if speed is None else speed
        if kind == "group":
            return mobile.group_plan(srv, ctx, lay, name, q, d, ctx["dt"], speed,
                                     SETTLE_S, self.primary(), self._group_world)
        ee = next(e for e in lay.ees if lay.ee_arm[e] == name)
        c = self._narrow(ctx, ee)
        g = float(c["gripper"])
        res = trajectory.plan_home(c["q"], c["tool"], d, c["dt"], q_goal=q, speed_scale=speed,
                                   settle_s=SETTLE_S, embodiment=self._plan_emb(c),
                                   check_table=srv.settings.table_obstacle)
        res.actions[:, -1] = g
        res = dataclasses.replace(res, kind="joints", gripper=np.full(len(res.gripper), g),
                                  start_gripper=g,
                                  extra={**res.extra, "joints": {"group": name, "q": vec(q)}})
        res = self._check_peers(self._check_held_path(res, c), c, ctx)
        return self._check_co_held(self._tag(res, c.get("ee")))

    def _group_world(self, clone, ee, q_arm) -> checks.CheckWorld:
        """ee's arm on the runtime clone (a joint group moved) against the episode's obstacles."""
        srv = self._srv
        table = srv.obstacles() == "table"
        with srv.cond:
            table_z = srv.table_z()
            sc = srv.world.scene
            scene = sc.copy() if sc is not None and not table else None
            held = None if scene is None else srv.world.held_by(ee)
            try:
                T_tcp = srv.obs.observed_tcp(ee)
            except KeyError:
                T_tcp = None
        held = held if scene is not None and held in scene.objects else None
        tool = clone.tool(ee)
        return checks.check_world(
            srv.obstacles(), scene, table_z=table_z, tool=tool, q_obs=q_arm,
            T_world_tcp=tool.fk_tcp(q_arm) if T_tcp is None else T_tcp,
            margin=self.planner.margin, held=held, embodiment=clone.view(ee),
            table=srv.settings.table_obstacle)

    def move_base(self, body) -> dict:
        """motion/base: move the base by (dx, dy, dyaw) in its own frame, once its swept footprint
        is free in the occupancy perceived from the current frames (unknown space is blocked)."""
        srv = self._srv
        emb = srv.embodiment
        lay = srv.executor.layout
        if emb.spec.base is None or lay.base is None:
            raise ValueError(f"{emb.name} has no mobile base to move")
        delta = mobile.parse_delta(body)
        with srv.cond:
            down = getattr(srv.observations_live(), "contacts_unavailable", False)
        if down:
            srv.log("error", f"motion/base refused: {BASE_UNAVAILABLE}")
            raise SessionConflict(BASE_UNAVAILABLE)

        def make(ctx):
            if ctx.get("T_world_base") is None:
                raise SessionConflict("the observation carries no T_world_base: the base pose is "
                                      "unknown")
            with srv.cond:
                obs = srv.observations_live()
                frames, q = dict(obs.cameras), np.asarray(obs.q, dtype=np.float64).copy()
                flange, token, seq = srv.obs.flange_ee(), srv.episode.token, srv.obs.frame_seq
                Tb = obs.T_world_base
            if Tb is None:
                raise SessionConflict("the observation carries no T_world_base: the base pose is "
                                      "unknown")
            free, why, info = mobile.check_base(emb, frames, q, flange,
                                                np.array(Tb, dtype=np.float64), delta,
                                                srv.obs.base_memory, token=token, stamp=seq)
            c = self._narrow(ctx, self.primary())
            res = mobile.base_plan(c, ctx, lay, emb.spec, delta, SPEED_SCALE,
                                   self._plan_emb(c), None if free else mobile.blocked(why),
                                   T_world_base=Tb)
            res.extra["base"]["check"] = {"free": free, "reason": why, **info}
            return self._tag(res, c.get("ee"))

        return srv.executor.start("base", {"base": vec(delta)}, make)

    def _tool_waypoints(self, plan_json, ee):
        """plan_json with a tool's waypoints that leave out "gripper" at 0 (it has none)."""
        wps = plan_json.get("waypoints") if isinstance(plan_json, dict) else None
        if self.no_gripper(ee) is None or not isinstance(wps, list):
            return plan_json
        return {**plan_json, "waypoints": [{"gripper": 0, **w} if isinstance(w, dict) else w
                                           for w in wps]}

    def _closes_no_tool(self, ee, waypoints, where="") -> None:
        why = self.no_gripper(ee, "its waypoints' gripper must be 0")
        if why is not None and any(w.gripper > 0.5 for w in waypoints):
            raise ValueError(f"{where}{why}")

    def _table_contact(self, plan_json, ee, waypoints) -> bool:
        """The plan's opt-in table_contact (a tool pressed on the table, e.g. a wiper)."""
        self._closes_no_tool(ee, waypoints)
        tc = plan_json.get("table_contact", False)
        if not isinstance(tc, bool):
            raise ValueError(f"table_contact must be true or false, got {tc!r}")  # noqa: TRY004
        return tc

    def parse_multi(self, plan_json) -> tuple[list[ArmPlan], str, float | None, str]:
        """(arm plans, interp, speed_scale, schedule) of {"ee_waypoints": {ee: [...]}, ...}."""
        if not self.scoped():
            raise ValueError("ee_waypoints is for a robot with several end effectors; send "
                             "waypoints")
        if "waypoints" in plan_json or "ee" in plan_json:
            raise ValueError("a plan takes waypoints (and an optional ee) or ee_waypoints, not both")
        raw = plan_json.get("ee_waypoints")
        if not isinstance(raw, dict) or not raw:
            raise ValueError("ee_waypoints must be {ee: [waypoint, ...]} naming at least one "
                             f"end effector, got {raw!r}")
        if plan_json.get("table_contact"):
            raise ValueError("table_contact is for one end effector's waypoints, not ee_waypoints")
        schedule = plan_json.get("schedule", "auto")
        if schedule not in SCHEDULES:
            raise ValueError(f"schedule must be one of {list(SCHEDULES)}, got {schedule!r}")
        common = {k: plan_json[k] for k in ("interp", "speed_scale") if k in plan_json}
        plans, interp, speed_scale = [], plan_json.get("interp", "minjerk"), None
        for key, wps in raw.items():
            ee = self.ee_name(key)
            if any(p.ee == ee for p in plans):
                raise ValueError(f"ee_waypoints names {ee} twice")
            if wps == []:
                plans.append(ArmPlan(ee=ee))
                continue
            try:
                waypoints, interp, speed_scale = trajectory.parse_plan(
                    self._tool_waypoints({**common, "waypoints": wps}, ee))
            except ValueError as exc:
                raise ValueError(f"ee_waypoints[{key}]: {exc}") from None
            self._closes_no_tool(ee, waypoints, f"ee_waypoints[{key}]: ")
            plans.append(ArmPlan(ee=ee, waypoints=waypoints))
        if all(not p.waypoints for p in plans):
            raise ValueError("ee_waypoints needs waypoints for at least one end effector")
        return plans, interp, speed_scale, schedule

    def manual_multi(self, plan_json, ctx) -> trajectory.PlanResult:
        """{"ee_waypoints": {ee: [...]}, "interp", "speed_scale", "schedule"}: one chunk moving
        several end effectors from their held targets, composed into the action layout."""
        srv = self._srv
        plans, interp, speed_scale, schedule = self.parse_multi(plan_json)
        lay = srv.executor.layout
        with srv.cond:
            tools = {e: srv.obs.tool_copy(e) for e in lay.ees}
        res = plan_chunk_multi(
            ctx["hold_q"], {e: ctx["grippers"].get(e, 0) for e in lay.ees}, tools, plans,
            ctx["dt"], interp=interp or "minjerk",
            speed_scale=speed_scale if speed_scale is not None else SPEED_SCALE,
            table_z=srv.obs.table_z, check_table=srv.settings.table_obstacle,
            settle_s=SETTLE_S, embodiment=srv.embodiment, schedule=schedule)
        rows = {}
        for ee, arm_res in res.arms.items():
            rows.setdefault(lay.ee_arm[ee], arm_res.q)
        for ee, arm_res in res.arms.items():
            c = self._narrow(ctx, ee)
            moving = {a: r for a, r in rows.items() if a != lay.ee_arm[ee]}
            issues = self._held_issues(arm_res, {**c, "q": c["hold_q"]}, moving)
            if issues:
                res.issues = sorted(
                    [*res.issues, *(dataclasses.replace(i, message=f"{ee}: {i.message}")
                                    for i in issues)],
                    key=lambda i: (i.step, i.severity != "error"))
                res.ok = False
        self._check_rigid_grip(res, plans)
        res.actions = lay.compose_rows(res.q_full, res.grip_by_ee)
        res.composed = True
        return res

    def _grip_reference(self, held, holders, res) -> dict:
        """Under the lock: each co-holder's planned TCP when the grip in held was taken."""
        ref = self._grip_ref
        if ref is None or ref[0] is not held or not set(holders) <= set(ref[1]):
            ref = self._grip_ref = (held, {e: np.asarray(res.arms[e].start_tcp, np.float64)
                                           for e in holders})
        return ref[1]

    def _check_rigid_grip(self, res, plans) -> None:
        """A co_held error if a coordinated plan moves some co-holders of an object and not others,
        or changes the hands' relative pose from where they took the grip."""
        with self._srv.cond:
            world = self._srv.world
            name, holders, held = world.held_name, list(world.held_ees), world.held
        if name is None or len(holders) < 2:
            return
        who = _hands(holders)
        moving = {p.ee for p in plans if p.waypoints}
        idle = [e for e in holders if e not in moving]
        if idle and len(idle) < len(holders):
            self._add_issue(res, trajectory.Issue(0, float(res.t[0]), CO_HELD_KIND, "error",
                                                  CO_HELD_IDLE.format(name=name, holders=who,
                                                                      idle=", ".join(idle))))
            return
        if idle:
            return
        with self._srv.cond:
            grip = self._grip_reference(held, holders, res)
        lead = holders[0]
        for other in holders[1:]:
            d_p, d_a = grip_drift(res.tcp_planned_by_ee[lead], res.tcp_planned_by_ee[other],
                                  grip[lead], grip[other])
            bad = (d_p > CO_HELD_POS_TOL_M) | (d_a > CO_HELD_ANG_TOL_RAD)
            if not bad.any():
                continue
            k = int(np.argmax(np.where(bad, np.maximum(d_p / CO_HELD_POS_TOL_M,
                                                       d_a / CO_HELD_ANG_TOL_RAD), 0.0)))
            self._add_issue(res, trajectory.Issue(
                k, float(res.t[k]), CO_HELD_KIND, "error",
                CO_HELD_DRIFT.format(name=name, holders=who, mm=d_p[k] * 1e3,
                                     deg=math.degrees(d_a[k]), step=k,
                                     tol_mm=CO_HELD_POS_TOL_M * 1e3,
                                     tol_deg=math.degrees(CO_HELD_ANG_TOL_RAD))))
            return

    def _check_held_path(self, res, ctx):
        """Add the held object's contacts with the scene along a manual plan as error issues."""
        issues = self._held_issues(res, ctx)
        if issues:
            res.issues = sorted([*res.issues, *issues],
                                key=lambda i: (i.step, i.severity != "error"))
            res.ok = False
        return res

    @staticmethod
    def _add_issue(res, issue):
        res.issues = sorted([*res.issues, issue], key=lambda i: (i.step, i.severity != "error"))
        res.ok = False
        return res

    def _check_peers(self, res, c, ctx):
        """c's arm along res against the other arms held at hold_q (on a robot with several) and
        against the robot's own body (on a whole_body robot: base, torso)."""
        emb = self._srv.embodiment
        if not len(res.q) or ("ee" not in c and not emb.whole_body):
            return res
        arm = self._srv.executor.layout.ee_arm[c.get("ee") or self.primary()]
        if "ee" in c and len(emb.spec.arms) >= 2:
            for issue in peer_path_issues(emb, arm, res.start_q, res.q, ctx["hold_q"], res.t):
                self._add_issue(res, issue)
        for issue in body_path_issues(emb, arm, res.start_q, res.q, res.t):
            self._add_issue(res, issue)
        return res

    def _co_held_why(self, ee) -> str | None:
        """Why ee's arm may not move alone (its hand and another hold one object), else None."""
        if not self.scoped():
            return None
        srv = self._srv
        with srv.cond:
            ee = ee or self.primary()
            name, holders = srv.world.held_by(ee), list(srv.world.held_ees)
        if name is None or len(holders) < 2:
            return None
        return CO_HELD.format(name=name, holders=_hands(holders))

    def _moves_co_held(self, res, ee=None, until_open=True) -> str | None:
        """_co_held_why of res's ee (or ee), if res moves its arm before its gripper opens."""
        if getattr(res, "composed", False):
            return None
        why = self._co_held_why(ee or getattr(res, "ee", None))
        return None if why is None or _first_move(res, until_open) is None else why

    def _check_co_held(self, res, until_open=True):
        """res with a co_held error if it moves its arm while that hand co-holds an object."""
        why = self._moves_co_held(res, until_open=until_open)
        if why is None or any(i.kind == CO_HELD_KIND for i in res.issues):
            return res
        k = _first_move(res, until_open)
        return self._add_issue(res, trajectory.Issue(k, float(res.t[k]), CO_HELD_KIND, "error",
                                                     why))

    def _step_co_held(self, st) -> str | None:
        """Under the lock: why plan step st may not start (it moves a co-holding arm), else None."""
        if st.spec.kind == "release" or st.result is None:
            return None
        return self._moves_co_held(st.result, self.plan_ee())

    def _held_issues(self, res, ctx, peer_path=None) -> list:
        """The held object's contacts with the scene along a plan of ctx's end effector."""
        srv = self._srv
        if srv.obstacles() == "table":
            return []
        ee = ctx.get("ee")
        with srv.cond:
            held = srv.world.held_name if ee is None else srv.world.held_by(ee)
            lead = ee is None or srv.world.held_ees[:1] == [ee]
            sc = srv.world.scene
            scene = sc.copy() if sc is not None and held else None
            g = srv.world.held_now()
        if scene is None or held not in scene.objects:
            return []
        if g is not None and g.get("T_tcp_obj") is not None and lead:
            T_tcp_obj = np.asarray(g["T_tcp_obj"], dtype=np.float64)
        else:
            T_tcp_obj = np.linalg.inv(ctx["tool"].fk_tcp(ctx["q"])) @ scene.objects[held].pose
        n = len(res.t) - int(res.settle_steps)
        opened = np.flatnonzero(np.asarray(res.gripper[:n]) < 0.5)
        n = int(opened[0]) if len(opened) else n
        return held_contacts(scene, held, T_tcp_obj, ctx["q"], res.q[:max(n, 0)], ctx["tool"],
                             ctx["dt"], t=res.t, embodiment=self._plan_emb(ctx),
                             peer_path=peer_path)

    def _validate(self, plan_json) -> None:
        if isinstance(plan_json, dict) and "ee_waypoints" in plan_json:
            self.parse_multi(plan_json)
            return
        if self._joints_body(plan_json):
            mobile.parse_joints(plan_json, self._srv.embodiment, self._srv.executor.layout)
            return
        ee = self.ee_of(plan_json)
        if isinstance(plan_json, dict) and ("table_contact" in plan_json
                                            or self.no_gripper(ee) is not None):
            self._table_contact(plan_json, ee, trajectory.parse_plan(
                self._tool_waypoints(plan_json, ee))[0])

    def preview(self, plan_json) -> dict:
        srv = self._srv
        if self._intent(plan_json) == "select":
            return self._select_only(plan_json)
        self._validate(plan_json)
        with srv.cond:
            if self._intent(plan_json) == "move":
                srv.executor.note_call(self._plan_select(plan_json))
            ctx = srv.executor.plan_context()
        return self.manual(plan_json, ctx).to_json()

    def execute(self, plan_json) -> dict:
        self._validate(plan_json)
        return self._srv.executor.start("plan", plan_json, lambda ctx: self.manual(plan_json, ctx),
                                        select=self._plan_select(plan_json))

    @staticmethod
    def _intent(body) -> str | None:
        """A preview or assist plan's intent: "move" (the planning half of a move being made) or "select"."""
        return body.get("intent") if isinstance(body, dict) else None

    def _select_only(self, body) -> dict:
        """A move that failed before it was sent: select its arm, plan nothing (an assist lift selects none)."""
        ee = self.ee_of(body)
        if body.get("kind") == "lift":
            return {"ok": False, "issues": [], "message": "an assist lift selects no arm"}
        with self._srv.cond:
            self._srv.executor.note_call([ee or self.primary()])
        return {"ok": False, "issues": [], "message": "the arm is selected; nothing was planned"}

    def _plan_select(self, plan_json) -> list | None:
        """The end effectors a plan call selects: a hand move does, a joints plan does not."""
        if not self.scoped() or self._joints_body(plan_json):
            return None
        if isinstance(plan_json, dict) and "ee_waypoints" in plan_json:
            return [self.ee_name(k) for k, wps in plan_json["ee_waypoints"].items() if wps]
        return [self.ee_of(plan_json) or self.primary()]

    @staticmethod
    def _with_ee(plan, ee) -> dict:
        return plan if ee is None else {**plan, "ee": ee}

    def hold(self, duration, ee=None) -> dict:
        d = _finite(duration, "duration", 1e-6, MAX_DURATION_S)
        ee = self.ee_name(ee)

        def make(ctx):
            c = self._narrow(ctx, ee)
            return self._tag(trajectory.plan_hold(c["hold_q"], c["gripper"], c["tool"], d, c["dt"],
                                                  embodiment=self._plan_emb(c)), c.get("ee"))

        return self._srv.executor.start("hold", self._with_ee({"duration": d}, ee), make)

    def gripper(self, closed, duration, ee=None) -> dict:
        if not (isinstance(closed, bool) or (isinstance(closed, int) and closed in (0, 1))):
            raise ValueError("closed must be true/false or 0/1")
        d = _finite(duration, "duration", 1e-6, MAX_DURATION_S)
        ee = self.ee_name(ee, star=True)
        lay = self._srv.executor.layout
        if ee == "*":
            if not lay.gripper_ees:
                raise ValueError(f"no end effector of {self._srv.embodiment.name} has a gripper")
        elif (why := self.no_gripper(ee, "there is nothing to open or close")) is not None:
            raise ValueError(why)

        def make(ctx):
            if ee == "*" and len(lay.gripper_ees) > 1:
                return self.gripper_all(bool(closed), d, ctx)
            c = self._narrow(ctx, (lay.gripper_ees[0] if self.scoped() else None) if ee == "*"
                             else ee)
            return self._tag(trajectory.plan_gripper(c["hold_q"], bool(closed), c["tool"], d,
                                                     c["dt"], embodiment=self._plan_emb(c)),
                             c.get("ee"))

        return self._srv.executor.start(
            "gripper", self._with_ee({"closed": bool(closed), "duration": d}, ee), make,
            select=[lay.gripper_ees[-1]] if ee == "*" else [ee or self.primary()])

    def gripper_all(self, closed: bool, duration, ctx) -> trajectory.PlanResult:
        """Every gripper to closed in one chunk, every arm held: one composed result."""
        lay = self._srv.executor.layout
        parts = {}
        for e in lay.gripper_ees:
            c = self._narrow(ctx, e)
            parts[e] = trajectory.plan_gripper(c["hold_q"], closed, c["tool"], duration, c["dt"],
                                               embodiment=c["embodiment"])
        lead = lay.primary if lay.primary in parts else lay.gripper_ees[0]
        first = parts[lead]
        hold = np.asarray(ctx["hold_q"], dtype=np.float64)
        q_full = np.tile(hold, (len(first.actions), 1))
        res = dataclasses.replace(
            first, ok=all(r.ok for r in parts.values()),
            issues=[i for r in parts.values() for i in r.issues], q=q_full, start_q=hold.copy(),
            actions=lay.compose_rows(q_full, {e: r.gripper for e, r in parts.items()}),
            extra={**first.extra, "ee": "*"})
        res.composed = True
        res.ee = lead
        res.tcp_planned_by_ee = {e: r.tcp_planned for e, r in parts.items()}
        return res

    def home(self, duration, ee=None) -> dict:
        srv = self._srv
        d = _finite(duration, "duration", 1e-6, MAX_DURATION_S)
        ee = self.ee_name(ee)

        def make(ctx):
            c = self._narrow(ctx, ee)
            res = self._tag(trajectory.plan_home(
                c["q"], c["tool"], d, c["dt"], speed_scale=SPEED_SCALE,
                settle_s=SETTLE_S, embodiment=self._plan_emb(c),
                check_table=srv.settings.table_obstacle), c.get("ee"))
            return self._check_co_held(self._check_peers(res, c, ctx), until_open=False)

        return srv.executor.start("home", self._with_ee({"duration": d}, ee), make)
