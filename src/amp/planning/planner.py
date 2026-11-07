#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The planner interface and the scene planner: grasp, place and step plans on scene snapshots."""

from __future__ import annotations

import dataclasses
import hashlib
from dataclasses import dataclass
from typing import Any, Protocol

import numpy as np

from amp.contracts.embodiment import DROID
from amp.geometry.collision import CollisionWorld
from amp.geometry.embodiment import DHArm, Embodiment, EmbodimentView
from amp.geometry.kinematics import ToolModel, make_pose, pose7
from amp.link import wire
from amp.planning import clock as clock_mod
from amp.planning import grasp, jaw, motion, place, robot
from amp.planning.options import (
    LiftOptions,
    MoveOptions,
    PickOptions,
    PlaceOptions,
    PushOptions,
    ReleaseOptions,
)
from amp.planning.scene_geom import STROKE, GripperModel, gripper_for
from amp.planning.search import REPORT_RESERVE_S
from amp.planning.trajectory import PlanResult

_GRASP_IGNORE = (("gripper", "table"),)
_PLACE_POOL = 16
_PLANNABLE_POOL = 2
_REFUSALS = ("singular", "unreachable", "collision", "table")
_PRE_POSE_COLLIDES = "every IK solution of the pre-pose collides"
_UNCHECKED = "not confirmed by a plan_pick check: it may still refuse this grasp"
_STAGE_WHERE = {"transit": "pregrasp", "approach": "approach", "close": "grasp", "lift": "lift",
                "lift_test": "lift"}
ARM_INSIDE_M = 0.010
GRIPPER_STROKE_M = STROKE
PLANNERS = (("builtin", "Built-in"),)
BUILTIN_STATUS = {"available": True, "ready": True, "reason": None}


class _Infer:
    def __repr__(self) -> str:
        return "INFER"


INFER: Any = _Infer()


class PlannerUnavailable(RuntimeError):
    """The selected planner cannot be used here."""


@dataclass(frozen=True)
class PlanContext:
    """The robot state a plan starts from, and how to plan from it."""

    tool: ToolModel
    q: np.ndarray
    gripper: float = 0.0
    dt: float = 1.0 / 15.0
    settle_s: float | None = None
    arm_inside_q: np.ndarray | None = None
    T_world_tcp: np.ndarray | None = None
    held: Any = INFER
    embodiment: Embodiment | None = None
    proposals: str = "v1"

    def replace(self, **kw) -> PlanContext:
        return dataclasses.replace(self, **kw)


class Planner(Protocol):
    """What assist plans and servers call; ScenePlanner implements it."""

    margin: float

    def grasps(self, scene, name, ctx: PlanContext, *, max_candidates, target=None, point=None,
               near=None) -> list: ...

    def check_grasp(self, scene, name, T_tcp, ctx: PlanContext): ...

    def places(self, scene, held, T_tcp_obj, ctx: PlanContext, *, target, point, max_candidates,
               ) -> list: ...

    def check_place(self, scene, held, T_tcp_obj, T_tcp, ctx: PlanContext, *, target, point): ...

    def plan_pick(self, scene, name, T_grasp, ctx: PlanContext, opts: PickOptions
                  ) -> PlanResult: ...

    def plan_place(self, scene, held, T_tcp_obj, T_place, ctx: PlanContext, opts: PlaceOptions,
                   *, target, point) -> PlanResult: ...


def is_droid(embodiment: Embodiment | None) -> bool:
    """Whether an embodiment is DROID itself: its name, and the built-in Panda's kinematics."""
    if embodiment is None:
        return True
    return embodiment.name == DROID and isinstance(robot.plan_arm(embodiment), DHArm)


def _ee_name(embodiment) -> str | None:
    return None if embodiment is None else embodiment.ee().name


def _extra(opts, keys) -> dict:
    return {k: float(getattr(opts, k)) for k in keys if getattr(opts, k) is not None}


def _mesh_digest(o) -> str:
    """The object's mesh by content: an id() can be reused and an array changed in place."""
    h = hashlib.sha1(np.ascontiguousarray(o.verts).tobytes())  # nosec B324  # nosemgrep: python.lang.security.insecure-hash-algorithms.insecure-hash-algorithm-sha1
    h.update(b"|" + np.ascontiguousarray(o.faces).tobytes())
    return h.hexdigest()


class ScenePlanner:
    """Grasp and place search and step plans on scene snapshots."""

    def __init__(self, *, margin=0.01, time_budget_s=6.0, seed=0, planner="builtin",
                 gripper: GripperModel | None = None,
                 grasp_budget_s: float = grasp.GRASP_BUDGET_S,
                 grasp_variant_budget_s: float = grasp.GRASP_VARIANT_BUDGET_S,
                 place_budget_s: float = place.PLACE_BUDGET_S,
                 clock: clock_mod.Clock | None = None,
                 embodiment: Embodiment | None = None):
        self.margin = float(margin)
        self.time_budget_s = float(time_budget_s)
        self.seed = int(seed)
        self.embodiment = embodiment
        self.gripper_error = None
        if gripper is None:
            try:
                gripper = gripper_for(embodiment)
            except ValueError as exc:
                self.gripper_error = str(exc)
        self.gripper = gripper
        self.grasp_budget_s = float(grasp_budget_s)
        self.grasp_variant_budget_s = float(grasp_variant_budget_s)
        self.place_budget_s = float(place_budget_s)
        self.clock = clock
        self._inside_cache = None
        self.planner = "builtin"
        if planner != "builtin":
            self.set_planner(planner)

    def planner_status(self, name) -> dict:
        """{"available", "ready", "reason"} of a planner by name."""
        if name == "builtin":
            return dict(BUILTIN_STATUS)
        raise ValueError(f"unknown planner {name!r}")

    def planners(self) -> list[dict]:
        """Every selectable planner with its status."""
        return [{"name": n, "label": label, **self.planner_status(n)} for n, label in PLANNERS]

    def planner_state(self) -> dict:
        """The selected planner and whether it is ready."""
        st = BUILTIN_STATUS
        return {"name": self.planner, "ready": bool(st["ready"]), "reason": st["reason"]}

    def set_planner(self, name) -> None:
        """Select a planner by name."""
        names = [n for n, _ in PLANNERS]
        if name not in names:
            raise ValueError(f"assist_planner must be one of {names}, got {name!r}")
        self.planner = name

    @staticmethod
    def _tag(res, provider, note):
        info = res.extra.setdefault("planner", {})
        info["backend"] = "builtin" if provider is None else provider.name
        if note:
            info["message"] = "; ".join(m for m in (info.get("message"), note) if m)
        return res

    def _emb(self, ctx: PlanContext) -> Embodiment | None:
        """The embodiment a request plans for: its own or the planner's (they must agree)."""
        e = ctx.embodiment
        if e is None or e is self.embodiment:
            return self.embodiment
        if self.embodiment is not None and e.spec != self.embodiment.spec:
            raise ValueError(f"the plan is for {e.name}, the planner for {self.embodiment.name}")
        return e

    def _same_ee(self, emb) -> bool:
        """Whether emb plans the planner's own end effector (so its gripper model applies)."""
        if emb is None or emb is self.embodiment:
            return True
        return emb == self.embodiment and _ee_name(emb) == _ee_name(self.embodiment)

    def _grip(self, emb: Embodiment | None = None) -> GripperModel:
        """The gripper model for a request's embodiment (the planner's own when it is that)."""
        if not self._same_ee(emb):
            return gripper_for(emb)
        if self.gripper is None:
            raise ValueError(self.gripper_error or "no gripper model")
        return self.gripper

    def _need_gripper(self, ctx: PlanContext) -> None:
        """Refuse a plan that grasps, holds or releases with an end effector that has no fingers."""
        emb = self._emb(ctx)
        if emb is not None and emb.gripper() is None:
            raise ValueError(f"{emb.name}: end effector {emb.ee().name} has no gripper: "
                             "it cannot grasp")

    def _arm_ok(self, scene, tool, *, exclude=(), attached=None, ignore=(), inside=(),
                embodiment=None):
        world = CollisionWorld(scene, exclude=exclude, attached=attached, margin=self.margin,
                               embodiment=embodiment)
        ignore = (*ignore, *(("arm", n) for n in inside))
        return lambda q: world.check(q, tool, ignore=ignore).ok

    def arm_inside_at(self, scene, q, tool, skip=(), embodiment=None) -> dict:
        """{obstacle: metres} the arm links penetrate beyond ARM_INSIDE_M at q."""
        embodiment = self.embodiment if embodiment is None else embodiment
        names = [n for n in scene.objects if n not in skip]
        if q is None or not names:
            return {}
        q = np.asarray(q, dtype=np.float64).reshape(robot.check_tool(embodiment, tool).n)
        emb_key = _inside_key(embodiment)
        key = (q.tobytes(), np.asarray(tool.T_flange_ee).tobytes(), self.margin,
               emb_key, tuple((n, _mesh_digest(o), np.asarray(o.pose).tobytes())
                              for n, o in sorted(scene.objects.items()) if n in names))
        cached = self._inside_cache
        if cached is not None and cached[0] == key:
            return dict(cached[1])
        world = CollisionWorld(scene, exclude=set(skip), margin=self.margin,
                               embodiment=emb_key[0].view(emb_key[1]) if len(emb_key) == 2
                               else embodiment)
        out = {}
        for n in names:
            if n not in world.obstacle_names:
                continue
            others = [("*", m) for m in world.obstacle_names if m not in (n, "table")]
            clr, _, _ = world.evaluate(q[None], tool, table_links_ignored=("*",),
                                       ignore=[*others, ("gripper", n)])
            if float(clr[0]) < -ARM_INSIDE_M:
                out[n] = -float(clr[0])
        self._inside_cache = (key, out)
        return dict(out)

    def arm_inside(self, scene, ctx: PlanContext, *, skip=()) -> dict:
        """arm_inside_at ctx.arm_inside_q; {} when the guard is off."""
        return self.arm_inside_at(scene, ctx.arm_inside_q, ctx.tool, skip, self._emb(ctx))

    @staticmethod
    def inside_notes(inside) -> list[str]:
        return [f"ignored {n} for the arm check: the arm is already {d * 1000:.0f} mm inside it "
                "(perception error)" for n, d in sorted(inside.items())]

    @classmethod
    def _noted(cls, cands, inside):
        notes = cls.inside_notes(inside)
        for c in cands:
            if isinstance(getattr(c, "notes", None), list):
                c.notes.extend(n for n in notes if n not in c.notes)
        return cands

    def _without(self, scene, ctx, keep):
        """(scene minus the obstacles the arm is already inside, their depths); keep stays."""
        inside = self.arm_inside(scene, ctx, skip=[k for k in keep if k is not None])
        if not inside:
            return scene, inside
        s = scene.copy()
        for n in inside:
            del s.objects[n]
        return s, inside

    def _tag_inside(self, res, inside):
        if inside:
            info = res.extra.setdefault("planner", {})
            info["message"] = "; ".join(m for m in (info.get("message"),
                                                    *self.inside_notes(inside)) if m)
            res.extra["arm_inside"] = {n: round(d, 4) for n, d in sorted(inside.items())}
        return res

    @staticmethod
    def _place_ignore(target):
        support = target or "table"
        return (("gripper", support), ("held", support), ("gripper", "table"))

    def _canonical(self, ctx: PlanContext) -> tuple[PlanContext, np.ndarray | None]:
        """(ctx in the planner's x-closing TCP frame, Rc that frame in the hand's; None: as is)."""
        emb = self._emb(ctx)
        if emb is None or getattr(ctx.tool, "canonical", False):
            return ctx, None
        Rc = jaw.canonical_rotation(robot.resolve(emb).ee(), emb.name)
        if Rc is None:
            return ctx, None
        T = ctx.T_world_tcp
        return ctx.replace(tool=jaw.canonical_tool(ctx.tool, Rc),
                           T_world_tcp=None if T is None else np.asarray(T) @ Rc), Rc

    def grasps(self, scene, name, ctx: PlanContext, *, max_candidates, target=None, point=None,
               near=None) -> list:
        """Ranked grasp candidates; with the intended target or point, ones placeable there lead."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        cands = self._grasps(scene, name, ctx, max_candidates=max_candidates, target=target,
                             point=point, near=near)
        return cands if Rc is None else [_grasp_out(c, Rc) for c in cands]

    def _grasps(self, scene, name, ctx: PlanContext, *, max_candidates, target=None, point=None,
                near=None) -> list:
        self._need_gripper(ctx)
        tool, emb = ctx.tool, self._emb(ctx)
        ranked = target is not None or point is not None
        inside = self.arm_inside(scene, ctx, skip=[name])
        extra = {} if near is None else {"near": near}
        if emb is not None:
            extra["embodiment"] = emb
        v2 = ctx.proposals == "v2"
        pool = max_candidates
        if ranked:
            pool = max(max_candidates, _PLACE_POOL)
        elif v2:
            pool = _PLANNABLE_POOL * max_candidates
        with clock_mod.use(self.clock):
            cands = grasp.propose_grasps(
                scene, name, q_seed=ctx.q, tool=tool, max_candidates=pool,
                arm_ok=self._arm_ok(scene, tool, exclude={name}, ignore=_GRASP_IGNORE,
                                    inside=inside, embodiment=emb),
                seed=self.seed, gripper=self._grip(emb), time_budget_s=self.grasp_budget_s,
                variant_budget_s=self.grasp_variant_budget_s, proposals=ctx.proposals, **extra,
            )
        self._noted(cands, inside)
        if v2:
            want = max(1, int(max_candidates))
            cands = self._plannable_first(scene, name, cands, ctx, want)
            if not ranked:
                return cands[:want]
        if not ranked:
            return cands

        def has_place(c):
            T_tcp_obj = np.linalg.inv(c.tcp) @ np.asarray(scene.objects[name].pose)
            seeded = ctx.replace(q=c.q if c.q is not None else ctx.q)
            return any(p.ok for p in self.places(scene, name, T_tcp_obj, seeded, target=target,
                                                  point=point, max_candidates=1))

        return grasp.rank_for_place(cands, has_place, target or "the point")[:max_candidates]

    def _plannable_first(self, scene, name, cands, ctx: PlanContext, want: int) -> list:
        """cands with the ok ones plan_pick refuses marked not ok: confirmed ok ones first, then
        the ok ones with no verdict (noted), then the rest."""
        found, checked = 0, set()
        with clock_mod.use(self.clock):
            until = clock_mod.now() + max(self.grasp_budget_s, want * self.time_budget_s)
            for c in cands:
                if found >= want or clock_mod.now() > until:
                    break
                if not c.ok:
                    continue
                verdict = self._pick_verdict(scene, name, c.tcp, ctx)
                if verdict is not None:
                    checked.add(id(c))
                if verdict is True:
                    found += 1
                elif verdict is not None:
                    kind, where, message = verdict
                    if kind in ("singular", "unreachable"):
                        c.reachable = False
                    else:
                        c.collision_free = False
                    c.notes.append(f"would be refused: {kind} at the {where}: {message}")
        out = sorted(cands, key=lambda c: (not c.ok, id(c) not in checked))
        for c in out:
            if c.ok and id(c) not in checked:
                c.notes.append(_UNCHECKED)
        return out

    def _pick_verdict(self, scene, name, T_grasp, ctx: PlanContext):
        """plan_pick's verdict: True (plans), the refusing (kind, where, message), None (unsure)."""
        budget = self.time_budget_s
        t0 = clock_mod.now()
        res = self.plan_pick(scene, name, T_grasp, ctx, PickOptions())
        if res.ok:
            return True
        if clock_mod.now() - t0 >= budget - min(REPORT_RESERVE_S, 0.25 * budget):
            return None
        segments = res.extra.get("segments") or []
        for i in res.issues:
            if i.severity != "error":
                continue
            if i.kind == "no_path" and _PRE_POSE_COLLIDES in i.message:
                return "collision", "pregrasp", i.message
            if i.kind not in _REFUSALS:
                continue
            seg = next((s for s in segments if s["start"] <= i.step <= s["end"]), None)
            stage = None if seg is None else seg["name"]
            if stage == "transit" and i.kind in ("collision", "table") and i.step < seg["end"]:
                continue
            return i.kind, _STAGE_WHERE.get(stage, stage or "pick"), i.message
        return None

    def check_grasp(self, scene, name, T_tcp, ctx: PlanContext):
        """Exactly T_tcp evaluated as a grasp of name."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _grasp_out(self.check_grasp(scene, name, np.asarray(T_tcp) @ Rc, ctx), Rc)
        tool, emb = ctx.tool, self._emb(ctx)
        inside = self.arm_inside(scene, ctx, skip=[name])
        cand = grasp.evaluate_grasp(
            scene, name, T_tcp, q_seed=ctx.q, tool=tool,
            arm_ok=self._arm_ok(scene, tool, exclude={name}, ignore=_GRASP_IGNORE, inside=inside,
                                embodiment=emb),
            gripper=self._grip(emb), proposals=ctx.proposals,
            **({} if emb is None else {"embodiment": emb}),
        )
        return self._noted([cand], inside)[0]

    def places(self, scene, held, T_tcp_obj, ctx: PlanContext, *, target, point, max_candidates,
               ) -> list:
        """Ranked placements of the held object over target or at a table point."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return [_place_out(c, Rc) for c in self.places(
                scene, held, Rc.T @ np.asarray(T_tcp_obj), ctx, target=target, point=point,
                max_candidates=max_candidates)]
        tool, emb = ctx.tool, self._emb(ctx)
        inside = self.arm_inside(scene, ctx, skip=[held])
        with clock_mod.use(self.clock):
            cands = place.propose_places(
                scene, held, T_tcp_obj, target=target, point=point, q_seed=ctx.q, tool=tool,
                max_candidates=max_candidates,
                arm_ok=self._arm_ok(scene, tool, attached=(held, T_tcp_obj),
                                    ignore=self._place_ignore(target), inside=inside,
                                    embodiment=emb),
                seed=self.seed, time_budget_s=self.place_budget_s, proposals=ctx.proposals,
                **self._robot_kw(emb),
            )
        return self._noted(cands, inside)

    def check_place(self, scene, held, T_tcp_obj, T_tcp, ctx: PlanContext, *, target, point):
        """Exactly T_tcp evaluated as a release pose of the held object."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _place_out(self.check_place(
                scene, held, Rc.T @ np.asarray(T_tcp_obj), np.asarray(T_tcp) @ Rc, ctx,
                target=target, point=point), Rc)
        tool, emb = ctx.tool, self._emb(ctx)
        inside = self.arm_inside(scene, ctx, skip=[held])
        cand = place.evaluate_place(
            scene, held, T_tcp_obj, T_tcp, target=target, point=point, q_seed=ctx.q,
            tool=tool, arm_ok=self._arm_ok(scene, tool, attached=(held, T_tcp_obj),
                                           ignore=self._place_ignore(target), inside=inside,
                                           embodiment=emb),
            **self._robot_kw(emb),
        )
        return self._noted([cand], inside)[0]

    def _common(self, provider) -> dict:
        out = {"time_budget_s": self.time_budget_s, "seed": self.seed}
        if provider is not None:
            out["provider"] = provider
        return out

    def _robot_kw(self, emb, gripper: bool = True) -> dict:
        """The embodiment (and its gripper model) a non-DROID plan passes on; {} for DROID."""
        if emb is None:
            return {}
        return {"embodiment": emb, **({"gripper": self._grip(emb)} if gripper else {})}

    def plan_pick(self, scene, name, T_grasp, ctx: PlanContext, opts: PickOptions) -> PlanResult:
        """A collision-free pick (kind "pick")."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _plan_out(self.plan_pick(scene, name, np.asarray(T_grasp) @ Rc, ctx, opts), Rc)
        scene, inside = self._without(scene, ctx, [name])
        emb = self._emb(ctx)
        with clock_mod.use(self.clock):
            res = motion.plan_pick(
                scene, name, T_grasp, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                gripper=self._grip(emb), **self._robot_kw(emb, gripper=False),
                approach=opts.approach, lift=opts.lift, speed_scale=opts.speed_scale,
                settle_s=opts.settle_s, **_extra(opts, ("close_s", "lift_test_m")),
                **self._common(None),
                **({"stop_after": opts.stop_after} if opts.stop_after else {}),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)

    def plan_place(self, scene, held, T_tcp_obj, T_place, ctx: PlanContext, opts: PlaceOptions,
                   *, target, point) -> PlanResult:
        """A collision-free place (kind "place")."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _plan_out(self.plan_place(
                scene, held, Rc.T @ np.asarray(T_tcp_obj), np.asarray(T_place) @ Rc, ctx, opts,
                target=target, point=point), Rc)
        scene, inside = self._without(scene, ctx, [held, target])
        with clock_mod.use(self.clock):
            res = motion.plan_place(
                scene, held, T_tcp_obj, T_place, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                target=target, approach=opts.approach, retreat=opts.retreat,
                speed_scale=opts.speed_scale, settle_s=opts.settle_s,
                **_extra(opts, ("open_s",)), **self._common(None),
                **self._robot_kw(self._emb(ctx)),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)

    def plan_lift(self, scene, held, T_tcp_obj, ctx: PlanContext, opts: LiftOptions
                  ) -> PlanResult:
        """The held object straight up to its carry pose (kind "lift")."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _plan_out(self.plan_lift(scene, held, Rc.T @ np.asarray(T_tcp_obj), ctx, opts),
                             Rc)
        scene, inside = self._without(scene, ctx, [held])
        with clock_mod.use(self.clock):
            res = motion.plan_lift(
                scene, held, T_tcp_obj, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                height=opts.height_m, settle_s=opts.settle_s, **_extra(opts, ("speed_scale",)),
                **self._robot_kw(self._emb(ctx)),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)

    def plan_release(self, scene, held, ctx: PlanContext, opts: ReleaseOptions) -> PlanResult:
        """Open where the gripper is and back off (kind "release")."""
        self._need_gripper(ctx)
        scene, inside = self._without(scene, ctx, [held])
        with clock_mod.use(self.clock):
            res = motion.plan_release(
                scene, held, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                retreat=opts.retreat_m, settle_s=opts.settle_s, **_extra(opts, ("open_s",)),
                **self._robot_kw(self._emb(ctx), gripper=False),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)

    def plan_move(self, scene, held, T_tcp_obj, T_goal, ctx: PlanContext, opts: MoveOptions
                  ) -> PlanResult:
        """A collision-free transit to T_goal with the held object (kind "move")."""
        if held is not None:
            self._need_gripper(ctx)
        scene, inside = self._without(scene, ctx, [held])
        with clock_mod.use(self.clock):
            res = motion.plan_move(
                scene, held, T_tcp_obj, T_goal, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                speed_scale=opts.speed_scale, settle_s=opts.settle_s, **self._common(None),
                **self._robot_kw(self._emb(ctx), gripper=False),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)

    def plan_push(self, scene, start, end, ctx: PlanContext, opts: PushOptions) -> PlanResult:
        """A straight closed-finger push from start to end (kind "push")."""
        self._need_gripper(ctx)
        ctx, Rc = self._canonical(ctx)
        if Rc is not None:
            return _plan_out(self.plan_push(scene, start, end, ctx, opts), Rc)
        scene, inside = self._without(scene, ctx, [])
        with clock_mod.use(self.clock):
            res = motion.plan_push(
                scene, start, end, ctx.q, ctx.gripper, ctx.tool, ctx.dt,
                gripper=self._grip(self._emb(ctx)),
                **self._robot_kw(self._emb(ctx), gripper=False),
                height=opts.height_m, speed_scale=opts.speed_scale, settle_s=opts.settle_s,
                **_extra(opts, ("close_s",)), **self._common(None),
            )
        return self._tag_inside(self._tag(res, None, ""), inside)


def _grasp_out(c, Rc):
    """A grasp candidate found in the planner's TCP frame, in the end effector's."""
    return dataclasses.replace(c, tcp=c.tcp @ Rc.T, pregrasp=c.pregrasp @ Rc.T)


def _place_out(c, Rc):
    """A place candidate found in the planner's TCP frame, in the end effector's."""
    return dataclasses.replace(c, tcp=c.tcp @ Rc.T)


def _tcp_rows(P, Rc) -> np.ndarray:
    """pose7 rows of the planner's TCP frame in the hand's."""
    P = np.asarray(P, dtype=np.float64)
    if P.size == 0:
        return P
    rows = P.reshape(-1, 7)
    out = np.array([pose7(make_pose(r[:3], r[3:]) @ Rc.T) for r in rows])
    return out.reshape(P.shape)


def _plan_out(res, Rc):
    """A plan made in the planner's TCP frame, its TCP poses in the end effector's."""
    for f in ("tcp_planned", "tcp_fk", "start_tcp"):
        if getattr(res, f, None) is not None:
            setattr(res, f, _tcp_rows(getattr(res, f), Rc))
    if res.extra.get("tcp_used") is not None:
        res.extra["tcp_used"] = [float(x) for x in _tcp_rows(res.extra["tcp_used"], Rc)]
    return res


def _inside_key(embodiment) -> tuple:
    """The embodiment as arm_inside_at's cache key: a view by its base and ee (peers aside)."""
    if isinstance(embodiment, EmbodimentView):
        return (embodiment.base, embodiment.ee_name)
    return (embodiment,)


def feasible_first(cands, what, name, *, stroke_m: float = GRIPPER_STROKE_M
                   ) -> tuple[list[dict], str]:
    """Feasible candidates' JSON only, unless there are none; then all of them and why."""
    feasible = [c for c in cands if c.ok]
    if feasible:
        shown, message = feasible, ""
    elif cands:
        shown = cands
        why = [
            (sum(not getattr(c, "reachable", True) for c in cands), "unreachable"),
            (sum(not getattr(c, "collision_free", True) for c in cands), "in collision"),
            (sum((getattr(c, "width", None) or 0.0) > stroke_m for c in cands),
             f"wider than {stroke_m * 100:.1f} cm"),
        ]
        detail = ", ".join(f"{n} {text}" for n, text in why if n) or "none passed"
        message = f"no feasible {what} of {name} among {len(cands)} candidates: {detail}"
    else:
        shown, message = [], f"no {what} candidates found for {name}"
    return [wire.jsonable(c.to_json(i)) for i, c in enumerate(shown)], message
