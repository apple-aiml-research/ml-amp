#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Step plans: pick, place, lift, release, move and push trajectories, and held-object contacts."""

from __future__ import annotations

import math
import weakref
from dataclasses import dataclass

import numpy as np
from scipy.spatial import cKDTree

from amp.geometry.collision import CollisionWorld, RobotSpheres, mesh_field
from amp.geometry.embodiment import Embodiment
from amp.geometry.kinematics import pose7
from amp.planning import candidates, clock, grasp, place, scene_geom, trajectory
from amp.planning.robot import arm_of, check_tool
from amp.planning.scene_geom import GripperModel, gripper_for
from amp.planning.search import (
    ACCEL_MAX,
    CART_SPEED,
    IK_POS_OK,
    IK_ROT_OK,
    MARGIN,
    PREFER_CLEARANCE,
    REST_GAIN,
    STAGE_SPEED_SCALE,
    HeldMargin,
    Piece,
    Setup,
    Stage,
    Variant,
    collision_issues,
    contact_rule,
    gripper_reach,
    lift_target,
    make_rule,
    pieces_free,
    plan_search,
    run_stages,
    sliding_rules,
)

LIFT_SLOW_M = 0.015
LIFT_SLOW_SPEED = 0.02
LIFT_SPEED = 0.15
DESCEND_SPEED = 0.20
APPROACH_SPEED = 0.20
CONTACT_SLOW_M = 0.02
PLACE_SLOW_M = 0.025
CONTACT_SPEED = 0.05
RETREAT_SPEED = 0.20
PUSH_SPEED = 0.05
CARRY_SPEED_SCALE = 0.35
CARRY_ACCEL = 1.5
HEAVY_KG = 0.4
LONG_M = 0.20
TALL_M = 0.12
GENTLE_CARRY = 0.6
LIFT_TEST_M = 0.015
LIFT_CONTACT_M = 0.02
PLACE_CONTACT_M = 0.03
HELD_LIFT_MARGIN = 0.004
SHELF_OVERHANG_M = 0.15
SHELF_LIFT_M = 0.02
SHELF_GAP_M = 0.01
SHELF_COVER = 0.3
SURFACE_CELL_M = 0.01
FLIP_ALIKE_M = 0.005
FLIP_SAMPLES = 4000
ESCAPE_NEIGHBOUR_M = 0.06
ESCAPE_CORRIDOR_M = 0.15
ESCAPE_EXTRA_M = 0.005
ESCAPE_MIN_M = 0.005
ESCAPE_MAX_M = 0.20
ESCAPE_SPEED = CART_SPEED
LIFT_MAX_M = 0.30
GRIPPER_HALF_M = 0.09
_FLIP = np.diag([-1.0, -1.0, 1.0, 1.0])


def _pose(T, name) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{name} must be a finite 4x4 transform")
    return T


def _obj(scene, name, what):
    if name not in scene.objects:
        raise ValueError(f"unknown {what} {name!r}")
    return scene.objects[name]


def _check_args(q_start, dt, speed_scale, tool=None):
    n = 7 if tool is None else arm_of(tool).n
    q = np.asarray(q_start, dtype=np.float64).reshape(n)
    if not dt > 0:
        raise ValueError(f"dt must be > 0, got {dt}")
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError(f"speed_scale must be in (0, 1], got {speed_scale}")
    return q


def _world_verts(o, T=None) -> np.ndarray:
    T = np.asarray(o.pose if T is None else T, dtype=np.float64)
    return np.asarray(o.verts, dtype=np.float64) @ T[:3, :3].T + T[:3, 3]


_SURFACE: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()


def _world_surface(o) -> np.ndarray:
    """World points covering o's surface (its vertices alone miss the middle of big faces)."""
    field = mesh_field(o.verts, o.faces)
    if field is None:
        return _world_verts(o)
    pts = _SURFACE.get(field)
    if pts is None:
        cand = field.samples[:: max(1, len(field.samples) // 20_000)]
        _, idx = np.unique(np.floor(cand / SURFACE_CELL_M), axis=0, return_index=True)
        pts = _SURFACE[field] = np.vstack([cand[idx], field.verts])
    T = np.asarray(o.pose, dtype=np.float64)
    return pts @ T[:3, :3].T + T[:3, 3]


def _spheres(spheres, embodiment):
    """The sphere model to plan with: the given one, else the embodiment's (DROID's fitted)."""
    if spheres is not None:
        return spheres
    return RobotSpheres.load() if embodiment is None else embodiment.spheres()


def _clear_rise(scene, skip, pts, T_tcp, toward=None, goal=None, body=None) -> float:
    """Straight-up rise that puts pts (a held object) and the gripper at T_tcp PREFER_CLEARANCE
    above every object beside or under them (within ESCAPE_NEIGHBOUR_M of the object, or
    GRIPPER_HALF_M of the TCP) or, given toward (xy), in the first ESCAPE_CORRIDOR_M of the
    way there; the goal object there is only counted beside the start (the transit clears it)."""
    body = scene_geom.ROBOTIQ_BODY if body is None else body
    low = min(float(pts[:, 2].min()), scene_geom.lowest_point(T_tcp, body)[0])
    high = float(pts[:, 2].max())
    xy = pts[:: max(1, len(pts) // 400), :2]
    tree = cKDTree(xy)
    c = xy.mean(axis=0)
    half = float(np.linalg.norm(xy - c, axis=1).max()) + ESCAPE_NEIGHBOUR_M
    seg = np.zeros(2)
    if toward is not None:
        d = np.asarray(toward, dtype=np.float64)[:2] - c
        n = float(np.linalg.norm(d))
        seg = d / n * min(n, ESCAPE_CORRIDOR_M) if n > 1e-9 else seg
    top = -np.inf
    for name, o in scene.objects.items():
        if name in skip:
            continue
        w = _world_surface(o)
        if w[:, 2].min() >= high:
            continue
        r = ESCAPE_NEIGHBOUR_M
        near = tree.query(w[:, :2], distance_upper_bound=r)[0] <= r
        near |= np.linalg.norm(w[:, :2] - T_tcp[:2, 3], axis=1) <= GRIPPER_HALF_M
        if seg.any() and name != goal:
            t = np.clip((w[:, :2] - c) @ seg / (seg @ seg), 0.0, 1.0)
            near |= np.linalg.norm(w[:, :2] - c - t[:, None] * seg, axis=1) <= half
        if near.any():
            top = max(top, float(w[:, 2].max()))
    return max(0.0, top + PREFER_CLEARANCE + ESCAPE_EXTRA_M - low)


def _escape(scene, held, T_tcp_obj, q_start, tool, dt, speed_scale, carry, window, toward,
            goal=None, body=None):
    """The place's first piece: straight up from q_start until the held object clears its
    neighbours and those on the way toward (xy), or None when it already does (or cannot)."""
    T0 = tool.fk_tcp(q_start)
    pts = _world_verts(scene.objects[held], T0 @ T_tcp_obj)
    rise = min(_clear_rise(scene, {held}, pts, T0, toward, goal, body), ESCAPE_MAX_M)
    if rise < ESCAPE_MIN_M:
        return None

    lifted = HeldMargin(carry, HELD_LIFT_MARGIN)

    def rising(tcp, q, q0):
        return sliding_rules(lifted, q0, q, tcp[:, 2] - T0[2, 3], window, tool,
                              table=("gripper", "held"))

    for scale in (1.0, 0.75, 0.5):
        T1 = T0.copy()
        T1[2, 3] += scale * rise
        stage = Stage("escape", 1.0, rising, T=T1, v_peak=ESCAPE_SPEED,
                       slow=(LIFT_SLOW_M, LIFT_SLOW_SPEED))
        pieces, ok = run_stages([stage], q_start, T0, tool, dt, STAGE_SPEED_SCALE)
        if ok and pieces_free(pieces, tool)[0]:
            return pieces[0]
    return None


def _pick_lift(scene, target, obj, T_grasp, q_start, tool, lift, body=None) -> float:
    """At least lift, and up to LIFT_MAX_M so the held object clears its tallest neighbour."""
    want = min(_clear_rise(scene, {target}, _world_verts(obj), T_grasp, body=body), LIFT_MAX_M)
    if want <= lift:
        return float(lift)
    arm = arm_of(tool)
    q = arm.ik(T_grasp, q_start, tool.T_flange_tcp, max_iters=150).q
    for h in (want, 0.5 * (want + lift)):
        T = T_grasp.copy()
        T[2, 3] += h
        r = arm.ik(T, q, tool.T_flange_tcp, q_rest=q, rest_gain=REST_GAIN)
        if r.pos_err <= IK_POS_OK and r.rot_err <= IK_ROT_OK:
            return float(h)
    return float(lift)


@dataclass
class _ShelfExit:
    """Under an overhang: rise `up` off the support, slide `out` (a world vector) clear of it."""

    over: str
    up: float
    out: np.ndarray


def _shelf_exit(scene, target, pts) -> _ShelfExit | None:
    """How the held object at pts leaves from under the lowest overhang above it, or None when
    nothing within a lift and PREFER_CLEARANCE is above it."""
    top = float(pts[:, 2].max())
    lo, hi = pts[:, :2].min(0) - PREFER_CLEARANCE, pts[:, :2].max(0) + PREFER_CLEARANCE
    best = None
    for name, o in scene.objects.items():
        if name == target:
            continue
        w = _world_surface(o)
        m = np.all((w[:, :2] >= lo) & (w[:, :2] <= hi), axis=1) & (w[:, 2] > top)
        if not m.any() or float(w[m, 2].min()) < top + SHELF_GAP_M:
            continue
        d, _ = cKDTree(w[m, :2]).query(pts[:, :2], distance_upper_bound=SHELF_GAP_M)
        covered = np.mean(d <= SHELF_GAP_M) >= SHELF_COVER
        if covered and (best is None or w[m, 2].min() < best[0]):
            best = (float(w[m, 2].min()), name, w)
    if best is None or best[0] > top + SHELF_OVERHANG_M:
        return None
    ceil, name, w = best
    c = pts[:, :2].mean(0)
    face = None
    for k in (0, 1):
        for sgn in (1.0, -1.0):
            n = np.zeros(3)
            n[k] = sgn
            at = float((w[:, :2] @ n[:2]).max())
            if at >= 0:
                continue
            score = float(n[:2] @ -c) / max(float(np.linalg.norm(c)), 1e-9)
            if face is None or score > face[0]:
                face = (score, n, at)
    if face is None:
        return None
    _, n, at = face
    out = max(0.0, at + PREFER_CLEARANCE - float((pts @ n).min()))
    up = float(np.clip(ceil - top - PREFER_CLEARANCE, ESCAPE_MIN_M, SHELF_LIFT_M))
    return _ShelfExit(name, up, out * n)


def _tall_thin(W) -> bool:
    """Whether world points W stand at least TALL_M high and TALL_RATIO x their narrowest width."""
    h = float(np.ptp(W[:, 2]))
    if h < TALL_M:
        return False
    a = np.radians(np.arange(0.0, 180.0, 2.0))
    narrow = float(np.ptp(W[:, :2] @ np.stack([np.cos(a), np.sin(a)]), axis=0).min())
    return h >= grasp.TALL_RATIO * max(narrow, 1e-3)


def _gentle(scene, obj) -> bool:
    """Whether obj is heavy (its physics mass), long or tall/thin."""
    V = np.asarray(obj.verts, dtype=np.float64)
    if not len(V):
        return False
    T = np.asarray(obj.pose, dtype=np.float64)
    W = V @ T[:3, :3].T + T[:3, 3]
    ph = getattr(obj, "physics", None)
    mass = getattr(ph, "mass_kg", None) if ph is not None else None
    return bool((mass is not None and mass >= HEAVY_KG) or float(np.ptp(V, axis=0).max()) >= LONG_M
                or _tall_thin(W))


def plan_pick(
    scene, target, T_grasp, q_start, gripper_start, tool, dt, *, approach=0.10, lift=0.10,
    speed_scale=0.5, settle_s=0.3, close_s=1.2, time_budget_s=4.0, seed=0, spheres=None,
    variants=True, provider=None, stop_after=None, lift_test_m=LIFT_TEST_M,
    gripper: GripperModel | None = None, embodiment: Embodiment | None = None,
) -> trajectory.PlanResult:
    """Transit to the pregrasp, approach, settle, close, lift: a "pick" PlanResult with segments."""
    t_begin = clock.now()
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, speed_scale, tool)
    if stop_after not in (None, "lift_test", "close"):
        raise ValueError(f"stop_after must be 'lift_test', 'close' or None, got {stop_after!r}")
    T_grasp = _pose(T_grasp, "T_grasp")
    obj = _obj(scene, target, "object")
    gripper = gripper_for(embodiment) if gripper is None else gripper
    body = gripper.body
    spheres = _spheres(spheres, embodiment)
    window = max(LIFT_CONTACT_M, gripper_reach(spheres, tool, embodiment) + MARGIN)
    test, closed = stop_after == "lift_test", stop_after == "close"
    shelf = None if test or closed else _shelf_exit(scene, target, _world_surface(obj))
    lift_v = LIFT_SPEED * (GENTLE_CARRY if _gentle(scene, obj) else 1.0)
    if test:
        lift = float(np.clip(lift_test_m, 0.002, LIFT_SLOW_M * 2))
    elif shelf is None and not closed:
        lift = _pick_lift(scene, target, obj, T_grasp, q_start, tool, float(lift), body)
    world = CollisionWorld(scene, spheres=spheres, margin=MARGIN, embodiment=embodiment)
    near_target = make_rule(world, table=("gripper",), ignore=[("gripper", target)])

    def setup(T_g) -> Setup:
        T_pre = T_g.copy()
        T_pre[:3, 3] -= float(approach) * T_g[:3, 2]
        T_tcp_obj = np.linalg.inv(T_g) @ np.asarray(obj.pose, dtype=np.float64)
        held_full = CollisionWorld(
            scene, exclude=(target,), attached=(target, T_tcp_obj), spheres=spheres,
            margin=MARGIN, embodiment=embodiment,
        )
        held = HeldMargin(held_full, HELD_LIFT_MARGIN)
        if test:
            T_lift = T_g.copy()
            T_lift[2, 3] += float(lift)
        elif closed:
            T_lift = T_g
        elif shelf is None:
            T_lift = lift_target(held_full, T_g, float(lift), HELD_LIFT_MARGIN)
        else:
            T_lift = T_g.copy()
            T_lift[2, 3] += shelf.up

        def at_target(tcp, q, q0):
            return [near_target] * len(q)

        issues = _closing_issues(scene, target, T_g, gripper)
        if world.arm is not None:
            issues += [("approach", "collision", "error", f"grasp model: {p}") for p in
                       grasp.gripper_problems(scene, target, T_g, approach_dist=float(approach),
                                              gripper=gripper, embodiment=embodiment)]

        def lifting(tcp, q, q0):
            carried: list = []
            rules = sliding_rules(held, q0, q, tcp[:, 2] - T_g[2, 3], window, tool,
                                   table=("gripper", "held"), carried=carried)
            if np.linalg.norm(tool.fk_tcp(q[-1])[:3, 3] - tcp[-1, :3]) > 5 * IK_POS_OK:
                carried.clear()
            for obj_name, other in carried:
                msg = f"{other} rests on {obj_name}: pick it first"
                if not any(i[3] == msg for i in issues):
                    issues.append(("lift", "collision", "error", msg))
            return rules

        stages = [
            Stage("approach", 0.0, at_target, T=T_g, v_peak=APPROACH_SPEED,
                   slow_end=(CONTACT_SLOW_M, CONTACT_SPEED)),
            Stage("approach", 0.0, at_target, hold_s=settle_s),
            Stage("close", 1.0, at_target, hold_s=close_s),
        ]
        if not closed:
            stages.append(
                Stage("lift_test", 1.0, lifting, T=T_lift, v_peak=LIFT_SLOW_SPEED) if test else
                Stage("lift", 1.0, lifting, T=T_lift, v_peak=lift_v,
                      slow=(LIFT_SLOW_M, LIFT_SLOW_SPEED)))
        if shelf is not None:
            free = make_rule(held, table=("gripper", "held"))

            def clear(tcp, q, q0):
                return [free] * len(q)

            T_out = T_lift.copy()
            T_out[:3, 3] += shelf.out
            T_up = T_out.copy()
            T_up[2, 3] += float(lift)
            stages += [Stage("lift", 1.0, clear, T=T_out, v_peak=ESCAPE_SPEED),
                       Stage("lift", 1.0, clear, T=T_up, v_peak=lift_v)]
        return Setup(T_g, T_pre, stages, make_rule(world), issues)

    tries = [Variant(False, 0.0, lambda: setup(T_grasp))]
    if variants:
        tries.append(Variant(True, 0.0, lambda: setup(T_grasp @ _FLIP)))
    res = plan_search(
        "pick", scene=scene, tool=tool, dt=dt, q_start=q_start, gripper_start=gripper_start,
        variants=tries, transit_grip=0.0, speed_scale=speed_scale, settle_s=settle_s,
        time_budget_s=time_budget_s, seed=seed, t_begin=t_begin, provider=provider,
    )
    if test or closed:
        res.extra["stop_after"] = stop_after
    return res


def _closing_issues(scene, target, T_grasp, gripper=None) -> list[tuple[str, str, str, str]]:
    """A warning when the closing fingers would reach the table: they stall there, open."""
    try:
        info = grasp.closure_at(scene, target, T_grasp, gripper)
    except (KeyError, ValueError):
        return []
    clear = info["sweep_clear"]
    if clear >= grasp.TIP_CLEAR:
        return []
    where = f"{-clear * 1e3:.0f} mm below" if clear < 0 else f"within {clear * 1e3:.0f} mm of"
    return [("close", "table", "warn",
             (f"the closing fingertips reach {where} the table (closure "
              f"{info['closure']:.2f}): they may stall before gripping; grasp higher"))]


def _yawed(T, center, yaw_deg) -> np.ndarray:
    """T turned by yaw_deg about the world vertical through center."""
    c, sn = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    G = np.eye(4)
    G[:2, :2] = [[c, -sn], [sn, c]]
    G[:2, 3] = center[:2] - G[:2, :2] @ center[:2]
    return G @ T


def _place_rise(scene, held, target, T_tcp_obj, T_pre, T_place, q_start, tool,
                body=None) -> float:
    """Extra height for the top of the descend so the held object arrives above the neighbours
    beside it and on the way in from q_start, within reach; 0 when none is needed."""
    pts = _world_verts(scene.objects[held], T_pre @ T_tcp_obj)
    toward = tool.fk_tcp(q_start)[:2, 3]
    want = min(_clear_rise(scene, {held, target}, pts, T_pre, toward, body=body),
               place.PLACE_RISE_MAX)
    if want < ESCAPE_MIN_M:
        return 0.0
    arm = arm_of(tool)
    r = arm.ik(T_place, q_start, tool.T_flange_tcp, max_iters=150)
    if r.pos_err > IK_POS_OK or r.rot_err > IK_ROT_OK:
        return 0.0
    q = r.q
    for h in (want, 0.5 * want):
        T = T_pre.copy()
        T[2, 3] += h + place.PLACE_REACH_MARGIN
        r = arm.ik(T, q, tool.T_flange_tcp, q_rest=q, rest_gain=REST_GAIN)
        if (r.pos_err <= IK_POS_OK and r.rot_err <= IK_ROT_OK
                and arm.limit_margin(r.q) >= candidates.LIMIT_MARGIN):
            return float(h)
    return 0.0


def plan_place(
    scene, held, T_tcp_obj, T_place, q_start, gripper_start, tool, dt, *, target=None,
    approach=0.10, retreat=0.10, speed_scale=0.5, settle_s=0.3, open_s=0.6, time_budget_s=4.0,
    seed=0, spheres=None, variants: bool = True, provider=None,
    gripper: GripperModel | None = None, embodiment: Embodiment | None = None,
) -> trajectory.PlanResult:
    """Escape, transit to above T_place, descend, release, retreat: a "place" PlanResult."""
    t_begin = clock.now()
    turned = [(True, 0.0)] if variants else []
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, speed_scale, tool)
    gripper = gripper_for(embodiment) if gripper is None else gripper
    body = gripper.body
    T_place = _pose(T_place, "T_place")
    T_tcp_obj = _pose(T_tcp_obj, "T_tcp_obj")
    obj = _obj(scene, held, "held object")
    if target is not None:
        _obj(scene, target, "target")
    support = "table" if target is None else target
    spheres = _spheres(spheres, embodiment)
    reach = gripper_reach(spheres, tool, embodiment)
    window = max(PLACE_CONTACT_M, reach + MARGIN)
    carry = CollisionWorld(
        scene, exclude=(held,), attached=(held, T_tcp_obj), spheres=spheres, margin=MARGIN,
        embodiment=embodiment,
    )
    escape = _escape(scene, held, T_tcp_obj, q_start, tool, dt, speed_scale, carry,
                     max(LIFT_CONTACT_M, reach + MARGIN), T_place[:2, 3], target, body)
    allowed = {support, "table"}
    if support == "table":
        base_carry = {"table": ("held", "gripper")}
    else:
        base_carry = {"ignore": [("held", support), ("gripper", support)]}
    gctx = None if target is None else scene_geom.SceneContext(scene, held, 0, gripper=gripper)
    T_obj_inv = np.linalg.inv(np.asarray(obj.pose, dtype=np.float64))

    def shelf_setup(T_p, path) -> Setup:
        """In under a level of the target and out again, every contact with it checked."""
        released = CollisionWorld(
            scene, poses={held: T_p @ T_tcp_obj}, spheres=spheres, margin=MARGIN,
            embodiment=embodiment,
        )
        strict = make_rule(carry)
        opening = make_rule(released, table=("gripper",), ignore=[("gripper", held)])
        out = make_rule(released, table=("gripper",))

        def rules(r):
            return lambda tcp, q, q0: [r] * len(q)

        stages = [
            Stage("descend", 1.0, rules(strict), T=T_p, v_peak=DESCEND_SPEED,
                   slow_end=(PLACE_SLOW_M, CONTACT_SPEED)),
            Stage("descend", 1.0, rules(strict), hold_s=settle_s),
            Stage("release", 0.0, rules(opening), hold_s=open_s),
        ]
        stages += [Stage("retreat", 0.0, rules(opening if k == 0 else out), T=T,
                          v_peak=RETREAT_SPEED) for k, T in enumerate(path.out)]
        return Setup(T_p, path.pre, stages, make_rule(carry), [])

    def setup(T_p) -> Setup:
        under = False
        if gctx is not None:
            H = gctx.tgt.moved(T_p @ T_tcp_obj @ T_obj_inv)
            path = place.place_path(gctx, H, T_p, target, float(approach), float(retreat))
            if path.side is not None:
                return shelf_setup(T_p, path)
            under = math.isfinite(path.ceil)
        T_pre = T_p.copy()
        length = None
        if under:
            ik = candidates.solve_best(T_p, q_start, tool)
            if ik.good:
                length = place.descend_length(T_p, ik.q, tool, float(approach),
                                              lambda q: carry.check(q, tool).ok)
        length = float(approach) if length is None else length
        T_pre[2, 3] += length
        back = min(float(retreat), length) if length < float(approach) else float(retreat)
        if gctx is not None and not place.is_down(T_p):
            back = max(back, place.back_off(gctx.tgt.moved(T_p @ T_tcp_obj @ T_obj_inv), T_p,
                                            body))
        T_pre[2, 3] += _place_rise(scene, held, target, T_tcp_obj, T_pre, T_p, q_start, tool,
                                   body)
        T_ret = T_p.copy()
        T_ret[:3, 3] -= back * T_p[:3, 2]
        released = CollisionWorld(
            scene, poses={held: T_p @ T_tcp_obj}, spheres=spheres, margin=MARGIN,
            embodiment=embodiment,
        )
        rel = make_rule(released, table=("gripper",),
                        ignore=[("gripper", held), ("gripper", support)])
        cache: dict = {}

        def carry_near(q_place):
            if "carry" not in cache:
                cache["carry"] = contact_rule(carry, q_place, tool, allowed=allowed, **base_carry)
            return cache["carry"]

        def descending(tcp, q, q0):
            cache.clear()
            near, far = carry_near(q[-1]), make_rule(carry)
            d = np.linalg.norm(tcp[:, :3] - T_p[:3, 3], axis=1)
            return [near if di < window else far for di in d]

        def holding(tcp, q, q0):
            return [carry_near(q0)] * len(q)

        def releasing(tcp, q, q0):
            return [rel] * len(q)

        stages = [
            Stage("descend", 1.0, descending, T=T_p, v_peak=DESCEND_SPEED,
                   slow_end=(PLACE_SLOW_M, CONTACT_SPEED)),
            Stage("descend", 1.0, holding, hold_s=settle_s),
            Stage("release", 0.0, releasing, hold_s=open_s),
            Stage("retreat", 0.0, releasing, T=T_ret, v_peak=RETREAT_SPEED),
        ]
        return Setup(T_p, T_pre, stages, make_rule(carry), [])

    T_obj = T_place @ T_tcp_obj
    center = (obj.verts.min(axis=0) + obj.verts.max(axis=0)) / 2
    center = T_obj[:3, :3] @ center + T_obj[:3, 3]
    rests: dict = {}

    def rests_clear(T_p) -> bool:
        """The held object released at T_p is no less clear of other objects than at T_place."""
        def ok(T):
            try:
                cand = place.evaluate_place(scene, held, T_tcp_obj, T, target=target,
                                            q_seed=q_start, tool=tool, gripper=gripper,
                                            embodiment=embodiment)
            except (KeyError, ValueError):
                return True
            return cand.collision_free
        if "exact" not in rests:
            rests["exact"] = ok(T_place)
        return ok(T_p) or not rests["exact"]

    def variant(flip, yaw):
        T_p = _yawed(T_place @ _FLIP if flip else T_place, center, yaw)
        return Variant(flip, yaw, lambda: setup(T_p) if rests_clear(T_p) else None)

    tries = [Variant(False, 0.0, lambda: setup(T_place))]
    swing = None
    if turned:
        tries += [variant(f, y) for f, y in turned]
        flip = tries[1]
        swing = Variant(True, 0.0,
                         lambda: flip.make() if _flip_alike(obj, T_tcp_obj) else None)
    gentle = GENTLE_CARRY if _gentle(scene, obj) else 1.0
    return plan_search(
        "place", scene=scene, tool=tool, dt=dt, q_start=q_start, gripper_start=gripper_start,
        variants=tries, transit_grip=1.0, speed_scale=speed_scale, settle_s=settle_s,
        time_budget_s=time_budget_s, seed=seed, t_begin=t_begin,
        transit_scale=gentle * min(speed_scale, CARRY_SPEED_SCALE),
        transit_accel=gentle * CARRY_ACCEL,
        prefix=() if escape is None else [escape], swing=swing, provider=provider,
        held=None if provider is None else (held, T_tcp_obj, tool.T_flange_tcp),
    )


def _flip_alike(o, T_tcp_obj) -> bool:
    """Whether o, held at T_tcp_obj, has the same surface turned pi about the TCP z."""
    field = mesh_field(o.verts, o.faces)
    pts = np.asarray(o.verts, dtype=np.float64) if field is None else field.samples
    tree = cKDTree(pts) if field is None else field.tree
    tol = FLIP_ALIKE_M + (0.0 if field is None else field.step)
    M = np.linalg.inv(T_tcp_obj) @ _FLIP @ T_tcp_obj
    pts = pts[:: max(1, len(pts) // FLIP_SAMPLES)]
    return bool(tree.query(pts @ M[:3, :3].T + M[:3, 3])[0].max() <= tol)


def _finish_pieces(kind, pieces, *, scene, q_start, gripper_start, tool, dt, settle_s, t_begin,
                   issues=(), message="") -> trajectory.PlanResult:
    """A PlanResult from Cartesian/hold pieces run from q_start (no transit)."""
    col_issues, clr = collision_issues(pieces, q_start, tool, dt)
    q = np.concatenate([p.q for p in pieces])
    segments, start = [], 0
    for p in pieces:
        end = start + len(p.q) - 1
        if segments and segments[-1]["name"] == p.name:
            segments[-1]["end"] = end
        else:
            segments.append({"name": p.name, "start": start, "end": end})
        start = end + 1
    segments[-1]["end"] += trajectory.settle_step_count(settle_s, dt)
    worlds = {id(rule[0]): rule[0] for p in pieces for rule in p.rules}
    extra = {"segments": segments, "planner": {
        "method": "cartesian", "time_s": round(clock.now() - t_begin, 3),
        "checks": int(sum(w.checks for w in worlds.values())),
        "min_clearance_m": round(float(np.min(clr) + MARGIN), 4), "message": message}}
    return trajectory.finish(
        kind, dt, (np.arange(len(q)) + 1) * dt, q, np.concatenate([p.tcp for p in pieces]),
        np.concatenate([np.full(len(p.q), p.grip) for p in pieces]), tool, q_start,
        gripper_start, pos_err=np.concatenate([p.pos_err for p in pieces]),
        rot_err=np.concatenate([p.rot_err for p in pieces]), speed_scale=STAGE_SPEED_SCALE,
        table_z=float(getattr(scene, "table_z", 0.0)), settle_s=settle_s,
        extra_issues=list(issues) + col_issues, extra=extra, singular_hold_error=True,
    )


def plan_lift(scene, held, T_tcp_obj, q_start, gripper_start, tool, dt, *, height=0.10,
              settle_s=0.3, spheres=None, speed_scale=1.0, gripper: GripperModel | None = None,
              embodiment: Embodiment | None = None) -> trajectory.PlanResult:
    """Raise the held object straight up by height (more to clear a taller neighbour, up to
    LIFT_MAX_M), slowly at first: a "lift" PlanResult ending at the carry pose."""
    t_begin = clock.now()
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, speed_scale, tool)
    spheres = _spheres(spheres, embodiment)
    T0 = tool.fk_tcp(q_start)
    window = max(LIFT_CONTACT_M, gripper_reach(spheres, tool, embodiment) + MARGIN)
    if held is not None:
        obj = _obj(scene, held, "held object")
        T_tcp_obj = _pose(T_tcp_obj, "T_tcp_obj")
        pts = _world_verts(obj, T0 @ T_tcp_obj)
        body = (gripper_for(embodiment) if gripper is None else gripper).body
        rise = max(float(height), min(_clear_rise(scene, {held}, pts, T0, body=body),
                                      LIFT_MAX_M))
        world = CollisionWorld(scene, exclude=(held,), attached=(held, T_tcp_obj),
                               spheres=spheres, margin=MARGIN, embodiment=embodiment)
        v = LIFT_SPEED * (GENTLE_CARRY if _gentle(scene, obj) else 1.0)
    else:
        rise, v = float(height), LIFT_SPEED
        world = CollisionWorld(scene, spheres=spheres, margin=MARGIN, embodiment=embodiment)
    v = max(LIFT_SLOW_SPEED, v * float(speed_scale))
    lifted = HeldMargin(world, HELD_LIFT_MARGIN)

    def rising(tcp, q, q0):
        return sliding_rules(lifted, q0, q, tcp[:, 2] - T0[2, 3], window, tool,
                              table=("gripper", "held"))

    issues, best = [], None
    for scale in (1.0, 0.75, 0.5):
        T1 = T0.copy()
        T1[2, 3] += scale * rise
        stage = Stage("lift", float(gripper_start >= 0.5), rising, T=T1, v_peak=v,
                       slow=(LIFT_SLOW_M, LIFT_SLOW_SPEED))
        pieces, ok = run_stages([stage], q_start, T0, tool, dt, STAGE_SPEED_SCALE)
        best = best or pieces
        if ok:
            best = pieces
            if scale < 1.0:
                why = (f"lifted {scale * rise * 100:.0f} cm of the {rise * 100:.0f} cm asked: "
                       "the arm reaches no higher")
                issues.append(trajectory.Issue(0, dt, "reach", "warn", why))
            break
    return _finish_pieces("lift", best, scene=scene, q_start=q_start, gripper_start=gripper_start,
                          tool=tool, dt=dt, settle_s=settle_s, t_begin=t_begin, issues=issues)


def plan_release(scene, held, q_start, gripper_start, tool, dt, *, retreat=0.08, open_s=0.6,
                 settle_s=0.3, spheres=None,
                 embodiment: Embodiment | None = None) -> trajectory.PlanResult:
    """Open the gripper where it is, then back off along -tcp z (up from a top-down grasp)."""
    t_begin = clock.now()
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, 1.0, tool)
    spheres = _spheres(spheres, embodiment)
    T0 = tool.fk_tcp(q_start)
    world = CollisionWorld(scene, spheres=spheres, margin=MARGIN, embodiment=embodiment)
    ignore = [] if held is None or held not in scene.objects else [("gripper", held)]
    rel = contact_rule(world, q_start, tool, table=("gripper",), ignore=ignore)

    def releasing(tcp, q, q0):
        return [rel] * len(q)

    T1 = T0.copy()
    T1[:3, 3] -= float(retreat) * T0[:3, 2]
    stages = [Stage("release", 0.0, releasing, hold_s=open_s)]
    if retreat > 0:
        stages.append(Stage("retreat", 0.0, releasing, T=T1, v_peak=RETREAT_SPEED))
    pieces, _ = run_stages(stages, q_start, T0, tool, dt, STAGE_SPEED_SCALE)
    return _finish_pieces("release", pieces, scene=scene, q_start=q_start,
                          gripper_start=gripper_start, tool=tool, dt=dt, settle_s=settle_s,
                          t_begin=t_begin)


def plan_move(scene, held, T_tcp_obj, T_goal, q_start, gripper_start, tool, dt, *,
              speed_scale=0.5, settle_s=0.3, time_budget_s=4.0, seed=0, spheres=None,
              provider=None, embodiment: Embodiment | None = None) -> trajectory.PlanResult:
    """A collision-free transit to the TCP pose T_goal, the held object (if any) attached."""
    t_begin = clock.now()
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, speed_scale, tool)
    T_goal = _pose(T_goal, "T_goal")
    spheres = _spheres(spheres, embodiment)
    grip = float(gripper_start >= 0.5)
    if held is not None:
        obj = _obj(scene, held, "held object")
        T_tcp_obj = _pose(T_tcp_obj, "T_tcp_obj")
        world = CollisionWorld(scene, exclude=(held,), attached=(held, T_tcp_obj),
                               spheres=spheres, margin=MARGIN, embodiment=embodiment)
        rule = contact_rule(world, q_start, tool, table=("held", "gripper"))
        gentle = GENTLE_CARRY if _gentle(scene, obj) else 1.0
        tscale, taccel = gentle * min(speed_scale, CARRY_SPEED_SCALE), gentle * CARRY_ACCEL
    else:
        world = CollisionWorld(scene, spheres=spheres, margin=MARGIN, embodiment=embodiment)
        rule = contact_rule(world, q_start, tool, table=("gripper",))
        tscale, taccel = speed_scale, ACCEL_MAX
    tries = [Variant(False, 0.0, lambda: Setup(T_goal, T_goal, [], rule, []))]
    return plan_search(
        "move", scene=scene, tool=tool, dt=dt, q_start=q_start, gripper_start=gripper_start,
        variants=tries, transit_grip=grip, speed_scale=speed_scale, settle_s=settle_s,
        time_budget_s=time_budget_s, seed=seed, t_begin=t_begin, transit_scale=tscale,
        transit_accel=taccel, provider=provider,
        held=None if provider is None or held is None else (held, T_tcp_obj, tool.T_flange_tcp),
    )


def push_poses(start, end, table_z, height=0.02, q_ref=None, tool=None,
               gripper: GripperModel | None = None):
    """(T_start, T_end) top-down TCP poses, fingers closed, their lowest point `height` above the
    table, the closing axis across the push (wide face first), yaw nearest the reference TCP."""
    p0, p1 = np.asarray(start, dtype=np.float64), np.asarray(end, dtype=np.float64)
    d = p1[:2] - p0[:2]
    n = float(np.linalg.norm(d))
    if n < 1e-3:
        raise ValueError("push start and end must be at least 1 mm apart")
    d /= n
    across = np.array([-d[1], d[0], 0.0])
    R = grasp.grasp_frame((0.0, 0.0, -1.0), across)
    if q_ref is not None and tool is not None:
        x_ref = tool.fk_tcp(np.asarray(q_ref, dtype=np.float64))[:3, 0]
        if float(R[:, 0] @ x_ref) < 0:
            R = R @ _FLIP[:3, :3]
    T = np.eye(4)
    T[:3, :3] = R
    gp = (gripper_for(None) if gripper is None else gripper).profile
    low = grasp.finger_low(T, gp.closed_angle, gp)
    z = float(table_z) + float(height) - low
    T0, T1 = T.copy(), T.copy()
    T0[:3, 3] = [p0[0], p0[1], z]
    T1[:3, 3] = [p1[0], p1[1], z]
    return T0, T1


def plan_push(scene, start, end, q_start, gripper_start, tool, dt, *, height=0.02,
              approach=0.10, retreat=0.10, speed_scale=0.5, settle_s=0.3, close_s=0.8,
              time_budget_s=4.0, seed=0, spheres=None, provider=None,
              gripper: GripperModel | None = None,
              embodiment: Embodiment | None = None) -> trajectory.PlanResult:
    """Close the fingers, transit above start, descend to `height` above the table, push straight to
    end, rise: a "push" PlanResult."""
    t_begin = clock.now()
    check_tool(embodiment, tool)
    q_start = _check_args(q_start, dt, speed_scale, tool)
    spheres = _spheres(spheres, embodiment)
    table_z = float(getattr(scene, "table_z", 0.0))
    gripper = gripper_for(embodiment) if gripper is None else gripper
    T_s, T_e = push_poses(start, end, table_z, height, q_start, tool, gripper)
    world = CollisionWorld(scene, spheres=spheres, margin=MARGIN, embodiment=embodiment)
    lo = np.minimum(T_s[:2, 3], T_e[:2, 3]) - GRIPPER_HALF_M
    hi = np.maximum(T_s[:2, 3], T_e[:2, 3]) + GRIPPER_HALF_M
    touched = [n for n, o in scene.objects.items()
               if np.any(np.all((_world_verts(o)[:, :2] >= lo) & (_world_verts(o)[:, :2] <= hi),
                                axis=1))]
    contact = make_rule(world, table=("gripper",), ignore=[("gripper", n) for n in touched])
    strict = make_rule(world, table=("gripper",))

    def rules(r):
        return lambda tcp, q, q0: [r] * len(q)

    T_pre = T_s.copy()
    T_pre[2, 3] += float(approach)
    T_up = T_e.copy()
    T_up[2, 3] += float(retreat)
    stages = [
        Stage("descend", 1.0, rules(strict), T=T_s, v_peak=APPROACH_SPEED,
               slow_end=(CONTACT_SLOW_M, CONTACT_SPEED)),
        Stage("push", 1.0, rules(contact), T=T_e, v_peak=PUSH_SPEED),
        Stage("retreat", 1.0, rules(contact), T=T_up, v_peak=RETREAT_SPEED),
    ]
    prefix = []
    if gripper_start < 0.5:
        q0 = np.asarray(q_start, dtype=np.float64)
        n = max(1, round(close_s / dt))
        prefix = [Piece("close", np.tile(q0, (n, 1)), np.tile(pose7(tool.fk_tcp(q0)), (n, 1)),
                         1.0, np.zeros(n), np.zeros(n), [strict] * n)]
    tries = [Variant(False, 0.0, lambda: Setup(T_s, T_pre, stages, strict, []))]
    return plan_search(
        "push", scene=scene, tool=tool, dt=dt, q_start=q_start, gripper_start=gripper_start,
        variants=tries, transit_grip=1.0, speed_scale=speed_scale, settle_s=settle_s,
        time_budget_s=time_budget_s, seed=seed, t_begin=t_begin, prefix=prefix,
        provider=provider,
    )


HELD_MANUAL_TOL_M = 0.003
HELD_MANUAL_STEP_M = 0.01


def _held_points(world, qs, tool) -> np.ndarray:
    """(N, m, 3) world's held surface points at each configuration."""
    n = len(world._local)
    return world.points(qs, tool)[:, n:n + len(world._att_pts)]


def _held_samples(world, q0, qs, tool, step) -> tuple[np.ndarray, np.ndarray]:
    """(configs, step index of each): qs thinned or subdivided so the held object moves <= step."""
    prev = np.vstack([q0[None], qs[:-1]])
    P = _held_points(world, np.vstack([q0[None], qs]), tool)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=2).max(axis=1)
    out, idx, acc = [], [], 0.0
    for k, s in enumerate(seg):
        m = math.ceil(s / step)
        if m > 1:
            out.extend(prev[k] + (qs[k] - prev[k]) * (np.arange(1, m + 1) / m)[:, None])
            idx += [k] * m
            acc = 0.0
            continue
        acc += s
        if acc >= step or k == len(qs) - 1:
            out.append(qs[k])
            idx.append(k)
            acc = 0.0
    return np.asarray(out), np.asarray(idx)


def _held_signed(world, P, k, cap, peer_sets=None) -> np.ndarray:
    """(N, m) signed distance (inside < 0, capped at cap) of held surface points P to obstacle k;
    peer_sets: the peers at each of the N configurations (default: world's, static)."""
    if k >= len(world._obs):
        g, hand = k - len(world._obs), k not in world.co_held
        if peer_sets is not None:
            return np.stack([ps.clearance(P[i], 0.0, g, cap, hand=hand)
                             for i, ps in enumerate(peer_sets)])
        return world.peers.clearance(P.reshape(-1, 3), 0.0, g, cap,
                                     hand=hand).reshape(P.shape[:-1])
    o = world._obs[k]
    L = (P - o.t) @ o.R
    box = np.linalg.norm(np.maximum(np.maximum(o.field.lo - L, L - o.field.hi), 0), axis=-1)
    out = np.full(box.shape, cap)
    near = box < cap
    if near.any():
        Lc = L[near]
        d = np.minimum(o.field.tree.query(Lc, distance_upper_bound=cap)[0], cap)
        out[near] = np.where(o.field.inside(Lc), -d, d)
    return out


def held_contacts(scene, held, T_tcp_obj, q_start, qs, tool, dt, *, t=None, spheres=None,
                  tol=HELD_MANUAL_TOL_M, step=HELD_MANUAL_STEP_M,
                  embodiment: Embodiment | None = None, peer_path=None) -> list:
    """Collision errors of the held object (attached at T_tcp_obj) along qs against the scene."""
    qs = np.asarray(qs, dtype=np.float64).reshape(-1, check_tool(embodiment, tool).n)
    if held not in scene.objects or not len(qs):
        return []
    try:
        world = CollisionWorld(
            scene, exclude=(held,), attached=(held, np.asarray(T_tcp_obj, dtype=np.float64)),
            spheres=_spheres(spheres, embodiment), margin=0.0, embodiment=embodiment)
    except ValueError:
        return []
    q0 = np.asarray(q_start, dtype=np.float64)
    configs, idx = _held_samples(world, q0, qs, tool, step)
    P = _held_points(world, np.vstack([q0[None], configs]), tool)
    peer_sets = None
    if peer_path is not None and world.peers is not None:
        rows = {a: np.asarray(r, dtype=np.float64) for a, r in peer_path.items()}
        peer_sets = [world.peers] + [
            embodiment.base.peer_set({a: r[j] for a, r in rows.items()},
                                     exclude_arm=embodiment.arm_name, held=embodiment.peer_held)
            for j in idx]
    depth, who = np.full(len(configs), np.inf), np.full(len(configs), -1)
    for k in range(len(world.obstacle_names) - 1):
        sd = _held_signed(world, P, k, 0.05, peer_sets)
        sd0, sd = sd[0], sd[1:]
        thr = np.full(sd.shape, -tol)
        if (sd0 < 0).any():
            sep = np.flatnonzero(sd.min(axis=1) > 0)
            thr[:sep[0] if len(sep) else len(sd)] = np.minimum(sd0, 0.0) - tol
        d = np.where(sd < thr, sd, np.inf).min(axis=1)
        worse = d < depth
        depth, who = np.where(worse, d, depth), np.where(worse, k, who)
    t = (np.arange(len(qs)) + 1) * dt if t is None else np.asarray(t)
    issues, i = [], 0
    while i < len(configs):
        if who[i] < 0:
            i += 1
            continue
        j = i
        while j + 1 < len(configs) and who[j + 1] == who[i]:
            j += 1
        a, b = int(idx[i]), int(idx[j])
        issues.append(trajectory.Issue(
            a, float(t[a]), "collision", "error",
            f"held {held} hits {world.obstacle_names[who[i]]} at {trajectory.step_range(a, b)} "
            f"(clearance {float(depth[i:j + 1].min()) * 1e3:.0f} mm)"))
        i = j + 1
    return issues
