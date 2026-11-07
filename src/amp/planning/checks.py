#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Collision checks for agents: what a configuration collides with, against a chosen world."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np

from amp.contracts.harness import OBSTACLE_MODES
from amp.geometry import kinematics as kin
from amp.geometry.collision import CollisionWorld
from amp.geometry.embodiment import Embodiment
from amp.geometry.world import Scene
from amp.planning import ik, robot

DEFAULT_MARGIN_M = 0.01
BATCH_MAX = 256
NO_OBJECTS = ("this robot API has no objects: held must be null and ignore may only name "
              "'table'")
NO_OBSTACLES = "this robot API has no objects: held must be null and ignore empty"


class UnknownObject(LookupError):
    """A check named an object the world does not have."""


@dataclass
class CheckWorld:
    """A collision world with what a check reports alongside it."""

    world: CollisionWorld
    tool: kin.ToolModel
    q_obs: np.ndarray
    margin: float
    base: list = field(default_factory=list)
    table_ignored: bool = False
    inside: dict = field(default_factory=dict)
    embodiment: Embodiment | None = None


def _ids(value, name) -> list[str]:
    if value is None:
        return []
    if not isinstance(value, (list, tuple)) or not all(isinstance(v, str) for v in value):
        raise ValueError(f"{name} must be a list of object ids, got {value!r}")
    return list(value)


def table_world(*, table_z: float, tool, q_obs, margin=DEFAULT_MARGIN_M, held=None,
                ignore=None, embodiment: Embodiment | None = None, table=True) -> CheckWorld:
    """The robot against the table only (table=False: against nothing); no object id is accepted."""
    robot.check_tool(embodiment, tool)
    ignore = [] if ignore is None else ignore
    names = ("table",) if table else ()
    if held is not None or not isinstance(ignore, list) or any(n not in names for n in ignore):
        raise ValueError(NO_OBJECTS if table else NO_OBSTACLES)
    world = CollisionWorld(Scene(0, {}, float(table_z), 0), margin=float(margin),
                           embodiment=embodiment)
    return CheckWorld(world, tool, np.asarray(q_obs, dtype=np.float64).copy(), float(margin),
                      table_ignored=not table or "table" in ignore, embodiment=embodiment)


def scene_world(scene: Scene, *, tool, q_obs, T_world_tcp, margin=DEFAULT_MARGIN_M, held=None,
                ignore=None, arm_inside: Callable | None = None,
                embodiment: Embodiment | None = None, table=True) -> CheckWorld:
    """The robot (holding held, fixed at the observed TCP) against scene and the table."""
    if held is not None and not isinstance(held, str):
        raise ValueError(f"held must be an object id or null, got {held!r}")
    ignore = _ids(ignore, "ignore")
    known = set(scene.objects) | ({"table"} if table else set())
    for n in [held, *ignore]:
        if n is not None and n not in known:
            raise UnknownObject(f"unknown object {n!r}; the scene has {sorted(scene.objects)}")
    if held == "table":
        raise ValueError("held must be an object id")
    robot.check_tool(embodiment, tool)
    q_obs = np.asarray(q_obs, dtype=np.float64).copy()
    objs = [n for n in ignore if n != "table"]
    inside = {}
    if arm_inside is not None:
        inside = arm_inside(scene, q_obs, tool, [n for n in (held, *objs) if n is not None])
    attached = None
    if held is not None:
        attached = (held, np.linalg.inv(T_world_tcp) @ scene.objects[held].pose)
    world = CollisionWorld(scene, exclude=set(objs), attached=attached, margin=float(margin),
                           embodiment=embodiment)
    return CheckWorld(world, tool, q_obs, float(margin), base=[("arm", n) for n in inside],
                      table_ignored=not table or "table" in ignore,
                      inside={n: round(d, 4) for n, d in sorted(inside.items())},
                      embodiment=embodiment)


def check_world(obstacles: str, scene: Scene | None, *, table_z: float, **kw) -> CheckWorld:
    """The world for an obstacles mode (contracts.harness.OBSTACLE_MODES); no scene = table."""
    if obstacles not in OBSTACLE_MODES:
        raise ValueError(f"obstacles {obstacles!r} not in {OBSTACLE_MODES}")
    if obstacles == "table":
        kw.pop("T_world_tcp", None)
        kw.pop("arm_inside", None)
        return table_world(table_z=table_z, **kw)
    return scene_world(scene if scene is not None else Scene(0, {}, float(table_z), 0), **kw)


def arm_qs(qs, n: int) -> np.ndarray:
    """qs as (k, n): one configuration (n,) or k of them; any other length is a ValueError."""
    qs = np.asarray(qs, dtype=np.float64)
    if qs.ndim == 1 and len(qs) == n:
        return qs[None]
    if (qs.ndim == 2 and qs.shape[1] == n) or qs.size == 0:
        return qs.reshape(-1, n)
    got = qs.shape[-1] if qs.ndim else 1
    raise ValueError(f"q has {got} values per configuration, the checked arm has {n} joints")


def collisions(w: CheckWorld, qs) -> list[dict]:
    """Per configuration of qs (N, n): collision_free, min_clearance_m, contacts, arm_inside."""
    world, tool, base = w.world, w.tool, w.base
    qs = arm_qs(qs, world.n)
    tl = ("*", "held") if w.table_ignored else ()
    clr, _, _ = world.evaluate(qs, tool, table_links_ignored=tl, ignore=base)
    out = []
    for k, q in enumerate(qs):
        contacts = []
        if float(clr[k]) < 0:
            for name in world.obstacle_names:
                if name == "table" and w.table_ignored:
                    continue
                others = [(sel, m) for m in world.obstacle_names if m not in (name, "table")
                          for sel in ("*", "held")]
                c, lk, ob = world.evaluate(
                    q, tool, table_links_ignored=() if name == "table" else ("*", "held"),
                    ignore=[*base, *others])
                ct = world.contact(c[0], lk[0], ob[0])
                if not ct.ok and ct.obstacle == name:
                    contacts.append({"link": ct.link, "obstacle": name,
                                     "clearance_m": round(ct.clearance, 5)})
            contacts.sort(key=lambda d: d["clearance_m"])
        out.append({"collision_free": bool(clr[k] >= 0),
                    "min_clearance_m": round(float(clr[k]), 5), "contacts": contacts,
                    "arm_inside": dict(w.inside)})
    return out


def pair_contacts(w: CheckWorld, q) -> dict[tuple[str, str], float]:
    """{(link, obstacle): clearance} of every link within clearance of every obstacle at q."""
    world, tool, base = w.world, w.tool, w.base
    q = arm_qs(q, world.n)[:1]
    tl = ["*", "held"] if w.table_ignored else []
    ign = list(base)
    out = {}
    for _ in range(len(world.link_names) * len(world.obstacle_names)):
        c, lk, ob = world.evaluate(q, tool, table_links_ignored=tuple(tl), ignore=ign)
        ct = world.contact(c[0], lk[0], ob[0])
        if ct.ok or ct.obstacle is None:
            break
        out[(str(ct.link), ct.obstacle)] = round(ct.clearance, 5)
        if ct.obstacle == "table":
            tl.append(str(ct.link))
        else:
            ign.append((str(ct.link), ct.obstacle))
    return out


def q_fields(q, reachable: bool, embodiment: Embodiment | None = None) -> dict:
    return {"q": [float(v) for v in q], "reachable": reachable,
            "in_limits": ik.in_limits(q, embodiment), "singular": ik.singular(q, embodiment)}


def check(w: CheckWorld, *, q=None, T_tcp=None) -> dict:
    """q, or the best IK of T_tcp (seeded at the observed q), or the observed q, checked in w."""
    if q is not None and T_tcp is not None:
        raise ValueError("check takes 'q' or 'tcp', not both")
    reply = {"ok": True, "margin_m": w.margin}
    if T_tcp is not None:
        sol = ik.solve(T_tcp, w.q_obs, w.tool, 1, w.embodiment)[0]
        q, reachable = np.array(sol.q), sol.accurate
    else:
        q = arm_qs(w.q_obs if q is None else q, w.world.n)[0]
        reachable = True
    reply.update(q_fields(q, reachable, w.embodiment))
    if not reachable:
        return {**reply, "collision_free": False, "min_clearance_m": None, "contacts": [],
                "arm_inside": {}}
    return {**reply, **collisions(w, q[None])[0]}


def check_batch(w: CheckWorld, qs) -> dict:
    """check per joint configuration, one collision world for all."""
    qs = arm_qs(qs, w.world.n)
    if not 1 <= len(qs) <= BATCH_MAX:
        raise ValueError(f"qs must be a list of 1..{BATCH_MAX} joint configurations")
    results = [{**q_fields(q, True, w.embodiment), **c} for q, c in zip(qs, collisions(w, qs))]
    return {"ok": True, "margin_m": w.margin, "results": results}
