#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Held-object inference: what a closed gripper holds, from perception."""

from __future__ import annotations

import numpy as np

from amp.geometry.embodiment import Embodiment
from amp.perception.objects import Perception, footprint, object_at, similar_size
from amp.perception.params import embodiment_or_droid

HELD_TCP_DIST_M = 0.06
HELD_GRIPPER_SAMPLES = 5
GRIP_SETTLED = 0.02
SPOT_MAX_M = 0.03
REST_M = 0.02
COLOR_MAX = 60.0


def grip_state(gripper_cmd, measured, *, embodiment: Embodiment | None = None,
               ee: str | None = None) -> str:
    """"open", "object" (closed, at or below the gripper's held_below) or "closed" (past it, toward
    its closed_empty reading: on nothing), for the embodiment's end effector (None = DROID's)."""
    gripper = embodiment_or_droid(embodiment).gripper(ee)
    if gripper is None or gripper_cmd != 1 or not measured:
        return "open"
    g = float(np.median(np.asarray(measured[-HELD_GRIPPER_SAMPLES:], dtype=np.float64)))
    return "object" if gripper.holds(g) else "closed"


def grip_lost_step(cmds, measured, *, embodiment: Embodiment | None = None,
                   ee: str | None = None) -> int | None:
    """The first step at which a settled hold, commanded closed, stops holding; None if never."""
    window, held = [], False
    for i, (c, g) in enumerate(zip(cmds, measured)):
        if c != 1:
            window, held = [], False
            continue
        if g is None:
            continue
        window = [*window, float(g)][-HELD_GRIPPER_SAMPLES:]
        if grip_state(1, window, embodiment=embodiment, ee=ee) == "object":
            held = held or (len(window) == HELD_GRIPPER_SAMPLES
                            and max(window) - min(window) <= GRIP_SETTLED)
        elif held:
            return i
    return None


def pick_signature(p: Perception, name: str) -> dict | None:
    """The footprint of name in p before a pick, with p's table_z; None if p lacks it."""
    o = p.objects.get(name) if p is not None else None
    return None if o is None else {**footprint(o), "table_z": float(p.table_z)}


def pick_verdict(name: str, sig0: dict, p: Perception, table_z: float, tcp, fresh) -> \
        tuple[bool, str]:
    """After a pick that reads fully closed: (held?, why), from where p sees the picked object."""
    tcp = np.asarray(tcp, np.float64)
    knocked = None
    for oid, o in p.objects.items():
        s = footprint(o)
        if (s["zmin"] > table_z + REST_M or float(np.linalg.norm(s["color"] - sig0["color"]))
                > COLOR_MAX or not similar_size(sig0, s)):
            continue
        d = float(np.linalg.norm(s["c"][:2] - sig0["c"][:2]))
        if d <= SPOT_MAX_M and abs(s["c"][2] - sig0["c"][2]) <= SPOT_MAX_M:
            return False, f"not held: {name} still at its pick spot"
        if ((oid == name or oid in fresh) and knocked is None
                and float(np.linalg.norm(s["c"] - tcp)) > HELD_TCP_DIST_M):
            knocked = s["c"]
    if knocked is not None:
        return False, (f"not held: {name} knocked to [{knocked[0]:.2f}, {knocked[1]:.2f}] on the "
                       "table")
    return True, f"held: {name} no longer at its pick spot (the gripper reads closed)"


def held_from(p: Perception, T_world_tcp) -> dict | None:
    """The perceived object a closed gripper at T_world_tcp holds, as held geometry."""
    T = np.asarray(T_world_tcp, np.float64)
    oid = object_at(p, T[:3, 3], max_dist=HELD_TCP_DIST_M)
    if oid is None or oid not in p.objects:
        return None
    o = p.objects[oid]
    return {"object": oid, "verts": np.asarray(o.verts, dtype=np.float64), "faces": o.faces,
            "color": tuple(o.color) if o.color is not None else None,
            "T_tcp_obj": np.linalg.inv(T)}
