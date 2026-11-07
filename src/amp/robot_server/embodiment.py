#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The server's embodiment runtime, built from what the backend declared, and its action layout."""

from __future__ import annotations

import threading

import numpy as np

from amp.contracts.embodiment import DROID
from amp.geometry.embodiment import Embodiment


def key(declared) -> tuple:
    """What identifies a declared robot's runtime: its spec (None = DROID) and description sha."""
    return declared.embodiment, declared.description_sha256


def runtime(declared) -> Embodiment | None:
    """The runtime of a DeclaredRobot; None while a chain arm still waits for /sim/robot."""
    if declared.embodiment is None:
        return Embodiment.droid()
    if not declared.complete:
        return None
    blobs = None
    if declared.description is not None:
        blobs = {}
        for name, (verts, faces) in declared.meshes.items():
            blobs[f"verts/{name}"], blobs[f"faces/{name}"] = verts, faces
    emb = Embodiment(declared.embodiment, declared.description, blobs)
    if not emb.spec.end_effectors:
        raise ValueError(f"embodiment {emb.name}: the robot server needs an end effector")
    Layout(emb)
    return emb


def warm_async(emb: Embodiment | None, log=None) -> threading.Thread | None:
    """Start building a whole_body runtime's sphere covers (about 17 s for R1Pro) off the request
    path, so the agent's first call finds them; None for any other robot."""
    if emb is None or not emb.whole_body:
        return None

    def run():
        try:
            emb.warm()
        except Exception as exc:  # noqa: BLE001
            if log is not None:
                log("warn", f"sphere warm-up for {emb.name} failed: {exc}")

    t = threading.Thread(target=run, name=f"warm-{emb.name}", daemon=True)
    t.start()
    return t


def is_droid(emb: Embodiment) -> bool:
    return emb.name == DROID


def limits(emb: Embodiment) -> dict[str, np.ndarray]:
    """q_min, q_max, qd_max, q_home over every arm's then joint group's joints, in proprio order."""
    arms = (*emb.spec.arms, *emb.spec.joint_groups)
    return {f: np.array([x for a in arms for x in getattr(a, f)], dtype=np.float64)
            for f in ("q_min", "q_max", "qd_max", "q_home")}


class Layout:
    """Where the joint targets, gripper commands and base motion sit in an embodiment's action."""

    def __init__(self, emb: Embodiment):
        spec = emb.spec
        start, k = {}, 0
        self.arm_q: dict[str, np.ndarray] = {}
        self.group_q: dict[str, np.ndarray] = {}
        for chains, into in ((spec.arms, self.arm_q), (spec.joint_groups, self.group_q)):
            for a in chains:
                start[a.name] = k
                into[a.name] = np.arange(k, k + len(a.joints))
                k += len(a.joints)
        pos = np.full(spec.dof, -1, dtype=np.int64)
        self.grippers: dict[str, int] = {}
        self.base: slice | None = None
        i = 0
        for s in spec.action:
            if s.kind == "joint_pos_abs":
                pos[start[s.target]:start[s.target] + s.dim] = np.arange(i, i + s.dim)
            elif s.kind == "base_pose_delta":
                self.base = slice(i, i + s.dim)
            else:
                self.grippers[s.target] = i
            i += s.dim
        if (pos[:spec.arm_dof] < 0).any():
            raise ValueError(f"embodiment {spec.name}: every arm joint needs a joint_pos_abs "
                             "action")
        if (pos < 0).any():
            raise ValueError(f"embodiment {spec.name}: every joint group needs a joint_pos_abs "
                             "action")
        self.dim, self.dof = i, spec.dof
        lo = int(pos[0])
        self.joints = slice(lo, lo + spec.dof) if np.array_equal(
            pos, np.arange(lo, lo + spec.dof)) else pos
        self.primary = emb.ee().name
        self.gripper = self.grippers.get(self.primary)
        self.columns = pos
        self.ee_arm: dict[str, str] = {e.name: e.arm for e in spec.end_effectors}
        self.ees: tuple[str, ...] = tuple(self.ee_arm)
        self.gripper_ees: tuple[str, ...] = tuple(
            e.name for e in spec.end_effectors if e.gripper is not None)
        self.ee_q: dict[str, np.ndarray] = {e: self.arm_q[a] for e, a in self.ee_arm.items()}
        self.ee_joints: dict[str, np.ndarray] = {e: pos[q] for e, q in self.ee_q.items()}

    def grip(self, action) -> float:
        """The primary end effector's gripper command (0 when it has none)."""
        return float(action[self.gripper]) if self.gripper is not None else 0.0

    def grip_of(self, action, ee: str | None = None) -> float:
        """An end effector's gripper command in an action (None: the primary; 0 without one)."""
        i = self.grippers.get(self.primary if ee is None else ee)
        return float(action[i]) if i is not None else 0.0

    def compose(self, q, gripper) -> np.ndarray:
        """An action holding q with every gripper at the given command."""
        a = np.zeros(self.dim, dtype=np.float64)
        a[self.joints] = np.asarray(q, dtype=np.float64)
        for i in self.grippers.values():
            a[i] = float(gripper)
        return a

    def compose_rows(self, q_full, grip_by_ee) -> np.ndarray:
        """(n, dim) actions from (n, dof) full q and per-ee gripper commands (scalars or (n,))."""
        q = np.asarray(q_full, dtype=np.float64)
        if q.ndim != 2 or q.shape[1] != self.dof:
            raise ValueError(f"q_full must be (n, {self.dof}), got {q.shape}")
        out = np.zeros((len(q), self.dim), dtype=np.float64)
        out[:, self.joints] = q
        for ee, i in self.grippers.items():
            g = grip_by_ee.get(ee)
            out[:, i] = 0.0 if g is None else np.asarray(g, dtype=np.float64)
        return out

    def plain(self, ee: str | None = None) -> bool:
        """Whether a legacy plan row [ee's arm joints..., gripper] already is the action."""
        ee = self.primary if ee is None else ee
        cols = self.ee_joints[ee]
        n = len(cols)
        return (self.dim == n + 1 and self.dof == n and np.array_equal(cols, np.arange(n))
                and self.grippers.get(ee) == n)

    def compose_plan(self, rows, ee, hold_q, grip_by_ee) -> np.ndarray:
        """Legacy plan rows [ee's arm joints..., gripper] as actions; every other arm at hold_q."""
        rows = np.asarray(rows, dtype=np.float64)
        idx = self.ee_q[ee]
        n = len(idx)
        if rows.ndim != 2 or rows.shape[1] != n + 1:
            raise ValueError(f"a plan step has {rows.shape[-1] if rows.ndim else 0} values; "
                             f"{ee}'s arm and gripper take {n + 1}")
        q = np.tile(np.asarray(hold_q, dtype=np.float64).reshape(1, self.dof), (len(rows), 1))
        q[:, idx] = rows[:, :n]
        return self.compose_rows(q, {**grip_by_ee, ee: rows[:, n]})


def _gripper_json(g, runtime=None) -> dict | None:
    """A gripper's observation block; None for a tool without fingers."""
    if g is None:
        return None
    out = {"kind": g.kind, "model": g.model, "stroke_m": g.stroke_m,
           "max_width_m": g.max_width_m, "closed_empty": g.closed_empty,
           "held_below": g.held_below}
    profile = getattr(runtime, "profile", None)
    if profile is not None:
        out["tip_reach_m"] = {"open": round(float(profile.tip_reach(0.0)), 4),
                              "closing": round(float(profile.max_tip_reach(profile.closed_angle)), 4)}
    elif g.tip_reach_m:
        out["tip_reach_m"] = {"open": round(g.tip_reach_m[0], 4),
                              "closing": round(g.tip_reach_m[1], 4)}
    return out


def observation_json(emb: Embodiment) -> dict:
    """api/agent/observation's embodiment block; format-2 keys only for a robot that has them."""
    spec = emb.spec
    out = {
        "name": spec.name,
        "cameras": [{"name": c.name, "width": int(c.width), "height": int(c.height),
                     "role": c.role, "depth": bool(c.depth)} for c in spec.cameras],
        "arms": [{"name": a.name, "joints": list(a.joints), "q_min": list(a.q_min),
                  "q_max": list(a.q_max), "q_home": list(a.q_home)} for a in spec.arms],
        "end_effectors": [{"name": e.name, "arm": e.arm, "closing_axis": e.closing_axis,
                           "approach_axis": e.approach_axis,
                           "gripper": _gripper_json(e.gripper, emb.gripper(e.name)
                                                    if e.gripper is not None else None)}
                          for e in spec.end_effectors],
        "workspace": {"support": spec.workspace.support, "box": list(spec.workspace.box)},
    }
    if spec.workspace.frame != "root":
        out["workspace"]["frame"] = spec.workspace.frame
    if spec.joint_groups:
        out["joint_groups"] = [{"name": g.name, "kind": g.kind, "joints": list(g.joints),
                                "q_min": list(g.q_min), "q_max": list(g.q_max),
                                "q_home": list(g.q_home), "carries": list(g.carries)}
                               for g in spec.joint_groups]
    if spec.base is not None:
        out["base"] = {"name": spec.base.name, "kind": spec.base.kind,
                       "qd_max": list(spec.base.qd_max)}
    return out
