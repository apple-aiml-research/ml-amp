#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The world model collision checks and assist plan against: the scene and the held object."""

from __future__ import annotations

import math

import numpy as np

from amp.geometry import world as scene_mod
from amp.link.wire import jsonable
from amp.planning.assist.predict import infer_held
from amp.planning.grasp import held_geometry

SCENE_IGNORED = "standard mode"
_EYE4 = np.eye(4)
CLEAR_OF_HAND_M = 0.08


def transform(T, verts) -> np.ndarray:
    v = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    return v @ T[:3, :3].T + T[:3, 3]


def _segment_distance(p, a, b):
    ab = b - a
    t = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-18), 0.0, 1.0)
    return np.linalg.norm(a + t[:, None] * ab - p, axis=1)


def point_mesh_distance(point, verts, faces) -> float:
    """Distance from point to the nearest triangle of a world-frame mesh."""
    p = np.asarray(point, dtype=np.float64).reshape(3)
    V = np.asarray(verts, dtype=np.float64).reshape(-1, 3)
    F = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    if not len(F):
        return float(np.min(np.linalg.norm(V - p, axis=1))) if len(V) else math.inf
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    ab, ac = b - a, c - a
    n = np.cross(ab, ac)
    nn = (n * n).sum(1)
    ok = nn > 1e-18
    s = np.where(ok, ((p - a) * n).sum(1) / np.where(ok, nn, 1.0), 0.0)
    w2 = p - s[:, None] * n - a
    d00, d01, d11 = (ab * ab).sum(1), (ab * ac).sum(1), (ac * ac).sum(1)
    d20, d21 = (w2 * ab).sum(1), (w2 * ac).sum(1)
    den = np.where(ok, d00 * d11 - d01 * d01, 1.0)
    v = (d11 * d20 - d01 * d21) / den
    w = (d00 * d21 - d01 * d20) / den
    inside = ok & (v >= 0) & (w >= 0) & (v + w <= 1)
    edges = np.minimum.reduce([_segment_distance(p, a, b), _segment_distance(p, b, c),
                               _segment_distance(p, c, a)])
    return float(np.where(inside, np.abs(s) * np.sqrt(nn), edges).min())


def _aabb(o) -> list:
    """The world AABB of a scene object at its pose (a held object's moves with the hand)."""
    v = o.verts if np.array_equal(o.pose, _EYE4) else o.world_verts()
    return [v.min(axis=0).tolist(), v.max(axis=0).tolist()]


class World:
    """Under the server lock: the episode's perceived scene and what the gripper holds."""

    def __init__(self):
        self.scene: scene_mod.Scene | None = None
        self.error: str | None = None
        self.meta: dict = {}
        self.version = 0
        self._display = None
        self.held: dict | None = None
        self.primary: str | None = None
        self.multi = False
        self._grippered: frozenset | None = None
        self._tcp_of = None
        self.held_geom: dict | None = None
        self.held_check = None
        self.held_reason: str | None = None
        self.ignored_episode = None
        self._ees: list = []
        self._clear: dict = {}

    def reset(self, episode, *, keep_scene: bool) -> None:
        """A new episode: drop the scene (unless it is already this episode's) and the held state."""
        if self.scene is not None and not (keep_scene and self.scene.episode == episode):
            self.scene, self.meta, self._display = None, {}, None
        if self.scene is None:
            self.error = None
        self._drop_held()
        self.scene, self.meta, self._display = None, {}, None
        self.held_geom = self.held_check = self.held_reason = None
        self._clear = {}

    def bind(self, ees, primary, tcp_of=None, grippered=None) -> None:
        """The robot's end effectors, its primary, tcp_of(ee): an ee's observed T_world_tcp, and
        grippered: those that can hold something (None = all; a tool without fingers cannot)."""
        self.primary, self.multi, self._tcp_of = primary, len(ees) > 1, tcp_of
        self._ees = list(ees)
        self._grippered = None if grippered is None else frozenset(grippered)

    @property
    def held_name(self) -> str | None:
        return self.held["object"] if self.held is not None else None

    @property
    def _holders(self) -> list:
        if self.held is None:
            return []
        return list(self.held.get("ees") or [self.primary])

    @property
    def held_ees(self) -> list:
        """The end effectors holding the held object, the one its geometry follows first."""
        return self._holders

    def held_by(self, ee=None) -> str | None:
        """The object an end effector (None: the primary) holds, or None."""
        ee = self.primary if ee is None else ee
        return self.held_name if ee in self.held_ees else None

    def _drop_held(self) -> None:
        self.held = None

    def _set_holders(self, holders) -> None:
        if self.multi and self.held is not None:
            self.held["ees"] = list(holders)

    def _hold(self, name, chunk_id, ee, **reason) -> bool:
        """ee holds name; True when it joins another end effector already holding it."""
        ee = self.primary if ee is None else ee
        if self._grippered is not None and ee not in self._grippered:
            raise ValueError(f"end effector {ee} has no gripper: it cannot hold {name}")
        holders = self._holders
        joins = (self.held is not None and self.held["object"] == name
                 and ee not in holders and self.multi)
        holders = [*holders, ee] if joins else [ee]
        self.held = {"object": name, "chunk_id": chunk_id, **reason}
        self._set_holders(holders)
        return joins

    def _tcp(self, ee):
        if self._tcp_of is None:
            return None
        try:
            return self._tcp_of(ee)
        except (KeyError, AttributeError, TypeError, ValueError):
            return None

    def _lead_tcp(self, T_world_tcp, ee=None):
        """The TCP the held geometry follows, given T_world_tcp of ee (None: the primary)."""
        ee = self.primary if ee is None else ee
        lead = self._holders[0] if self._holders else self.primary
        if lead == ee or not self.multi:
            return T_world_tcp
        return self._tcp(lead)

    def held_now(self) -> dict | None:
        """The held object's geometry, if it is still the held object."""
        g = self.held_geom
        if g is None or self.held is None or g["object"] != self.held["object"]:
            return None
        return g

    def set_held(self, name, chunk_id, geom, reason, ee=None) -> None:
        g = self.held_now()
        if self._hold(name, chunk_id, ee, reason=reason):
            if g is not None:
                geom = g
            elif geom is not None:
                self._set_holders([self._holders[-1], *self._holders[:-1]])
        self.held_geom = geom
        self.held_reason = reason

    def release(self, T_world_tcp):
        """The gripper opens on the held object: (name, posed world verts, faces, colour) or None."""
        g = self.held_now()
        self.held_geom = self.held_reason = None
        if g is None or T_world_tcp is None:
            return None
        return g["object"], transform(T_world_tcp @ g["T_tcp_obj"], g["verts"]), g["faces"], \
            g["color"]

    def gripper_opened(self, T_world_tcp, ee=None):
        """An end effector's gripper (None: the primary's) opened at T_world_tcp: (released name or
        None, posed mesh for perception or None)."""
        ee = self.primary if ee is None else ee
        if self.held_check is not None and (
                not self.multi or self.held_check.get("ee", self.primary) == ee):
            self.held_check = None
        if self.held is None or (self.multi and ee not in self._holders):
            return None, None
        rest = [e for e in self._holders if e != ee] if self.multi else []
        if rest:
            self._hand_over(T_world_tcp, ee, rest)
            return None, None
        name = self.held["object"]
        mesh = self.release(T_world_tcp)
        self._drop_held()
        return name, mesh

    def grip_held(self, chunk_id, T_world_tcp, ee=None) -> str | None:
        """held_tracking gripper: a closed gripper reads something held; hold the scene object
        nearest its TCP (as the planner infers it), or None when there is none."""
        sc = self.scene
        if sc is None or T_world_tcp is None or self.held is not None:
            return None
        T = np.asarray(T_world_tcp, dtype=np.float64)
        name = infer_held(sc, T, list(self._clear.items()))
        if name is not None and name not in sc.objects:
            self._reinstate(sc, name)
        if name is None:
            return None
        o = sc.objects[name]
        geom = held_geometry(name, o, o.pose, T)
        self.set_held(name, chunk_id, geom, "gripper", ee)
        return name

    def dropped(self, chunk_id, ee=None) -> str | None:
        """held_tracking gripper: ee's (None: the primary's) closed gripper lost its hold; the
        name it dropped, or None when it held nothing or another hand still holds it."""
        ee = self.primary if ee is None else ee
        if self.held is None or (self.multi and ee not in self._holders):
            return None
        name = self.held["object"]
        rest = [e for e in self._holders if e != ee] if self.multi else []
        if rest:
            self._hand_over(self._tcp(ee), ee, rest)
            return None
        self.held_geom = None
        self.held_reason = f"not held: {name} slipped from the gripper (chunk {chunk_id})"
        self._drop_held()
        return name

    def _hand_over(self, T_world_tcp, ee, rest) -> None:
        """ee lets go while rest hold on: the geometry follows the next holder's TCP."""
        g = self.held_now()
        if g is not None and self._holders[0] == ee:
            T_new = self._tcp(rest[0])
            if T_world_tcp is not None and T_new is not None:
                self.held_geom = {**g, "T_tcp_obj": np.linalg.inv(T_new) @ T_world_tcp
                                  @ g["T_tcp_obj"]}
        self._set_holders(rest)

    def placed(self, ee=None) -> str | None:
        """A place chunk of ee (None: the primary) finished: ee holds nothing; the name it held
        when nothing else holds it."""
        ee = self.primary if ee is None else ee
        if self.held is not None and self.multi:
            if ee not in self._holders:
                return None
            rest = [e for e in self._holders if e != ee]
            if rest:
                self._set_holders(rest)
                return None
        name = self.held_name
        self._drop_held()
        return name

    def picked(self, name, chunk_id, *, state=None, grip=None, geom=None, sig0=None,
               check=None, ee=None) -> tuple[str, str]:
        """A pick chunk finished: decide what is held; (log level, message), from the gripper
        (state "object" | "closed" | "open"), a fully closed one deferred to perception via check."""
        who = self.primary if ee is None else ee
        if self._grippered is not None and who not in self._grippered:
            return "warn", (f"pick chunk {chunk_id}: end effector {who} has no gripper: "
                            f"it holds nothing")
        if state == "object":
            self.set_held(name, chunk_id, geom, f"held: gripper {grip:.2f}", ee)
            return "info", f"chunk {chunk_id}: held: gripper {grip:.2f}"
        if state in ("closed", "jammed") and geom is not None and sig0 is not None:
            self.held_check = {"chunk_id": chunk_id, "check": check}
            if ee is not None and ee != self.primary:
                self.held_check["ee"] = ee
            reads = "closed" if state == "closed" else "still open (jammed)"
            return "info", (f"pick chunk {chunk_id}: the gripper reads {reads} ({grip:.4f}); "
                            f"checking whether {name} is still at its pick spot")
        why = (f"not held: the gripper closed fully ({grip:.4f}) and {name} has no pre-pick "
               "perception to compare" if state == "closed" else
               f"not held: the fingers did not close ({grip:.4f}) and {name} has no pre-pick "
               "perception to compare" if state == "jammed" else "not held: the gripper is open")
        self.held_reason = why
        return "warn", f"pick chunk {chunk_id}: {why}"

    def settle_check(self, check, verdict) -> tuple[str, str] | None:
        """Perception's verdict on a pending held check: (log level, message) or None."""
        if check is None or self.held_check is not check:
            return None
        self.held_check = None
        ee = check.get("ee")
        if verdict is None or (self.held is not None and not (
                self.multi and verdict.get("object") == self.held["object"]
                and ee not in self._holders)):
            return None
        if "faces" in verdict:
            self.set_held(verdict["object"], check["chunk_id"], verdict, verdict["reason"], ee)
            return "info", f"chunk {check['chunk_id']}: {verdict['reason']}"
        self.held_reason = verdict["reason"]
        return "warn", f"pick chunk {check['chunk_id']}: {verdict['reason']}"

    def ignore_scene(self, key) -> bool:
        """Standard mode ignores /sim/scene; True the first time for this episode key."""
        first = self.ignored_episode != key
        self.ignored_episode = key
        return first

    def held_mesh(self, T_world_tcp, ee=None):
        """(name, world verts, faces) of the held geometry, T_world_tcp being ee's (None: the
        primary's) TCP, or None."""
        g = self.held_now()
        T_world_tcp = self._lead_tcp(T_world_tcp, ee)
        if g is None or T_world_tcp is None:
            return None
        return g["object"], transform(T_world_tcp @ g["T_tcp_obj"], g["verts"]), g["faces"]

    def install_perceived(self, scene, p, T_world_tcp, ee=None) -> None:
        """Standard: perception's scene, the held object's mesh posed at the TCP."""
        g = self.held_now()
        T_world_tcp = self._lead_tcp(T_world_tcp, ee)
        if g is not None and T_world_tcp is not None:
            scene.objects[g["object"]] = scene_mod.SceneObject(
                g["object"], "object", np.asarray(g["verts"], dtype=np.float64),
                np.asarray(g["faces"]), T_world_tcp @ g["T_tcp_obj"], g["color"])
        self.version += 1
        scene.version = self.version
        self.scene = scene
        self._note_clear(scene, T_world_tcp, None if g is None else g["object"])
        self.meta = {n: {"aabb": _aabb(o),
                         "source": getattr(p.objects.get(n), "source", "perception")}
                     for n, o in scene.objects.items() if len(o.verts)}
        self._display, self.error = None, None

    def _note_clear(self, scene, T_world_tcp, held) -> None:
        """Keep each object's box from this pass if every TCP is clear of it, and the box of
        one this pass lost while a hand is near it."""
        tcps = [self._tcp(e) for e in self._ees if e != self.primary] if self.multi else []
        tcps = [np.asarray(T)[:3, 3] for T in [T_world_tcp, *tcps] if T is not None]

        def clear_of_hands(v):
            lo, hi = v.min(axis=0), v.max(axis=0)
            return all(np.linalg.norm(t - np.clip(t, lo, hi)) > CLEAR_OF_HAND_M for t in tcps)

        for n in [n for n in self._clear if n not in scene.objects]:
            if not tcps or n == held or clear_of_hands(self._clear[n].verts):
                del self._clear[n]
        if not tcps:
            return
        for n, o in scene.objects.items():
            if o.kind != "object" or n == held or not len(o.verts):
                continue
            v = o.world_verts()
            if clear_of_hands(v):
                self._clear[n] = scene_mod.SceneObject(n, o.kind, v, o.faces, np.eye(4), o.color)

    def _reinstate(self, sc, name) -> None:
        """Put back an object perception lost under the hand, as last seen clear of it."""
        r = self._clear[name]
        sc.objects[name] = scene_mod.SceneObject(name, r.kind, r.verts.copy(), r.faces,
                                                 np.eye(4), r.color)
        self.meta[name] = {"aabb": _aabb(sc.objects[name]), "source": "perception"}
        self.version += 1
        sc.version, self._display = self.version, None

    def track_held(self, T_world_tcp, ee=None) -> None:
        """Standard: move the held object's scene mesh with the observed TCP of ee (None: the
        primary), or with the holding end effector's own when that is another."""
        g = self.held_now()
        sc = self.scene
        T_world_tcp = self._lead_tcp(T_world_tcp, ee)
        if g is not None and sc is not None and g["object"] in sc.objects and \
                T_world_tcp is not None:
            o = sc.objects[g["object"]]
            o.pose = T_world_tcp @ g["T_tcp_obj"]
            if g["object"] in self.meta:
                self.meta[g["object"]]["aabb"] = _aabb(o)

    def set_table_z(self, z) -> None:
        if self.scene is not None:
            self.scene.table_z = z

    def scene_json(self) -> dict | None:
        sc = self.scene
        if sc is None:
            return None
        return {"episode": sc.episode, "version": sc.version,
                "objects": {n: o.kind for n, o in sc.objects.items()}}

    def boxes(self, episode) -> list:
        """World AABBs of this episode's scene objects (the table estimate ignores them)."""
        sc = self.scene if self.scene is not None and self.scene.episode == episode else None
        out = []
        for o in (sc.objects.values() if sc is not None else ()):
            v = o.world_verts()
            if len(v):
                out.append((v.min(axis=0), v.max(axis=0)))
        return out

    def display_snapshot(self):
        """Under the lock: (scene copy, meta, cached display meshes or None), or None."""
        if self.scene is None:
            return None
        cached = self._display
        meshes = cached[1] if cached is not None and cached[0] == self.scene.version else None
        return self.scene.copy(), dict(self.meta), meshes

    def cache_display(self, version, meshes) -> None:
        if self.scene is not None and self.scene.version == version:
            self._display = (version, meshes)

    def held_json(self) -> dict | None:
        return None if self.held is None else jsonable(dict(self.held))
