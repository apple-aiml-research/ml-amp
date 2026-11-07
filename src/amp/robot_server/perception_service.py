#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp.perception behind the server: the background pass, locks, info-mode gating, object ids."""

from __future__ import annotations

import dataclasses
import math
import threading
import time

import numpy as np

from amp.perception import (
    PerceptionConflict,
    PerceptionState,
    define_object,
    forget_object,
    object_at,
    object_json,
    perceive_pass,
    points_of,
    scan_from_frames,
    segment,
    to_scene,
)
from amp.perception.held import pick_signature
from amp.perception.params import params_for
from amp.perception.tools import box_from_aabb, box_from_obb, check_label_kind
from amp.robot_server import mobile
from amp.robot_server.errors import NotFound, SessionConflict
from amp.robot_server.world import point_mesh_distance

SEGMENT_RADIUS_MM = 11.0
OBJECT_AT_MAX_M = 0.03
MAX_ASSIST_DIST_M = 0.3
MEMORY_FEED_S = 1.0


class PerceptionService:
    """The perceived scene."""

    def __init__(self, srv, *, interval_s: float):
        self._srv = srv
        self.interval_s = interval_s
        self.reset()
        self.lock = threading.Lock()
        self._thread = None
        self._last = -math.inf
        self._fed = -math.inf
        self.seq = -1
        self.error: str | None = None
        self.info: dict | None = None
        self.closed = False

    def reset(self) -> None:
        """Under the server lock, at an episode start: a fresh state for the new episode."""
        emb, st = self._srv.embodiment, self._srv.settings
        params = dataclasses.replace(params_for(emb), floor_at_support=st.perception_floor)
        self.state = PerceptionState(embodiment=emb, params=params)
        self.info = self.error = None

    def due(self) -> bool:
        srv = self._srv
        return srv.obs.obs is not None and self.seq != srv.obs.frame_seq

    def kick(self) -> None:
        """Under the lock: start the background pass thread when a pass is due."""
        if (self._thread is not None or self._srv.executor.chunk is not None or self.closed
                or not self.due()):
            return
        self._thread = threading.Thread(target=self._loop, name="amp-perception", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        srv = self._srv
        try:
            while True:
                with srv.cond:
                    if srv.executor.chunk is not None or self.closed or not self.due():
                        self._thread = None
                        return
                    wait = self._last + self.interval_s - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                    continue
                self.ensure(background=True)
        except Exception as exc:  # noqa: BLE001
            srv.log("error", f"the perception thread failed: {exc!r}")
            with srv.cond:
                self._thread = None

    def ensure(self, background=False) -> None:
        """Perceive the latest observation unless that is done; call outside the server lock."""
        srv = self._srv
        with self.lock:
            with srv.cond:
                if not self.due() or (background and srv.executor.chunk is not None):
                    return
                self._last = time.monotonic()
                job = self._job()
            self._run(job)

    def _job(self) -> dict:
        """Under the lock: everything one pass needs, copied; the cloud is fused later by scan()."""
        srv = self._srv
        obs, world = srv.obs.obs, srv.world
        T_world_tcp = srv.obs.observed_tcp()
        g = world.held_now()
        check = world.held_check if g is None else None
        return {"seq": srv.obs.frame_seq, "token": srv.episode.token, "state": self.state,
                "frames": dict(obs.cameras), "q": np.asarray(obs.q, np.float64).copy(),
                "T_flange_ee": srv.obs.flange_ee(), "embodiment": srv.embodiment,
                "T_world_tcp": T_world_tcp, "T_world_base": obs.T_world_base,
                "table_z": float(srv.obs.table_z), "held": world.held_mesh(T_world_tcp),
                "check": check, "wanted": srv.planning.wanted_ids()}

    @staticmethod
    def scan(job):
        """Outside the server lock: the job's fused, cleaned scan (milliseconds of work)."""
        return scan_from_frames(job["frames"], q=job["q"], T_flange_ee=job["T_flange_ee"],
                                T_world_tcp=job["T_world_tcp"], embodiment=job["embodiment"],
                                T_world_base=job.get("T_world_base"))

    def _remember(self, job) -> None:
        """Outside the lock: a whole_body robot's frames into its episode's base memory."""
        emb, Tb = job["embodiment"], job.get("T_world_base")
        now = time.monotonic()
        if not emb.whole_body or Tb is None or now - self._fed < MEMORY_FEED_S:
            return
        self._fed = now
        try:
            mobile.remember(emb, job["frames"], job["q"], job["T_flange_ee"], Tb,
                            self._srv.obs.base_memory, token=job["token"], stamp=job["seq"])
        except Exception as exc:  # noqa: BLE001
            self._srv.log("warn", f"base memory: {type(exc).__name__}: {exc}")

    def _run(self, job) -> None:
        """Outside the server lock: one pass, then install its result unless it is stale."""
        srv = self._srv
        result, error = None, None
        check = job["check"]
        try:
            result = perceive_pass(job["state"], self.scan(job), table_z=job["table_z"],
                                   held=job["held"],
                                   check=None if check is None else check["check"],
                                   wanted=job["wanted"], tracer=srv.tracer_now())
        except Exception as exc:  # noqa: BLE001
            error = f"{type(exc).__name__}: {exc}"
        held_log, first = None, False
        with srv.cond:
            if job["token"] != srv.episode.token or job["seq"] < self.seq:
                return
            self.seq = job["seq"]
            if error is None:
                held_log = srv.world.settle_check(check, result.held)
                error = self._install(job["seq"])
            if error is not None:
                first = error != self.error
                self.error = error
        self._remember(job)
        if error is not None:
            if first:
                srv.log("warn", f"perception failed: {error}")
        elif held_log is not None:
            srv.log(*held_log)

    def _install(self, seq) -> str | None:
        """Under the lock: the state's perception becomes the episode's scene; an error or None."""
        srv = self._srv
        p = self.state.perception
        try:
            scene = to_scene(p, srv.episode.episode, srv.world.version + 1)
        except Exception as exc:  # noqa: BLE001
            return f"to_scene: {type(exc).__name__}: {exc}"
        srv.world.install_perceived(scene, p, srv.obs.observed_tcp())
        self.info = {"seq": seq, "time_s": float(p.time_s), "table_z": float(p.table_z),
                     "objects": [object_json(o) for o in p.objects.values()],
                     "notes": [str(n) for n in (p.notes or ())]}
        self.error = None
        srv.planning.kick()
        srv.cond.notify_all()
        return None

    def state_json(self) -> dict | None:
        """Under the lock: api/state's "perception"."""
        if self.info is None:
            return None
        world = self._srv.world
        return {**self.info, "objects": [dict(o) for o in self.info["objects"]],
                "notes": list(self.info["notes"]), "held": world.held_name,
                "held_reason": world.held_reason}

    def note_release(self, released) -> None:
        """Under the lock: the executor let an object go; expect it where it was released."""
        if released is not None:
            name, verts, faces, color = released
            self.state.note_release(name, verts.mean(axis=0), verts, faces, color)

    def pre_pick_sig(self, name):
        return pick_signature(self.state.perception, name)

    def no_scene_reason(self) -> str:
        return self.error or "it has not run yet"

    def resolve(self, oid):
        """Under the lock: (current id, resolution or None) of an object id."""
        sc = self._srv.world.scene
        return self.state.resolve(oid, set(sc.objects) if sc is not None else set())

    def resolve_request_ids(self, body):
        """(body with every stale object id replaced by its successor, [resolutions])."""
        if not isinstance(body, dict):
            return body, []
        with self._srv.cond:
            sc = self._srv.world.scene
            return self.state.resolve_request_ids(body, set(sc.objects) if sc else set())

    def _perceived_locked(self):
        self._srv.observations_live()
        if self.state.perception is None:
            raise SessionConflict(f"no perception yet ({self.error or 'it has not run yet'})")

    def _tool_job(self):
        """Under the lock: (job, held mesh, episode token) for an agent tool; scan(job) later."""
        self._perceived_locked()
        job = self._job()
        return job, job["held"], job["token"]

    def _commit(self, token) -> None:
        """Under the lock: install the state after an agent tool edited it."""
        if token != self._srv.episode.token:
            raise SessionConflict("the episode changed; look again")
        seq = self.info["seq"] if self.info is not None else self.seq
        err = self._install(seq)
        if err is not None:
            raise SessionConflict(f"perception failed: {err}")

    def _object_reply(self, oid) -> dict:
        for o in self.info["objects"]:
            if o["id"] == oid:
                return {"ok": True, "object": dict(o)}
        return {"ok": True, "object": None}

    def segment(self, body) -> dict:
        """POST api/agent/segment: region-grow the cleaned cloud from the point under (u, v)."""
        srv = self._srv
        cam = body.get("cam")
        if not isinstance(cam, str) or not cam:
            raise ValueError(f"cam must be a camera name, got {cam!r}")
        for k in ("u", "v"):
            x = body.get(k)
            if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
                raise ValueError(f"{k} must be a pixel coordinate, got {x!r}")
        r = body.get("radius_mm", SEGMENT_RADIUS_MM)
        if isinstance(r, bool) or not isinstance(r, (int, float)) or not 2.0 <= r <= 50.0:
            raise ValueError(f"radius_mm must be a number in [2, 50], got {r!r}")
        tol = body.get("color_tol")
        if tol is not None and (isinstance(tol, bool) or not isinstance(tol, (int, float))
                                or not tol > 0):
            raise ValueError(f"color_tol must be a positive number or null, got {tol!r}")
        label, kind = body.get("label"), body.get("kind")
        check_label_kind(label, kind)
        with self.lock:
            with srv.cond:
                job, held, token = self._tool_job()
                frames = srv.obs.obs.cameras
                if cam not in frames:
                    raise NotFound(f"unknown camera {cam!r}; the observation has "
                                   f"{sorted(frames)}")
                fr = frames[cam]
            hit = srv.obs.pick(fr, cam, float(body["u"]), float(body["v"]))
            if not hit.get("ok"):
                raise SessionConflict(f"the pixel hits nothing: {hit.get('message')}")
            state, scan = self.state, self.scan(job)
            obj = self._tool(lambda: segment(state, scan, hit["point"], radius=float(r) / 1000,
                                             color_tol=tol, label=label, kind=kind, held=held,
                                             tracer=srv.tracer_now()))
            with srv.cond:
                self._commit(token)
                return self._object_reply(obj.id)

    @staticmethod
    def _tool(fn):
        try:
            return fn()
        except PerceptionConflict as exc:
            raise SessionConflict(str(exc)) from None

    def define_object(self, body) -> dict:
        """POST api/agent/define_object: an agent object from the cloud in a box, or from points."""
        srv = self._srv
        given = [k for k in ("aabb", "obb", "points") if body.get(k) is not None]
        if len(given) != 1:
            raise ValueError("define_object takes exactly one of 'aabb', 'obb' or 'points'")
        label, kind = body.get("label"), body.get("kind")
        check_label_kind(label, kind)
        replace = body.get("replace")
        if replace is None:
            replace = []
        if not isinstance(replace, (list, tuple)) or not all(isinstance(v, str) for v in replace):
            raise ValueError(f"replace must be a list of object ids, got {replace!r}")
        box = points = None
        if given[0] == "points":
            points = body["points"]
        else:
            box = box_from_aabb(body["aabb"]) if given[0] == "aabb" else box_from_obb(body["obb"])
        with self.lock:
            with srv.cond:
                job, held, token = self._tool_job()
            state, scan = self.state, self.scan(job)
            obj = self._tool(lambda: define_object(state, scan, box=box, points=points,
                                                   label=label, kind=kind, replace=replace,
                                                   held=held, tracer=srv.tracer_now()))
            with srv.cond:
                self._commit(token)
                out = self._object_reply(obj.id)
        if replace:
            out["replaced"] = list(replace)
        return out

    def forget_object(self, body) -> dict:
        """POST api/agent/forget_object: drop an object; nothing is perceived there again."""
        srv = self._srv
        oid = body.get("id")
        if not isinstance(oid, str) or not oid:
            raise ValueError(f"id must be an object id, got {oid!r}")
        with self.lock, srv.cond:
            self._perceived_locked()
            token = srv.episode.token
            self._tool(lambda: forget_object(self.state, oid, held=srv.world.held_name,
                                             tracer=srv.tracer_now()))
            self._commit(token)
        return {"ok": True}

    def object_points(self, oid):
        """GET api/agent/object_points/<id>: ((N, 6) float32 array, id, resolution or None)."""
        with self._srv.cond:
            oid, resolved = self.resolve(oid)
            self._perceived_locked()
            arr = self._tool(lambda: points_of(self.state, oid, held=self._srv.world.held_name))
        return arr, oid, resolved

    def reperceive(self, body) -> dict:
        """POST api/agent/reperceive: a perception pass now, on the current frame."""
        srv = self._srv
        with self.lock:
            with srv.cond:
                srv.observations_live()
                self._last = time.monotonic()
                job = self._job()
            self._run(job)
        with srv.cond:
            info = self.info
            if info is None or info["seq"] != job["seq"] or self.error is not None:
                raise SessionConflict(f"perception failed: {self.error or 'superseded'}")
            return {"ok": True, "objects": [dict(o) for o in info["objects"]],
                    "notes": list(info["notes"]), "perception_seq": int(info["seq"])}

    def object_at_reply(self, point, max_dist) -> dict:
        """POST api/assist/object_at: the object whose surface is nearest a world point."""
        srv = self._srv
        self.ensure()
        with srv.cond:
            scene = srv.world.scene.copy() if srv.world.scene is not None else None
            p = self.state.perception
            held = srv.world.held_name
        best, best_d = None, None
        if p is not None:
            oid = object_at(p, point, max_dist=max_dist)
            if oid is not None:
                ds = []
                if oid in p.objects and len(p.objects[oid].points):
                    pts = np.asarray(p.objects[oid].points, dtype=np.float64).reshape(-1, 3)
                    ds.append(float(np.min(np.linalg.norm(pts - point, axis=1))))
                if scene is not None and oid in scene.objects:
                    o = scene.objects[oid]
                    ds.append(point_mesh_distance(point, o.world_verts(), o.faces))
                best, best_d = oid, (min(ds) if ds else None)
        names = () if scene is None else (
            [held] if held in scene.objects else list(scene.objects))
        if best is None:
            for n in names:
                o = scene.objects[n]
                d = point_mesh_distance(point, o.world_verts(), o.faces)
                if d <= max_dist and (best_d is None or d < best_d):
                    best, best_d = n, d
        return {"ok": True, "id": best, "distance_m": best_d}

    def observation_fields(self) -> tuple:
        """Under the lock: (perception_seq, objects, notes) of api/agent/observation."""
        if self.info is None:
            return None, [], []
        return (int(self.info["seq"]), [dict(o) for o in self.info["objects"]],
                [str(n) for n in self.info["notes"]])
