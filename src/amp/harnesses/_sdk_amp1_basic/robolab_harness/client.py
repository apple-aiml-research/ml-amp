#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The agent's client: one robot episode through the harness gateway."""

from __future__ import annotations

import functools
import io
import json
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Mapping
from contextlib import contextmanager
from pathlib import Path

import numpy as np
from PIL import Image

from . import _robot, _trace, tlf
from . import geometry as _geometry
from .geometry import (
    OBB,
    Camera,
    View,
    make_pose,
    pose_to_T,
    waypoint,
)
from .render import (
    DEFAULT_LAYERS,
    Obj,
    SceneData,
    annotate_image,
    grid,
    outside_spheres,
    render_scene,
    spheres_from_links,
)
from .render import crop as _crop_image

CAMERAS = ("ego_l", "ego_r", "wrist")
_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))
# POST routes that take an "ee" (absent = the primary end effector); EEHandle injects it.
EE_ROUTES = frozenset({
    "api/agent/ik", "api/agent/ik_batch", "api/agent/fk", "api/agent/check",
    "api/agent/check_batch", "api/agent/robot", "api/plan/preview", "api/plan/execute",
    "api/hold", "api/home", "api/gripper"})


def _ep_stack() -> list[str]:
    """The public Episode methods on the call stack, outermost first: which ep calls led here."""
    out, f = [], sys._getframe(2)
    while f is not None:
        name = f.f_code.co_name
        if not name.startswith("_") and isinstance(f.f_locals.get("self"), Episode):
            out.append(name)
        f = f.f_back
    return out[::-1]


def _trace_request(method: str, path: str, status, t0: float, nbytes) -> None:
    """One harness request event per HTTP call; "stack" is the chain of public ep methods that
    sent it (empty when none did)."""
    if not _trace.enabled():
        return
    try:
        _trace.emit("request", {"method": method, "path": path, "status": status,
                                "latency_s": round(time.monotonic() - t0, 4), "bytes": nbytes,
                                "stack": _ep_stack()})
    except Exception:  # noqa: BLE001, S110 - a trace failure must never reach the model's code
        pass


def issues(reply, severity="error") -> list:
    """The issue messages of a preview / refusal (also under its "preview"); severity=None: all."""
    if not isinstance(reply, dict):
        return []
    found = reply.get("issues")
    if found is None and isinstance(reply.get("preview"), dict):
        found = reply["preview"].get("issues")
    return [str(i.get("message")) for i in found or [] if isinstance(i, dict) and i.get("message")
            and (severity is None or i.get("severity", "error") == severity)]


def explain(reply, message=None) -> str:
    """One line: the reply's message plus every error issue it does not already mention."""
    r = reply if isinstance(reply, dict) else {}
    msg = str(message or r.get("message") or ("ok" if r.get("ok") else "refused"))
    extra = [m for m in issues(reply) if m not in msg]
    if extra:
        msg += ": " + "; ".join(extra)
    if not isinstance(reply, dict):
        return msg
    pv = reply.get("preview") if isinstance(reply.get("preview"), dict) else reply
    md = pv.get("min_duration")
    if md is not None and "min_duration" not in msg and any(
            i.get("kind") == "velocity" for i in pv.get("issues") or [] if isinstance(i, dict)):
        msg += f" (min_duration {float(md):.2f} s)"
    return msg


class HarnessError(RuntimeError):
    """A refused or failed request; `message` is the server's explanation, with every error issue
    of the reply appended; `reply` the reply JSON."""

    def __init__(self, message, status=None, path=None, reply=None):
        super().__init__(f"{message} [{status} {path}]" if status else message)
        self.message, self.status, self.path, self.reply = message, status, path, reply


class EpisodeOver(HarnessError):
    """The episode has ended (HTTP 410, or a 409 saying so): nothing more can be done in it."""


# The robot server's 409 replies once the episode has ended (or is ending on publish).
_OVER_MESSAGES = ("the episode is over", "mode 'episode_over'", "the episode is being published")


def _over_reply(reply, msg) -> bool:
    """Does a 409 reply say the episode has ended?"""
    if isinstance(reply, dict) and reply.get("mode") == "episode_over":
        return True
    return any(m in str(msg) for m in _OVER_MESSAGES)


class Objects(list):
    """A list of Obj that can also be indexed by id: objs["obj3"]; `.notes` perception's notes."""

    notes: list
    seq: int | None

    def __init__(self, items=(), notes=None, seq=None):
        super().__init__(items)
        self.notes, self.seq = list(notes or []), seq

    def __getitem__(self, key):
        if isinstance(key, str):
            for o in self:
                if o.id == key:
                    return o
            raise KeyError(f"no object {key!r}; perceived: {[o.id for o in self]}")
        return super().__getitem__(key)

    def get(self, key, default=None):
        try:
            return self[key]
        except KeyError:
            return default

    def ids(self) -> list:
        return [o.id for o in self]

    def find(self, label=None, color=None, kind=None, near=None, min_size=None,
             max_size=None) -> Objects:
        """Objects matching every given filter, nearest to `near` first: label / color substrings
        (color by its basic name), kind ("object" | "fixture"), min/max aabb size (m, scalar or
        3-vector)."""
        out = []
        for o in self:
            if label and label.lower() not in (o.label or "").lower():
                continue
            if color and (o.color is None or color.lower() not in color_name(o.color)):
                continue
            if kind and o.kind != kind:
                continue
            if o.aabb is not None and min_size is not None and np.any(o.size < min_size):
                continue
            if o.aabb is not None and max_size is not None and np.any(o.size > max_size):
                continue
            out.append(o)
        if near is not None:
            p = np.asarray(near, float).reshape(3)
            out.sort(key=lambda o: _dist(o, p))
        return Objects(out, self.notes, self.seq)

    def nearest(self, point, kind=None, max_dist=None):
        """The object whose centroid is nearest to point (within max_dist m), or None."""
        found = self.find(kind=kind, near=point)
        if not found or (max_dist is not None and _dist(found[0], point) > max_dist):
            return None
        return found[0]


def _dist(o, p) -> float:
    if o.centroid is None:
        return float("inf")
    return float(np.linalg.norm(np.asarray(o.centroid, float) - np.asarray(p, float).reshape(3)))


class Cloud(tuple):
    """(xyz float32 (N, 3), rgb uint8 (N, 3), cam_index uint8 (N,)) with `.cams` and `.seq`."""

    cams: list
    seq: int | None

    def __new__(cls, xyz, rgb, cam_index, cams, seq=None):
        self = super().__new__(cls, (xyz, rgb, cam_index))
        self.cams, self.seq = list(cams), seq
        return self

    @property
    def xyz(self):
        return self[0]

    @property
    def rgb(self):
        return self[1]

    @property
    def cam_index(self):
        return self[2]


def _obj_id(obj) -> str:
    if isinstance(obj, Obj):
        return obj.id
    if isinstance(obj, dict):
        return obj.get("id") or obj["object"]
    return str(obj)


def _tcp(tcp) -> list:
    """A Pose7 from a Pose7, a dict with "tcp", or a 4x4."""
    if isinstance(tcp, dict):
        tcp = tcp["tcp"]
    a = np.asarray(tcp, dtype=np.float64)
    if a.shape == (4, 4):
        from .geometry import T_to_pose

        return T_to_pose(a).tolist()
    return a.reshape(7).tolist()


def _q(q, ee=None):
    """q as a list: the full q (every arm's joints in order) or the arm of `ee` (default the primary's)."""
    if q is None:
        return None
    a = np.asarray(q, dtype=np.float64).reshape(-1)
    rob = _robot.ACTIVE
    arm = rob.arm_dof(ee)
    if a.size not in (rob.full_dof, arm):
        want = f"{rob.full_dof}" if arm == rob.full_dof else f"{rob.full_dof} (all arms) or {arm}"
        raise ValueError(f"q has {a.size} values; expected {want}")
    return a.tolist()


def _make_obj(name, p: dict, meta: dict, v, f) -> Obj:
    """An Obj from its A1 JSON `p`, the scene header entry `meta` and its world mesh."""
    aabb = p.get("aabb") or meta.get("aabb") or (
        None if v is None or not len(v) else [np.min(v, 0).tolist(), np.max(v, 0).tolist()])
    aabb = None if aabb is None else np.asarray(aabb, np.float64)
    color = p.get("color") or meta.get("color")
    centroid = p.get("centroid")
    if centroid is None and aabb is not None:
        centroid = aabb.mean(0)
    label = p.get("label") or meta.get("label")
    if not label and aabb is not None:
        cm = np.round((aabb[1] - aabb[0]) * 100).astype(int)
        label = (f"{color_name(color)} · " if color is not None else "") + \
            f"{cm[0]}x{cm[1]}x{cm[2]} cm"
    source = p.get("source")
    if source is None and meta.get("source") in ("perception", "agent"):
        source = meta["source"]
    return Obj(id=name, label=label or "",
               color=None if color is None else tuple(int(c) for c in color),
               centroid=None if centroid is None else np.asarray(centroid, float),
               aabb=aabb, verts=v, faces=f, kind=p.get("kind") or meta.get("kind") or "object",
               n_points=p.get("n_points"), obb=p.get("obb") or meta.get("obb"),
               source=source,
               pose=meta.get("pose"))


def color_name(rgb) -> str:
    """A basic colour name for an RGB triple (by hue, saturation and value ranges)."""
    import colorsys

    r, g, b = (float(c) / 255.0 for c in np.asarray(rgb, dtype=np.float64).reshape(3))
    h, sat, v = colorsys.rgb_to_hsv(r, g, b)
    h *= 360
    if v < 0.18:
        return "black"
    if sat < 0.22:
        return "white" if v > 0.8 else ("grey" if v > 0.35 else "dark grey")
    if (h < 45 or h >= 340) and v < 0.55 and h >= 10:
        name = "brown"
    else:
        bins = [(15, "red"), (42, "orange"), (70, "yellow"), (165, "green"), (195, "cyan"),
                (255, "blue"), (290, "purple"), (340, "pink"), (361, "red")]
        name = next(n for hi, n in bins if h < hi)
    return ("dark " + name) if v < 0.35 and name != "brown" else name


class Episode:
    """One episode. Every call is an HTTP request under `base_url` (the gateway's s/<token>/)."""

    _ee_routes = EE_ROUTES  # a subclass that adds ee-taking routes widens it

    def __init__(self, base_url: str | None = None, *, worker_url: str | None = None,
                 info: dict | None = None, timeout_s: float = 120.0):
        if (base_url is None) == (worker_url is None):
            raise ValueError("give exactly one of base_url and worker_url")
        # worker_url talks to a worker with no gateway: for the SDK's own tests only.
        self.direct = worker_url is not None
        self.base_url = (base_url or worker_url).rstrip("/") + "/"
        self.info = dict(info or {})
        self.timeout_s = timeout_s
        self.instruction = self.info.get("instruction")
        self._obs: dict | None = None
        self._robot = _robot.DROID
        self.resolved: list = []
        self._tl = threading.local()  # .ee: the end effector an EEHandle call is bound to

    # ---- construction -------------------------------------------------------------------

    @classmethod
    def from_file(cls, path="episode.json") -> Episode:
        info = json.loads(Path(path).read_text())
        base = info.get("base") or f"s/{info['token']}/"
        return cls(info["gateway"].rstrip("/") + "/" + base.lstrip("/"), info=info)

    def __repr__(self) -> str:
        return f"Episode({self.base_url!r})"

    # ---- HTTP ---------------------------------------------------------------------------

    def _request(self, method: str, path: str, body=None, *, binary=False, timeout=None,
                 retries=None):
        url = self.base_url + path.lstrip("/")
        data = None if body is None else json.dumps(body).encode()
        headers = {"Content-Type": "application/json"} if data is not None else {}
        n_try = (3 if method == "GET" else 1) if retries is None else retries
        last = None
        for attempt in range(n_try):
            req = urllib.request.Request(url, data=data, headers=headers, method=method)
            t0 = time.monotonic()
            try:
                with _OPENER.open(req, timeout=timeout or self.timeout_s) as r:
                    raw = r.read()
                    _trace_request(method, path, r.status, t0, len(raw))
                    if binary:
                        return raw, r.headers
                    reply = json.loads(raw) if raw else {}
                    self._note_resolved(reply)
                    return reply
            except urllib.error.HTTPError as e:
                raw = e.read()
                _trace_request(method, path, e.code, t0, len(raw))
                try:
                    reply = json.loads(raw)
                    msg = reply.get("message") or reply.get("error") or raw.decode()[:300]
                    msg = explain(reply, msg)
                except (ValueError, AttributeError):
                    reply, msg = None, raw.decode(errors="replace")[:300] or e.reason
                over = e.code == 410 or e.code == 409 and _over_reply(reply, msg)
                cls = EpisodeOver if over else HarnessError
                raise cls(str(msg), e.code, path, reply) from None
            except (urllib.error.URLError, ConnectionError, TimeoutError) as e:
                _trace_request(method, path, None, t0, None)
                last = e
                if attempt + 1 < n_try:
                    time.sleep(1.0 + attempt)
        raise HarnessError(f"cannot reach {url}: {last}", None, path)

    def _get(self, path, **kw):
        return self._request("GET", path, **kw)

    def _post(self, path, body=None, *, check=True, **kw):
        body = {} if body is None else body
        ee = self._ee
        if ee is not None and path.lstrip("/") in self._ee_routes and isinstance(body, dict) \
                and "ee" not in body and "ee_waypoints" not in body:
            body = {**body, "ee": ee}
        reply = self._request("POST", path, body, **kw)
        if check and isinstance(reply, dict) and reply.get("ok") is False:
            raise HarnessError(explain(reply), 200, path, reply)
        return reply

    def _note_resolved(self, reply) -> None:
        """A stale id the server resolved to its successor: keep it in .resolved and print it."""
        if not isinstance(reply, dict):
            return
        res = reply.get("resolved_all") or reply.get("resolved")
        if not res:
            return
        for r in res if isinstance(res, list) else [res]:
            self.resolved.append(r)
            why = f" ({r.get('why')})" if r.get("why") else ""
            print(f"[sdk] note: {r.get('from')} is now {r.get('to')}{why}", flush=True)

    def _missing(self, e: HarnessError) -> bool:
        return self.direct and e.status == 404

    @property
    def _ee(self) -> str | None:
        return getattr(self._tl, "ee", None)

    @contextmanager
    def _bind(self, name):
        prev = self._ee
        self._tl.ee = name
        try:
            with _robot.bound_ee(name):
                yield
        finally:
            self._tl.ee = prev

    def _robot_block(self, obs, ee=None) -> dict:
        """obs["robot"] with the bound (or given) end effector's own pose and gripper state laid over it."""
        rb = obs.get("robot") or {}
        ee = ee or self._ee
        sub = (rb.get("end_effectors") or {}).get(ee) if ee else None
        if isinstance(sub, dict):
            rb = {**rb, **{k: v for k, v in sub.items() if k != "q_slice"}}
        if not self._robot.for_ee(ee, strict=False).has_gripper:
            rb = {k: v for k, v in rb.items() if not k.startswith("gripper_")}
        return rb

    def _need_gripper(self, call: str, rob=None) -> None:
        """Refuse a gripper call on a tool without fingers, naming the end effector."""
        rob = (rob or self._robot).for_ee(self._ee, strict=False)
        if not rob.has_gripper:
            raise HarnessError(f"end effector {self._ee or rob.primary_ee or rob.name!r} has no "
                               f"gripper (a tool without fingers): {call}() needs one", None, call)

    def _check_ee(self, name) -> None:
        """Refuse an end-effector name the robot (as last observed) does not have."""
        try:
            rob = self._robot_of(self._last_obs())
        except HarnessError:
            return
        rob._ee(str(name))

    # ---- inputs ------------------------------------------------------------------------

    def observe(self) -> dict:
        """The latest observation (A1): instruction, step budget, robot, cameras, held."""
        try:
            obs = self._get("api/agent/observation")
        except HarnessError as e:
            if not self._missing(e):
                raise
            obs = self._observation_from_state(self._get("api/state"))
        self._obs = obs
        self._robot = _robot.activate(_robot.from_embodiment(obs.get("embodiment")))
        if obs.get("instruction"):
            self.instruction = obs["instruction"]
        return obs

    def _last_obs(self) -> dict:
        return self._obs if self._obs is not None else self.observe()

    def camera(self, name: str, depth=False, obs=None) -> Camera:
        """The camera model (K, current T_world_cam) of `name` in the latest observation."""
        obs = obs or self.observe()
        if name not in obs["cameras"]:
            raise HarnessError(f"no camera {name!r}; have {sorted(obs['cameras'])}")
        return Camera.from_obs(name, obs, depth=depth)

    def cameras(self, obs=None, depth=False) -> dict:
        """name -> Camera of every camera (depth=True: the depth image's model, if it has one)."""
        obs = obs or self.observe()
        return {n: Camera.from_obs(n, obs, depth=depth) for n, c in obs["cameras"].items()
                if not depth or c.get("K_depth") is not None}

    def rgb(self, cam: str) -> np.ndarray:
        """The latest full-resolution RGB of a camera, uint8 (H, W, 3)."""
        raw, _ = self._get(f"api/frame/{cam}", binary=True)
        return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))

    def depth(self, cam: str) -> np.ndarray:
        """The latest depth of a camera, float32 (depth_height, depth_width) m; 0 = invalid."""
        try:
            raw, _ = self._get(f"api/agent/depth/{cam}", binary=True)
            return np.load(io.BytesIO(raw), allow_pickle=False).astype(np.float32)
        except HarnessError as e:
            if not self._missing(e):
                raise
        return self._depth_from_cloud(cam)

    def cloud(self, cams=None, stride=1, max_depth=None, crop=True) -> Cloud:
        """Fused RGB point cloud (robot-root frame): (xyz, rgb, cam_index) with .cams; max_depth
        None = the server's cutoff (2 m, or a scene camera's whole cropped workspace)."""
        q = {"stride": int(stride), "crop": 1 if crop else 0}
        if max_depth is not None:
            q["max_depth"] = float(max_depth)
        if cams:
            q["cams"] = ",".join([cams] if isinstance(cams, str) else cams)
        raw, _ = self._get("api/cloud?" + urllib.parse.urlencode(q), binary=True)
        header, blobs = tlf.decode(raw)
        per_cam = header.get("per_cam") or {}
        names = list(per_cam)
        idx = np.zeros(0, np.uint8)
        if per_cam:
            idx = np.concatenate([np.full(int(n), i, np.uint8)
                                  for i, n in enumerate(per_cam.values())])
        xyz = blobs.get("xyz", np.zeros((0, 3), np.float32))
        rgb = blobs.get("rgb", np.zeros((0, 3), np.uint8))
        if len(idx) != len(xyz):
            idx = np.zeros(len(xyz), np.uint8)
        return Cloud(xyz, rgb, idx, names, header.get("seq"))

    def objects(self, obs=None) -> Objects:
        """Perceived and agent-defined objects (Obj: id, label, colour, centroid, aabb, obb, mesh,
        source), from the latest perception; `.notes` its notes."""
        obs = obs if obs is not None else self.observe()
        try:
            raw, _ = self._get("api/scene", binary=True)
            header, blobs = tlf.decode(raw)
        except HarnessError as e:
            if e.status != 404:
                raise
            header, blobs = {}, {}
        perc_list = obs.get("objects")
        if perc_list is None:
            perc_list = (obs.get("perception") or {}).get("objects") or []
        perc = {o["id"]: o for o in perc_list if isinstance(o, dict) and o.get("id")}
        metas = dict(header.get("objects") or {})
        out = []
        for name in list(metas) + [n for n in perc if n not in metas]:
            meta = metas.get(name) or {}
            v = blobs.get(f"verts/{name}")
            f = blobs.get(f"faces/{name}")
            if v is not None and meta.get("pose") is not None:
                T = pose_to_T(meta["pose"])
                if not np.allclose(T, np.eye(4)):
                    v = np.asarray(v, np.float64) @ T[:3, :3].T + T[:3, 3]
            out.append(_make_obj(name, perc.get(name) or {}, meta, v, f))
        out.sort(key=lambda o: (len(o.id), o.id))
        notes = obs.get("perception_notes")
        if notes is None:
            notes = (obs.get("perception") or {}).get("notes")
        return Objects(out, notes, obs.get("perception_seq"))

    # ---- agent-defined objects and perception tools -----------------------------------

    def segment(self, cam: str, u: float, v: float, radius_mm=11, color_tol=None) -> Obj:
        """Region-grow an object from the 3D point under pixel (u, v) of `cam` (robot and support
        surface removed; radius_mm the growth step) and add it to the scene as an agent object.
        color_tol (e.g. 0.08) also stops the growth at colour edges between touching objects."""
        body = {"cam": cam, "u": float(u), "v": float(v), "radius_mm": float(radius_mm)}
        if color_tol is not None:
            body["color_tol"] = float(color_tol)
        return self._obj_reply(self._post("api/agent/segment", body))

    def define_object(self, aabb=None, obb=None, points=None, label=None, kind=None,
                      replace=()) -> Obj:
        """Add an agent object from the cloud points inside an aabb [[lo], [hi]] or an obb
        (OBB / {"center", "size", "yaw_deg"}), or from given points (N, 3); `replace` removes
        those ids (e.g. a merged segment you are splitting). kind "object" | "fixture"."""
        if aabb is None and obb is None and points is None:
            raise ValueError("give aabb, obb or points")
        body = {}
        if aabb is not None:
            body["aabb"] = np.asarray(aabb, float).reshape(2, 3).round(5).tolist()
        if obb is not None:
            body["obb"] = OBB.from_json(obb).to_json()
        if points is not None:
            body["points"] = np.asarray(points, float).reshape(-1, 3).round(5).tolist()
        if label is not None:
            body["label"] = str(label)
        if kind is not None:
            body["kind"] = str(kind)
        if replace:
            body["replace"] = [_obj_id(o) for o in
                               ([replace] if isinstance(replace, (str, Obj)) else replace)]
        return self._obj_reply(self._post("api/agent/define_object", body))

    def forget_object(self, obj) -> dict:
        """Remove an object (agent-defined or perceived) from the scene; it is not re-added
        where it was."""
        return self._post("api/agent/forget_object", {"id": _obj_id(obj)})

    def object_points(self, obj) -> tuple[np.ndarray, np.ndarray]:
        """The cloud points of one object: (xyz float32 (N, 3), rgb uint8 (N, 3))."""
        oid = _obj_id(obj)
        raw, headers = self._get(f"api/agent/object_points/{oid}", binary=True)
        if headers.get("X-Resolved-From"):
            self._note_resolved({"resolved": {"from": headers["X-Resolved-From"],
                                              "to": headers.get("X-Object-Id"),
                                              "why": headers.get("X-Resolved-Why")}})
        a = np.load(io.BytesIO(raw), allow_pickle=False).astype(np.float32).reshape(-1, 6)
        rgb = np.clip(np.round(a[:, 3:6] * 255.0), 0, 255).astype(np.uint8)
        return a[:, :3].copy(), rgb

    def reperceive(self) -> Objects:
        """Run perception now on the current frame (no motion, no steps) and return the objects."""
        r = self._post("api/agent/reperceive", {})
        obs = {"objects": r.get("objects") or [], "perception_notes": r.get("notes") or [],
               "perception_seq": r.get("perception_seq")}
        self._obs = None
        return self.objects(obs)

    def _obj_reply(self, r: dict) -> Obj:
        d = r.get("object") or {}
        try:
            o = self.objects().get(d.get("id"))
        except HarnessError:
            o = None
        return o if o is not None else _make_obj(d.get("id", "?"), d, {}, None, None)

    def pick(self, cam: str, u: float, v: float, search_px=16) -> dict:
        """3D point under full-resolution pixel (u, v): {"point", "depth", "normal", "cam"}.
        On a depth hole, the nearest valid pixel within search_px ("offset_px" says which)."""
        try:
            return self._post("api/pick", {"cam": cam, "u": float(u), "v": float(v)})
        except HarnessError as e:
            if "depth" not in e.message or not search_px:
                raise
            first = e
        for r in sorted({max(2, int(search_px) // 4), max(2, int(search_px) // 2), int(search_px)}):
            for du, dv in ((r, 0), (-r, 0), (0, r), (0, -r), (r, r), (-r, -r), (r, -r), (-r, r)):
                try:
                    out = self._post("api/pick", {"cam": cam, "u": float(u + du),
                                                  "v": float(v + dv)})
                except HarnessError:
                    continue
                return {**out, "offset_px": [du, dv]}
        raise first

    def object_at(self, point, max_dist=0.03) -> dict:
        """{"id": objN | None, "distance_m"}: the object whose surface is within max_dist."""
        return self._post("api/assist/object_at",
                          {"point": np.asarray(point, float).reshape(3).tolist(),
                           "max_dist": float(max_dist)})

    def object_at_pixel(self, cam: str, u: float, v: float, max_dist=0.03) -> dict:
        """pick() then object_at(): {"id", "distance_m", "point"} under a pixel."""
        p = self.pick(cam, u, v)
        r = self.object_at(p["point"], max_dist)
        return {**r, "point": p["point"]}

    # ---- manual motion ------------------------------------------------------------------

    def preview(self, waypoints, interp="minjerk", speed_scale=None) -> dict:
        """Densify and validate TCP waypoints (Plan JSON): preview JSON with `issues`; no motion."""
        own = waypoints.get("ee") if isinstance(waypoints, dict) else None
        if own is not None and own != self._ee:
            # A Plan JSON naming its end effector is previewed, filled and executed on that one.
            with self._bind(own):
                return self.preview(waypoints, interp, speed_scale)
        if not isinstance(waypoints, dict):
            waypoints = list(waypoints)  # read twice below: a generator would be spent by the first
        r = self._post("api/plan/preview", self._plan(waypoints, interp, speed_scale), check=False)
        if isinstance(r, dict):
            # The waypoints as given, so execute(preview) fills an unset gripper when it runs.
            r["request"] = self._plan(waypoints, interp, speed_scale, fill=False)
            # execute(preview) then runs on the end effector it was checked for, the primary's
            # when unbound, never on whichever one a handle executing it is bound to.
            ee = self._ee or (self._robot.primary_ee if self._robot.multi else None)
            if ee is not None:
                r["request"]["ee"] = ee
        return r

    def _plan(self, waypoints, interp="minjerk", speed_scale=None, fill=True) -> dict:
        """Plan JSON; with fill, a waypoint without "gripper" keeps the current gripper command."""
        if isinstance(waypoints, dict) and "waypoints" in waypoints:
            plan = dict(waypoints)
        else:
            plan = {"waypoints": list(waypoints), "interp": interp}
        if speed_scale is not None:
            plan["speed_scale"] = float(speed_scale)
        wps = [dict(w) for w in plan["waypoints"]]
        if fill and any("gripper" not in w or w["gripper"] is None for w in wps):
            robot = self._robot_block(self.observe())
            if self._robot.for_ee(self._ee, strict=False).has_gripper:
                cmd = int(round(float(robot.get("gripper_cmd") or 0)))  # noqa: RUF046
                for w in wps:
                    if w.get("gripper") is None:
                        w["gripper"] = cmd
        plan["waypoints"] = wps
        return plan

    def execute(self, plan, wait=True, timeout_s=300) -> dict:
        """Run a preview, or waypoints / Plan JSON. With wait, returns after the motion with the
        reply's "observation" and "last_chunk" filled in."""
        p = plan
        if isinstance(p, dict) and "waypoints" not in p and isinstance(p.get("request"), dict):
            if not p.get("ok"):
                raise HarnessError(explain(p, p.get("message") or "the preview has errors"),
                                   None, "execute", p)
            req = p["request"]
            with self._bind(req.get("ee")):
                r = self._post("api/plan/execute", self._plan(req))
        elif isinstance(p, (list, tuple)) or isinstance(p, dict) and "waypoints" in p:
            r = self._post("api/plan/execute", self._plan(p))
        else:
            reply = p if isinstance(p, dict) else {}
            raise HarnessError(explain(reply, reply.get("message") or "not an executable plan (no "
                                       "preview and no waypoints)"), None, "execute", p)
        return self._after_execute(r, wait, timeout_s)

    def _after_execute(self, r: dict, wait: bool, timeout_s: float) -> dict:
        if wait:
            obs = self.wait("idle", timeout_s=timeout_s)
            r = {**r, "observation": obs, "last_chunk": obs.get("last_chunk")}
        return r

    def move_to(self, pose7, duration, gripper=None, wait=True, interp="minjerk") -> dict:
        """Move the TCP to pose7 in `duration` s (one waypoint); refuses on preview errors."""
        wp = waypoint(duration, pose7, gripper)
        pv = self.preview([wp], interp=interp)
        errors = [i for i in pv.get("issues") or [] if i.get("severity") == "error"]
        if errors:
            msg = "preview has errors: " + "; ".join(i["message"] for i in errors)
            if any(i.get("kind") == "velocity" for i in errors) and pv.get("min_duration"):
                msg += f" (min_duration {float(pv['min_duration']):.2f} s)"
            raise HarnessError(msg, None, "api/plan/preview", pv)
        return self.execute({"waypoints": [wp], "interp": interp}, wait=wait)

    def gripper(self, closed: bool, duration=1.0, wait=True) -> dict:
        """Close (True) or open (False) the gripper; the arm holds."""
        self._need_gripper("gripper")
        r = self._post("api/gripper", {"closed": bool(closed), "duration": float(duration)})
        return self._after_execute(r, wait, duration + 60)

    def hold(self, seconds=1.0, wait=True) -> dict:
        """Hold the current targets for `seconds` (physics settles; costs steps)."""
        r = self._post("api/hold", {"duration": float(seconds)})
        return self._after_execute(r, wait, seconds + 60)

    def home(self, duration=3.0, wait=True) -> dict:
        """Joint-space move to the home pose, gripper open."""
        r = self._post("api/home", {"duration": float(duration)})
        return self._after_execute(r, wait, duration + 60)

    # ---- two arms ----------------------------------------------------------------------

    def ee(self, name: str) -> EEHandle:
        """A handle whose calls act on end effector `name` (ep itself acts on the primary one)."""
        self._check_ee(name)
        return EEHandle(self, str(name))

    def _plan_both(self, ee_waypoints, interp="minjerk", speed_scale=None) -> dict:
        """Plan JSON {"ee_waypoints": {ee: [waypoint]}, "interp"}; a waypoint without "gripper"
        keeps that end effector's current command."""
        if not isinstance(ee_waypoints, Mapping) or not ee_waypoints:
            raise ValueError("ee_waypoints is {end effector name: [waypoint, ...]}")
        out, obs = {}, None
        for name, wps in ee_waypoints.items():
            self._check_ee(name)
            wps = [dict(w) for w in (wps.get("waypoints") if isinstance(wps, Mapping) else wps)]
            if any(w.get("gripper") is None for w in wps):
                obs = obs or self.observe()
                if not self._robot.for_ee(str(name), strict=False).has_gripper:
                    out[str(name)] = wps
                    continue
                cmd = int(round(float(self._robot_block(obs, str(name)).get("gripper_cmd") or 0)))  # noqa: RUF046
                for w in wps:
                    if w.get("gripper") is None:
                        w["gripper"] = cmd
            out[str(name)] = wps
        plan = {"ee_waypoints": out, "interp": interp}
        if speed_scale is not None:
            plan["speed_scale"] = float(speed_scale)
        return plan

    def preview_both(self, ee_waypoints, interp="minjerk", speed_scale=None) -> dict:
        """preview() of a coordinated two-arm move, {end effector: [waypoints]}: preview JSON with
        `issues` and "schedule" ("simultaneous" | "serial"); no motion."""
        return self._post("api/plan/preview", self._plan_both(ee_waypoints, interp, speed_scale),
                          check=False)

    def execute_both(self, plan, wait=True, timeout_s=300) -> dict:
        """execute() of a coordinated move: {"ee_waypoints": {ee: [...]}, "interp"} (or just the
        {ee: [...]} dict); a one-arm preview is run as in execute()."""
        if isinstance(plan, Mapping) and "ee_waypoints" in plan:
            body = self._plan_both(plan["ee_waypoints"], plan.get("interp", "minjerk"),
                                   plan.get("speed_scale"))
        elif isinstance(plan, Mapping) and plan and "plan_id" not in plan and all(
                isinstance(v, (list, tuple)) for v in plan.values()):
            body = self._plan_both(plan)
        elif isinstance(plan, Mapping):
            return self.execute(plan, wait=wait, timeout_s=timeout_s)
        else:
            raise HarnessError("not an executable plan (give {end effector: [waypoints]})",
                               None, "execute_both", None)
        return self._after_execute(self._post("api/plan/execute", body), wait, timeout_s)

    def move_both(self, poses, duration, grippers=None, wait=True, interp="minjerk") -> dict:
        """Move several TCPs at once, {end effector: Pose7}, each in `duration` s (the arms that
        are not named hold); grippers {end effector: 0 | 1}. Refuses on preview errors."""
        grippers = dict(grippers or {})
        if set(grippers) - set(poses):
            raise ValueError(f"grippers names {sorted(set(grippers) - set(poses))} not in poses")
        wps = {name: [waypoint(duration, pose, grippers.get(name))] for name, pose in poses.items()}
        pv = self.preview_both(wps, interp=interp)
        errors = [i for i in pv.get("issues") or [] if i.get("severity") == "error"]
        if errors or pv.get("ok") is False:
            msg = "preview has errors: " + "; ".join(i["message"] for i in errors) if errors \
                else explain(pv)
            if any(i.get("kind") == "velocity" for i in errors) and pv.get("min_duration"):
                msg += f" (min_duration {float(pv['min_duration']):.2f} s)"
            raise HarnessError(msg, None, "api/plan/preview", pv)
        return self.execute_both({"ee_waypoints": wps, "interp": interp}, wait=wait)

    def gripper_both(self, closed: bool, duration=1.0, wait=True) -> dict:
        """Close (True) or open (False) every gripper at once; the arms hold."""
        if self._robot.end_effectors and not any(e.has_gripper for e in self._robot.end_effectors):
            self._need_gripper("gripper_both", self._robot.for_ee(self._robot.primary_ee))
        r = self._post("api/gripper", {"closed": bool(closed), "duration": float(duration),
                                       "ee": "*"})
        return self._after_execute(r, wait, duration + 60)

    def stop(self) -> dict:
        """Drop the rest of the running motion; the robot holds where it is."""
        return self._post("api/stop", {})

    # ---- kinematics --------------------------------------------------------------------

    def fk(self, q=None) -> dict:
        """{"q", "links": 11 points, "ee_pose", "tcp_pose"} at q (default the observed q)."""
        return self._post("api/agent/fk", {} if q is None else {"q": _q(q, self._ee)})

    def ik(self, tcp, seed=None, max=4) -> dict:
        """{"reachable", "solutions": [{"q", "pos_err_mm", "rot_err_deg", ...}]} for a TCP pose."""
        body = {"tcp": _tcp(tcp), "max": int(max)}
        if seed is not None:
            body["seed"] = _q(seed, self._ee)
        return self._post("api/agent/ik", body)

    def ik_batch(self, tcps, seed=None) -> list:
        """ik() of many TCP poses at once: one A4 reply ({"reachable", "solutions"}) per pose."""
        tcps = [_tcp(t) for t in tcps]
        out = []
        for i in range(0, len(tcps), 256):
            body = {"tcps": tcps[i:i + 256]}
            if seed is not None:
                body["seed"] = _q(seed, self._ee)
            try:
                out += self._post("api/agent/ik_batch", body)["results"]
            except HarnessError as e:
                if e.status not in (403, 404):
                    raise
                return out + [self.ik(t, seed) for t in tcps[i:]]
        return out

    def check_batch(self, qs, held=None, ignore=()) -> list:
        """check() of many joint vectors at once: one A5 reply per q (held/ignore as check)."""
        qs = [_q(q, self._ee) for q in qs]
        out = []
        for i in range(0, len(qs), 256):
            body = {"qs": qs[i:i + 256], **self._held_ignore(held, ignore)}
            try:
                out += self._post("api/agent/check_batch", body)["results"]
            except HarnessError as e:
                if e.status not in (403, 404):
                    raise
                return out + [self.check(q=q, held=held, ignore=ignore) for q in qs[i:]]
        return out

    @staticmethod
    def _held_ignore(held, ignore) -> dict:
        body = {}
        if held is False:
            body["held"] = None
        elif held is not None:
            body["held"] = _obj_id(held)
        if ignore:
            body["ignore"] = [_obj_id(i) for i in ignore]
        return body

    def check(self, q=None, tcp=None, held=None, ignore=()) -> dict:
        """Collision/limit check of q or a TCP pose against the perceived scene.

        held=None checks with whatever is held now, held=False with nothing held, held="objN"
        with that object in the gripper; ignore lists ids to leave out.
        """
        body = {}
        if q is not None:
            body["q"] = _q(q, self._ee)
        if tcp is not None:
            body["tcp"] = _tcp(tcp)
        body.update(self._held_ignore(held, ignore))
        return self._post("api/agent/check", body)

    def robot(self, q=None, gripper=None) -> dict:
        """The robot's collision spheres [[x, y, z, r]] at q (default observed), for drawing."""
        body = {}
        if q is not None:
            body["q"] = _q(q, self._ee)
        if gripper is not None:
            body["gripper"] = float(gripper)
        try:
            return self._post("api/agent/robot", body)
        except HarnessError as e:
            if not self._missing(e) or q is not None:
                raise
        st = self._get("api/state")
        links = (st.get("robot") or {}).get("links") or []
        return {"ok": True, "spheres": spheres_from_links(links).tolist() if links else [],
                "approximate": True}

    # ---- flow --------------------------------------------------------------------------

    def wait(self, until="idle", timeout_s=120, after_seq=None) -> dict:
        """Block until the robot is idle (or, until="frame", a newer frame); the observation."""
        if until not in ("idle", "frame"):
            raise ValueError("until must be 'idle' or 'frame'")
        if until == "frame" and after_seq is None:
            after_seq = int(self._last_obs().get("frame_seq") or 0)
        deadline = time.monotonic() + float(timeout_s)
        while True:
            left = max(1.0, min(120.0, deadline - time.monotonic()))
            body = {"until": until, "timeout_s": left}
            if after_seq is not None:
                body["after_seq"] = int(after_seq)
            try:
                r = self._post("api/agent/wait", body, timeout=left + 30)
            except HarnessError as e:
                if not self._missing(e):
                    raise
                return self._poll_state(until, after_seq, deadline)
            obs = r.get("observation") or {}
            self._obs = obs
            if "embodiment" in obs:
                self._robot = _robot.activate(_robot.from_embodiment(obs["embodiment"]))
            if not r.get("timed_out"):
                return obs
            if obs.get("mode") == "episode_over":
                raise EpisodeOver("the episode is over", None, "api/agent/wait", r)
            if time.monotonic() >= deadline:
                raise HarnessError(f"wait({until!r}) timed out after {timeout_s} s", None,
                                   "api/agent/wait", r)

    def status(self) -> dict:
        """The gateway's episode status: {"state": loading|ready|over, "end", "idle_s", ...}."""
        if self.direct:
            st = self._get("api/state")
            over = st.get("mode") == "episode_over"
            return {"ok": True, "state": "over" if over else "ready",
                    "instruction": st.get("instruction"), "end": "ended" if over else None}
        return self._get("_episode")

    def finish(self) -> dict:
        """End the episode now, scored as it stands. Call once the task is done (or hopeless)."""
        if self.direct:
            return self._post("api/episode/publish", {})
        return self._post("_episode/finish", {})

    # ---- rendering ---------------------------------------------------------------------

    def scene(self, cloud_stride=1, cams=None, with_cloud=True, remove_robot=True) -> SceneData:
        """Fetch everything the renderer draws (observation, cloud, objects, robot spheres);
        remove_robot drops cloud points on the robot's own body."""
        obs = self.observe()
        sd = SceneData()
        sd.cameras = self.cameras(obs)
        sd.depth_cameras = self.cameras(obs, depth=True)
        if with_cloud:
            c = self.cloud(cams=cams, stride=cloud_stride)
            sd.cloud_xyz, sd.cloud_rgb, sd.cloud_cam, sd.cloud_cams = c[0], c[1], c[2], c.cams
        sd.objects = self.objects(obs)
        try:
            sd.spheres = np.asarray(self.robot().get("spheres") or np.zeros((0, 4)), float)
        except HarnessError:
            sd.spheres = None
        if remove_robot and with_cloud and sd.spheres is not None and len(sd.spheres):
            keep = outside_spheres(sd.cloud_xyz, sd.spheres, inflate=0.012)
            sd.cloud_xyz, sd.cloud_rgb = sd.cloud_xyz[keep], sd.cloud_rgb[keep]
            sd.cloud_cam = sd.cloud_cam[keep]
        rb = obs.get("robot") or {}
        sd.tcp_pose = rb.get("tcp_pose")
        sd.gripper = float(rb.get("gripper_measured") or 0.0) if self._robot.has_gripper else None
        sd.held = (obs.get("held") or {}).get("object")
        if rb.get("end_effectors"):  # multi-ee only; not a SceneData field (its signature is surface)
            sd.end_effectors = {
                n: {"tcp_pose": e.get("tcp_pose"),
                    "gripper": float(e.get("gripper_measured") or 0.0)
                    if self._robot.for_ee(n, strict=False).has_gripper else None}
                for n, e in rb["end_effectors"].items() if isinstance(e, dict)}
        return sd

    def render(self, view="iso", layers=DEFAULT_LAYERS, overlays=None, size=(960, 540),
               scene: SceneData | None = None, **kw) -> Image.Image:
        """Render the scene from a View (or a standard view name / camera name) to a PIL image.

        overlays: {"grasps", "places", "plan", "paths", "frames", "grippers", "points", "boxes",
        "text", "highlight", "ghosts" (sphere arrays), "ghost_q" (joint vectors),
        "ghost_steps" (step indices into overlays["plan"]["q"], or "segments")}.
        """
        sd = scene or self.scene()
        v = self._view(view)
        ov = self._resolve_ghosts(dict(overlays or {}))
        return render_scene(sd, v, layers=layers, overlays=ov, size=size, **kw)

    @staticmethod
    def _view(view) -> View:
        if isinstance(view, View):
            return view
        from .geometry import STANDARD_VIEWS

        if view in STANDARD_VIEWS:
            return View.standard(view)
        return View.camera(view)

    def _resolve_ghosts(self, ov: dict) -> dict:
        qs = [q for q in ov.pop("ghost_q", None) or ()]
        steps = ov.pop("ghost_steps", None)
        plan = ov.get("plan")
        if steps is not None and isinstance(plan, dict) and plan.get("q"):
            if steps == "segments":
                steps = [s["end"] for s in plan.get("segments") or []] or [len(plan["q"]) - 1]
            qs += [plan["q"][min(int(i), len(plan["q"]) - 1)] for i in steps]
        if qs:
            ov["ghosts"] = list(ov.get("ghosts") or []) + \
                [np.asarray(self.robot(q)["spheres"]) for q in qs]
        return ov

    def views(self, names=("iso", "top", "front", "side"), target=None, layers=DEFAULT_LAYERS,
              overlays=None, size=(960, 540), cols=2, scene=None, distance=None) -> Image.Image:
        """Several standard views of the same scene in one grid image."""
        from .geometry import DEFAULT_TARGET

        sd = scene or self.scene()
        ov = self._resolve_ghosts(dict(overlays or {}))
        t = DEFAULT_TARGET if target is None else _target(target)
        ims = [render_scene(sd, View.standard(n, t, distance), layers=layers, overlays=ov,
                            size=size) for n in names]
        return grid(ims, cols=cols, titles=list(names))

    def render_plan(self, plan, view=None, ghosts="segments", scene=None, **kw) -> Image.Image:
        """A preview's TCP path (coloured start blue -> end red) with ghost robots at its
        segment ends (ghosts="segments"), at given step indices, or none (ghosts=None).
        view=None frames the path from in front of the robot (front-right)."""
        if view is None:
            pts = np.asarray([p[:3] for p in plan.get("tcp_fk") or plan.get("tcp_planned")])
            lo, hi = pts.min(0), pts.max(0)
            ext = float(np.linalg.norm(hi - lo))
            view = View.look_at((lo + hi) / 2, 320, 35, max(0.7, 2.2 * ext))
        ov = dict(kw.pop("overlays", None) or {})
        ov["plan"] = plan
        if ghosts is not None:
            ov["ghost_steps"] = ghosts
        return self.render(view, overlays=ov, scene=scene, **kw)

    def annotate(self, cam, points=None, labels=None, boxes=None, plan=None, poses=None,
                 grasps=None, objects=False, **kw) -> Image.Image:
        """The camera's current RGB with 3D items projected onto it (see render.annotate_image).
        objects=True marks every perceived object (id + aabb); a list marks those."""
        obs = self.observe()
        img = self.rgb(cam)
        camera = Camera.from_obs(cam, obs)
        dcam = Camera.from_obs(cam, obs, depth=True)
        try:
            dep = self.depth(cam)
        except HarnessError:
            dep, dcam = None, None
        objs = None
        if objects is True:
            objs = list(self.objects(obs))
        elif objects:
            allo = self.objects(obs)
            objs = [o if isinstance(o, Obj) else allo[str(o)] for o in objects]
        return annotate_image(img, camera, points=points, labels=labels, boxes=boxes, plan=plan,
                              poses=poses, grasps=grasps, objects=objs, depth=dep,
                              depth_cam=dcam, **kw)

    def closeup(self, obj, n_views=4, size=(480, 360), distance=None, cams=("ego_l", "ego_r"),
                elevation_deg=40.0, scene=None, overlays=None) -> Image.Image:
        """One grid image of an object: n_views renders orbiting it (the object highlighted, its
        obb drawn) plus a crop of each camera's real image around it."""
        sd = scene or self.scene()
        o = obj if isinstance(obj, Obj) else Objects(sd.objects)[_obj_id(obj)]
        c = np.asarray(o.centroid, float)
        ext = float(np.max(o.size)) if o.aabb is not None else 0.1
        dist = distance or max(0.3, 3.2 * ext)
        ov = dict(overlays or {})
        ov.setdefault("highlight", [o.id])
        if o.obb is not None:
            ov["boxes"] = list(ov.get("boxes") or []) + [{"aabb": o.obb, "label": o.id}]
        if not self._robot.droid and tuple(cams or ()) == ("ego_l", "ego_r"):
            cams = self._robot.scene_cameras
        ims, titles = [], []
        for k in range(int(n_views)):
            az = 180.0 + 45.0 + k * 360.0 / max(1, int(n_views))
            ims.append(self.render(View.look_at(c, az, elevation_deg, dist), overlays=ov,
                                   size=size, scene=sd))
            titles.append(f"{o.id} az {az % 360:.0f}")
        for cam in cams or ():
            if cam not in sd.cameras:
                continue
            camera = sd.cameras[cam]
            uv, z = camera.project(c.reshape(1, 3))
            if z[0] <= 0:
                continue
            img = annotate_image(self.rgb(cam), camera, objects=[o])
            w = int(min(camera.width, max(160, 2.5 * ext * camera.K[0, 0] / max(z[0], 0.05))))
            h = int(w * size[1] / size[0])
            ims.append(_crop_image(img, uv[0], (w, h)).resize(size))
            titles.append(f"{cam} crop")
        return grid(ims, cols=min(3, len(ims)), titles=titles)

    def budget(self) -> dict:
        """Steps and wall time left."""
        obs = self.observe()
        left = obs.get("steps_left")
        dt = float(obs.get("control_dt") or 1 / 15)
        out = {"step": obs.get("step"), "max_steps": obs.get("max_steps"), "steps_left": left,
               "motion_s_left": None if left is None else round(left * dt, 1)}
        try:
            st = self.status()
        except HarnessError:
            st = {}
        if st.get("episode_timeout_s") is not None and st.get("wall_s") is not None:
            out["wall_left_min"] = round((st["episode_timeout_s"] - st["wall_s"]) / 60.0, 1)
        out["idle_timeout_s"] = st.get("idle_timeout_s")
        return out

    # ---- verified pick and place --------------------------------------------------------

    def _robot_of(self, obs) -> _robot.Robot:
        if isinstance(obs, dict) and "embodiment" in obs:
            return _robot.from_embodiment(obs["embodiment"])
        return self._robot

    def _segment_near(self, xyz, size, radius, cams=None):
        """A fresh segment at a 3D point, seen from the first camera that frames it, accepted only
        if it lies within radius of the point and matches size (None to skip that check)."""
        obs = self.observe()
        if cams is None:
            rob = self._robot
            cams = rob.cameras if rob.droid else rob.scene_cameras + tuple(
                c for c in rob.cameras if c not in rob.scene_cameras)
        for cam in cams:
            if cam not in (obs.get("cameras") or {}):
                continue
            camera = Camera.from_obs(cam, obs)
            uv, z = camera.project(np.asarray(xyz, float).reshape(1, 3))
            u, v = uv[0]
            if z[0] <= 0 or not (0 <= u < camera.width and 0 <= v < camera.height):
                continue
            try:
                o = self.segment(cam, u, v)
            except HarnessError:
                continue
            if _dist(o, xyz) <= radius and (size is None or o.aabb is None
                                            or _size_match(o.size, size)):
                return o
        return None

    # ---- direct-mode fallbacks (no /api/agent/* on the worker) ------------------------

    @staticmethod
    def _observation_from_state(st: dict) -> dict:
        rb = st.get("robot") or {}
        lim = st.get("limits") or {}
        perc = st.get("perception")
        held = (st.get("assist") or {}).get("held")
        robot = None
        if rb:
            robot = {"q": rb.get("q"), "gripper_cmd": rb.get("gripper_cmd"),
                     "gripper_measured": rb.get("gripper_measured"),
                     "ee_pose": make_pose(rb["ee_pos"], rb["ee_quat"]),
                     "tcp_pose": make_pose(rb["tcp_pos"], rb["tcp_quat"]),
                     "tcp_offset": (st.get("tool") or {}).get("tcp_offset"),
                     "q_min": lim.get("q_min"), "q_max": lim.get("q_max")}
        steps_left = None if st.get("max_steps") is None else st["max_steps"] - st.get("step", 0)
        ex = st.get("execution")
        hist = st.get("history") or []
        return {
            "ok": True, "info_mode": st.get("info_mode"), "episode": st.get("episode"),
            "instruction": st.get("instruction"), "mode": st.get("mode"),
            "publishing": st.get("publishing"), "step": st.get("step"),
            "max_steps": st.get("max_steps"), "steps_left": steps_left,
            "control_dt": st.get("control_dt"), "frame_seq": st.get("frame_seq"),
            "robot": robot, "cameras": st.get("cameras") or {},
            "execution": None if not ex else {k: ex.get(k) for k in ("chunk_id", "kind", "index")}
            | {"n_steps": ex.get("total")},
            "last_chunk": None if not hist else {k: hist[-1].get(k) for k in
                                                 ("chunk_id", "kind", "status", "n_steps")}
            | ({"dropped": hist[-1]["dropped"]} if "dropped" in hist[-1] else {}),
            "held": None if not held else {"object": held.get("object"),
                                           "reason": held.get("reason")},
            "perception_seq": None if perc is None else perc.get("seq"),
            "perception": perc, "_fallback": "api/state",
        }

    def _poll_state(self, until, after_seq, deadline) -> dict:
        while True:
            obs = self._observation_from_state(self._get("api/state"))
            self._obs = obs
            busy = obs["mode"] == "executing" or bool(
                ((self._get("api/state").get("assist") or {}).get("plan") or {}).get("executing"))
            if until == "idle" and not busy:
                return obs
            if until == "frame" and (obs.get("frame_seq") or 0) > (after_seq or 0):
                return obs
            if time.monotonic() >= deadline:
                raise HarnessError(f"wait({until!r}) timed out", None, "api/state")
            time.sleep(0.2)

    def _depth_from_cloud(self, cam: str) -> np.ndarray:
        obs = self._last_obs()
        dcam = Camera.from_obs(cam, obs, depth=True)
        c = self.cloud(cams=[cam], stride=1, max_depth=100.0, crop=False)
        uv, z = dcam.project(c.xyz)
        u, v = np.round(uv[:, 0]).astype(int), np.round(uv[:, 1]).astype(int)
        ok = (u >= 0) & (u < dcam.width) & (v >= 0) & (v < dcam.height) & (z > 0)
        out = np.zeros((dcam.height, dcam.width), np.float32)
        out[v[ok], u[ok]] = z[ok]
        return out


class EEHandle:
    """ep.ee(name): the Episode's per-end-effector calls, each acting on that end effector."""

    def __init__(self, episode: Episode, name: str):
        self._ep, self.name = episode, name

    def __repr__(self) -> str:
        return f"EEHandle({self.name!r})"

    def __getattr__(self, attr):
        if attr in ("_ep", "name"):
            raise AttributeError(attr)
        target = getattr(self._ep, attr)
        if attr.startswith("_") or not callable(target):
            return target

        @functools.wraps(target)
        def bound(*args, **kw):
            with self._ep._bind(self.name):
                return target(*args, **kw)

        return bound


def _follows_ee(fn):
    @functools.wraps(fn)
    def call(self, *args, **kw):
        with _robot.bound_ee(self.name):
            return fn(*args, **kw)

    return call


for _name in ("top_down", "side_pose", "pose_from_axes", "finger_yaw", "approach_axis",
              "waypoint_top_down"):
    setattr(EEHandle, _name, _follows_ee(getattr(_geometry, _name)))


def _size_match(a, b, ratio=1.7, slack=0.025) -> bool:
    """Do two extents (3,) agree, sorted, within a ratio or an absolute slack per axis?"""
    a, b = np.sort(np.asarray(a, float)), np.sort(np.asarray(b, float))
    return bool(np.all((np.abs(a - b) <= slack) | (np.maximum(a, b) <= ratio * np.minimum(a, b))))


def _target(t):
    if isinstance(t, Obj):
        return tuple(t.centroid)
    return tuple(np.asarray(t, float).reshape(3))


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = Episode.from_file(path)
    deadline = time.monotonic() + float(wait_ready_s)
    last = None
    while True:
        st = ep.status()
        state = st.get("state")
        if state == "ready":
            ep.instruction = st.get("instruction") or ep.instruction
            return ep
        if state == "over":
            raise EpisodeOver(f"the episode is over (end: {st.get('end')})", 410, "_episode", st)
        if verbose and state != last:
            print(f"[sdk] episode {state}; waiting for ready", flush=True)
        last = state
        if time.monotonic() >= deadline:
            raise HarnessError(f"episode not ready after {wait_ready_s} s (state {state})")
        time.sleep(min(5.0, max(0.5, deadline - time.monotonic())))


_trace.instrument(Episode)
connect = _trace.traced(connect, "connect")
