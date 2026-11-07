#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""L5 of the assist layers: verified pick and place, multi-step plans and relation, on top of
proposals."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from PIL import Image

from .._manual import _trace
from .._manual.client import (
    EpisodeOver,
    HarnessError,
    Objects,
    _dist,
    _obj_id,
    _size_match,
    _tcp,
    explain,
)
from .._manual.geometry import _as_obb, footprint_overlap, pose_to_T
from . import proposals, transit

CALLS = {"add_step": "plan", "execute_steps": "execute", "pick_verified": "verify",
         "place_verified": "verify"}
ENDED_AFTER_RELEASE = "the episode ended after the release"
UNCONFIRMED = "cannot confirm from perception"


@dataclass
class Relation:
    """How box a sits relative to box b: see `relation`."""

    inside: bool
    on_top: bool
    overlap: float
    a_bottom_z: float
    b_bottom_z: float
    b_top_z: float

    def __bool__(self) -> bool:
        return self.inside or self.on_top


def relation(a, b, min_overlap=0.8, tol=0.015) -> Relation:
    """Is a inside b (footprint over b, bottom between b's bottom and top) or on top of it
    (bottom within tol of b's top)? From the perceived boxes only: verify with images."""
    oa, ob = _as_obb(a), _as_obb(b)
    ov = footprint_overlap(oa, ob)
    over = ov >= min_overlap
    inside = over and ob.bottom_z - tol <= oa.bottom_z < ob.top_z - tol
    on_top = over and abs(oa.bottom_z - ob.top_z) <= tol
    return Relation(bool(inside), bool(on_top), round(ov, 3), round(oa.bottom_z, 4),
                    round(ob.bottom_z, 4), round(ob.top_z, 4))


class Verified:
    """The result of pick_verified / place_verified: .ok, .object (its id now), .chosen
    (the candidate that worked), .attempts (one dict per try: stage, ok, why, gripper, ...),
    .message, .steps_left. Truthy when ok; ok None: released but not confirmed either way (the
    episode ended, or perception does not show the object). Print it for a summary."""

    def __init__(self, kind: str, obj):
        self.kind, self.object = kind, obj
        self.ok, self.chosen, self.attempts, self.message = False, None, [], ""
        self.steps_left = None

    def __bool__(self) -> bool:
        return bool(self.ok)

    def __repr__(self) -> str:
        verdict = "UNCONFIRMED" if self.ok is None else "OK" if self.ok else "FAILED"
        lines = [f"{self.kind}_verified {self.object}: {verdict}"
                 + (f" ({self.message})" if self.message and not self.ok else "")
                 + f", steps_left {self.steps_left}"]
        for i, a in enumerate(self.attempts):
            keys = ("gripper_measured", "gripper", "wrist_points", "dist_m", "relation", "steps")
            extra = ", ".join(f"{k}={a[k]}" for k in keys if a.get(k) is not None)
            res = "unconfirmed" if "ok" in a and a["ok"] is None else "ok" if a.get("ok") \
                else "fail"
            lines.append(f"  try {i} cand {a.get('candidate')} [{a.get('stage')}] "
                         f"{res}: {a.get('why', '')}"
                         + (f" ({extra})" if extra else ""))
        return "\n".join(lines)


def _predicted_centre(obj, cand):
    """Where a place candidate puts the object's centre: its object_pose applied to the mesh's
    centre in the object's current scene pose; None when either is unknown."""
    if obj is None or obj.pose is None or not isinstance(cand, dict) or \
            cand.get("object_pose") is None:
        return None
    if obj.verts is not None and len(obj.verts):
        c = np.asarray(obj.verts, float).mean(0)
    elif obj.centroid is not None:
        c = np.asarray(obj.centroid, float)
    else:
        return None
    T = pose_to_T(cand["object_pose"]) @ np.linalg.inv(pose_to_T(obj.pose))
    return T[:3, :3] @ c + T[:3, 3]


def _observed_match(seen, oid, want, size, radius):
    """The freshly perceived object that is the placed one: same id or nearest to the predicted
    centre (within radius), of a matching size; None if perception does not show it."""
    cands = [x for x in seen if size is None or x.aabb is None or _size_match(x.size, size)]
    if want is None:
        return next((x for x in cands if x.id == oid), None)
    same = [x for x in cands if x.id == oid and _dist(x, want) <= radius]
    if same:
        return same[0]
    return Objects(cands).nearest(want, kind="object", max_dist=radius)


def _containing(objs, want, tol=0.005):
    """The perceived object whose box holds the predicted centre (the placed one merged with its
    support in perception), smallest first; None if none does."""
    if want is None:
        return None
    w = np.asarray(want, float).reshape(3)
    hits = [x for x in objs if x.source != "agent" and x.aabb is not None
            and np.all(np.asarray(x.aabb[0]) - tol <= w)
            and np.all(w <= np.asarray(x.aabb[1]) + tol)]
    return min(hits, key=lambda x: float(np.prod(x.size)), default=None)


def _near_any(c, tried, pos_tol=0.015, yaw_tol=15.0) -> bool:
    """Is grasp c (nearly) one already tried: TCP within pos_tol and finger yaw within yaw_tol?"""
    from .._manual.geometry import finger_yaw

    p = np.asarray(_tcp(c))
    for t in tried:
        q = np.asarray(_tcp(t))
        dyaw = abs((finger_yaw(p) - finger_yaw(q) + 90.0) % 180.0 - 90.0)
        if np.linalg.norm(p[:3] - q[:3]) < pos_tol and dyaw < yaw_tol:
            return True
    return False


class Episode(proposals.Episode):
    """The manual Episode plus the assisted skills."""

    _ee_routes = proposals.Episode._ee_routes | {"api/assist/steps/add"}

    def steps(self) -> dict:
        """The multi-step assist plan JSON: {"valid", "steps": [...], "preview"}."""
        return self._get("api/assist/steps")

    def add_step(self, kind, obj, tcp, target=None, point=None, options=None) -> dict:
        """Append a pick or place step (tcp evaluated at the plan's end); returns the plan JSON."""
        self._need_gripper("add_step")
        body = {"kind": kind, "object": _obj_id(obj), "tcp": _tcp(tcp)}
        if target is not None:
            body["target"] = _obj_id(target)
        if point is not None:
            body["point"] = np.asarray(point, float).reshape(3).tolist()
        if options:
            body["options"] = dict(options)
        return self._post("api/assist/steps/add", body, check=False)

    def update_step(self, step_id, tcp=None, options=None) -> dict:
        body = {"id": int(step_id)}
        if tcp is not None:
            body["tcp"] = _tcp(tcp)
        if options is not None:
            body["options"] = dict(options)
        return self._post("api/assist/steps/update", body, check=False)

    def delete_step(self, step_id) -> dict:
        return self._post("api/assist/steps/delete", {"id": int(step_id)}, check=False)

    def clear_steps(self) -> dict:
        return self._post("api/assist/steps/clear", {}, check=False)

    def execute_steps(self, wait=True, timeout_s=600) -> dict:
        """Run every planned step back to back (each re-planned from the observed state)."""
        r = self._post("api/assist/steps/execute", {})
        return self._after_execute(r, wait, timeout_s)

    def pick_verified(self, obj, candidates=None, max_tries=3, target=None, point=None, near=None,
                      lift=True, height_m=None, lift_test_m=None, options=None,
                      require_wrist=False, wrist_image=True) -> Verified:
        """Pick with a lift test: plan_pick(stop_after="lift_test") -> execute -> verify_grasp
        (and the wrist view) -> lift() on success, else release() and the next candidate
        (re-proposed from the new state unless `candidates` is given). Returns a Verified."""
        self._need_gripper("pick_verified")
        res = Verified("pick", _obj_id(obj))
        given = None if candidates is None else [c for c in candidates
                                                 if not isinstance(c, dict) or c.get("ok", True)]
        queue = list(given or [])
        tried = []
        executed = 0
        last = self.objects().get(res.object) if int(max_tries) > 1 else None  # for retries
        while executed < int(max_tries):
            if executed and not self._still_known(res, last):
                break
            if executed and given is None:
                queue = []  # re-propose from the state the last try left
            if not queue:
                if given is not None:
                    res.message = res.message or "no candidate left"
                    break
                g = self.grasps(res.object, target=target, point=point, near=near)
                queue = [c for c in g.feasible if not _near_any(c, tried)]
                if not queue:
                    res.message = g.message or "no feasible grasp left"
                    break
            c = queue.pop(0)
            tried.append(c)
            att = {"candidate": c.get("id") if isinstance(c, dict) else None,
                   "tcp": np.round(_tcp(c), 4).tolist()}
            res.attempts.append(att)
            pv = self.plan_pick(res.object, c, options=options, stop_after="lift_test",
                                lift_test_m=lift_test_m)
            if not pv.get("ok") or pv.get("plan_id") is None:
                att.update(stage="plan", ok=False, why=explain(pv, pv.get("message")))
                continue
            step0 = self._last_obs().get("step") or 0
            r = self.execute(pv)
            executed += 1
            if (r.get("last_chunk") or {}).get("status") == "stopped":
                att["stopped"] = (r.get("last_chunk") or {}).get("message")
            v = self.verify_grasp(c.get("closure") if isinstance(c, dict) else None)
            att.update(stage="lift_test", **v)
            if require_wrist and not (v["wrist_points"] or 0):
                att.update(ok=False, why=v["why"] + "; no object in the wrist view")
            if wrist_image:
                wcam = "wrist" if self._robot.droid else \
                    self._robot.for_ee(self._ee, strict=False).wrist_camera
                try:
                    if wcam is not None:
                        att["wrist"] = Image.fromarray(self.rgb(wcam))
                except HarnessError:
                    pass
            if att["ok"]:
                held = v["held"] or res.object
                res.object = held
                if lift:
                    try:
                        lr = self.lift(held, height_m)
                        # Against the lift test's reading: fingers that closed further lost it.
                        lv = self.verify_grasp(v["gripper_measured"])
                        att["lift"] = {"ok": lv["ok"], "why": lv["why"],
                                       "gripper_measured": lv["gripper_measured"]}
                        if (lr.get("last_chunk") or {}).get("status") == "stopped":
                            att["lift"]["stopped"] = lr["last_chunk"].get("message")
                        att["ok"] = lv["ok"]
                        if not lv["ok"]:
                            att["why"] = "slipped during the lift: " + lv["why"]
                    except HarnessError as e:
                        att["lift"] = {"ok": False, "why": e.message}
                        att["ok"] = False
                        att["why"] = "lift failed: " + e.message
                if att["ok"]:
                    res.ok, res.chosen = True, c
                    att["steps"] = int((self._last_obs().get("step") or step0) - step0)
                    break
            try:
                self.release()
                att["released"] = True
            except HarnessError as e:
                att["released"] = False
                att["release_error"] = e.message
                try:
                    self.gripper(False)
                except HarnessError:
                    pass
            obs = self._last_obs()
            att["steps"] = int((obs.get("step") or step0) - step0)
        res.steps_left = self._last_obs().get("steps_left")
        if not res.ok and not res.message:
            res.message = res.attempts[-1].get("why", "") if res.attempts else "no attempt"
        return res

    def _still_known(self, res, last) -> bool:
        """Before a retry: follow the target if perception renumbered it, or gave its id to
        something else (the object of its size nearest where it was); False when it is gone."""
        objs = self.objects()
        cur = objs.get(res.object)
        if cur is not None and (last is None or last.aabb is None or cur.aabb is None or (
                _size_match(cur.size, last.size) and _dist(cur, last.centroid) <= 0.1)):
            return True
        o = None
        if last is not None and last.aabb is not None:
            near = [x for x in objs if x.source != "agent" and x.aabb is not None
                    and _size_match(x.size, last.size)]
            o = Objects(near).nearest(last.centroid, kind="object", max_dist=0.1)
        if o is None:
            res.message = (f"{res.object} is no longer perceived (moved or renumbered); find it "
                           "again and pick it by its new id")
            return False
        res.attempts[-1]["renumbered"] = f"{res.object} -> {o.id}"
        res.object = o.id
        return True

    def place_verified(self, obj=None, target=None, point=None, candidates=None, max_tries=2,
                       options=None, tol_m=0.04) -> Verified:
        """places -> plan_place -> execute, then check the release (nothing held, fingers open),
        re-perceive, and find the object near the predicted pose (and, with a target, inside or
        on it by the perceived boxes). Retries the next candidate only while still holding. ok is
        None once released if the episode ended or perception cannot show the object."""
        self._need_gripper("place_verified")
        oid = _obj_id(obj) if obj is not None else self._held_id()
        res = Verified("place", oid)
        before = self.objects()
        tgt = before.get(_obj_id(target)) if target is not None else None
        held_obj = before.get(oid)
        if candidates is None:
            cs = self.places(oid, target=target, point=point)
            queue = cs.feasible
            if not queue:
                res.message = cs.message or "no feasible place"
                return res
        else:
            queue = [c for c in candidates if not isinstance(c, dict) or c.get("ok", True)]
        executed = 0
        for c in queue:
            if executed >= int(max_tries):
                break
            att = {"candidate": c.get("id") if isinstance(c, dict) else None,
                   "tcp": np.round(_tcp(c), 4).tolist()}
            res.attempts.append(att)
            pv = self.plan_place(oid, c, target=target, point=point, options=options)
            if not pv.get("ok") or pv.get("plan_id") is None:
                att.update(stage="plan", ok=False, why=explain(pv, pv.get("message")))
                continue
            executed += 1
            att["stage"] = "place"
            try:
                r = self.execute(pv)
                obs = r.get("observation") or self.observe()
                if not self._released(att, obs):
                    continue
                if obs.get("mode") == "episode_over":
                    raise EpisodeOver("the episode is over", None, "place_verified")
                self._check_place(res, att, c, oid, held_obj, tgt, tol_m, before)
            except EpisodeOver:
                # A successful place often ends the episode: what wait() last saw tells whether
                # the release happened.
                if "released" not in att and not self._released(att, self._obs or {}):
                    att["why"] = "the episode is over before the release: " + att["why"]
                    break
                att.update(ok=None, why=ENDED_AFTER_RELEASE)
                res.ok, res.chosen, res.message = None, c, ENDED_AFTER_RELEASE
            break
        try:
            res.steps_left = self._last_obs().get("steps_left")
        except EpisodeOver:
            pass
        if not res.ok and res.ok is not None and not res.message:
            res.message = res.attempts[-1].get("why", "") if res.attempts else "no attempt"
        return res

    def _released(self, att: dict, obs: dict) -> bool:
        """Record in att whether obs shows the place released (nothing held, fingers open)."""
        held = (obs.get("held") or {}).get("object")
        shut = float(self._robot_block(obs).get("gripper_cmd") or 0) >= 0.5
        att["released"] = held is None and not shut
        if not att["released"]:
            att.update(ok=False, why=f"still holding {held or 'something'} (gripper closed)"
                       if shut else f"still holding {held}")
        return att["released"]

    def _check_place(self, res, att, c, oid, held_obj, tgt, tol_m, before=()) -> None:
        """After a release: find the placed object in fresh perception and judge the place."""
        try:
            objs = self.reperceive()
        except EpisodeOver:
            raise
        except HarnessError as e:
            if e.status not in (403, 404):
                att.update(ok=None, why=f"{UNCONFIRMED}: cannot re-perceive after the place: "
                           f"{e.message}")
                res.ok, res.chosen, res.message = None, c, UNCONFIRMED
                return
            objs = self.objects()
        want = _predicted_centre(held_obj, c)
        t_now = None
        if tgt is not None:
            t_now = objs.get(tgt.id) or objs.nearest(tgt.centroid, kind=tgt.kind, max_dist=0.1)
        size = None if held_obj is None or held_obj.aabb is None else held_obj.size
        merged = None
        if held_obj is not None and held_obj.source == "agent":
            o = None if want is None else self._segment_near(want, size, max(tol_m, 0.08))
            if o is None:
                att.update(ok=False, why="cannot verify an agent-defined object; look "
                           "(render) or define it again")
                return
            att["resegmented"] = o.id
        else:
            seen = [x for x in objs if x.source != "agent"
                    and (t_now is None or x.id != t_now.id)]
            o = _observed_match(seen, oid, want, size, max(tol_m, 0.08))
            if o is None:
                # Still perceived under its id, away from the predicted pose: not merged there.
                o = next((x for x in seen if x.id == oid), None)
            if o is None:
                # A new object of its size away from the predicted pose: likely it, renumbered.
                old = {x.id for x in before}
                new = [x for x in seen if x.id not in old and x.aabb is not None and size is not None
                       and _size_match(x.size, size)]
                if new:
                    att.update(ok=None, why=f"{UNCONFIRMED}: a new object of its size ({new[0].id}) "
                               "is perceived away from the predicted pose")
                    res.ok, res.chosen, res.message = None, c, UNCONFIRMED
                    return
                o = merged = _containing(objs, want)
            if o is None:
                # Released (nothing held, fingers open) but not seen: merged or cropped away.
                att.update(ok=None, why=f"{UNCONFIRMED}: no perceived object of its size near "
                           "the predicted pose, but the gripper is open and nothing is held")
                res.ok, res.chosen, res.message = None, c, UNCONFIRMED
                return
        att["object"] = o.id
        bad = []
        if merged is not None:
            att["merged"] = merged.id
        else:
            res.object = o.id
            if want is not None:
                att["dist_m"] = round(float(np.linalg.norm(o.centroid[:2] - want[:2])), 4)
                if att["dist_m"] > tol_m:
                    bad.append(f"{att['dist_m'] * 100:.1f} cm from the predicted pose")
        if t_now is not None and (merged is None or merged.id != t_now.id):
            rel = relation(o, t_now)
            att["relation"] = {"inside": rel.inside, "on_top": rel.on_top,
                               "overlap": rel.overlap}
            if not rel:
                bad.append(f"not in/on {t_now.id} by the perceived boxes "
                           f"(overlap {rel.overlap:.2f})")
        why = "; ".join(bad) or ("placed" if merged is None else
                                 f"placed (merged with {merged.id} in perception)")
        att.update(ok=not bad, why=why)
        res.ok, res.chosen = not bad, c


transit._trace_calls(Episode, CALLS)


def _attempt(a: dict, notes: dict) -> dict:
    keys = ("stage", "ok", "why", "gripper", "gripper_measured", "wrist_points", "dist_m",
            "candidate")
    out = _trace.jsonable({k: a.get(k) for k in keys if k in a})
    if a.get("ok") is False:
        out["code"] = _trace.failure_code(a.get("why"))
    if a.get("candidate") in notes:
        out["notes"] = _trace.jsonable(notes[a["candidate"]])
    return out


def _outcome(result, args: dict | None = None) -> dict | None:
    """The trace outcome of a Verified: its attempts, and a failure code when it failed."""
    jsonable, failure_code = _trace.jsonable, _trace.failure_code
    if not isinstance(result, Verified):
        return None
    # The candidates the call was given carry the notes the agent saw before choosing.
    given = (args or {}).get("candidates")
    notes = {c.get("id"): c.get("notes") for c in given or ()
             if isinstance(c, dict) and c.get("notes") is not None}
    ok = None if result.ok is None else bool(result.ok)
    out = {"ok": ok, "object": jsonable(result.object),
           "message": jsonable(result.message), "steps_left": result.steps_left,
           "attempts": [_attempt(a, notes) for a in result.attempts]}
    if ok is False:
        failed = next((a.get("why") for a in reversed(result.attempts)
                       if a.get("ok") is False and a.get("why")), None)
        out["code"] = failure_code(failed or result.message)
    return out


transit._register_outcome(_outcome)


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = proposals.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
