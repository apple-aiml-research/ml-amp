#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""L4 of the assist layers: grasp and place proposals on top of critic, and top_down_across."""

from __future__ import annotations

import numpy as np

from .._manual import _trace
from .._manual.client import EEHandle, _follows_ee, _obj_id
from .._manual.geometry import _as_obb, _wrap_yaw, top_down
from . import critic, transit

CALLS = {"grasps": "candidates", "places": "candidates"}
MAX_FEASIBLE = 8


class Candidates(list):
    """A list of grasp or place candidates, best first, plus the server's `message`."""

    def __init__(self, items, message="", reply=None):
        super().__init__(items)
        self.message = message or ""
        self.reply = reply or {}
        self.resolved = self.reply.get("resolved")

    @property
    def feasible(self) -> list:
        return [c for c in self if c.get("ok")]

    @property
    def best(self) -> dict | None:
        """The first feasible candidate, or None."""
        f = self.feasible
        return f[0] if f else None

    def table(self, n=8) -> str:
        """One line per candidate: id, ok, width, score, notes."""
        rows = [f"{c.get('id', i)}: ok={c.get('ok')} width={c.get('width')} "
                f"score={c.get('score')} {c.get('notes') or ''}".rstrip()
                for i, c in enumerate(self[:n])]
        return "\n".join(rows + ([f"({self.message})"] if self.message else []))


def top_down_across(obj, z=None, yaw_offset_deg=0.0) -> list:
    """Top-down Pose7 over an object's (Obj / OBB) centre whose fingers close across its short
    side; z defaults to the box's middle height."""
    o = _as_obb(obj)
    yaw = o.long_yaw_deg + 90.0 + yaw_offset_deg
    zz = o.center[2] if z is None else float(z)
    return top_down([o.center[0], o.center[1], zz], _wrap_yaw(yaw))


class Episode(critic.Episode):
    """The manual Episode plus planned skills, checks and grasp and place proposals."""

    _ee_routes = critic.Episode._ee_routes | {"api/assist/grasps", "api/assist/places"}

    def grasps(self, obj, max=8, point=None, target=None, at=None, near=None) -> Candidates:
        """Grasp candidates for obj, best first (`.message` says why when none is feasible).
        target (an object) or point (xyz on the support surface) is where you will PLACE it,
        and ranks the grasps that allow that place first (give one, not both); near (xyz) prefers
        grasps whose contact is near that point of the object."""
        self._need_gripper("grasps")
        body = {"object": _obj_id(obj), "max": int(max)}
        if target is not None:
            body["target"] = _obj_id(target)
        if point is not None:
            body["point"] = np.asarray(point, float).reshape(3).tolist()
        if near is not None:
            body["near"] = np.asarray(near, float).reshape(3).tolist()
        if at is not None:
            body["at"] = int(at)
        r = self._post("api/assist/grasps", body)
        return Candidates(r.get("candidates") or [], r.get("message"), r)

    def places(self, obj, target=None, point=None, max=8, at=None) -> Candidates:
        """Place candidates for the held obj: set down on `target` (an object), on whatever is
        highest under it there, or at `point` on the support surface."""
        self._need_gripper("places")
        body = {"object": _obj_id(obj), "max": int(max)}
        if target is not None:
            body["target"] = _obj_id(target)
        if point is not None:
            body["point"] = np.asarray(point, float).reshape(3).tolist()
        if at is not None:
            body["at"] = int(at)
        r = self._post("api/assist/places", body)
        return Candidates(r.get("candidates") or [], r.get("message"), r)


EEHandle.top_down_across = _follows_ee(top_down_across)
transit._trace_calls(Episode, CALLS)


def _outcome(result, args: dict | None = None) -> dict | None:
    """The trace outcome of Candidates: counts, the best and the feasible ones' notes."""
    jsonable, failure_code = _trace.jsonable, _trace.failure_code
    if not isinstance(result, Candidates):
        return None
    best = result.best
    out = {"n": len(result), "n_feasible": len(result.feasible),
           "message": jsonable(result.message),
           "best": None if best is None else jsonable(
               {k: best.get(k) for k in ("id", "tcp", "closure", "width", "notes")}),
           "feasible": [jsonable({"id": c.get("id"), "notes": c.get("notes")})
                        for c in list(result.feasible)[:MAX_FEASIBLE]]}
    if not result.feasible:
        out["code"] = failure_code(result.message)
    return out


transit._register_outcome(_outcome)


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = critic.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
