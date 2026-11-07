#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""L2 of the assist layers: planned picks, places, lifts, releases and pushes on top of transit,
and pick counts in budget()."""

from __future__ import annotations

import numpy as np

from .._manual.client import _obj_id, _tcp
from . import transit

CALLS = {"plan_pick": "plan", "plan_lift": "plan", "plan_release": "plan", "plan_push": "plan",
         "plan_place": "plan", "lift": "execute", "release": "execute", "push": "execute"}


class Episode(transit.Episode):
    """The manual Episode plus planned moves, picks, places, lifts, releases and pushes."""

    def plan_pick(self, obj, tcp, options=None, stop_after=None, lift_test_m=None) -> dict:
        """Collision-free pick (transit, approach, close, lift): preview JSON with plan_id.
        stop_after="lift_test" ends after the close and a lift_test_m (default 0.015) lift: then
        verify, and lift() or release()."""
        self._need_gripper("plan_pick")
        body = {"kind": "pick", "object": _obj_id(obj), "tcp": _tcp(tcp)}
        opts = dict(options or {})
        if stop_after is not None:
            opts["stop_after"] = str(stop_after)
        if lift_test_m is not None:
            opts["lift_test_m"] = float(lift_test_m)
        if opts:
            body["options"] = opts
        return self._post("api/assist/plan", body, check=False)

    def _held_id(self):
        return ((self.observe().get("held") or {}).get("object"))

    def plan_lift(self, obj=None, height_m=None, options=None) -> dict:
        """Continue a lift-tested pick: lift the held obj (default: what is held) by height_m."""
        self._need_gripper("plan_lift")
        body = {"kind": "lift"}
        oid = _obj_id(obj) if obj is not None else self._held_id()
        if oid is not None:
            body["object"] = oid
        return self._plan_kind(body, options, height_m=height_m)

    def plan_release(self, retreat_m=None, options=None) -> dict:
        """Open the gripper where it is and back off along the approach by retreat_m."""
        self._need_gripper("plan_release")
        return self._plan_kind({"kind": "release"}, options, retreat_m=retreat_m)

    def plan_push(self, start, end, height_m=None, options=None) -> dict:
        """A straight push with closed fingers from start to end (xyz; the TCP path) at
        height_m (default 0.02) above the support surface."""
        body = {"kind": "push", "start": np.asarray(start, float).reshape(3).tolist(),
                "end": np.asarray(end, float).reshape(3).tolist()}
        return self._plan_kind(body, options, height_m=height_m)

    def lift(self, obj=None, height_m=None, options=None, wait=True) -> dict:
        """plan_lift + run: the execute reply (with "observation") or HarnessError."""
        return self.run(self.plan_lift(obj, height_m, options), wait)

    def release(self, retreat_m=None, options=None, wait=True) -> dict:
        """plan_release + run."""
        return self.run(self.plan_release(retreat_m, options), wait)

    def push(self, start, end, height_m=None, options=None, wait=True) -> dict:
        """plan_push + run."""
        return self.run(self.plan_push(start, end, height_m, options), wait)

    def plan_place(self, obj, tcp, target=None, point=None, options=None, force=False) -> dict:
        """Collision-free place (transit, descend, release, retreat) of the held obj; a place
        with nothing held is refused unless force=True."""
        self._need_gripper("plan_place")
        body = {"kind": "place", "object": _obj_id(obj), "tcp": _tcp(tcp)}
        if target is not None:
            body["target"] = _obj_id(target)
        if point is not None:
            body["point"] = np.asarray(point, float).reshape(3).tolist()
        opts = dict(options or {})
        if opts:
            body["options"] = opts
        if force:
            body["force"] = True
        return self._post("api/assist/plan", body, check=False)

    def budget(self, pick_steps=None, place_steps=None) -> dict:
        """Steps and wall time left; given a pick's (and a place's) step cost, such as a plan's
        n_steps, also how many picks / pick-and-place cycles that allows."""
        out = super().budget()
        left = out.get("steps_left")
        if pick_steps:
            out["picks_left"] = None if left is None else left // int(pick_steps)
            if place_steps:
                out["pick_place_left"] = None if left is None else \
                    left // (int(pick_steps) + int(place_steps))
        return out


transit._trace_calls(Episode, CALLS)


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = transit.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
