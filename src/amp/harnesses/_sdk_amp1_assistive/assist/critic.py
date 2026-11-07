#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""L3 of the assist layers: the critic on top of skills, which checks a given grasp or place and
verifies a grasp from the gripper and the wrist view."""

from __future__ import annotations

import numpy as np

from .._manual.client import HarnessError, _obj_id, _tcp
from .._manual.geometry import Camera, backproject, pose_to_T
from . import skills, transit

OVERCLOSE = 0.25           # closed this far past the expected closure, unseen: on air
SHUT_NO_EXPECTED = 0.85    # with no expected closure: this shut and nothing seen fails
PAD_CLEAR = 0.006          # a counted point keeps this far from either pad face (pads, noise)
MIN_HALF = 0.002           # a narrower view between the pads cannot tell an object from them
CALLS = {"check_grasp": "candidates", "check_place": "candidates", "verify_grasp": "verify",
         "fingers_view": "look"}


class Episode(skills.Episode):
    """The manual Episode plus planned skills and checks of a grasp or place."""

    _ee_routes = skills.Episode._ee_routes | {"api/assist/check"}

    def check_grasp(self, obj, tcp, at=None) -> dict:
        """Evaluate exactly this grasp TCP: the grasp candidate JSON (ok, width, notes, ...)."""
        self._need_gripper("check_grasp")
        body = {"kind": "grasp", "object": _obj_id(obj), "tcp": _tcp(tcp)}
        if at is not None:
            body["at"] = int(at)
        return self._post("api/assist/check", body)["candidate"]

    def check_place(self, obj, tcp, target=None, point=None, at=None) -> dict:
        """Evaluate exactly this release TCP: the place candidate JSON."""
        self._need_gripper("check_place")
        body = {"kind": "place", "object": _obj_id(obj), "tcp": _tcp(tcp)}
        if target is not None:
            body["target"] = _obj_id(target)
        if point is not None:
            body["point"] = np.asarray(point, float).reshape(3).tolist()
        if at is not None:
            body["at"] = int(at)
        return self._post("api/assist/check", body)["candidate"]

    def fingers_view(self, obs=None, cam="wrist") -> int | None:
        """Wrist-depth points well inside the gap between the finger pads (TCP frame box), or
        None without depth or when the gap is too narrow to tell an object from the pads."""
        obs = obs or self.observe()
        self._need_gripper("fingers_view", self._robot_of(obs))
        rob = self._robot_of(obs).for_ee(self._ee, strict=False)
        if not rob.droid and cam == "wrist" and "wrist" not in (obs.get("cameras") or {}):
            cam = rob.wrist_camera
        c = (obs.get("cameras") or {}).get(cam) or {}
        rb = self._robot_block(obs)
        if cam is None or c.get("K_depth") is None or rb.get("tcp_pose") is None:
            return None
        try:
            d = self.depth(cam)
        except HarnessError:
            return None
        pts = backproject(d, Camera.from_obs(cam, obs, depth=True), min_depth=0.02, max_depth=0.6)
        T = pose_to_T(rb["tcp_pose"])
        loc = (pts - T[:3, 3]) @ T[:3, :3]
        if rob.droid:
            gap = 0.085 * (1.0 - float(rb.get("gripper_measured") or 0.0))
        elif rb.get("gripper_gap_m") is not None:
            gap = float(rb["gripper_gap_m"])
        else:
            gap = rob.gap_m(float(rb.get("gripper_measured") or 0.0))
        # The pads' own faces sit at +-gap/2: only points clear of both are an object between.
        half = gap / 2 - PAD_CLEAR
        if half < MIN_HALF:
            return None
        ci, wi, ai = rob.closing, rob.width_axis, rob.approach
        m = (np.abs(loc[:, ci]) < half) & (np.abs(loc[:, wi]) < 0.012) & \
            (loc[:, ai] > -0.025) & (loc[:, ai] < 0.015)
        return int(m.sum())

    def verify_grasp(self, expected_closure=None, min_wrist_points=15) -> dict:
        """Is something held? {"ok", "why", "held", "gripper_measured", "gripper" ("closed" |
        "empty" ~1.0 | "jammed" ~0.0), "wrist_points"}: ok = held and a partial closure (never
        empty, whatever the wrist view), not closed well past the expected closure unseen."""
        obs = self.observe()
        self._need_gripper("verify_grasp", self._robot_of(obs))
        m = float(self._robot_block(obs).get("gripper_measured") or 0.0)
        held = (obs.get("held") or {}).get("object")
        rob = self._robot_of(obs).for_ee(self._ee, strict=False)
        empty_at, shut_at = 0.97, SHUT_NO_EXPECTED
        if not rob.droid:
            # The declared reading closed on nothing, and below which a closed jaw holds something.
            empty_at = min(rob.held_below, rob.closed_empty - 0.03)
            shut_at = rob.closed_empty - (1.0 - SHUT_NO_EXPECTED)
        verdict = "jammed" if m <= 0.03 else "empty" if m >= empty_at else "closed"
        wp = self.fingers_view(obs)
        # A reading closed on nothing is never a hold, whatever the wrist view or held says.
        seen = verdict != "empty" and wp is not None and wp >= min_wrist_points
        # Closing well past the expected closure (or near-shut with none known) and nothing in
        # the wrist view between the pads: the fingers closed on air.
        if expected_closure is not None:
            overclosed = m - float(expected_closure) > OVERCLOSE and not seen
        else:
            overclosed = m >= shut_at and wp == 0
        ok = held is not None and verdict == "closed" and not overclosed
        why = []
        if held is None:
            why.append("nothing is held")
        if verdict == "jammed":
            why.append(f"gripper_measured {m:.2f}: the fingers did not close (jammed)")
        elif verdict == "empty":
            why.append(f"gripper_measured {m:.2f}: nothing between the fingers")
        elif overclosed:
            exp = "" if expected_closure is None else f" vs expected {float(expected_closure):.2f}"
            why.append(f"gripper_measured {m:.2f}{exp} and {wp or 0} wrist points between the "
                       f"pads: probably closed on air")
        elif expected_closure is not None and abs(m - float(expected_closure)) > 0.15:
            why.append(f"closure {m:.2f} vs expected {float(expected_closure):.2f}")
        return {"ok": bool(ok), "why": "; ".join(why) or "held", "held": held,
                "gripper_measured": round(m, 3), "gripper": verdict, "wrist_points": wp}


transit._trace_calls(Episode, CALLS)


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = skills.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
