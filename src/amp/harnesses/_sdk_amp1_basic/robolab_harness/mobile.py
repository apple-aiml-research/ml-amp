#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Episode for a robot with a holonomic base and joint groups: move_joints, move_base, base_pose."""

from __future__ import annotations

import numpy as np

from . import _trace
from . import client as _client
from .client import HarnessError, explain

BASE_ROUTE = "api/motion/base"


class Episode(_client.Episode):
    """The Episode plus a mobile base and joint groups."""

    def move_joints(self, group, q, duration=2.0, wait=True) -> dict:
        """Joint-space move of one arm (by arm or end-effector name) or joint group ("torso") to q
        in `duration` s, the rest holding; a torso move carries the arms. Arm and torso moves are
        checked against the scene and the robot's own body. Refuses on preview errors."""
        body = {"joints": {"group": str(group), "q": [float(v) for v in np.ravel(q)]},
                "duration": float(duration)}
        pv = self._post("api/plan/preview", body, check=False)
        errors = [i for i in pv.get("issues") or [] if i.get("severity") == "error"]
        if errors or pv.get("ok") is False:
            msg = ("preview has errors: " + "; ".join(i["message"] for i in errors)) if errors \
                else explain(pv)
            raise HarnessError(msg, None, "api/plan/preview", pv)
        return self._after_execute(self._post("api/plan/execute", body), wait, duration + 60)

    def base_pose(self, obs=None) -> np.ndarray | None:
        """T_world_base (4x4): the robot root's pose in the world, which every other pose is
        relative to; None for a robot without a mobile base."""
        T = ((obs if obs is not None else self.observe()).get("robot") or {}).get("T_world_base")
        return None if T is None else np.asarray(T, dtype=np.float64)

    def move_base(self, dx=0.0, dy=0.0, dyaw=0.0, wait=True, timeout_s=120) -> dict:
        """Drive the base dx, dy (m) and turn dyaw (rad) in its own frame (x ahead, y left); the
        arms hold. Refused with the reason when the floor it sweeps is occupied or unseen."""
        r = self._post(BASE_ROUTE, {"dx": float(dx), "dy": float(dy), "dyaw": float(dyaw)},
                       check=False)
        if not r.get("ok"):
            raise HarnessError(explain(r, r.get("message")), None, BASE_ROUTE, r)
        r = self._after_execute(r, wait, timeout_s)
        if wait:
            r["T_world_base"] = self.base_pose(r["observation"])
            r["base"] = (r.get("last_chunk") or {}).get("base")
        return r


for _name in ("move_joints", "move_base"):
    setattr(Episode, _name, _trace.traced(Episode.__dict__[_name], "execute"))


def connect(path="episode.json", wait_ready_s=900, verbose=True) -> Episode:
    """Open the episode in episode.json and wait (up to wait_ready_s) until it is ready."""
    ep = _client.connect(path, wait_ready_s=wait_ready_s, verbose=verbose)
    ep.__class__ = Episode  # it adds methods only, so the instance needs no new state
    return ep
