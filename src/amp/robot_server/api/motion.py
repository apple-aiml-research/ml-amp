#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Motion routes: manual waypoint plans, holds, the gripper, home, base moves, stop, pixel picks."""

from __future__ import annotations

from amp.robot_server.api import Reply, ok
from amp.robot_server.api.assist import call
from amp.robot_server.config import _finite
from amp.robot_server.errors import SessionConflict

preview = call("planning", "preview")
plan_execute = call("planning", "execute")
stop = call("executor", "stop", with_body=False)


def pick(srv, req) -> Reply:
    """POST api/pick: the 3D point under full-resolution pixel (u, v) of a camera."""
    u, v = _finite(req.body.get("u"), "u"), _finite(req.body.get("v"), "v")
    cam = req.body.get("cam")
    with srv.cond:
        obs = srv.obs.obs
        if obs is None:
            raise SessionConflict("no observation yet")
        fr = obs.cameras.get(cam)
    return ok(srv.obs.pick(fr, cam, u, v))


def hold(srv, req) -> Reply:
    return ok(srv.planning.hold(req.body.get("duration", 1.0), req.body.get("ee")))


def gripper(srv, req) -> Reply:
    if "closed" not in req.body:
        raise ValueError("gripper needs 'closed'")
    return ok(srv.planning.gripper(req.body["closed"], req.body.get("duration", 1.0),
                                   req.body.get("ee")))


def home(srv, req) -> Reply:
    return ok(srv.planning.home(req.body.get("duration", 3.0), req.body.get("ee")))


def base(srv, req) -> Reply:
    """POST api/motion/base: {dx, dy, dyaw} in the base's own frame (m, m, rad)."""
    return ok(srv.planning.move_base(req.body))
