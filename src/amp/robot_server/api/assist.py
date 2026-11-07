#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Assist routes: grasp and place search, one-shot assist plans, multi-step plans."""

from __future__ import annotations

import numpy as np

from amp.planning.options import number
from amp.robot_server.api import Reply, ok
from amp.robot_server.config import _finite
from amp.robot_server.perception_service import MAX_ASSIST_DIST_M, OBJECT_AT_MAX_M


def call(component, method, with_body=True):
    def handler(srv, req) -> Reply:
        obj = getattr(srv, component)
        return ok(getattr(obj, method)(req.body) if with_body else getattr(obj, method)())
    handler.__name__ = method
    return handler


grasps, places = call("planning", "grasps"), call("planning", "places")
check, plan = call("planning", "check_assist"), call("planning", "plan_assist")
execute = call("planning", "execute_assist")
steps = call("planning", "steps", with_body=False)
steps_add, steps_update = call("planning", "steps_add"), call("planning", "steps_update")
steps_delete, steps_clear = call("planning", "steps_delete"), call("planning", "steps_clear")
steps_execute = call("planning", "steps_execute")


def object_at(srv, req) -> Reply:
    point = req.body.get("point")
    if not isinstance(point, list) or len(point) != 3:
        raise ValueError(f"point must be 3 numbers, got {point!r}")
    point = np.array([number(v, "point") for v in point])
    max_dist = _finite(req.body.get("max_dist", OBJECT_AT_MAX_M), "max_dist", 1e-6,
                       MAX_ASSIST_DIST_M)
    return ok(srv.perception.object_at_reply(point, max_dist))
