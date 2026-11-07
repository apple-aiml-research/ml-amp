#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot server's HTTP API as data: the server implements exactly these routes, and the gateway,
harness conformance, the audit and the API docs read them from here."""

from __future__ import annotations

from amp.contracts.server import Route

GRADE = Route("POST", "api/episode/grade", "lifecycle",
              summary='{"cell": n}: grade the state at code cell n\'s end, no physics stepped')

ROUTES: tuple[Route, ...] = (
    Route("GET", "api/tasks", "lifecycle"),
    Route("POST", "api/tasks/select", "lifecycle"),
    Route("GET", "api/state", "lifecycle"),
    Route("POST", "api/episode/release", "lifecycle"),
    Route("POST", "api/episode/publish", "lifecycle"),
    GRADE,
    Route("GET", "api/runs/{task}/{k}", "lifecycle", summary="the saved record of run k"),
    Route("GET", "api/runs/{run}/{task}/{k}", "lifecycle", summary="the same, in one run_id"),
    Route("GET", "api/frame/{cam}", "sensors"),
    Route("GET", "api/frame/{cam}/{seq}", "sensors"),
    Route("GET", "api/cloud", "sensors"),
    Route("GET", "api/cloud/{query*}", "sensors", summary="query parameters encoded in the path"),
    Route("GET", "api/scene", "sensors"),
    Route("GET", "api/agent/observation", "agent"),
    Route("POST", "api/agent/wait", "agent"),
    Route("GET", "api/agent/depth/{cam}", "agent"),
    Route("GET", "api/agent/depth/{cam}/{seq}", "agent"),
    Route("GET", "api/agent/object_points/{name}", "agent"),
    Route("POST", "api/agent/robot", "agent"),
    Route("POST", "api/agent/fk", "agent"),
    Route("POST", "api/agent/ik", "agent"),
    Route("POST", "api/agent/ik_batch", "agent"),
    Route("POST", "api/agent/check", "agent"),
    Route("POST", "api/agent/check_batch", "agent"),
    Route("POST", "api/agent/segment", "agent"),
    Route("POST", "api/agent/define_object", "agent"),
    Route("POST", "api/agent/forget_object", "agent"),
    Route("POST", "api/agent/reperceive", "agent"),
    Route("POST", "api/assist/grasps", "assist"),
    Route("POST", "api/assist/places", "assist"),
    Route("POST", "api/assist/check", "assist"),
    Route("POST", "api/assist/plan", "assist"),
    Route("POST", "api/assist/execute", "assist", executes=True),
    Route("POST", "api/assist/object_at", "assist"),
    Route("GET", "api/assist/steps", "assist"),
    Route("POST", "api/assist/steps/add", "assist"),
    Route("POST", "api/assist/steps/update", "assist"),
    Route("POST", "api/assist/steps/delete", "assist"),
    Route("POST", "api/assist/steps/clear", "assist"),
    Route("POST", "api/assist/steps/execute", "assist", executes=True),
    Route("POST", "api/pick", "motion"),
    Route("POST", "api/plan/preview", "motion"),
    Route("POST", "api/plan/execute", "motion", executes=True),
    Route("POST", "api/stop", "motion"),
    Route("POST", "api/hold", "motion", executes=True),
    Route("POST", "api/gripper", "motion", executes=True),
    Route("POST", "api/home", "motion", executes=True),
    Route("POST", "api/motion/base", "motion", executes=True,
          summary="move a mobile base by (dx, dy, dyaw) in its own frame, checked against the "
                  "perceived occupancy"),
    *(Route("POST", f"sim/{n}", "link")
      for n in ("hello", "catalog", "episode", "step", "poll", "end", "next", "saved", "scene")),
    Route("POST", "sim/robot", "link", summary="the robot description a chain embodiment needs"),
)


def find(method: str, path: str) -> Route | None:
    """The served route for a request path (no leading slash), or None."""
    for r in ROUTES:
        if r.match(method, path) is not None:
            return r
    return None
