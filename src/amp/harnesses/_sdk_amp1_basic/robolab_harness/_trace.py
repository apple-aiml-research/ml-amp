#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Harness trace events from inside the sandbox. The kernel puts amp_trace (amp.trace.kernel_writer)
next to this package; without it, or before the bootstrap names the harness, nothing is written.
Events are harness.<HARNESS>.<kind>; tracing never changes what a call returns or raises."""

from __future__ import annotations

import functools
import inspect
import re
import threading
import time

try:
    import amp_trace
except ImportError:
    amp_trace = None

HARNESS = None  # set by the harness bootstrap, e.g. rh._trace.HARNESS = "amp1_basic"

# Episode method -> event kind; data["call"] names the method.
CALLS = {
    "preview": "plan", "preview_both": "plan", "execute_both": "execute", "move_both": "execute",
    "gripper_both": "execute",
    "execute": "execute", "move_to": "execute",
    "gripper": "execute", "hold": "execute", "home": "execute", "stop": "execute",
    "segment": "objects", "define_object": "objects", "forget_object": "objects",
    "reperceive": "objects",
    "objects": "perceive", "object_points": "perceive", "object_at": "perceive",
    "object_at_pixel": "perceive", "pick": "perceive",
    "rgb": "look", "depth": "look", "cloud": "look", "render": "look", "views": "look",
    "render_plan": "look", "annotate": "look", "closeup": "look", "scene": "look",
    "camera": "look",
    "ik": "query", "ik_batch": "query", "check": "query", "check_batch": "query", "fk": "query",
    "robot": "query",
    "finish": "finish",
}
MAX_ITEMS = 16
MAX_STR = 300
MAX_ARRAY = 64  # elements; a bigger array (an image, a cloud) is traced as its shape
# fn(result, args) -> dict or None, tried before base_outcome: a layer on this SDK adds its own.
OUTCOMES: list = []
_local = threading.local()

# A failure's reason -> a fixed code; first match wins, so specific causes come before generic.
FAILURES = [(re.compile(p, re.IGNORECASE), code) for p, code in (
    (r"episode is over|episode_?over|\b410\b", "EPISODE_OVER"),
    (r"wait\(.*\) timed out", "WAIT_TIMEOUT"),
    (r"cannot reach|connection refused|timed out", "UNREACHABLE_GATEWAY"),
    (r"not an executable plan|needs a non-empty|is missing '|not both", "API_MISUSE"),
    (r"robot moved since this plan|unknown plan_id", "STALE_PLAN"),
    (r"singularity", "SINGULAR"),
    (r"pixel hits nothing|depth near that pixel", "PIXEL_MISS"),
    (r"too few points|points grew from|cloud points inside", "SEGMENT_TOO_SMALL"),
    # "obj3 is held; release it first", not "nothing is held; ..."
    (r"(?<!nothing )\bis held\b[;:]", "HELD_CONFLICT"),
    (r"too wide", "TOO_WIDE"),
    (r"no feasible grasp", "GRASP_INFEASIBLE"),
    (r"no feasible place|no feasible placement", "PLACE_INFEASIBLE"),
    (r"jammed|did not close", "GRIPPER_JAMMED"),
    (r"nothing between the fingers|empty grasp|closed on air|slip|dropped|lift.?test",
     "GRASP_LOST"),
    (r"nothing is held", "NOT_HELD"),
    (r"still holding", "RELEASE_FAILED"),
    (r"not observed after the place|cannot verify", "NOT_OBSERVED_AFTER_PLACE"),
    (r"not in/on|overlap", "RELATION_FAILED"),
    (r"from the predicted pose", "PLACE_OFF_TARGET"),
    (r"collision|collides|in contact|\bhits\b", "PLAN_COLLISION"),
    (r"unreachable|no ik|ik fail|joint limit", "PLAN_UNREACHABLE"),
    (r"too fast|min_duration", "TOO_FAST"),
    (r"no plan", "NO_PLAN"),
    (r"not perceived|unknown object|no object", "UNKNOWN_OBJECT"),
    (r"budget|steps left|needs ~", "OVER_BUDGET"),
)]


def failure_code(reason, status=None) -> str | None:
    """The FAILURES code of a failure's reason: None without one, "OTHER" when none matches."""
    if status == 410:
        return "EPISODE_OVER"
    if not reason:
        return None
    return next((code for pattern, code in FAILURES if pattern.search(str(reason))), "OTHER")


def enabled() -> bool:
    return amp_trace is not None and bool(HARNESS)


def emit(kind: str, data: dict, *, lvl: str = "info", image_png_bytes=None) -> bool:
    if not enabled():
        return False
    return amp_trace.emit(f"harness.{HARNESS}.{kind}", data, image_png_bytes, lvl=lvl)


def jsonable(v, depth=0):
    """A small JSON-safe rendering of an argument or result: ids for objects, capped lists."""
    if v is None or isinstance(v, (bool, int)):
        return v
    if isinstance(v, float):
        return round(v, 5)
    if isinstance(v, str):
        return v[:MAX_STR]
    if hasattr(v, "id") and hasattr(v, "label"):  # an Obj
        return {"id": v.id, "label": str(v.label)[:80]}
    if hasattr(v, "getbands") and hasattr(v, "size"):  # a PIL image
        return {"image": list(v.size)}
    if hasattr(v, "dtype") and hasattr(v, "shape") and getattr(v, "size", 0) > MAX_ARRAY:
        return {"shape": list(v.shape), "dtype": str(v.dtype)}
    if depth >= 3:
        return repr(v)[:80]
    if hasattr(v, "tolist") and hasattr(v, "dtype"):
        v = v.tolist()
    if isinstance(v, dict):
        return {str(k): jsonable(x, depth + 1) for k, x in list(v.items())[:MAX_ITEMS * 2]}
    if isinstance(v, (list, tuple)):
        out = [jsonable(x, depth + 1) for x in v[:MAX_ITEMS]]
        return out + [f"... {len(v)} items"] if len(v) > MAX_ITEMS else out
    if hasattr(v, "item"):  # numpy scalars
        return jsonable(v.item(), depth)
    return repr(v)[:MAX_STR]


def _errors(reply: dict) -> list:
    return [str(i.get("message"))[:MAX_STR] for i in reply.get("issues") or []
            if isinstance(i, dict) and i.get("severity") == "error"][:MAX_ITEMS]


def outcome(result, args: dict | None = None) -> dict:
    """What a traced call returned, reduced to what a reader of the trace needs: the first
    OUTCOMES hook that knows the result, else base_outcome."""
    for hook in OUTCOMES:
        out = hook(result, args)
        if out is not None:
            return out
    return base_outcome(result, args)


def base_outcome(result, args: dict | None = None) -> dict:
    """outcome() for the results this SDK itself returns."""
    if hasattr(result, "getbands") or (hasattr(result, "dtype") and hasattr(result, "shape")):
        summary = jsonable(result)
        return summary if isinstance(summary, dict) else {"result": summary}
    if hasattr(result, "id") and hasattr(result, "label"):
        return {"object": jsonable(result)}
    if hasattr(result, "ids") and callable(result.ids):  # Objects
        return {"objects": result.ids()[:MAX_ITEMS * 2]}
    if not isinstance(result, dict):
        return {"result": jsonable(result)}
    out = {k: jsonable(result[k]) for k in (
        "ok", "chunk_id", "message", "n_steps", "duration", "valid", "reachable", "held",
        "gripper", "gripper_measured") if k in result}
    if result.get("issues"):
        out["errors"] = _errors(result)
    if result.get("ok") is False:
        out["code"] = failure_code(result.get("message") or result.get("why")
                                   or next(iter(out.get("errors") or ()), None))
    obs = result.get("observation")
    if isinstance(obs, dict):
        robot = obs.get("robot") or {}
        out["after"] = {"step": obs.get("step"), "steps_left": obs.get("steps_left"),
                        "held": jsonable((obs.get("held") or {}).get("object")),
                        "gripper_measured": jsonable(robot.get("gripper_measured")),
                        "tcp_pose": jsonable(robot.get("tcp_pose"))}
        chunk = result.get("last_chunk")
        if isinstance(chunk, dict):
            out["last_chunk"] = {k: jsonable(chunk.get(k)) for k in ("status", "message")
                                 if k in chunk}
    return out


def traced(fn, kind: str, call: str | None = None):
    """fn, emitting one harness event per call: its arguments and outcome or error."""
    call = call or fn.__name__
    sig = inspect.signature(fn)

    @functools.wraps(fn)
    def wrapper(*args, **kw):
        if not enabled():
            return fn(*args, **kw)
        depth = getattr(_local, "depth", 0)
        _local.depth = depth + 1
        t0 = time.monotonic()
        try:
            result = fn(*args, **kw)
        except Exception as e:
            _local.depth = depth
            _safe_emit(kind, call, sig, args, kw, depth, t0, error=e)
            raise
        _local.depth = depth
        _safe_emit(kind, call, sig, args, kw, depth, t0, result=result)
        return result

    return wrapper


def _safe_emit(kind, call, sig, args, kw, depth, t0, result=None, error=None) -> None:
    try:
        bound = sig.bind_partial(*args, **kw).arguments
        bound.pop("self", None)
        data = {"call": call, "args": jsonable(dict(bound)), "depth": depth,
                "dur_s": round(time.monotonic() - t0, 3)}
        if error is not None:
            status = getattr(error, "status", None)
            data.update(ok=False, error=type(error).__name__, message=str(error)[:MAX_STR],
                        status=status, code=failure_code(str(error), status) or "OTHER")
            emit(kind, data, lvl="warn")
        else:
            data.update(outcome(result, bound))
            emit(kind, data)
    except Exception:  # noqa: BLE001, S110 - a trace failure must never reach the model's code
        pass


def instrument(cls) -> None:
    """Wrap cls's methods named in CALLS."""
    for name, kind in CALLS.items():
        fn = cls.__dict__.get(name)
        if fn is not None and not hasattr(fn, "__wrapped__"):
            setattr(cls, name, traced(fn, kind))
