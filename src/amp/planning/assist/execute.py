#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Planning one step from its predicted state: the backend call and the state it leads to."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amp.geometry.world import pose_from7
from amp.planning import options as opt
from amp.planning.assist.predict import (
    Infeasible,
    PredictedState,
    after_pick,
    after_place,
    anchor_of_tcp,
)
from amp.planning.assist.steps import StepSpec
from amp.planning.planner import PlanContext, Planner
from amp.planning.trajectory import PlanResult

DEFAULT_SETTLE_S = opt.SETTLE_S


@dataclass
class StepOutcome:
    """A planned step: its plan, the TCP it used, the state after it, and a re-anchoring."""

    result: PlanResult
    tcp: np.ndarray
    after: PredictedState
    release_height: float | None = None
    anchor: np.ndarray | None = None

    @property
    def ok(self) -> bool:
        return bool(self.result.ok)

    @property
    def message(self) -> str:
        if self.ok:
            return ""
        errors = [i for i in self.result.issues if i.severity == "error"]
        return errors[0].message if errors else "the plan has errors"


def step_options(spec: StepSpec, settle_s: float | None):
    """spec's options over the defaults, with the plan's settle time."""
    return opt.parse(spec.kind, spec.options,
                     settle_s=DEFAULT_SETTLE_S if settle_s is None else float(settle_s))


def used_tcp(res: PlanResult, T_tcp) -> tuple[np.ndarray, bool]:
    """(the TCP the plan used, whether it was a variant of T_tcp)."""
    used = (res.extra or {}).get("tcp_used")
    if not (res.extra or {}).get("variant") or used is None:
        return T_tcp, False
    return pose_from7(used, "tcp_used"), True


def release_height(backend: Planner, scene, spec: StepSpec, T_tcp_obj, T_tcp, ctx) -> float:
    """How far the released object drops onto its support; 0 when the backend cannot say."""
    try:
        cand = backend.check_place(scene, spec.object, T_tcp_obj, T_tcp, ctx,
                                   target=spec.target, point=spec.point)
        h = float(cand.release_height)
    except Exception:  # noqa: BLE001
        return 0.0
    return h if math.isfinite(h) else 0.0


def plan_step(backend: Planner, base_scene, base_ctx: PlanContext, spec: StepSpec,
              state: PredictedState, T_tcp, opts) -> StepOutcome:
    """Plan spec at the world TCP T_tcp from state (on base_scene and base_ctx moved to it)."""
    scene, ctx = state.scene(base_scene), state.ctx(base_ctx)
    if spec.kind == "pick":
        res = backend.plan_pick(scene, spec.object, T_tcp, ctx, opts)
        T_used, variant = used_tcp(res, T_tcp)
        after = after_pick(state, spec.object, T_used, res.q[-1], ctx.tool)
        height = None
    else:
        res = backend.plan_place(scene, spec.object, state.T_tcp_obj, T_tcp, ctx, opts,
                                 target=spec.target, point=spec.point)
        T_used, variant = used_tcp(res, T_tcp)
        height = release_height(backend, scene, spec, state.T_tcp_obj, T_used, ctx)
        after = after_place(state, spec.object, T_used, res.q[-1], height)
    res.kind = spec.kind
    return StepOutcome(res, T_used, after, height,
                       anchor_of_tcp(spec, state, T_used) if variant else None)


def check_pickable(spec: StepSpec, state: PredictedState) -> None:
    """Infeasible when a pick starts with the gripper already holding something."""
    if spec.kind == "pick" and state.held is not None:
        raise Infeasible(f"the gripper already holds {state.held}; place it before "
                         f"picking {spec.object}")


__all__ = ["StepOutcome", "check_pickable", "plan_step", "release_height", "step_options",
           "used_tcp"]
