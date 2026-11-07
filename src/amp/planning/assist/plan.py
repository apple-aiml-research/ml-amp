#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""AssistPlan: an ordered list of pick/place steps, each re-planned when anything before it
changes, from the state the steps before it predict."""

from __future__ import annotations

import dataclasses

import numpy as np

from amp.planning import options as opt
from amp.planning.assist import execute, predict, preview
from amp.planning.assist.predict import Infeasible, PredictedState
from amp.planning.assist.steps import KINDS, PlannedStep, StepSpec, as_pose
from amp.planning.planner import PlanContext, Planner

AFTER_INFEASIBLE = "after an infeasible step"


class AssistPlan:
    """The steps, the observed start they are planned from, and the backend that plans them."""

    def __init__(self, backend: Planner, scene, ctx: PlanContext, specs=(), *, version=0):
        self.backend = backend
        self.version = int(version)
        self.steps: list[PlannedStep] = []
        self._epoch = 0
        self._set_base(scene, ctx)
        for spec in specs:
            self._check_new_id(spec.id)
            self.steps.append(PlannedStep(spec))
        self._replan(0)

    def _set_base(self, scene, ctx: PlanContext):
        self.scene = scene
        self.ctx = ctx
        self._epoch += 1
        self._start = predict.observed_state(scene, ctx)

    def __len__(self) -> int:
        return len(self.steps)

    @property
    def valid(self) -> bool:
        return all(s.ok for s in self.steps)

    def index(self, id) -> int:
        for i, s in enumerate(self.steps):
            if s.spec.id == id:
                return i
        raise KeyError(f"no plan step with id {id!r}")

    def step(self, id) -> PlannedStep:
        return self.steps[self.index(id)]

    def next_id(self) -> int:
        return max((s.spec.id for s in self.steps), default=0) + 1

    def state_at(self, n: int) -> PredictedState:
        """The predicted state before step n (0 = the observed start, len = the plan's end)."""
        if isinstance(n, bool) or not isinstance(n, (int, np.integer)) or not 0 <= n <= len(self):
            raise ValueError(f"at must be an integer in [0, {len(self)}], got {n!r}")
        if n == 0:
            return self._start
        prev = self.steps[n - 1]
        return prev.after if prev.after is not None else prev.before

    def anchor_for(self, kind, name, T_world_tcp, n, *, target=None, point=None) -> np.ndarray:
        """The anchor of a step whose world TCP T_world_tcp was evaluated in state_at(n)."""
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {list(KINDS)}, got {kind!r}")
        return predict.anchor_for(kind, name, T_world_tcp, self.state_at(n), target=target,
                                  point=point)

    def _check_new_id(self, id):
        if any(s.spec.id == id for s in self.steps):
            raise ValueError(f"step id {id} is already in the plan")

    def add(self, spec: StepSpec) -> int:
        """Append spec and plan it from the plan's end; returns its id."""
        self._check_new_id(spec.id)
        self.steps.append(PlannedStep(spec))
        self._replan(len(self.steps) - 1)
        self.version += 1
        return spec.id

    def update(self, id, *, anchor=None, options=None) -> None:
        i = self.index(id)
        spec = self.steps[i].spec
        if anchor is not None:
            spec.anchor = as_pose(anchor, "anchor")
        if options is not None:
            if not isinstance(options, dict):
                raise ValueError("options must be a JSON object")
            spec.options = opt.validate(spec.kind, options)
        self._replan(i)
        self.version += 1

    def delete(self, id) -> None:
        i = self.index(id)
        del self.steps[i]
        self._replan(i)
        self.version += 1

    def clear(self) -> None:
        self.steps = []
        self.version += 1

    def rebase(self, scene, ctx) -> None:
        """A new observed start: every step is re-planned from it."""
        self._set_base(scene, ctx)
        self._replan(0)
        self.version += 1

    def pop_first(self) -> PlannedStep | None:
        """Drop the first step (just executed); the rest keep their plans until the next rebase."""
        if not self.steps:
            return None
        first = self.steps.pop(0)
        self._start = first.after if first.after is not None else first.before or self._start
        self._replan(0)
        self.version += 1
        return first

    def _replan(self, i0: int) -> None:
        """Re-plan steps i0.. in order; past the first infeasible one, only locate them."""
        feasible = all(s.ok for s in self.steps[:i0])
        state = self.state_at(min(i0, len(self.steps)))
        for s in self.steps[i0:]:
            s.before = state
            if feasible:
                self._plan_step(s, state)
                feasible = s.ok
            else:
                s.clear()
                s.message = AFTER_INFEASIBLE
                try:
                    s.tcp = predict.world_tcp(s.spec, state)
                except Infeasible:
                    s.tcp = None
            if s.after is not None:
                state = s.after

    def _key(self, spec, state) -> tuple:
        opts = execute.step_options(spec, self.ctx.settle_s)
        return (
            self._epoch, spec.kind, spec.object, spec.target,
            None if spec.point is None else spec.point.tobytes(),
            None if spec.anchor is None else spec.anchor.tobytes(),
            tuple(sorted((k, repr(v)) for k, v in dataclasses.asdict(opts).items())),
            state.fingerprint(),
        )

    def _plan_step(self, s: PlannedStep, state: PredictedState) -> None:
        """Plan s from state unless nothing it depends on changed since it was last planned."""
        key = self._key(s.spec, state)
        if s.key == key:
            return
        s.clear()
        s.key = key
        spec = s.spec
        try:
            execute.check_pickable(spec, state)
            s.tcp = predict.world_tcp(spec, state)
            out = execute.plan_step(self.backend, self.scene, self.ctx, spec, state, s.tcp,
                                    execute.step_options(spec, self.ctx.settle_s))
        except Infeasible as exc:
            s.ok, s.message = False, str(exc)
            return
        except Exception as exc:  # noqa: BLE001
            s.ok, s.message = False, f"planning failed: {type(exc).__name__}: {exc}"
            return
        if out.anchor is not None:
            spec.anchor = out.anchor
            s.key = self._key(spec, state)
        s.tcp, s.release_height = out.tcp, out.release_height
        s.result, s.after, s.ok, s.message = out.result, out.after, out.ok, out.message

    def preview(self, dt: float) -> dict | None:
        """The combined preview JSON of the planned prefix (see preview.preview)."""
        return preview.preview(self.steps, dt)

    def to_json(self) -> dict:
        """The plan JSON without preview and draft."""
        return preview.plan_json(self.steps, version=self.version)
