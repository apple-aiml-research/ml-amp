#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""TCP-pose IK for agents: distinct solutions from several seeds, accurate ones nearest the seed."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amp.geometry import kinematics as kin
from amp.geometry.embodiment import Embodiment
from amp.planning import robot

POS_TOL_MM = 5.0
ROT_TOL_DEG = 3.0
MAX_DEFAULT = 4
MAX_LIMIT = 16
RANDOM_SEEDS = 14
BATCH_RANDOM_SEEDS = 4
ITERS = 200
SAME_RAD = 0.05


@dataclass(frozen=True)
class IKSolution:
    """One IK solution with its residuals, rounded as reported."""

    q: tuple[float, ...]
    pos_err_mm: float
    rot_err_deg: float
    limit_margin_rad: float
    singular: bool
    dist_from_seed_rad: float

    @property
    def accurate(self) -> bool:
        return self.pos_err_mm <= POS_TOL_MM and self.rot_err_deg <= ROT_TOL_DEG

    def to_json(self) -> dict:
        return {"q": list(self.q), "pos_err_mm": self.pos_err_mm,
                "rot_err_deg": self.rot_err_deg, "limit_margin_rad": self.limit_margin_rad,
                "singular": self.singular, "dist_from_seed_rad": self.dist_from_seed_rad}


def singular(q, embodiment: Embodiment | None = None, ee: str | None = None) -> bool:
    """Whether q is within a declared singularity band of the arm of ee (None: the default)."""
    return not robot.plan_arm(embodiment, ee).singular_ok(q)


def in_limits(q, embodiment: Embodiment | None = None, ee: str | None = None) -> bool:
    arm = robot.plan_arm(embodiment, ee)
    q = np.asarray(q, dtype=np.float64)
    return bool(np.all(q >= arm.q_min) and np.all(q <= arm.q_max))


def reachable(sols: list[IKSolution]) -> bool:
    return bool(sols) and sols[0].accurate


def seeds(seed, n_random: int, embodiment: Embodiment | None = None,
          ee: str | None = None) -> np.ndarray:
    """The seed, the arm's home and n_random fixed pseudo-random configurations."""
    arm = robot.plan_arm(embodiment, ee)
    rng = np.random.default_rng(0)
    rand = rng.uniform(arm.q_min + 0.1, arm.q_max - 0.1, size=(n_random, arm.n))
    return np.vstack([seed, arm.q_home, rand])


def rank(results, seed, n_max: int, embodiment: Embodiment | None = None,
         ee: str | None = None) -> list[IKSolution]:
    """Accurate solutions nearest seed first, then the rest by error; near-duplicates dropped."""
    arm = robot.plan_arm(embodiment, ee)
    sols = []
    for r in results:
        q = np.asarray(r.q, dtype=np.float64)
        sols.append({"q": q, "pos": r.pos_err * 1000.0, "rot": math.degrees(r.rot_err),
                     "dist": float(np.linalg.norm(q - seed))})

    def ok(d):
        return d["pos"] <= POS_TOL_MM and d["rot"] <= ROT_TOL_DEG

    sols.sort(key=lambda d: (not ok(d), d["dist"] if ok(d) else d["pos"] + d["rot"]))
    kept = []
    for d in sols:
        if any(np.max(np.abs(d["q"] - k["q"])) < SAME_RAD for k in kept):
            continue
        kept.append(d)
        if len(kept) >= n_max:
            break
    return [IKSolution(q=tuple(float(v) for v in d["q"]), pos_err_mm=round(d["pos"], 4),
                       rot_err_deg=round(d["rot"], 4),
                       limit_margin_rad=round(float(arm.limit_margin(d["q"])), 5),
                       singular=not arm.singular_ok(d["q"]),
                       dist_from_seed_rad=round(d["dist"], 5))
            for d in kept]


def solve(T_tcp, seed, tool: kin.ToolModel, n_max: int = MAX_DEFAULT,
          embodiment: Embodiment | None = None, ee: str | None = None) -> list[IKSolution]:
    """Distinct IK solutions for a TCP pose from seed, the arm's home and RANDOM_SEEDS more."""
    arm = robot.check_tool(embodiment, tool, ee)
    seed = _seed(seed, arm)
    results = arm.ik_batch(T_tcp, seeds(seed, RANDOM_SEEDS, embodiment, ee), tool.T_flange_tcp,
                           max_iters=ITERS)
    return rank(results, seed, n_max, embodiment, ee)


def solve_batch(T_tcps, seed, tool: kin.ToolModel, n_max: int = 1,
                embodiment: Embodiment | None = None,
                ee: str | None = None) -> list[list[IKSolution]]:
    """solve per TCP pose, from fewer seeds (BATCH_RANDOM_SEEDS), all iterated together."""
    arm = robot.check_tool(embodiment, tool, ee)
    Ts = np.asarray(T_tcps, dtype=np.float64).reshape(-1, 4, 4)
    seed = _seed(seed, arm)
    S = seeds(seed, BATCH_RANDOM_SEEDS, embodiment, ee)
    res = arm.ik_batch(np.repeat(Ts, len(S), axis=0), np.tile(S, (len(Ts), 1)),
                       tool.T_flange_tcp, max_iters=ITERS)
    return [rank(res[i * len(S):(i + 1) * len(S)], seed, n_max, embodiment, ee)
            for i in range(len(Ts))]


def _seed(seed, arm) -> np.ndarray:
    q = np.asarray(seed, dtype=np.float64).reshape(-1)
    if len(q) != arm.n:
        raise ValueError(f"seed has {len(q)} values, arm {arm.name} has {arm.n} joints")
    return q
