#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Candidate IK and scoring for grasp and place search: one solve per pose, best of seeds."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from amp.geometry.kinematics import matrix_to_quat
from amp.planning.poses import EZ, FLIP
from amp.planning.robot import arm_of, plan_arm
from amp.planning.scene_geom import MAX_WIDTH, MIN_WIDTH

__all__ = [
    "IK_POS_TOL",
    "IK_ROT_TOL",
    "LIMIT_MARGIN",
    "MAX_INFEASIBLE",
    "CandidateIK",
    "candidate_flags",
    "joint_terms",
    "solve_at",
    "solve_best",
    "weighted_score",
    "wrist_options",
]

IK_POS_TOL = 1e-3
IK_ROT_TOL = math.radians(0.5)
LIMIT_MARGIN = 0.02
MAX_INFEASIBLE = 3


def _twist(R_rel, n) -> float:
    q = matrix_to_quat(R_rel)
    return 2.0 * math.atan2(float(q[1:] @ n), float(q[0]))


def wrist_options(R, q_seed, tool, flips=(False, True)) -> list[tuple[bool, float]]:
    """Per 180-deg flip about the approach, the last joint's target nearest q_seed within limits
    (the last joint turning the TCP about its approach, as the Panda's joint 7 does)."""
    arm = arm_of(tool)
    w = arm.n - 1
    R_seed = tool.fk_tcp(q_seed)[:3, :3]
    n = tool.T_flange_tcp[:3, :3].T @ EZ
    lo, hi = arm.q_min[w] + 0.05, arm.q_max[w] - 0.05
    opts = []
    for flip in flips:
        tw = _twist(R_seed.T @ (R @ FLIP[:3, :3] if flip else R), n)
        best = None
        for k in (-1, 0, 1):
            q7 = float(q_seed[w]) + tw + 2.0 * math.pi * k
            key = (max(0.0, lo - q7, q7 - hi), abs(tw + 2.0 * math.pi * k))
            if best is None or key < best[0]:
                best = (key, flip, q7)
        opts.append(best)
    opts.sort(key=lambda o: o[0])
    return [(flip, q7) for _, flip, q7 in opts]


@dataclass
class CandidateIK:
    """One IK solve for a candidate pose: q, residuals, convergence and limit margins."""

    q: np.ndarray
    pos_err: float
    rot_err: float
    converged: bool
    limit_margin: np.ndarray

    @property
    def good(self) -> bool:
        return self.converged and float(self.limit_margin.min()) >= LIMIT_MARGIN

    def note(self, what: str) -> str | None:
        if not self.converged:
            return (
                f"{what} unreachable: IK off by {self.pos_err * 1000:.0f} mm, "
                f"{math.degrees(self.rot_err):.1f} deg"
            )
        j = int(np.argmin(self.limit_margin))
        if self.limit_margin[j] < LIMIT_MARGIN:
            return f"{what}: joint {j + 1} within {LIMIT_MARGIN:.2f} rad of its limit"
        return None


def solve_at(T, seed: np.ndarray, tool) -> CandidateIK:
    """IK of TCP pose T from seed, judged against the candidate tolerances."""
    arm = arm_of(tool)
    r = arm.ik(T, seed, tool.T_flange_tcp, pos_tol=2e-4, rot_tol=2e-3, q_rest=seed)
    conv = r.pos_err <= IK_POS_TOL and r.rot_err <= IK_ROT_TOL
    return CandidateIK(r.q, r.pos_err, r.rot_err, conv,
                       np.minimum(r.q - arm.q_min, arm.q_max - r.q))


def solve_best(T, q_seed: np.ndarray, tool) -> CandidateIK:
    """IK from q_seed (the last joint pre-turned), falling back to the arm's home."""
    arm = arm_of(tool)
    best = None
    for s in (q_seed, arm.q_home):
        if best is not None and np.allclose(s, q_seed, atol=1e-3):
            continue
        seed = s.copy()
        seed[arm.n - 1] = wrist_options(T[:3, :3], s, tool, flips=(False,))[0][1]
        ik = solve_at(T, np.clip(seed, arm.q_min, arm.q_max), tool)
        if best is None or (ik.good, ik.converged, -ik.pos_err) > (
            best.good, best.converged, -best.pos_err
        ):
            best = ik
        if best.good:
            break
    return best


def joint_terms(q, q_seed, arm=None) -> tuple[float, float]:
    """(limit-margin term, seed-travel term) of q, each in [0, 1]; arm None = DROID's."""
    if q is None:
        return 0.0, 0.0
    arm = plan_arm(None) if arm is None else arm
    margin = float(np.min(np.minimum(q - arm.q_min, arm.q_max - q)))
    travel = float(np.linalg.norm(q - q_seed))
    return min(1.0, max(0.0, margin) / 0.3), 1.0 - min(1.0, travel / np.pi)


def weighted_score(terms: dict[str, float], weights: dict[str, float]) -> float:
    """The weighted sum of score terms over the keys of weights."""
    return float(sum(weights[k] * terms[k] for k in weights))


def candidate_flags(c) -> int:
    """How many of reachable, collision-free and graspable-width a candidate is."""
    width_ok = MIN_WIDTH <= getattr(c, "width", MIN_WIDTH) <= MAX_WIDTH
    return int(c.reachable) + int(c.collision_free) + int(width_ok)
