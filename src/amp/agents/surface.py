#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Model-visible text the agent loop injects, and its version (locked as agent/<version>)."""

from __future__ import annotations

AGENT_SURFACE_VERSION = 1
SELF_JUDGED_PROMPTS = frozenset({"amp1"})
SELF_JUDGED = {
    "finish_episode": ("End the episode now, scored as it stands. Use it once you have verified "
                       "the task is done, or when it has become impossible."),
    "submit_report": ("Your required last action once the episode is over: your account of the "
                      "episode. diagnosis is required unless you are confident the task "
                      "succeeded."),
    "diagnosis_required": ("diagnosis is required unless outcome_belief is success. Explain why "
                           "it failed or may have failed."),
    "episode_over": "The episode is over.",
    "rh": "rh (the robot SDK)",
}

EPISODE_OVER = {
    "over": "The episode is over (end: {end}).",
    "ended": (" It ended by itself: RoboLab ends an episode when its success check holds "
              "or when the step budget runs out."),
    "last_obs": (" The last observation before the end: step {step} of {max_steps} "
                 "({steps_left} left), mode {mode}."),
    "tail": (" Nothing more can be done in it. Now call submit_report (view_image is "
             "still available to look at saved images)."),
}
EPISODE_OVER_BENCHMARK = {
    "ended": (" It ended by itself: The benchmark ends an episode when its success "
              "check holds or when the step budget runs out."),
}


def self_judged(prompt: str) -> bool:
    return prompt in SELF_JUDGED_PROMPTS


def episode_over_text(end, obs: dict, benchmark: bool = False,
                      judged_by_self: bool = False) -> str:
    """The model's episode-over tool result; benchmark: not RoboLab's robot; judged_by_self: says neither why nor when."""
    if judged_by_self:
        return SELF_JUDGED["episode_over"] + EPISODE_OVER["tail"]
    msg = EPISODE_OVER["over"].format(end=end)
    if end == "ended":
        msg += (EPISODE_OVER_BENCHMARK if benchmark else EPISODE_OVER)["ended"]
    if obs.get("step") is not None:
        msg += EPISODE_OVER["last_obs"].format(
            step=obs.get("step"), max_steps=obs.get("max_steps"),
            steps_left=obs.get("steps_left"), mode=obs.get("mode"))
    return msg + EPISODE_OVER["tail"]
