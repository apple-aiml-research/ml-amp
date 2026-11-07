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


def episode_over_text(end, obs: dict, benchmark: bool = False, judged_by_self: bool = False,
                      vdm: bool = False) -> str:
    """The model's episode-over tool result; benchmark: not RoboLab's robot; judged_by_self: says
    neither why nor when; vdm: a model that sees no images (no view_image)."""
    if judged_by_self:
        return SELF_JUDGED["episode_over"] + (VDM_NOTES["over_tail"] if vdm else
                                              EPISODE_OVER["tail"])
    msg = EPISODE_OVER["over"].format(end=end)
    if end == "ended":
        msg += (EPISODE_OVER_BENCHMARK if benchmark else EPISODE_OVER)["ended"]
    if obs.get("step") is not None:
        msg += EPISODE_OVER["last_obs"].format(
            step=obs.get("step"), max_steps=obs.get("max_steps"),
            steps_left=obs.get("steps_left"), mode=obs.get("mode"))
    return msg + (VDM_NOTES["over_tail"] if vdm else EPISODE_OVER["tail"])

VDM_SURFACE_VERSION = 1

VDM_NOTES = {
    "diff": "[vision model] What changed between step {before} and step {after}, seen by {seen}:"
            "\n{text}",
    "look": "[vision model] What the last motion changed (step {before} to step {after}), seen by "
            "{seen}:\n{text}",
    "scene": "[vision model] The scene at step {after}, seen by {seen}:\n{text}",
    "views": "{cams}, and your views: {paths}",
    "camera_view": "the {cam} camera",
    "unavailable": "[vision model] No description this time: {error}",
    "camera_only": "[vision model] Only cameras are described in this episode: show(camera=<name>, "
                   "caption=...). {n} image(s) you show()ed were saved but not described.",
    "no_camera": "[vision model] No camera {cam!r} in this episode; its cameras: {cams}.",
    "refused": "vision vdm_strict: this episode's code gets no camera, depth or perception data; "
               "show(camera=<name>, caption=...) asks the vision model",
    "over_tail": " Nothing more can be done in it. Now call submit_report.",
    "report_only": "The episode is over: only submit_report is available now.",
}

VDM_PROMPTS = {
    "task": "The robot's task: {instruction}",
    "initial_system": (
        "You are a helpful assistant that describes the initial state of the environment with "
        "the goal of the task in mind. You should try to provide objective information and no "
        "assumptions. Do *NOT* write any code."),
    "initial_user": (
        "Describe the initial state of the environment with the goal of the task in mind. You "
        "should try to provide objective information and no assumptions. Do *NOT* write any "
        "code."),
    "view": "{cam} camera view:",
    "diff_system": (
        "You are a helpful assistant that describes the difference between the current state of "
        "the environment and the previous state of the environment with the goal of the task in "
        "mind and whether the task has been completed. You should try to provide objective "
        "information and no assumptions. Do *NOT* write any code."),
    "diff_user": (
        "Describe the difference between the current state of the environment and the previous "
        "state of the environment with the goal of the task in mind and whether the task has "
        "been completed. You should try to provide objective information and no assumptions. "
        "Do *NOT* write any code.."),
    "previous": "Previous state ({cam} camera):",
    "current": "Current state ({cam} camera):",
    "agent_view": "The agent controlling the robot also looked at the current state through this "
                  "view ({path}){caption}:",
    "caption": ', captioned "{caption}"',
    "agent_camera": "The agent controlling the robot asks about the {cam} camera{caption}.",
    "focus": 'The agent controlling the robot asks: "{captions}". Pay particular attention to '
             "this in your description.",
    "qualitative": ("Describe positions, sizes and distances only in words: relative to the "
                    "gripper, the robot and the other objects (left of, behind, touching, about "
                    "as wide as the open gripper). Give no pixel coordinates, distances, angles, "
                    "sizes or other numbers, even if the agent asks for them."),
}

_SEEN_BY = """\
## How you see in this episode

This section overrides everything above, the reference included, about seeing images. You see no
images yourself: a separate vision model looks at the cameras for you and answers in text, inside
the result of your `python` call.

- Your first `python` result carries its description of the scene at the start. After every call
  that moved the robot (a motion or a gripper action), it describes what changed in every camera
  between just before the motion and just after it.
"""
_JUDGE = """\
- Wherever the text above tells you to look at the cameras or at an image, or to judge from
  images, ask the vision model instead, and judge from its answers together with the numbers you
  have. Its descriptions can be vague or wrong: check them against the numbers, and ask about a
  second camera before you rely on one.
- If you must decide yourself that the task is done, ask the vision model about each part of the
  instruction, about at least two cameras, before you finish: ask it to describe where each
  object is relative to its target rather than to answer yes or no, and fix anything unclear while
  steps remain.
"""
VISION_SECTIONS = {
    "vdm_strict": _SEEN_BY + """\
- `show(camera=<name>, caption=None)` asks it about one of the robot's cameras, by the name the
  robot's description gives it; write the caption as your question, for example
  `show(camera="wrist", caption="is the cube between the fingers?")`. It sees the plain camera
  frames: nothing you draw reaches it. An image you pass as `show(x)` is saved but not described.
  `view_image` is not available.
- **It answers only in words**, never with pixel coordinates, distances, angles or other numbers,
  even if you ask. Ask for relations: left or right of, in front of, behind, above, touching, and
  how far relative to the gripper or to other objects.
- **Your code gets no camera images**: the calls for them (camera frames, and annotating or
  rendering a camera view) are refused with "not available in the sandbox: vision vdm_strict".
  The "Perception" section at the start of the reference says what the harness withholds from
  every agent alike (depth, perceived objects, the table height, camera calibration). Where the
  text above or the reference advises camera images, depth, point clouds, picked pixels,
  perceived objects, calibration or triangulation, that does not apply: the vision model's words
  are your only view of the scene. You have the robot's proprioception (joint positions, gripper
  command and measured opening, end-effector and TCP pose), its model and kinematics, whatever
  the collision checks report, the instruction, the budget and the vision model's text.
- You know an object's position only roughly: close in on it in small steps from the gripper's
  known pose (move, ask where it now is relative to the fingers, correct) before you descend, and
  descend in short steps, since you are not given the table's height.
- In your report, `key_images` may be empty: the camera views you asked about are not saved.
""" + _JUDGE,
}


def vision_section(prompt: str, vision: str) -> str:
    """The section a vision harness configuration's system text ends with: none for image ones."""
    if vision == "image":
        return ""
    return VISION_SECTIONS[vision]
