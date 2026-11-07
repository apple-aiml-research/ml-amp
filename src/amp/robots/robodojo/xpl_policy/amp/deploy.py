#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""AMP's in-sim loop: RoboDojo's EvalEnv calls eval_one_episode, which relays over AMP's bridge."""

from __future__ import annotations

HOLD_STEPS = 20000


def eval_one_episode(TASK_ENV, model_client=None):
    """One episode, driven by AMP's launcher over the bridge (XPolicyLab's hook)."""
    from amp.robots.robodojo import bridge as B

    link = B.connect_from_env()
    try:
        _run(TASK_ENV, link)
    finally:
        link.close()


def _run(TASK_ENV, link) -> None:
    from amp.robots.robodojo import frames as F

    while True:
        obs = TASK_ENV.get_obs()
        link.send({"type": "obs", "obs": obs})
        message = link.recv_skipping_keepalives()
        kind = message.get("type")
        if kind == "stop":
            return
        if kind == "hold":
            hold_until_end(TASK_ENV, obs, F.hold_action)
            link.send({"type": "end"})
            return
        if kind != "act":
            raise ValueError(f"[amp.deploy] the launcher sent {kind!r}, not act, hold or stop")
        TASK_ENV.take_action(message["action"])
        if TASK_ENV.is_episode_end():
            link.send({"type": "end"})
            return


def hold_until_end(TASK_ENV, obs, hold_action, max_steps: int = HOLD_STEPS) -> int:
    """Repeat the hold action until RoboDojo ends the episode; the steps it took."""
    for step in range(max_steps):
        if TASK_ENV.is_episode_end():
            return step
        TASK_ENV.take_action(hold_action(obs))
        obs = TASK_ENV.get_obs()
    raise RuntimeError(f"[amp.deploy] the episode did not end within {max_steps} hold steps")
