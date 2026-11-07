#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The no-op XPolicyLab model: AMP's actions come over the bridge, not from this server."""

from __future__ import annotations


class Model:
    """Satisfies the policy server's handshake and reset; it never produces an action."""

    def __init__(self, model_cfg=None):
        self.model_cfg = dict(model_cfg or {})
        self.model = None
        print("[amp.model] no-op policy server up; AMP drives the episode over its bridge",
              flush=True)

    def update_obs(self, obs):
        return None

    def update_obs_batch(self, obs_list):
        return None

    def get_action(self):
        return []

    def get_action_batch(self, env_idx_list=None):
        return [] if env_idx_list is None else [[] for _ in env_idx_list]

    def reset(self):
        return None

    def prepare_case(self, case_meta=None):
        return None

    def on_trial_end(self, result=None):
        return None
