#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The launchers' --vision and --vdm-model."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from amp.contracts.eval import Arm


def add_arguments(ap: argparse.ArgumentParser) -> None:
    """The vision flags, as `amp eval robolab` and `amp eval capgym` take them."""
    ap.add_argument("--vision", choices=("image", "vdm_strict"), default="image",
                    help="how the model sees: image (default); vdm_strict: no images, a vision "
                    "model describes the cameras in text, and the model's code "
                    "gets no camera images either")
    ap.add_argument("--vdm-model", default=None,
                    help="the vision model of --vision vdm_strict (default opus-5.5)")


def profile_kwargs(a: argparse.Namespace) -> dict:
    """make_profile's vision and vdm_model from the parsed flags."""
    return {"vision": a.vision, "vdm_model": a.vdm_model}


def check(arms: Sequence[Arm], sandbox: str) -> None:
    """ValueError for a vdm_strict harness configuration that cannot hold: its sandbox's relay
    withholds the camera images, and only a harness without perception keeps the rest out."""
    strict = [a for a in arms if a.agent.vision == "vdm_strict"]
    from amp.agents.models import strict_problem

    for a in strict:
        why = strict_problem(a.harness)
        if why:
            raise ValueError(f"--vision vdm_strict: {why}")
