#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The system prompt: the profile's template from prompts/, then the harness's reference docs."""

from __future__ import annotations

import re
from pathlib import Path

PROMPTS = Path(__file__).resolve().parent / "prompts"
PROMPT_ID = re.compile(r"^[A-Za-z0-9_.-]+$")


def prompt_text(prompt: str) -> str:
    """prompts/<prompt>.md."""
    if not PROMPT_ID.match(prompt) or prompt.startswith("."):
        raise ValueError(f"bad prompt id {prompt!r}")
    return (PROMPTS / f"{prompt}.md").read_text()


def compose_system(template: str, reference: str) -> str:
    """The template, then the harness reference one blank line after it (none: the template)."""
    if not reference:
        return template
    return template.rstrip("\n") + "\n\n" + reference


def system_text(prompt: str, reference: str) -> str:
    return compose_system(prompt_text(prompt), reference)
