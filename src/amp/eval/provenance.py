#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What a run executes besides the job's own AMP code: its runtime commit, RoboLab, CaP-X and
RoboDojo."""

from __future__ import annotations

import os
import re
import subprocess
import sys
from collections.abc import Callable, Mapping
from pathlib import Path

from . import envs

CAPX_ROOT_ENV = "AMP_CAPX_ROOT"
ROBOLAB_ROOT_ENV = "AMP_ROBOLAB_ROOT"
ROBODOJO_ROOT_ENV = "AMP_ROBODOJO_ROOT"
PINS = {"robolab_commit": ("RoboLab", "9db0aaf09d9fe5d4f37b168320788258c7012463"),
        "capx_commit": ("CaP-X", "53e9966d7a8e2fa7494676772bccc35280f5c0ed"),
        "robodojo_commit": ("RoboDojo", "09bd5f5e762b949f5858ece31347ce62057b3a5e")}
CAPGYM_BOUNDARY_KEY = "capgym_boundary_v"
CAPGYM_BOUNDARY_V = 1


def git_head(path: str | Path | None) -> str | None:
    """The full sha checked out at path; None if path is not the root of a checkout."""
    if not path or not Path(path).exists():
        return None
    try:
        p = envs.run(["git", "-c", f"safe.directory={path}", "-C", str(path), "rev-parse",
                      "--show-toplevel", "HEAD"], capture_output=True, text=True, timeout=10,
                     check=False, stdin=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired):
        return None
    top, _, sha = p.stdout.strip().rpartition("\n")
    if p.returncode or not top or Path(top).resolve() != Path(path).resolve():
        return None
    return sha if re.fullmatch(r"[0-9a-f]{40}", sha) else None


def pin_warnings(facts: Mapping) -> list[str]:
    """One line per checkout that is not at the commit in docs/reproducibility.md."""
    return [f"warning: {name} is checked out at {facts[key]}, not the pinned {pin} "
            "(docs/reproducibility.md); results may not match the published ones"
            for key, (name, pin) in PINS.items() if facts.get(key) and facts[key] != pin]


def runtime_facts(*, amp_runtime_commit: str | None = None, runtime_src: str | Path | None = None,
                  robolab_root: str | Path | None = None, capx_root: str | Path | None = None,
                  robodojo_root: str | Path | None = None,
                  environ: Mapping[str, str] | None = None,
                  warn: Callable[[str], None] = lambda m: print(m, file=sys.stderr)) -> dict:
    """The runtime commits, read once at start; warns for a checkout off its pin,
    never refuses."""
    env = os.environ if environ is None else environ
    src = Path(runtime_src) if runtime_src else Path(__file__).resolve().parents[2]
    root = robolab_root or env.get(ROBOLAB_ROOT_ENV) or src.parent / "external" / "robolab"
    capx = capx_root or env.get(CAPX_ROOT_ENV) or src.parent / "external" / "cap-x"
    dojo = robodojo_root or env.get(ROBODOJO_ROOT_ENV) or src.parent / "external" / "robodojo"
    facts = {"amp_runtime_commit": amp_runtime_commit or git_head(src.parent) or None,
             "robolab_commit": git_head(root),
             "capx_commit": git_head(capx),
             "robodojo_commit": git_head(dojo),
             CAPGYM_BOUNDARY_KEY: CAPGYM_BOUNDARY_V}
    for line in pin_warnings(facts):
        warn(line)
    return facts
