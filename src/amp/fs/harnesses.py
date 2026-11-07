#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Generate an overlay harness in a checkout for layers that may not import amp.harnesses."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

MANIFEST = "manifest.json"
CHILD_ENV = ("PATH", "LANG", "LC_ALL", "LC_CTYPE", "TMPDIR", "SYSTEMROOT")


def child_env(src: Path, environ=None) -> dict[str, str]:
    environ = os.environ if environ is None else environ
    return {**{k: environ[k] for k in CHILD_ENV if k in environ}, "PYTHONPATH": str(src),
            "PYTHONDONTWRITEBYTECODE": "1"}


def ensure_harness(harness_root: str | Path, name: str, timeout_s: float = 120.0) -> bool:
    """True if <harness_root>/<name> is a harness, after generating it when name is an overlay
    harness."""
    root = Path(harness_root)
    if (root / name / MANIFEST).is_file():
        return True
    src = root.resolve().parents[1]
    if not (src / "amp" / "harnesses" / "overlays.py").is_file():
        return False
    env = child_env(src)
    try:
        subprocess.run([sys.executable, "-m", "amp.harnesses.overlays", "materialize", name,
                        "--root", str(root)], env=env, capture_output=True, timeout=timeout_s,
                       check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return (root / name / MANIFEST).is_file()
