#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The per-slot overlay root: RoboDojo's read-only checkout, symlinked, plus AMP's writable parts."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

REAL_DIRS = ("eval_result", "env", "env_cfg", "XPolicyLab")
STAMP = ".amp_overlay.json"
LAYOUT = 2
ENV_CFG = "arx_x5_amp.yml"
CAMERA_CFG = "camera_config_amp.yml"
POLICY = "amp"


class OverlayError(RuntimeError):
    """The RoboDojo checkout is not where or what the overlay needs it to be."""


def _link(target: Path, where: Path) -> None:
    where.symlink_to(target)


def _link_entries(src: Path, dest: Path, *, skip: tuple[str, ...] = ()) -> None:
    for entry in sorted(src.iterdir()):
        if entry.name in skip:
            continue
        _link(entry, dest / entry.name)


def _digest(paths: list[Path]) -> str:
    """A fingerprint of AMP's own files, so editing one rebuilds the overlay."""
    h = hashlib.sha256()
    for root in paths:
        h.update(str(root).encode())
        if not root.exists():
            continue
        for p in sorted(root.rglob("*")) if root.is_dir() else [root]:
            if p.is_file():
                st = p.stat()
                h.update(f"|{p.relative_to(root)}|{st.st_size}|{int(st.st_mtime_ns)}".encode())
    return h.hexdigest()


def stamp(robodojo_root: Path, amp_policy_dir: Path, amp_env_cfg_dir: Path) -> dict:
    return {"robodojo_root": str(robodojo_root), "layout": LAYOUT,
            "amp": _digest([amp_policy_dir, amp_env_cfg_dir])}


def build_overlay(robodojo_root: Path | str, overlay: Path | str, *, amp_policy_dir: Path | str,
                  amp_env_cfg_dir: Path | str) -> Path:
    """Build (or keep) the overlay root and return it; idempotent per (checkout, AMP files)."""
    root = Path(robodojo_root).absolute()
    out = Path(overlay).absolute()
    policy_dir, env_cfg_dir = Path(amp_policy_dir).absolute(), Path(amp_env_cfg_dir).absolute()
    for needed in (root, root / "env", root / "env_cfg", root / "XPolicyLab",
                   root / "XPolicyLab" / "policy", root / "scripts"):
        if not needed.is_dir():
            raise OverlayError(f"{needed} is not a directory: {root} is not a RoboDojo checkout")
    if not (policy_dir / "deploy.py").is_file():
        raise OverlayError(f"{policy_dir} has no deploy.py: AMP's policy folder is incomplete")
    want = stamp(root, policy_dir, env_cfg_dir)
    current = out / STAMP
    if current.is_file() and (out / "env").is_dir() and not (out / "env").is_symlink():
        try:
            if json.loads(current.read_text()) == want:
                return out
        except ValueError:
            pass
    if out.exists() or out.is_symlink():
        shutil.rmtree(out, ignore_errors=True) if out.is_dir() and not out.is_symlink() \
            else out.unlink()
    for d in ("eval_result", "env", "env_cfg", "env_cfg/camera", "XPolicyLab",
              "XPolicyLab/policy", "XPolicyLab/utils"):
        (out / d).mkdir(parents=True)
    for entry in sorted(root.iterdir()):
        if entry.name in REAL_DIRS or (entry.name.startswith(".") and entry.name != ".cache"):
            continue
        _link(entry, out / entry.name)
    _link_entries(root / "env", out / "env")
    _link_entries(root / "env_cfg", out / "env_cfg", skip=("camera",))
    if (root / "env_cfg" / "camera").is_dir():
        _link_entries(root / "env_cfg" / "camera", out / "env_cfg" / "camera")
    _link(env_cfg_dir / ENV_CFG, out / "env_cfg" / ENV_CFG)
    _link(env_cfg_dir / "camera" / CAMERA_CFG, out / "env_cfg" / "camera" / CAMERA_CFG)
    _link_entries(root / "XPolicyLab", out / "XPolicyLab", skip=("policy", "utils"))
    if (root / "XPolicyLab" / "utils").is_dir():
        _link_entries(root / "XPolicyLab" / "utils", out / "XPolicyLab" / "utils")
    _link_entries(root / "XPolicyLab" / "policy", out / "XPolicyLab" / "policy")
    _link(policy_dir, out / "XPolicyLab" / "policy" / POLICY)
    current.write_text(json.dumps(want))
    return out


def assets_dir(overlay: Path | str) -> Path:
    """Where RoboDojo's own paths find the Assets (a symlink into the checkout's .cache)."""
    return Path(overlay) / "Assets"


def policy_dir() -> Path:
    """AMP's XPolicyLab policy folder in this checkout."""
    return Path(__file__).resolve().parent / "xpl_policy" / POLICY


def env_cfg_dir() -> Path:
    """AMP's env_cfg additions in this checkout."""
    return Path(__file__).resolve().parent / "env_cfg"
