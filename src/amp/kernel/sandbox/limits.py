#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Per-kernel limits: parsing, resolution (defaults, $AMP_KERNEL_LIMITS, argument) and rlimits."""

from __future__ import annotations

import json
import os

GIB = 1 << 30
DEFAULT_LIMITS = {"as": 48 * GIB, "fsize": 4 * GIB, "nproc": 256, "core": 0, "disk": 20 * GIB,
                  "cpus": 4}
LIMITS_ENV = "AMP_KERNEL_LIMITS"
RLIMITS = ("as", "fsize", "nproc", "core")
LIMIT_EXEC = ("import json, os, resource, sys\n"
              "for k, v in json.loads(sys.argv[1]).items():\n"
              "    r = getattr(resource, 'RLIMIT_' + k.upper())\n"
              "    hard = resource.getrlimit(r)[1]\n"
              "    v = v if hard == resource.RLIM_INFINITY else min(v, hard)\n"
              "    resource.setrlimit(r, (v, v))\n"
              "os.execvp(sys.argv[2], sys.argv[2:])\n")
_UNITS = {"k": 1 << 10, "m": 1 << 20, "g": GIB, "t": 1 << 40}


def limit_value(v) -> int | None:
    """An int, "8G" / "512MiB" style sizes, or None for None, "", "none" and "off"."""
    if v is None or (isinstance(v, str) and v.strip().lower() in ("", "none", "off")):
        return None
    if isinstance(v, str):
        s = v.strip().lower().removesuffix("ib").removesuffix("b")
        mult = _UNITS.get(s[-1:], 1)
        return int(float(s[:-1] if s[-1:] in _UNITS else s) * mult)
    return int(v)


def parse_limits(text: str) -> dict:
    """'as=48G,fsize=4G,nproc=1024,core=0,disk=20G' (or a JSON object); 'off' disables one."""
    text = text.strip()
    if not text:
        return {}
    if text.startswith("{"):
        d = json.loads(text)
    else:
        d = dict(kv.split("=", 1) for kv in text.split(",") if kv.strip())
    return {k.strip(): limit_value(v) for k, v in d.items()}


def resolve_limits(limits: dict | None = None, environ=os.environ) -> dict:
    """DEFAULT_LIMITS, overridden by $AMP_KERNEL_LIMITS, then by limits; None disables one."""
    out: dict[str, int | None] = dict(DEFAULT_LIMITS)
    for layer in (parse_limits(environ.get(LIMITS_ENV, "")), limits or {}):
        for k, v in layer.items():
            if k not in DEFAULT_LIMITS:
                raise ValueError(f"unknown kernel limit {k!r}; known: {sorted(DEFAULT_LIMITS)}")
            out[k] = limit_value(v)
    return out


def rlimits_for(limits: dict) -> tuple[dict, dict]:
    """(rlimits to set, {name: why skipped}): Linux sets every one, so none is skipped."""
    return {k: limits[k] for k in RLIMITS if limits.get(k) is not None}, {}


def limit_argv(python: str, rlimits: dict, cmd: list[str]) -> list[str]:
    """cmd, prefixed so that it starts with rlimits in force (unchanged when there are none)."""
    if not rlimits:
        return cmd
    return [python, "-I", "-c", LIMIT_EXEC, json.dumps(rlimits), *cmd]
