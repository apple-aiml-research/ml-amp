#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Helpers every robot backend shares (RobotBackend: amp.contracts.robot)."""

from __future__ import annotations

import os
from pathlib import Path
from urllib.parse import urlsplit

SRC_DIR = Path(__file__).resolve().parents[2]
ISAAC_ABSENT = ("ISAAC_*", "CARB_*", "EXP_PATH", "LD_LIBRARY_PATH")


def server_address(url: str) -> tuple[str, int]:
    """(host, port) of a robot server URL; ValueError unless it is http://host:port."""
    parts = urlsplit(url)
    if parts.scheme != "http" or not parts.hostname or parts.port is None:
        raise ValueError(f"robot server URL {url!r}: expected http://<host>:<port>")
    return parts.hostname, parts.port


def pythonpath(*dirs, inherited: str = "") -> str:
    """AMP's src first, then `dirs`, then `inherited`, without duplicates."""
    out: list[str] = []
    for p in [str(SRC_DIR), *map(str, dirs), *inherited.split(os.pathsep)]:
        if p and p not in out:
            out.append(p)
    return os.pathsep.join(out)
