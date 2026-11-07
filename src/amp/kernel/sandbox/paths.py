#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Path helpers for the sandbox: symlinks met while resolving, ancestors, containment."""

from __future__ import annotations

import os
from collections.abc import Iterable
from pathlib import Path

MAX_SYMLINKS = 64


def symlinks(p) -> set[str]:
    """Every symlink met while resolving p (each needs its own allowance or bind)."""
    out: set[str] = set()
    todo = [os.path.abspath(p)]
    while todo and len(out) < MAX_SYMLINKS:
        parts = Path(todo.pop()).parts
        for i in range(2, len(parts) + 1):
            q = Path(*parts[:i])
            if q.is_symlink():
                out.add(str(q))
                t = Path(os.readlink(q))
                t = t if t.is_absolute() else q.parent / t
                todo.append(os.path.abspath(Path(t, *parts[i:])))
                break
    return out


def ancestors(p: str) -> set[str]:
    """The proper ancestors of an absolute path, "/" included."""
    parts = Path(p).parts
    return {str(Path(*parts[:i])) for i in range(1, len(parts))}


def under(p: str, roots: Iterable[str]) -> bool:
    """Whether p is one of roots or lies below one."""
    return any(p == r or p.startswith(r.rstrip("/") + "/") for r in roots)
