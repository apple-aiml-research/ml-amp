#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Environment scrubbing for the eval's children and for git: secret-looking names are dropped."""

from __future__ import annotations

import os
import subprocess
from collections.abc import Iterable, Mapping

SECRET_WORDS = frozenset({"TOKEN", "SECRET", "SECRETS", "PASSWORD", "PASSWD", "PASS", "KEY",
                          "KEYS", "CREDENTIAL", "CREDENTIALS", "PRIVATE", "AUTH", "COOKIE",
                          "TOKENS", "BEARER"})


SECRET_SUFFIXES = ("SECRET", "TOKEN", "TOKENS", "PASSWORD", "PASSWD", "PRIVATE_KEY", "BEARER",
                   "APIKEY", "APIKEYS")


GIT_CONFIG_OVERRIDES = frozenset({"GIT_CONFIG_COUNT", "GIT_CONFIG_PARAMETERS", "GIT_CONFIG_GLOBAL",
                                  "GIT_CONFIG_SYSTEM"})


def git_config_name(name: str) -> bool:
    """A git config override: COUNT with KEY_n / VALUE_n (valid only whole), PARAMETERS, GLOBAL, SYSTEM."""
    return name in GIT_CONFIG_OVERRIDES or (name.startswith(("GIT_CONFIG_KEY_", "GIT_CONFIG_VALUE_"))
                                            and name.rsplit("_", 1)[1].isdigit())


def secret_name(name: str) -> bool:
    up = name.upper()
    return bool(set(up.split("_")) & SECRET_WORDS) or up.endswith(SECRET_SUFFIXES)


def node_env(environ: Mapping[str, str] | None = None, *, keep: Iterable[str] = ()) -> dict[str, str]:
    """environ (default os.environ) minus secret_name() and git config names, but keep."""
    env = os.environ if environ is None else environ
    keep = set(keep)
    return {k: v for k, v in env.items()
            if k in keep or not (secret_name(k) or git_config_name(k))}


def run(args, *, env: Mapping[str, str] | None = None, keep: Iterable[str] = (),
        check: bool = False, **kw) -> subprocess.CompletedProcess:
    """subprocess.run whose child gets node_env(env, keep=keep): nothing secret by default."""
    return subprocess.run(args, env=node_env(env, keep=keep), check=check, **kw)


def popen(args, *, env: Mapping[str, str] | None = None, keep: Iterable[str] = (),
          **kw) -> subprocess.Popen:
    """subprocess.Popen whose child gets node_env(env, keep=keep): nothing secret by default."""
    return subprocess.Popen(args, env=node_env(env, keep=keep), **kw)
