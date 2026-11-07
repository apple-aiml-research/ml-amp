#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The Linux backend: bubblewrap with every namespace unshared and a root built from binds."""

from __future__ import annotations

import functools
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from .paths import symlinks, under

INNER_SOCK = "/run/amp/relay.sock"
INNER_SITE = "/run/amp/site"
INNER_PORT = 8999
INNER_ID = 1000
SYSTEM_LIBS = ("/usr/lib", "/usr/lib64", "/lib", "/lib64", "/usr/lib32", "/lib32")
SYSTEM_FILES = ("/etc/ld.so.cache", "/etc/ld.so.conf", "/etc/localtime")
TMPFS_BYTES = 256 << 20
TOOL_ENV = {"PATH": "/usr/bin:/bin", "LANG": "C.UTF-8"}
SIZE_FLAG, NO_USERNS = "--size", "--disable-userns"
SHIM_SOURCE = Path(__file__).with_name("netshim.py").read_text()
_EXEC_IGN = ("import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_IGN); "
             "os.execv(sys.argv[1], sys.argv[1:])")
_EXEC_DFL = ("import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); "
             "os.execv(sys.argv[1], sys.argv[1:])")


def _with_deadline(source: str, *pairs: tuple[str, str]) -> str:
    """source with each line followed by the same line for SIGUSR2, the kernel's deadline signal."""
    for line, sig_line in pairs:
        if source.count(line) != 1:
            raise RuntimeError(f"the launch source changed: {line!r}")
        source = source.replace(line, line + sig_line)
    return source


_EXEC_IGN_DEADLINE = _with_deadline(
    _EXEC_IGN, ("signal.signal(signal.SIGINT, signal.SIG_IGN); ",
                "signal.signal(signal.SIGUSR2, signal.SIG_IGN); "))
_EXEC_DFL_DEADLINE = _with_deadline(
    _EXEC_DFL, ("signal.signal(signal.SIGINT, signal.SIG_DFL); ",
                "signal.signal(signal.SIGUSR2, signal.SIG_DFL); "))
SHIM_DEADLINE_SOURCE = _with_deadline(
    SHIM_SOURCE,
    ("        signal.signal(signal.SIGINT, signal.SIG_DFL)\n",
     "        signal.signal(signal.SIGUSR2, signal.SIG_DFL)\n"),
    ("    signal.signal(signal.SIGINT, signal.SIG_IGN)\n",
     "    signal.signal(signal.SIGUSR2, signal.SIG_IGN)\n"))
LAUNCH = r"""
import ctypes, json, os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_IGN)
spec, a = json.loads(sys.argv[1]), sys.argv[2:]
libc = ctypes.CDLL(None, use_errno=True)
libc.mount.argtypes = (ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_ulong,
                       ctypes.c_void_p)


def ck(r, what):
    if r != 0:
        e = ctypes.get_errno()
        sys.exit(f"amp-launch: {what}: {os.strerror(e)}")


ck(libc.unshare(0x20000), "unshare(CLONE_NEWNS)")
ck(libc.mount(None, b"/", None, 0x4000 | 0x40000, None), "make / private")
stage = os.path.join(spec["stage"], str(os.getpid()))
os.mkdir(stage, 0o711)
one = {"--hostname", "--dev", "--tmpfs", "--proc", "--cap-drop", "--chdir", "--uid", "--gid",
       "--size"}
i, n = 1, 0
while i < len(a) and a[i] != "--":
    if a[i] in ("--bind", "--ro-bind"):
        dst = os.path.join(stage, str(n))
        n += 1
        if os.path.isdir(a[i + 1]):
            os.mkdir(dst)
        else:
            open(dst, "w").close()
        ck(libc.mount(a[i + 1].encode(), dst.encode(), None, 0x1000 | 0x4000, None),
           "bind " + a[i + 1])
        a[i + 1] = dst
        i += 3
    else:
        i += 3 if a[i] == "--symlink" else 2 if a[i] in one else 1
os.setgroups([])
os.setresgid(spec["gid"], spec["gid"], spec["gid"])
os.setresuid(spec["uid"], spec["uid"], spec["uid"])
os.execv(a[0], a)
"""

LAUNCH_DEADLINE = _with_deadline(LAUNCH, ("signal.signal(signal.SIGINT, signal.SIG_IGN)\n",
                                          "signal.signal(signal.SIGUSR2, signal.SIG_IGN)\n"))

_PROBE = r"""
import json, os, sys, sysconfig
lib, ld = sysconfig.get_config_var("LIBDIR") or "", sysconfig.get_config_var("LDLIBRARY") or ""
p = sysconfig.get_paths()
print(json.dumps({"executable": sys.executable, "prefix": sys.prefix,
                  "base_prefix": sys.base_prefix, "path": [x for x in sys.path if x],
                  "stdlib": p["stdlib"], "platstdlib": p["platstdlib"],
                  "libpython": os.path.join(lib, ld) if lib and ld else "",
                  "version": sys.version.split()[0]}))
"""


@functools.lru_cache(maxsize=8)
def features(bwrap: str) -> frozenset[str]:
    """Which optional flags (SIZE_FLAG, NO_USERNS) this bwrap's --help lists."""
    try:
        r = subprocess.run([bwrap, "--help"], capture_output=True, text=True, timeout=30,
                           check=False, env=TOOL_ENV)
    except (OSError, subprocess.SubprocessError):
        return frozenset()
    text = r.stdout + r.stderr
    return frozenset(f for f in (SIZE_FLAG, NO_USERNS) if f in text.split())


def find_bwrap() -> str | None:
    return shutil.which("bwrap") or next(
        (p for p in ("/usr/bin/bwrap", "/usr/local/bin/bwrap") if os.access(p, os.X_OK)), None)


@functools.lru_cache(maxsize=8)
def interpreter(python: str) -> dict:
    """What the kernel interpreter loads, asked of the interpreter itself (isolated mode)."""
    out = subprocess.run([python, "-I", "-c", _PROBE], capture_output=True, text=True,
                         timeout=60, check=False, env=TOOL_ENV)
    if out.returncode != 0:
        raise RuntimeError(f"cannot probe {python}: {out.stderr.strip()[-500:]}")
    return json.loads(out.stdout)


def read_only_paths(python: str) -> tuple[list[str], list[str]]:
    """(paths to bind read-only, sys.path entries refused)."""
    info = interpreter(python)
    prefix = os.path.realpath(info["prefix"])
    stdlib = {os.path.realpath(info["stdlib"]), os.path.realpath(info["platstdlib"])}
    allowed = {prefix, *stdlib}
    binds, refused = set(allowed), []
    for p in info["path"]:
        if not os.path.exists(p):
            continue
        rp = os.path.realpath(p)
        if under(rp, allowed):
            binds.add(rp)
        else:
            refused.append(p)
    for f in (python, info["executable"], info["libpython"]):
        if f and os.path.exists(f):
            binds.add(os.path.realpath(f))
    return sorted(binds), refused


def fixed_binds(python: str) -> list[str]:
    """The host paths argv binds into every sandbox on this python, whatever the episode."""
    sys_paths = [p for p in [*SYSTEM_LIBS, *SYSTEM_FILES] if os.path.lexists(p)]
    return sorted({*sys_paths, *read_only_paths(python)[0]})


def exposing(path: str, python: str, dirs: tuple[str, ...] | list[str] = ()) -> str | None:
    """The bind (fixed_binds, or one of dirs: an episode, slot or state root) that would put path
    inside a sandbox, or one inside path; None when neither holds."""
    p = os.path.realpath(path)
    for b in [*fixed_binds(python), *dirs]:
        rb = os.path.realpath(b)
        if under(p, [rb]) or under(rb, [p]):
            return b
    return None


def _symlink_ops(paths, bound) -> list[str]:
    ops, seen = [], set()
    for p in paths:
        for s in sorted(symlinks(p)):
            if s in seen or under(os.path.dirname(s), bound):
                continue
            seen.add(s)
            ops += ["--symlink", os.readlink(s), s]
    return ops


def _try(bwrap: str, python: str, ids: tuple[int, int] | None, proc: bool,
         flags: frozenset[str] = frozenset()) -> str | None:
    """None when a trivial command runs in this configuration, else bwrap's complaint."""
    with tempfile.TemporaryDirectory(prefix="amp-probe-") as d:
        ep, sdk, stage = (os.path.join(d, x) for x in ("ep", "sdk", "stage"))
        for x in (ep, sdk, stage):
            os.mkdir(x)
        drop = None
        if ids is not None:
            os.chmod(d, 0o711)  # nosec B103  # nosemgrep: python.lang.security.audit.insecure-file-permissions.insecure-file-permissions
            os.chown(ep, *ids)
            drop = {"uid": ids[0], "gid": ids[1], "stage": stage}
        a = argv([python, "-I", "-c", "pass"], bwrap=bwrap, episode_dir=ep, sdk_dir=sdk,
                 python=python, relay_sock=None, masks={}, proc=proc, drop=drop, flags=flags)
        r = subprocess.run(a, capture_output=True, text=True, timeout=60, check=False,
                           env=TOOL_ENV)
        return None if r.returncode == 0 else (r.stderr.strip() or f"exit {r.returncode}")[-300:]


@functools.lru_cache(maxsize=8)
def launch_mode(bwrap: str, python: str, ids: tuple[int, int] | None
                ) -> tuple[bool, bool, str, frozenset[str]]:
    """(run as the unprivileged ids, mount a fresh /proc, why not ids, optional flags that work)."""
    have = features(bwrap)
    tiers = list(dict.fromkeys([have, have - {NO_USERNS}, frozenset()]))
    why = ""
    for flags in tiers:
        if ids is not None:
            for proc in (True, False):
                err = _try(bwrap, python, ids, proc, flags)
                if err is None:
                    return True, proc, "", flags
                why = why or err
    if ids is not None:
        return False, False, why, frozenset()
    for flags in tiers:
        for proc in (True, False):
            if _try(bwrap, python, None, proc, flags) is None:
                return False, proc, "", flags
    return False, False, why, frozenset()


def argv(cmd: list[str], *, bwrap: str, episode_dir: str, sdk_dir: str, python: str,
         relay_sock: str | None, masks: dict[str, str], proc: bool = True,
         view: tuple[str, list[str]] | None = None, drop: dict | None = None,
         site: str | None = None, flags: frozenset[str] = frozenset(),
         deadline: bool = False) -> list[str]:
    """The bwrap command line for cmd; masks maps an in-sandbox path to what is bound over it."""
    ro, _ = read_only_paths(python)
    sys_paths = [p for p in [*SYSTEM_LIBS, *SYSTEM_FILES] if os.path.lexists(p)]
    real_dirs = [p for p in sys_paths if not os.path.islink(p)] + ro
    a = [bwrap, "--unshare-all", "--unshare-user", "--die-with-parent", "--cap-drop", "ALL",
         "--hostname", "sandbox", "--dev", "/dev"]
    if NO_USERNS in flags:
        a += [NO_USERNS]
    size = [SIZE_FLAG, str(TMPFS_BYTES)] if SIZE_FLAG in flags else []
    a += [*size, "--tmpfs", "/tmp"]  # nosec B108
    if size:
        a += [*size, "--tmpfs", "/dev/shm"]  # nosec B108
    if drop is not None:
        a += ["--uid", str(INNER_ID), "--gid", str(INNER_ID)]
    if proc:
        a += ["--proc", "/proc"]
    for p in sys_paths:
        if not os.path.islink(p):
            a += ["--ro-bind", p, p]
    for p in ro:
        a += ["--ro-bind", p, p]
    a += _symlink_ops([*sys_paths, python, episode_dir], real_dirs)
    ep, sdk = os.path.realpath(episode_dir), os.path.realpath(sdk_dir)
    if view is None:
        a += ["--bind", ep, ep]
    else:
        a += ["--bind", view[0], ep]
        for name in view[1]:
            a += ["--bind", os.path.join(ep, name), os.path.join(ep, name)]
    a += ["--ro-bind", sdk, sdk]
    for inner, src in sorted(masks.items()):
        a += ["--ro-bind", src, inner]
    if relay_sock is not None:
        a += ["--bind", relay_sock, INNER_SOCK]
    if site is not None:
        a += ["--ro-bind", site, INNER_SITE]
    a += ["--chdir", ep, "--"]
    if relay_sock is not None:
        a += [python, "-I", "-c", SHIM_DEADLINE_SOURCE if deadline else SHIM_SOURCE,
              INNER_SOCK, str(INNER_PORT), "--"]
    else:
        a += [python, "-I", "-c", _EXEC_DFL_DEADLINE if deadline else _EXEC_DFL]
    if drop is not None:
        return [python, "-I", "-c", LAUNCH_DEADLINE if deadline else LAUNCH, json.dumps(drop),
                *a, *cmd]
    return [python, "-I", "-c", _EXEC_IGN_DEADLINE if deadline else _EXEC_IGN, *a, *cmd]
