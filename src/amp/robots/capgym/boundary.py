#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The runtime CaP-Gym boundary: what a capgym process loaded, read and spawned, per unit."""

from __future__ import annotations

import hashlib
import importlib.util
import itertools
import json
import os
import re
import sys
import time
from pathlib import Path

from amp.robots.capgym.boundary_sets import FAMILY_SETS, FamilySet

ROOT_ENV = "AMP_CAPX_ROOT"
MAX_VIOLATIONS = 20
SCHEMA = 1
_SPAWN_EVENTS = frozenset({"subprocess.Popen", "os.system", "os.exec", "os.posix_spawn",
                           "os.spawn"})
_MAX_CACHED_DIRS = 20000
MAX_SPAWNS = 64


_MODULE_FILE = re.compile(
    r"(?P<dir>capx(?:/[A-Za-z_]\w*)*)/(?:(?P<src>[A-Za-z_]\w*)\.py"
    r"|__pycache__/(?P<pyc>[A-Za-z_]\w*)\.[\w.-]+\.pyc(?:\.\d+)?)")

_root: str | None = None
_prefixes: tuple[str, ...] = ()
_real_prefix = ""
_imports: set[str] = set()
_opens: set[str] = set()
_spawn_names: set[str] = set()
_spawn_bad: list[str] = []
_spawn_n = 0
_dir_real: dict[str, str] = {}
_clear: set[str] = set()
_rel: dict[str, str | None] = {}
_installed = False
_family: str | None = None
_configs: frozenset[str] | None = None

_seen_modules: set[str] = set()
_seen_files: set[str] = set()
_seen_spawns = 0
_seen_spawn_n = 0
_tainted = False


def find_root() -> str | None:
    """The cap-x checkout, as official.capx_root finds it, without importing capx; None if none."""
    if os.environ.get(ROOT_ENV):
        return os.path.abspath(os.environ[ROOT_ENV])
    try:
        spec = importlib.util.find_spec("capx")
    except (ImportError, ValueError):
        return None
    if spec is None or not spec.origin:
        return None
    return str(Path(spec.origin).parents[1])


def _strings(args) -> list[str]:
    out: list[str] = []
    for a in args:
        if isinstance(a, bytes):
            a = a.decode(errors="replace")
        if isinstance(a, str):
            out.append(a)
        elif isinstance(a, (list, tuple)):
            out += [x.decode(errors="replace") if isinstance(x, bytes) else x
                    for x in a if isinstance(x, (str, bytes))]
    return out


def _spawn(event: str, args: tuple) -> None:
    global _spawn_n

    cwd = args[2] if event == "subprocess.Popen" and len(args) > 2 else None
    if event == "subprocess.Popen":
        args = args[:2]
    parts = _strings([a for a in args if not isinstance(a, dict)])
    if not parts:
        return
    if len(_spawn_names) < 32:
        _spawn_names.add(os.path.basename(parts[0].split(" ", 1)[0]))
    if os.path.basename(parts[0]) == "git" and "rev-parse" in parts:
        return
    module = any(a == "-m" and (b == "capx" or b.startswith("capx."))
                 for a, b in itertools.pairwise(parts))
    if isinstance(cwd, str):
        parts += [os.path.join(cwd, p) for p in parts[:1] + parts[1:2] if "/" in p and p[:1] != "/"]
    if module or any((p + "/").startswith(_prefixes) for p in parts):
        _spawn_n += 1
        if len(_spawn_bad) < MAX_SPAWNS:
            _spawn_bad.append(" ".join(parts)[:200])


def _outside(p: str) -> bool:
    """Whether the absolute p, once its symlinks resolve, still lies outside the root."""
    d, name = os.path.split(p)
    real = _dir_real.get(d)
    if real is None:
        real = os.path.realpath(d)
        if len(_dir_real) < _MAX_CACHED_DIRS:
            _dir_real[d] = real
    r = os.path.join(real, name)
    if os.path.islink(r):
        r = os.path.realpath(r)
    return not r.startswith(_prefixes)


def _hook(event: str, args: tuple) -> None:

    try:
        if event == "open":
            p = args[0]
            if p.__class__ is not str:
                if isinstance(p, bytes):
                    p = p.decode(errors="replace")
                elif isinstance(p, os.PathLike):
                    p = os.fspath(p)
                    if p.__class__ is not str:
                        return
                else:
                    return
            if p.startswith(_prefixes):
                _opens.add(p)
            elif _prefixes and p not in _clear:
                if p[:1] != "/":
                    p = os.path.join(os.getcwd(), p)
                if not _outside(p):
                    _opens.add(p)
                elif len(_clear) < _MAX_CACHED_DIRS and p[:1] == "/":
                    _clear.add(p)
        elif event == "import":
            n = args[0]
            if n.startswith("capx") and (len(n) == 4 or n[4] == "."):
                _imports.add(n)
        elif event in _SPAWN_EVENTS:
            _spawn(event, args)
    except Exception:  # noqa: BLE001, S110
        pass



def install(root: str | None = None) -> bool:
    """Start recording capx imports, opens under the CaP-X root and spawns, once per process."""
    global _installed, _root, _prefixes, _real_prefix
    if _installed:
        return False
    _root = root if root is not None else find_root()
    if _root:
        real = os.path.realpath(_root)
        _prefixes = tuple(dict.fromkeys(os.path.join(r, "") for r in (_root, real)))
        _real_prefix = os.path.join(real, "")
    _installed = True
    sys.addaudithook(_hook)
    return True


def bind(family: str | None) -> None:
    """Name the family this process serves; a no-op where no hook is installed (tests)."""
    global _family
    if _installed:
        _family = family


def module_of(rel: str) -> str | None:
    """The capx module a .py or __pycache__ .pyc path holds; None for any other file."""
    m = None if rel.startswith("capx/third_party/") else _MODULE_FILE.fullmatch(rel)
    if m is None:
        return None
    leaf = m["src"] or m["pyc"]
    pkg = m["dir"].replace("/", ".")
    return pkg if leaf == "__init__" else f"{pkg}.{leaf}"


def env_configs(family: str) -> frozenset[str]:
    """The env config YAMLs the capgym manifest names, under the family's config dir."""
    global _configs
    spec = FAMILY_SETS.get(family)
    if spec is None or spec.env_config_dir is None:
        return frozenset()
    if _configs is None:
        from amp.benchmarks.capgym import manifest_path

        found: set[str] = set()

        def walk(x) -> None:
            if isinstance(x, dict):
                for k, v in x.items():
                    if k in ("source", "base_config") and isinstance(v, str) and \
                            v.startswith("env_configs/") and v.endswith(".yaml"):
                        found.add(v)
                    walk(v)
            elif isinstance(x, list):
                for v in x:
                    walk(v)

        try:
            walk(json.loads(manifest_path().read_text()))
        except (OSError, ValueError):
            pass
        _configs = frozenset(found)
    return frozenset(c for c in _configs if c.startswith(spec.env_config_dir))


def _other_family(name: str, family: str) -> str:
    owners = sorted(k for k, v in FAMILY_SETS.items() if k != family and name in v.allowed())
    return f" (in {', '.join(owners)}'s set, not {family}'s)" if owners else ""


def file_violation(rel: str, spec: FamilySet, configs: frozenset[str]) -> str | None:
    """Why reading `rel` (relative to the CaP-X root) crosses the boundary; None if it may."""
    if rel in spec.files or rel in configs or rel.startswith(tuple(p for p, _ in spec.trees)):
        return None
    mod = module_of(rel)
    if mod is not None:
        return None if mod in spec.allowed() else f"file {rel} (module {mod})"
    return f"file {rel}"


def check(loaded, files, *, family: str, attempted=None, new_modules=None, new_files=None,
          spawns=(), configs: frozenset[str] | None = None) -> dict:
    """The family's verdict on loaded modules, import events and files."""

    loaded = set(loaded)
    seen = loaded | set(loaded if attempted is None else attempted)
    paths = sorted(set(files))
    out = {"ok": False, "family": family, "modules": len(seen), "files": len(paths)}
    spec = FAMILY_SETS.get(family)
    if spec is None:
        bad = [f"no CaP-Gym allow-list for family {family!r}"]
    else:
        configs = env_configs(family) if configs is None else configs
        fresh = seen if new_modules is None else seen & set(new_modules)
        bad = [f"module {m}{_other_family(m, family)}" for m in sorted(fresh - spec.allowed())]
        bad += [f"module {m} loaded, but it is a failed attempt" for m in
                sorted(spec.attempts & loaded)]
        bad += [f"missing module {m}" for m in sorted(spec.modules - loaded)]
        bad += [f"missing import attempt {m}" for m in sorted(spec.attempts - seen)]
        bad += [v for p in (paths if new_files is None else sorted(set(new_files)))
                if (v := file_violation(p, spec, configs))]
        bad += [f"spawn {s}" for s in spawns]
    digest = hashlib.sha256(json.dumps([sorted(seen), paths]).encode()).hexdigest()
    out.update(ok=not bad, violations=bad[:MAX_VIOLATIONS], sha256=digest)
    if len(bad) > MAX_VIOLATIONS:
        out["violations_total"] = len(bad)
    return out


def loaded_modules() -> set[str]:
    """capx modules in sys.modules."""
    return {m for m in list(sys.modules) if m == "capx" or m.startswith("capx.")}


def attempted_modules() -> set[str]:
    """capx modules the hook saw an import event for, failed attempts included."""
    return set(_imports)


def opened_files() -> set[str]:
    """Files opened under the CaP-X root so far, symlinks resolved, relative to the root."""
    out = set()
    for p in list(_opens):
        if p not in _rel:
            rp = os.path.realpath(p)
            _rel[p] = rp[len(_real_prefix):] if rp.startswith(_real_prefix) else None
        if _rel[p] is not None:
            out.add(_rel[p])
    return out


def _unrecorded(why: str) -> dict:
    return {"ok": None, "family": _family, "root": _root or "unknown", "error": why}


def record() -> dict:
    """The unit's boundary record, closing its window; ok is None if the check could not run."""
    global _seen_modules, _seen_files, _seen_spawns, _seen_spawn_n, _tainted
    t0 = time.perf_counter()
    if not _installed:
        return _unrecorded("hook not installed")
    if _family is None:
        return _unrecorded("no family bound")
    if not _prefixes:
        return _unrecorded("root unknown")
    loaded, attempted, files = loaded_modules(), attempted_modules(), opened_files()
    seen = loaded | attempted
    out = check(loaded, files, family=_family, attempted=attempted,
                new_modules=seen - _seen_modules, new_files=files - _seen_files,
                spawns=_spawn_bad[_seen_spawns:])
    unlisted = _spawn_n - _seen_spawn_n - len(_spawn_bad[_seen_spawns:])
    if unlisted:
        out["violations_total"] = out.get("violations_total", len(out["violations"])) + unlisted
        out["ok"] = False
    out = {"ok": out.pop("ok"), "family": _family, "root": _root, **out}
    out["new_modules"], out["new_files"] = len(seen - _seen_modules), len(files - _seen_files)
    if _spawn_names:
        out["spawned"] = sorted(_spawn_names)[:8]
    if _tainted:
        out["process_tainted"] = True
    _seen_modules, _seen_files, _seen_spawns = seen, files, len(_spawn_bad)
    _seen_spawn_n = _spawn_n
    _tainted = _tainted or not out["ok"]
    out["check_ms"] = round((time.perf_counter() - t0) * 1e3, 3)
    return out
