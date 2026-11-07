#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Perception overlays: one change to what a harness's agent perceives, applied to any harness."""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

from amp.contracts.harness import OBSTACLE_MODES, RoutePolicy

ROOT = Path(__file__).resolve().parent / "_overlays"
PERCEPTION_FILE = "perception.json"
REFERENCE_FILE = "perception.md"
FIELDS = {"tag", "description", "server_env", "server", "deny_routes", "response_drop",
          "bootstrap"}
SERVER_FIELDS = {"obstacles": OBSTACLE_MODES}
GENERATED_MARK = ".generated"
GENERATOR_VERSION = "1"


def names(root: Path = ROOT) -> list[str]:
    return sorted(p.parent.name for p in root.glob("*/overlay.json"))


def load(name: str, root: Path = ROOT) -> dict:
    """An overlay's overlay.json plus its reference paragraph; KeyError for an unknown name."""
    d = root / name
    if not (d / "overlay.json").is_file():
        raise KeyError(f"no overlay {name!r}; have {names(root)}")
    ov = json.loads((d / "overlay.json").read_text())
    unknown = set(ov) - FIELDS
    if unknown or not ov.get("tag"):
        raise ValueError(f"overlay {name!r}: needs a tag; unknown fields {sorted(unknown)}")
    for k, v in (ov.get("server") or {}).items():
        if v not in SERVER_FIELDS.get(k, ()):
            raise ValueError(f"overlay {name!r}: server may set only {sorted(SERVER_FIELDS)} "
                             f"({k}={v!r}; obstacles one of {list(OBSTACLE_MODES)})")
    ov["reference"] = (d / "reference.md").read_text().strip()
    return ov


def fingerprint(overlays, root: Path = ROOT) -> str:
    """8 hex digits of the generator and the overlays' files, in order: part of an overlaid
    harness's version."""
    h = hashlib.sha256(f"generator/{GENERATOR_VERSION}\0".encode())
    for name in overlays:
        for f in ("overlay.json", "reference.md"):
            h.update(f"{name}/{f}\0".encode())
            h.update((Path(root) / name / f).read_bytes())
    return h.hexdigest()[:8]


def _matches(rule: dict, method: str, path: str, name: str) -> bool:
    one = RoutePolicy.from_dict({"format": "amp-route-policy-1", "name": name, "rules": [rule],
                                 "executes": []})
    return one.key(method, path) is not None


def apply(harness_dir: Path, base: str, overlays, root: Path = ROOT) -> None:
    """Apply overlays, in order, to a fresh copy of the base harness and record them."""
    harness_dir = Path(harness_dir)
    manifest_p, routes_p = harness_dir / "manifest.json", harness_dir / "routes.json"
    manifest, routes = json.loads(manifest_p.read_text()), json.loads(routes_p.read_text())
    env = dict(manifest["server"].get("env") or {})
    drop = {k: list(v) for k, v in (routes.get("response_drop") or {}).items()}
    paragraphs, server = [], {}
    for name in overlays:
        ov = load(name, root)
        for k, v in (ov.get("server_env") or {}).items():
            if k in env and env[k] != v:
                raise ValueError(f"overlay {name!r} sets server env {k}={v!r}, already {env[k]!r}")
            env[k] = v
        for k, v in (ov.get("server") or {}).items():
            if k in server and server[k] != v:
                raise ValueError(f"overlay {name!r} sets server {k}={v!r}, already {server[k]!r}")
            server[k] = manifest["server"][k] = v
        for method, path in ov.get("deny_routes") or []:
            routes["rules"] = [r for r in routes["rules"]
                               if not _matches(r, method, path, routes["name"])]
        for key, fields in (ov.get("response_drop") or {}).items():
            drop[key] = drop.get(key, []) + [f for f in fields if f not in drop.get(key, [])]
        if ov.get("bootstrap"):
            manifest["bootstrap"] = manifest["bootstrap"].rstrip("\n") + "\n" + \
                ov["bootstrap"].rstrip("\n") + "\n"
        paragraphs.append(ov["reference"])
    base_version = str(manifest["version"])
    manifest["version"] = f"{base_version}.{fingerprint(overlays, root)}"
    if env:
        manifest["server"]["env"] = env
    if drop:
        routes["response_drop"] = drop
    pol = RoutePolicy.from_dict(routes)
    routes["executes"] = [k for k in routes.get("executes", [])
                          if pol.key("POST", k) is not None or pol.key("GET", k) is not None]
    (harness_dir / REFERENCE_FILE).write_text("## Perception\n\n" + "\n\n".join(paragraphs) + "\n")
    manifest["reference_docs"] = [REFERENCE_FILE, *[d for d in manifest.get("reference_docs", [])
                                                    if d != REFERENCE_FILE]]
    manifest_p.write_text(json.dumps(manifest, indent=2) + "\n")
    routes_p.write_text(json.dumps(routes, indent=2) + "\n")
    (harness_dir / PERCEPTION_FILE).write_text(
        json.dumps({"base": base, "base_version": base_version, "overlays": list(overlays)},
                   indent=2) + "\n")


def perception(harness_dir: Path) -> dict | None:
    """A harness's perception.json ({"base", "base_version", "overlays"}), or None for one built
    without overlays."""
    p = Path(harness_dir) / PERCEPTION_FILE
    return json.loads(p.read_text()) if p.is_file() else None


def resolve(name: str, harness_root: Path, root: Path = ROOT) -> tuple[str, list[str]] | None:
    """(base, overlays) if name is <base>_<tag>... for a committed base harness and known overlay
    tags (which may hold "_"), else None."""
    harness_root = Path(harness_root)
    tags = {load(o, root)["tag"]: o for o in names(root)}
    bases = sorted((p.parent.name for p in harness_root.glob("*/manifest.json")
                    if not (p.parent / GENERATED_MARK).exists() and not p.parent.name.startswith("_")),
                   key=len, reverse=True)
    for base in bases:
        if name.startswith(base + "_"):
            found = _split_tags(name[len(base) + 1:], tags)
            if found:
                return base, found
    return None


def _split_tags(rest: str, tags: dict[str, str]) -> list[str] | None:
    """The overlays whose tags, joined by "_", spell rest (a tag may hold "_"; the longest tag
    that fits is tried first), or None."""
    if not rest:
        return []
    for tag in sorted(tags, key=len, reverse=True):
        if rest == tag or rest.startswith(tag + "_"):
            tail = _split_tags(rest[len(tag) + 1:], tags)
            if tail is not None:
                return [tags[tag], *tail]
    return None


def materialize(name: str, harness_root: Path, root: Path = ROOT) -> Path:
    """<harness_root>/<name>, generated from its base and overlays if it is not there yet, or was
    generated from other overlay or base files (its version says which); KeyError if name is
    neither a harness nor an overlay harness."""
    from amp.harnesses import registry

    harness_root = Path(harness_root)
    dst = harness_root / name
    generated = (dst / GENERATED_MARK).is_file()
    if (dst / "manifest.json").is_file() and not generated:
        return dst
    found = resolve(name, harness_root, root)
    if found is None:
        raise KeyError(f"no harness {name!r} under {harness_root}, and it is not "
                       f"<base>_<overlay tag>...; overlays: {names(root)}")
    base, ovs = found
    want = (f"{json.loads((harness_root / base / 'manifest.json').read_text())['version']}."
            f"{fingerprint(ovs, root)}")
    if generated and (dst / "manifest.json").is_file():
        if json.loads((dst / "manifest.json").read_text()).get("version") == want:
            return dst
        stale = harness_root / f".{name}.stale.{os.getpid()}"
        with contextlib.suppress(OSError):
            os.rename(dst, stale)
            shutil.rmtree(stale, ignore_errors=True)
    tmp = Path(tempfile.mkdtemp(prefix=f".{name}.", dir=harness_root))
    try:
        work = tmp / name
        shutil.copytree(harness_root / base, work, ignore=registry.COPY_IGNORE)
        for pattern in registry.RENAMED_FILES:
            for f in work.glob(pattern):
                f.write_text(registry._rename(f.read_text(), base, name))
        apply(work, base, ovs, root)
        (work / GENERATED_MARK).write_text("generated by amp.harnesses.overlays.materialize\n")
        (work / ".gitignore").write_text("*\n")
        if dst.is_dir() and {p.name for p in dst.iterdir()} <= {"__pycache__"}:
            shutil.rmtree(dst, ignore_errors=True)
        try:
            os.rename(work, dst)
        except OSError:
            if not (dst / "manifest.json").is_file():
                raise
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return dst


def main(argv=None) -> int:
    import argparse

    ap = argparse.ArgumentParser(prog="python -m amp.harnesses.overlays")
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("materialize", help="generate an overlay harness if it is not there yet")
    m.add_argument("name")
    m.add_argument("--root", default=str(Path(__file__).resolve().parent),
                   help="the harnesses directory (default: this checkout's)")
    a = ap.parse_args(argv)
    try:
        print(materialize(a.name, Path(a.root)))
    except (KeyError, ValueError, OSError) as e:
        print(f"overlays: {e.args[0] if isinstance(e, KeyError) and e.args else e}",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
