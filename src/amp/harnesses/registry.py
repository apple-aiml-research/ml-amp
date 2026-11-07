#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Harness discovery and loading: one dir per harness under amp/harnesses/, described by its
manifest.json (a contracts.harness.HarnessSpec, paths relative to that dir)."""

from __future__ import annotations

import dataclasses
import json
import re
import shutil
from pathlib import Path

from amp.contracts.harness import HarnessSpec

MANIFEST = "manifest.json"
ROOT = Path(__file__).resolve().parent
RESERVED = ("_sdk", "_overlays")
NAME = re.compile(r"[a-z][a-z0-9_]*")
RENAMED_FILES = ("manifest.json", "routes.json", "*.py")
COPY_IGNORE = shutil.ignore_patterns("__pycache__", "*.pyc")


def check_name(name: str) -> str:
    if not isinstance(name, str) or not NAME.fullmatch(name):
        raise KeyError(f"bad harness name {name!r}: must match {NAME.pattern}")
    return name


class Registry:
    """The harnesses under one root dir; the module functions use the package's own."""

    def __init__(self, root: str | Path = ROOT):
        self.root = Path(root).resolve()
        self._dirs: dict[int, tuple[HarnessSpec, Path]] = {}

    def names(self) -> list[str]:
        """The registered harnesses: committed dirs, not generated overlay harnesses."""
        return sorted(p.parent.name for p in self.root.glob(f"*/{MANIFEST}")
                      if p.parent.name not in RESERVED and NAME.fullmatch(p.parent.name)
                      and not (p.parent / ".generated").exists())

    def harness_dir(self, name: str) -> Path:
        """name's dir; an overlay harness (amp.harnesses.overlays) is generated on first use, and
        again when its overlays or base changed since."""
        d = self.root / check_name(name)
        if not (d / MANIFEST).is_file() or (d / ".generated").is_file():
            from amp.harnesses import overlays

            try:
                own = self.root / overlays.ROOT.name
                return overlays.materialize(name, self.root, own if own.is_dir() else overlays.ROOT)
            except KeyError:
                raise KeyError(f"no harness {name!r} under {self.root}; have {self.names()}, or "
                               f"<harness>_<overlay tag>... with overlays {overlays.names()}") from None
        return d

    def load(self, name: str) -> HarnessSpec:
        d = self.harness_dir(name)
        spec = spec_from_manifest(d / MANIFEST)
        if spec.name != name:
            raise ValueError(f"{d / MANIFEST}: name {spec.name!r} != directory {name!r}")
        self._dirs[id(spec)] = (spec, d)
        return spec

    def dir_of(self, spec: HarnessSpec) -> Path:
        hit = self._dirs.get(id(spec))
        return hit[1] if hit and hit[0] is spec else self.harness_dir(spec.name)

    def path(self, spec: HarnessSpec, rel: str) -> Path:
        """A manifest path, resolved against the harness dir; it may not leave this root."""
        p = (self.dir_of(spec) / rel).resolve()
        if not rel or Path(rel).is_absolute() or not p.is_relative_to(self.root):
            raise ValueError(f"{spec.name}: path {rel!r} must be relative and stay under "
                             f"{self.root}")
        return p

    def reference_text(self, spec: HarnessSpec) -> str:
        """The reference docs joined as the original agent appended them (one blank line)."""
        text = ""
        for i, rel in enumerate(spec.reference_docs):
            doc = self.path(spec, rel).read_text()
            text = doc if i == 0 else text.rstrip("\n") + "\n\n" + doc
        return text

    def sdk_source(self, spec: HarnessSpec) -> Path:
        """The SDK dir; the agent copies it to <episode>/sdk/<basename> minus sdk_exclude."""
        p = self.path(spec, spec.sdk_package)
        if not p.is_dir():
            raise FileNotFoundError(f"{spec.name}: sdk_package {spec.sdk_package!r} is not a dir")
        return p


def _rename(text: str, old: str, new: str) -> str:
    return re.sub(rf"\b{re.escape(old)}\b", new, text)


def spec_from_manifest(path: str | Path) -> HarnessSpec:
    """Parse one manifest.json; unknown fields are an error, not ignored."""
    path = Path(path)
    d = json.loads(path.read_text())
    if not isinstance(d, dict):
        raise ValueError(f"{path}: a manifest is a JSON object")  # noqa: TRY004
    known = {f.name for f in dataclasses.fields(HarnessSpec)}
    unknown = sorted(set(d) - known)
    if unknown:
        raise ValueError(f"{path}: unknown fields {unknown}")
    try:
        return HarnessSpec.from_dict(d)
    except TypeError as e:
        raise ValueError(f"{path}: {e}") from None


DEFAULT = Registry()


def names() -> list[str]:
    return DEFAULT.names()


def harness_dir(name: str) -> Path:
    return DEFAULT.harness_dir(name)


def load(name: str) -> HarnessSpec:
    return DEFAULT.load(name)


def reference_text(spec: HarnessSpec) -> str:
    return DEFAULT.reference_text(spec)


def sdk_source(spec: HarnessSpec) -> Path:
    return DEFAULT.sdk_source(spec)


def perception(name: str) -> dict | None:
    """A harness's perception stack, {"base", "overlays"}, or None if it was built without one."""
    from amp.harnesses import overlays

    return overlays.perception(DEFAULT.harness_dir(name))
