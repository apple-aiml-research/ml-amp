#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Agent-owned workspace memory: a task's folder of files, written by the agent in practice
debriefs, carried in AgentProfile.extra and unpacked as ./workspace (docs/practice.md)."""

from __future__ import annotations

import base64
import dataclasses
import hashlib
import json
import os
from pathlib import Path, PurePosixPath

from amp.contracts.model import AgentProfile

WORKSPACE_KEY = "workspace"
DIR = "workspace"
MAX_BYTES = 60_000
POINTER = ("Your workspace for this task is ./workspace, written by you in earlier practice "
           "episodes; read what you need and check it against the scene.")
NOTEBOOK_KEY = "notebook"
NOTEBOOK_VERSIONS = (1, 2)
POINTER_V2 = (POINTER + " Changes you make to ./workspace during this episode are not kept: only "
              "the debrief after practice updates the workspace.")


class WorkspaceError(ValueError):
    """A workspace that cannot be packed, delivered or trusted (bad path, sha, or link)."""


def check_path(rel: str) -> str:
    """rel as a safe relative POSIX path inside the workspace, or WorkspaceError."""
    p = PurePosixPath(rel)
    if not rel or rel == "." or "\x00" in rel or p.is_absolute() or "\\" in rel \
            or any(x in ("", ".", "..") for x in p.parts) or str(p) != rel:
        raise WorkspaceError(f"workspace path {rel!r} is not a plain relative path")
    return rel


def sha_of(files: dict[str, bytes]) -> str:
    """The content address of a set of files: sha256 over each sorted path and its bytes' sha256."""
    h = hashlib.sha256()
    for rel in sorted(files):
        h.update(f"{rel}\0{hashlib.sha256(files[rel]).hexdigest()}\n".encode())
    return h.hexdigest()


def encode(files: dict[str, bytes]) -> dict:
    """The extra["workspace"] value for these files."""
    out: dict[str, object] = {}
    for rel in sorted(files):
        data = files[check_path(rel)]
        try:
            out[rel] = data.decode("utf-8")
        except UnicodeDecodeError:
            out[rel] = {"b64": base64.b64encode(data).decode()}
    return {"sha": sha_of(files), "files": out}


def decode(ws: dict) -> dict[str, bytes]:
    """{path: bytes} of an extra["workspace"] value; WorkspaceError if its sha does not match."""
    if not isinstance(ws, dict) or not isinstance(ws.get("files"), dict):
        raise WorkspaceError("a workspace is {sha, files: {path: text | {b64}}}")
    files: dict[str, bytes] = {}
    for rel, v in ws["files"].items():
        check_path(rel)
        if isinstance(v, str):
            files[rel] = v.encode("utf-8")
        elif isinstance(v, dict) and isinstance(v.get("b64"), str):
            try:
                files[rel] = base64.b64decode(v["b64"], validate=True)
            except ValueError:
                raise WorkspaceError(f"workspace file {rel!r} has invalid base64") from None
        else:
            raise WorkspaceError(f"workspace file {rel!r} is neither text nor {{b64}}")
    if ws.get("sha") != sha_of(files):
        raise WorkspaceError(f"workspace sha {str(ws.get('sha'))[:12]} does not match its files")
    if NOTEBOOK_KEY in ws and ws[NOTEBOOK_KEY] not in NOTEBOOK_VERSIONS[1:]:
        raise WorkspaceError(f"workspace notebook {ws[NOTEBOOK_KEY]!r} is not one of "
                             f"{NOTEBOOK_VERSIONS[1:]} (leave it out for 1)")
    return files


def notebook_of(ws: dict | None) -> int:
    """The notebook version a workspace or its unpack record carries (1 without the key)."""
    v = ws.get(NOTEBOOK_KEY) if isinstance(ws, dict) else None
    return v if v in NOTEBOOK_VERSIONS else 1


def with_notebook(ws: dict, version: int) -> dict:
    """ws marked with notebook version (1: unmarked, as every payload before notebook v2)."""
    if version not in NOTEBOOK_VERSIONS:
        raise WorkspaceError(f"notebook version {version!r} is not one of {NOTEBOOK_VERSIONS}")
    out = {k: v for k, v in ws.items() if k != NOTEBOOK_KEY}
    return out if version == 1 else {**out, NOTEBOOK_KEY: version}


def read_dir(root: Path) -> dict[str, bytes]:
    """Every regular file under root as {relative path: bytes}; WorkspaceError on a link."""
    root = Path(root)
    files: dict[str, bytes] = {}
    for dirpath, dirnames, filenames in os.walk(root):
        d = Path(dirpath)
        for name in dirnames + filenames:
            if (d / name).is_symlink():
                raise WorkspaceError(f"{d / name} is a link: a workspace holds regular files only")
        dirnames.sort()
        for name in sorted(filenames):
            p = d / name
            if not p.is_file():
                raise WorkspaceError(f"{p} is not a regular file")
            files[check_path(p.relative_to(root).as_posix())] = p.read_bytes()
    return files


def write_dir(files: dict[str, bytes], dest: Path) -> None:
    """Write files under dest (created); refuses a path that leaves it."""
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    for rel, data in sorted(files.items()):
        p = dest / check_path(rel)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)


def size(ws: dict) -> int:
    """Bytes of the workspace as the profile carries it (compact JSON)."""
    return len(json.dumps(ws, separators=(",", ":")))


def describe(ws: dict) -> dict:
    """What the trace records: the sha, the file count and the files' total bytes."""
    files = decode(ws)
    return {"sha": ws["sha"], "files": len(files), "bytes": sum(len(b) for b in files.values())}


def of(profile: AgentProfile) -> dict | None:
    """The profile's workspace, or None without one."""
    ws = profile.extra.get(WORKSPACE_KEY)
    return ws if isinstance(ws, dict) else None


def with_workspace(profile: AgentProfile, ws: dict) -> AgentProfile:
    """profile carrying the workspace ws (decode() checks it first)."""
    decode(ws)
    return dataclasses.replace(profile, extra={**profile.extra, WORKSPACE_KEY: ws})


WORKSPACES_KEY = "workspaces"


def read_manifest(path: Path) -> dict[str, tuple[Path | None, str | None]]:
    """{task: (folder, sha)} of a notebooks manifest, a JSON list of {task, folder, sha}."""
    path = Path(path)
    try:
        entries = json.loads(path.read_text())
    except (OSError, ValueError) as e:
        raise WorkspaceError(f"{path}: {e}") from None
    if not isinstance(entries, list) or not entries:
        raise WorkspaceError(f"{path}: expected a non-empty list of {{task, folder, sha}}")
    root = path.parent.resolve()
    out: dict[str, tuple[Path | None, str | None]] = {}
    for e in entries:
        if not isinstance(e, dict) or not isinstance(e.get("task"), str) or "folder" not in e:
            raise WorkspaceError(f"{path}: entry {e!r} is not {{task, folder, sha}}")
        task, folder, sha = e["task"], e["folder"], e.get("sha")
        if task in out:
            raise WorkspaceError(f"{path}: task {task!r} is listed twice")
        if folder is not None and not isinstance(folder, str):
            raise WorkspaceError(f"{path}: {task}: folder must be a string or null")
        if sha is not None and not isinstance(sha, str):
            raise WorkspaceError(f"{path}: {task}: sha must be a string or null")
        if folder is None:
            out[task] = (None, sha)
            continue
        if not sha:
            raise WorkspaceError(f"{path}: {task}: folder {folder!r} has no sha to check it "
                                 "against")
        try:
            check_path(folder)
        except WorkspaceError:
            raise WorkspaceError(f"{path}: {task}: folder {folder!r} is not a plain relative path "
                                 "under the manifest's directory") from None
        resolved = (path.parent / folder).resolve()
        if resolved == root or not resolved.is_relative_to(root):
            raise WorkspaceError(f"{path}: {task}: folder {folder!r} leaves the manifest's "
                                 "directory")
        out[task] = (path.parent / folder, sha)
    return out


def with_table(profile: AgentProfile, by_task: dict[str, dict],
               configs: dict[str, str | None]) -> AgentProfile:
    """profile carrying a workspaces table: each config's task names its workspace in by_task."""
    for ws in by_task.values():
        decode(ws)
    missing = sorted({t for t in configs.values() if t is not None} - by_task.keys())
    if missing:
        raise WorkspaceError(f"no workspace for task(s) {', '.join(missing)}")
    if WORKSPACE_KEY in profile.extra:
        raise WorkspaceError("a profile carries one workspace or a workspaces table, not both")
    table = {"by_task": by_task, "configs": dict(configs)}
    return dataclasses.replace(profile, extra={**profile.extra, WORKSPACES_KEY: table})


def _table_parts(table: object) -> tuple[dict, dict]:
    """(configs, by_task) of a workspaces table; WorkspaceError if it is not {by_task, configs}."""
    configs = table.get("configs") if isinstance(table, dict) else None
    by_task = table.get("by_task") if isinstance(table, dict) else None
    if not isinstance(configs, dict) or not isinstance(by_task, dict):
        raise WorkspaceError("the workspaces table is {by_task: {task: workspace}, "
                             "configs: {config: task or null}}")
    return configs, by_task


def for_task(profile: AgentProfile, config: str) -> AgentProfile:
    """profile with its workspaces table resolved to this config's workspace, or to none."""
    table = profile.extra.get(WORKSPACES_KEY)
    if table is None:
        return profile
    configs, by_task = _table_parts(table)
    if config not in configs:
        raise WorkspaceError(f"{config!r} is not a task of the workspaces table")
    rest = dataclasses.replace(profile, extra={k: v for k, v in profile.extra.items()
                                               if k != WORKSPACES_KEY})
    task = configs[config]
    if task is None:
        return rest
    if not isinstance(task, str) or task not in by_task:
        raise WorkspaceError(f"the workspaces table has no workspace for {task!r}")
    return with_workspace(rest, by_task[task])


def unpack(ws: dict, ep_dir: Path, kernel_cwd: Path | None = None) -> dict:
    """Write the workspace to <ep_dir>/workspace, and to <kernel_cwd>/workspace when the kernel's
    working directory is not the episode dir (bwrap); describe(ws) plus where it went."""
    files = decode(ws)
    targets = [Path(ep_dir) / DIR]
    if kernel_cwd is not None and Path(kernel_cwd).resolve() != Path(ep_dir).resolve():
        targets.append(Path(kernel_cwd) / DIR)
    for t in targets:
        write_dir(files, t)
    rec = {**describe(ws), "dirs": [os.path.relpath(t, ep_dir) for t in targets]}
    v = notebook_of(ws)
    return rec if v == 1 else {**rec, NOTEBOOK_KEY: v}


def first_user_block(rec: dict | None = None) -> dict:
    """The text block appended to the episode's first user message (rec: unpack's record)."""
    return {"type": "text", "text": POINTER_V2 if notebook_of(rec) == 2 else POINTER}
