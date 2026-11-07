#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Past practice episodes as plain files for the debrief (docs/practice.md)."""
from __future__ import annotations

import base64
import csv
import json
import re
import shutil
from pathlib import Path

from amp.agents.recording import BLOB_KEY
from amp.practice.replay import TraceView

ROBOT_RESULT = "robot_result.json"
SUCCESS, BUDGET, NOT_DONE = "success", "step budget exhausted", "task not completed"
OUTCOME_FIELDS = ("episode", "layout", "workspace_sha", "success", "end_reason", "steps_used",
                  "step_budget", "result")
BLOB_REF = re.compile(r"blobs/[0-9a-f]{64}\.[A-Za-z0-9.]+")


def blob_refs(text: str) -> list[str]:
    """Every blob ref a trace stream mentions, once each."""
    return sorted(set(BLOB_REF.findall(text)))


def fetch_episode(artifact_uri: str, dest: Path) -> list[str]:
    """robot_result.json, trace/agent.jsonl and its blobs, copied from the episode folder
    artifact_uri names to dest; the paths that were missing."""
    src, dest = Path(artifact_uri), Path(dest)
    (dest / "trace" / "blobs").mkdir(parents=True, exist_ok=True)
    failed = []

    def copy(rel: str) -> None:
        if (dest / rel).is_file():
            return
        try:
            shutil.copyfile(src / rel, dest / rel)
        except OSError:
            failed.append(rel)

    for rel in (ROBOT_RESULT, "trace/agent.jsonl"):
        copy(rel)
    if (dest / "trace" / "agent.jsonl").is_file():
        for ref in blob_refs((dest / "trace" / "agent.jsonl").read_text()):
            copy(f"trace/{ref}")
    return failed


def step_budget(robot: dict, view: TraceView | None) -> int | None:
    """The episode's step budget: the robot record's, else the agent's last observation of it."""
    h = robot.get("horizon")
    if isinstance(h, dict) and isinstance(h.get("steps_allowed"), int):
        return h["steps_allowed"]
    p = robot.get("amp_protocol")
    if isinstance(p, dict) and isinstance(p.get("max_episode_length"), int):
        return p["max_episode_length"]
    for e in (view.kinds("agent.episode.over") if view else []):
        if isinstance(e.data.get("max_steps"), int):
            return e.data["max_steps"]
    return None


def outcome(robot: dict, view: TraceView | None = None, **meta) -> dict:
    """The official outcome: success, how it ended and the steps used of the budget, from the
    benchmark's episode record."""
    success = bool(robot.get("success"))
    used = robot.get("episode_step")
    if not isinstance(used, int):
        over = view.kinds("agent.episode.over") if view else []
        used = next((e.data["last_step"] for e in over if isinstance(e.data.get("last_step"), int)),
                    None)
    budget = step_budget(robot, view)
    if success:
        end = SUCCESS
    elif robot.get("truncated") or (used is not None and budget is not None and used >= budget):
        end = BUDGET
    else:
        end = NOT_DONE
    out = {k: meta.get(k) for k in ("episode", "layout", "workspace_sha")}
    out.update(success=success, end_reason=end, steps_used=used, step_budget=budget)
    out["result"] = result_text(out)
    return out


def result_text(o: dict) -> str:
    """SUCCESS (148 of 300 steps), FAILED: step budget exhausted (300 of 300 steps used) or
    FAILED: task not completed (212 of 300 steps used)."""
    used = "?" if o.get("steps_used") is None else o["steps_used"]
    budget = "?" if o.get("step_budget") is None else o["step_budget"]
    if o["success"]:
        return f"SUCCESS ({used} of {budget} steps)"
    return f"FAILED: {o['end_reason']} ({used} of {budget} steps used)"


def _fence(text: str, lang: str = "") -> str:
    ticks = "```"
    while ticks in text:
        ticks += "`"
    return f"{ticks}{lang}\n{text.rstrip()}\n{ticks}"


def _save_image(view: TraceView, block: dict, dest: Path, name: str) -> str | None:
    src = block.get("source") or {}
    ref = src.get(BLOB_KEY)
    if not isinstance(ref, dict) or not ref.get("ref"):
        return None
    ext = (src.get("media_type") or "image/png").split("/")[-1].replace("jpeg", "jpg")
    p = dest / f"{name}.{ext}"
    try:
        raw = view.blob(ref["ref"])
    except (OSError, ValueError, KeyError):
        return None
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_bytes(raw if ref.get("b64", True) else base64.b64decode(raw))
    return p.name


def transcript(view: TraceView, out: Path) -> str:
    """The episode as markdown: each turn's text and tool calls, then each tool's output, uncut."""
    lines: list[str] = []
    begin = next(iter(view.kinds("agent.run.begin")), None)
    if begin is not None:
        lines += [f"Instruction: {begin.data.get('instruction')}", ""]
        if begin.data.get("first_user"):
            lines += ["## First message", "", begin.data["first_user"], ""]
    for e in view.events:
        d = e.data
        if e.kind == "agent.llm.response":
            lines += [f"## Turn {d.get('turn')}", ""]
            for b in d.get("content") or []:
                if b.get("type") == "text" and b.get("text", "").strip():
                    lines += [b["text"].strip(), ""]
        elif e.kind == "agent.tool.call":
            inp = d.get("input") or {}
            if d.get("name") == "python":
                lines += ["### python", "", _fence(str(inp.get("code") or ""), "python"), ""]
            else:
                lines += [f"### {d.get('name')}", "", _fence(json.dumps(inp, indent=1), "json"), ""]
        elif e.kind == "agent.tool.result":
            obs = d.get("obs") or {}
            head = f"result{' (error)' if d.get('is_error') else ''}"
            if obs.get("step") is not None:
                head += f", step {obs.get('step')}, {obs.get('steps_left')} left"
            lines += [f"#### {head}", ""]
            if d.get("name") == "python":
                for k in ("stdout", "stderr", "error"):
                    if d.get(k):
                        lines += [f"{k}:", _fence(str(d[k])), ""]
            imgs = 0
            for i, b in enumerate(d.get("content") or []):
                if b.get("type") == "text" and (d.get("name") != "python" or i > 0):
                    lines += [_fence(b.get("text", "")), ""]
                elif b.get("type") == "image":
                    imgs += 1
                    name = _save_image(view, b, out / "images", f"turn{d.get('turn')}_{imgs}")
                    if name:
                        lines += [f"[image shown to the agent: images/{name}]", ""]
    return "\n".join(lines).rstrip() + "\n"


def frames(view: TraceView, out: Path) -> int:
    """Storyboard frames as out/frames/step<NNNNN>_<camera>.jpg; how many were written."""
    n = 0
    for e in view.kinds("agent.storyboard"):
        for b in e.blobs:
            cam = (b.role or "rgb:cam").split(":", 1)[-1]
            p = out / "frames" / f"step{int(e.data.get('step') or 0):05d}_{cam}.jpg"
            try:
                raw = view.blob(b.ref)
            except (OSError, ValueError, KeyError):
                continue
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_bytes(raw)
            n += 1
    return n


def build(ep_dir: Path, dest: Path, **meta) -> dict:
    """dest/: transcript.md, frames/, images/, report.json and outcome.json of one fetched
    episode (robot_result.json + trace/); returns the outcome."""
    ep_dir, dest = Path(ep_dir), Path(dest)
    robot = json.loads((ep_dir / ROBOT_RESULT).read_text())
    view = TraceView(ep_dir) if (ep_dir / "trace" / "agent.jsonl").is_file() else None
    dest.mkdir(parents=True, exist_ok=True)
    o = outcome(robot, view, **meta)
    if view is not None:
        (dest / "transcript.md").write_text(transcript(view, dest))
        frames(view, dest)
        rep = next(iter(view.kinds("agent.report")), None)
        if rep is not None:
            (dest / "report.json").write_text(json.dumps(rep.data.get("report"), indent=1))
    (dest / "outcome.json").write_text(json.dumps(o, indent=1))
    return o


def write_outcomes(history: Path, outcomes: list[dict], fields=OUTCOME_FIELDS) -> Path:
    """history/outcomes.csv: one row per episode, the official record only (and, from shared.py,
    the episode's instruction and its efficiency from its own trace)."""
    p = Path(history) / "outcomes.csv"
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for o in outcomes:
            w.writerow({k: o.get(k) for k in fields})
    return p
