#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Run finalization: each harness configuration's run_config.json, a status.json tally of its unit records, and
summary.json, successes per task and per corpus."""

from __future__ import annotations

import contextlib
import json
import time
from collections import Counter
from pathlib import Path

from amp.contracts.eval import Arm, outcome, unit_name

from .layout import RunLayout
from .store import parse_record, write_json
from .study import Study, expand

STATUS_FILE = "status.json"
SUMMARY_FILE = "summary.json"


def viewer_of(harness: str) -> str | None:
    from amp.harnesses import registry

    with contextlib.suppress(Exception):
        return registry.load(harness).viewer
    return None


def perception_of(harness: str) -> dict | None:
    """{"base", "overlays"} of an overlay harness, else None (descriptive: never fails a run)."""
    from amp.harnesses import registry

    with contextlib.suppress(Exception):
        return registry.perception(harness)
    return None


def _corpus_meta(study: Study) -> tuple[dict | None, str | None]:
    """(study.corpus_meta, None), or (None, why) when it cannot be computed."""
    try:
        return study.corpus_meta, None
    except Exception as e:  # noqa: BLE001
        return None, f"{type(e).__name__}: {e}"


def run_config(study: Study, arm: Arm, units: list[str], *, started_at: float | None = None,
               viewer: str | None = None,
               perception: dict | None = None) -> dict:
    """agentic/run_config.json, field for field as amp.contracts.eval pins it."""
    meta, why = _corpus_meta(study)
    return {"run_id": f"{study.job_id}__{arm.id}", "job_id": study.job_id, "arm_id": arm.id,
            "arm": arm.to_dict(), "code_ref": study.code_ref,
            "corpus": study.corpus,
            "eval": study.eval_name, "corpus_meta": meta,
            **({"corpus_meta_error": why} if why else {}),
            "protocol": study.to_dict()["protocol"], "units": list(units),
            "started_at": time.time() if started_at is None else started_at, "viewer": viewer,
            "perception": perception}


def arm_units(study: Study, arm: Arm) -> list[str]:
    return [unit_name(u.ref.task, u.ref.episode) for u in expand(study) if u.ref.arm_id == arm.id]


def write_run_config(layout: RunLayout, study: Study, arm: Arm, *, units: list[str] | None = None,
                     started_at: float | None = None) -> dict:
    """Write the harness configuration's run_config.json unless it already exists; return what is there."""
    if layout.run_config.is_file():
        with contextlib.suppress(OSError, ValueError):
            return json.loads(layout.run_config.read_text())
    cfg = run_config(study, arm, arm_units(study, arm) if units is None else units,
                     started_at=started_at, viewer=viewer_of(arm.harness),
                     perception=perception_of(arm.harness))
    write_json(layout.run_config, cfg)
    return cfg


def arm_status(layout: RunLayout, units: list[str]) -> dict:
    """Counts of the harness configuration's unit records by status, with successes over scored units."""
    statuses: Counter[str] = Counter()
    scored = succeeded = reconciled = 0
    missing = []
    for name in units:
        p = layout.units / f"{name}.json"
        rec = parse_record(p.read_bytes()) if p.is_file() else None
        if rec is None:
            missing.append(name)
            continue
        statuses[rec.get("status") or "unknown"] += 1
        reconciled += "reconciled" in rec
        o = outcome(rec)
        if o is not None:
            scored += 1
            succeeded += bool(o)
    return {"units": len(units), "statuses": dict(sorted(statuses.items())), "scored": scored,
            "succeeded": succeeded, "success_rate": succeeded / scored if scored else None,
            "reconciled": reconciled, "missing": missing}


def finalize_run(study: Study, root: str | Path) -> dict:
    """Write every harness configuration's status.json under root/<arm_id>/agentic/; return them by harness configuration id."""
    out: dict = {"root": str(root), "arms": {}, "errors": []}
    for arm in study.arms:
        layout = RunLayout(Path(root) / arm.id)
        try:
            cfg = write_run_config(layout, study, arm)
            status = arm_status(layout, list(cfg.get("units") or arm_units(study, arm)))
            write_json(layout.agentic / STATUS_FILE, status)
            out["arms"][arm.id] = status
        except Exception as e:  # noqa: BLE001
            out["errors"].append(f"{arm.id}: {type(e).__name__}: {e}")
    return out


def _tally(recs: list[dict | None]) -> dict:
    scored = succeeded = 0
    for rec in recs:
        o = outcome(rec) if rec is not None else None
        if o is not None:
            scored += 1
            succeeded += bool(o)
    return {"units": len(recs), "scored": scored, "succeeded": succeeded,
            "success_rate": succeeded / scored if scored else None,
            "missing": sum(r is None for r in recs)}


def success_summary(study: Study, root: str | Path) -> dict:
    """Successes per task and per corpus for every harness configuration, written to root/summary.json."""
    out: dict = {"job_id": study.job_id, "corpus": study.corpus, "arms": {}}
    for arm in study.arms:
        layout = RunLayout(Path(root) / arm.id)
        by_task: dict[str, list[dict | None]] = {}
        for u in expand(study):
            if u.ref.arm_id != arm.id:
                continue
            p = layout.unit_path(u.ref.task, u.ref.episode)
            by_task.setdefault(u.ref.task, []).append(
                parse_record(p.read_bytes()) if p.is_file() else None)
        out["arms"][arm.id] = {"total": _tally([r for rs in by_task.values() for r in rs]),
                               "tasks": {t: _tally(rs) for t, rs in sorted(by_task.items())}}
    write_json(Path(root) / SUMMARY_FILE, out)
    return out
