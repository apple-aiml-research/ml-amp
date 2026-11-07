#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What a chain of one shared notebook is made of, for the continual driver (continual.py): its
state and lock, notebook versions, episode claims, efficiency numbers and their paired statistics
(docs/practice.md).
"""
from __future__ import annotations

import json
import math
import os
import random
import shutil
import socket
import statistics
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path

from amp.practice import fewshot as LP
from amp.practice import workspace as W

EMPTY = LP.EMPTY
METRICS = ("turns", "cost_usd", "wall_s", "steps")
UNKNOWN = "unknown"
MAX_STEP_FAILS = 20
MAX_ROUND_ATTEMPTS = 3
MAX_TASK_ATTEMPTS = 3
MAX_TOPUP_TASKS = 24
FETCH_POLLS = 4
DEBRIEF_ATTEMPTS = 3
DEBRIEF_BACKOFF_S = 120
NOT_RERUN = ("attempt_error", "error")
DONE_TOOL = {
    "name": "done",
    "description": ("End the debrief and save ./workspace as it stands, changed or not, for every "
                    "later episode of any task. summary: one or two sentences on what you changed "
                    "and why, or that you left it as it was (kept in the debrief's log; later "
                    "episodes see only the notebook)."),
    "input_schema": {"type": "object", "properties": {"summary": {"type": "string"}},
                     "required": ["summary"]}}


class ChainFailed(RuntimeError):
    """The chain cannot go on without its operator (run_chain does not retry it)."""


class ChainLocked(RuntimeError):
    """Another live process drives this chain."""


class DebriefFailed(RuntimeError):
    """A debrief that ended before it produced a usable result (not recorded; retried)."""

    def __init__(self, msg: str, cost_usd: float = 0.0):
        super().__init__(msg)
        self.cost_usd = cost_usd


def log(*a, **kw) -> None:
    kw.setdefault("flush", True)
    print(*a, **kw)


def config(state: Path) -> dict:
    return json.loads((state / "config.json").read_text())


def load(state: Path) -> dict:
    p = state / "state.json"
    if p.is_file():
        return json.loads(p.read_text())
    return {"version": EMPTY, "rounds": [], "frozen": None, "eval": None}


def save(state: Path, st: dict) -> None:
    tmp = state / f"state.json.tmp.{os.getpid()}.{threading.get_ident()}"
    tmp.write_text(json.dumps(st, indent=1))
    os.replace(tmp, state / "state.json")


def chain_seeds(base: int, n: int) -> list[int]:
    """n distinct chain seeds derived from one base seed."""
    rng, out = random.Random(base), []  # nosec B311
    while len(out) < n:
        x = rng.randrange(2 ** 32)
        if x not in out:
            out.append(x)
    return out


def chain_dirs(path: Path) -> list[Path]:
    """The chain states under a chains root, or [path] for one state."""
    f = path / "chains.json"
    if f.is_file():
        return [path / d for d in json.loads(f.read_text())["dirs"]]
    return [path]


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


@contextmanager
def chain_lock(state: Path):
    """STATE/driver.lock (pid, host) held while this process drives the chain; a lock whose
    process is gone from this host is stale and taken over, any other refuses."""
    p = state / "driver.lock"
    me = {"pid": os.getpid(), "host": socket.gethostname(), "since": now(),
          "token": uuid.uuid4().hex}
    for _ in range(3):
        try:
            fd = os.open(p, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            try:
                held = json.loads(p.read_text())
            except (OSError, ValueError):
                held = None
            if held is None:
                try:
                    young = now() - p.stat().st_mtime < 60
                except FileNotFoundError:
                    continue
                if young:
                    raise ChainLocked(f"{state} is being locked by another process ({p})") from None
                log(f"removing an unreadable stale lock {p}")
                p.unlink(missing_ok=True)
                continue
            if held.get("host") == me["host"] and not _pid_alive(int(held.get("pid", -1))):
                log(f"removing a stale lock {p}: pid {held.get('pid')} on {held.get('host')} is gone")
                p.unlink(missing_ok=True)
                continue
            raise ChainLocked(
                f"{state} is driven by pid {held.get('pid')} on {held.get('host')} since "
                f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(held.get('since') or 0))}: "
                f"stop that process first; if it is gone (a process on another host cannot be "
                f"checked from here), delete {p}") from None
        with os.fdopen(fd, "w") as f:
            json.dump(me, f)
        break
    else:
        raise ChainLocked(f"{state}: could not take {p}")
    try:
        yield me
    finally:
        try:
            if json.loads(p.read_text()).get("token") == me["token"]:
                p.unlink()
        except (OSError, ValueError):
            pass


def store_version(state: Path, files: dict[str, bytes], parent: str, meta: dict) -> str:
    sha = W.sha_of(files)
    d = state / "versions" / sha
    if not d.is_dir():
        tmp = d.with_name(sha + ".tmp")
        W.write_dir(files, tmp)
        tmp.mkdir(parents=True, exist_ok=True)
        os.replace(tmp, d)
    with (state / "versions.jsonl").open("a") as f:
        f.write(json.dumps({"sha": sha, "parent": parent, "bytes": W.size(W.encode(files)),
                            "files": len(files), **meta}) + "\n")
    return sha


def version_files(state: Path, sha: str) -> dict[str, bytes]:
    return {} if sha == EMPTY else W.read_dir(state / "versions" / sha)


def version_dir(state: Path, sha: str) -> Path:
    d = state / "versions" / sha
    if W.sha_of(W.read_dir(d)) != sha:
        raise RuntimeError(f"{d} no longer holds version {sha[:12]}: refusing to submit it")
    return d


def efficiency(row: dict, summary: dict | None = None, robot: dict | None = None) -> dict:
    """turns (model calls), cost_usd, wall_s and simulator steps of one episode: its summary.json
    (from its own trace), else the unit record's totals and the robot record's step."""
    s = summary if summary is not None else row.get("summary")
    tot = (s or {}).get("totals") or {}
    out = (s or {}).get("outcome") or {}

    def num(*xs):
        return next((x for x in xs if isinstance(x, (int, float)) and not isinstance(x, bool)), None)

    return {"turns": num(tot.get("turns"), row.get("turns")),
            "cost_usd": num(tot.get("cost_usd"), row.get("cost_usd")),
            "wall_s": num(tot.get("wall_s"), row.get("wall_s")),
            "steps": num(out.get("steps"), (robot or {}).get("episode_step"), row.get("steps"))}


def now() -> float:
    return time.time()


def outcomes(state: Path) -> list[dict]:
    return [json.loads(p.read_text()) for p in sorted((state / "history").glob("*/outcome.json"))]


def instruction_of(ep: Path) -> str | None:
    p = ep / "trace" / "agent.jsonl"
    for line in p.read_text().splitlines() if p.is_file() else ():
        if '"agent.run.begin"' in line:
            ins = (json.loads(line).get("data") or {}).get("instruction")
            return ins.strip() if isinstance(ins, str) else None
    return None


def scored(row: dict | None) -> bool:
    return row is not None and row.get("status") == "ok" and row.get("attempted") is not False \
        and row.get("success") is not None


def claim(ep: Path, job: str) -> None:
    """ep/ for this job's episode: files another job (an earlier, lost attempt) left go first."""
    mark = ep / ".job"
    if ep.exists() and (not mark.is_file() or mark.read_text() != job):
        shutil.rmtree(ep)
    ep.mkdir(parents=True, exist_ok=True)
    mark.write_text(job)


def shown(o: dict) -> dict:
    """An outcome for history/outcomes.csv: a missing efficiency value reads UNKNOWN."""
    return {**o, **{k: UNKNOWN for k in ("turns", "cost_usd", "wall_s", "steps_used")
                    if k in o and o[k] is None}}


def money(x) -> str:
    return UNKNOWN if x is None else f"${x:.2f}"


def known(x) -> str:
    return UNKNOWN if x is None else str(x)


def debrief_kind(d: dict) -> str:
    """changed, unchanged (done() on an unchanged notebook), ended_without_done (a turn or cost
    limit, or no done(), on an unchanged notebook), refused_by_gate, skipped_infra or failed."""
    if "skipped" in d:
        return "skipped_infra"
    if "failed" in d:
        return "failed"
    if d.get("changed"):
        return "changed"
    if not d.get("saved"):
        return "refused_by_gate"
    return "unchanged" if d.get("end_reason") == "done" else "ended_without_done"


KINDS = ("changed", "unchanged", "ended_without_done", "refused_by_gate", "skipped_infra",
         "failed")


def failed_debriefs_cost(rnd: dict) -> float:
    return sum((f.get("cost_usd") or 0.0) for f in (rnd.get("debrief_failures") or {}).values())


def round_timing(rnd: dict) -> dict:
    """Seconds spent per part of a round: in its job (submit to collect), in debriefs, in all."""
    t = rnd.get("timing") or {}
    deb = [d["timing"]["end"] - d["timing"]["start"] for d in rnd["debriefs"].values()
           if d.get("timing")]
    out = {"pool_s": None, "debriefs_s": round(sum(deb)), "debrief_max_s": round(max(deb or [0])),
           "debriefs": len(deb), "total_s": None}
    if t.get("submitted") and t.get("collected"):
        out["pool_s"] = round(t["collected"] - t["submitted"])
    if t.get("planned") and t.get("done"):
        out["total_s"] = round(t["done"] - t["planned"])
    return out


def hms(s) -> str:
    if s is None:
        return "-"
    s = int(s)
    return f"{s // 3600}h{s % 3600 // 60:02d}m" if s >= 3600 else f"{s // 60}m{s % 60:02d}s"


def unit_metrics(rows: list[dict]) -> dict:
    """{(task, episode): {success, turns, cost_usd, wall_s, steps}} of the scored rows."""
    return {(r["task"], r["episode"]): {"success": bool(r["success"]), **efficiency(r)}
            for r in rows if scored(r)}


def sign_flip_p(diffs: list[float], n: int = 20_000, seed: int = 0) -> float | None:
    """Two-sided paired sign-flip p of the mean of per-task differences: exact up to 16 tasks,
    else Monte Carlo with n flips (the +1 estimate)."""
    if not diffs:
        return None
    obs = abs(sum(diffs)) - 1e-12
    if len(diffs) <= 16:
        hits = sum(1 for k in range(2 ** len(diffs))
                   if abs(sum(d if k >> i & 1 else -d for i, d in enumerate(diffs))) >= obs)
        return hits / 2 ** len(diffs)
    rng = random.Random(seed)  # nosec B311
    hits = sum(1 for _ in range(n)
               if abs(sum(d if rng.random() < 0.5 else -d for d in diffs)) >= obs)
    return (hits + 1) / (n + 1)


def efficiency_vs(run: dict, base: dict, tasks=None, *, boot: int = 4000, seed: int = 0) -> dict:
    """Per metric over the units successful in both (paired per unit): medians and means of each,
    the mean paired difference with a task-clustered bootstrap 95% CI and a sign-flip p over
    tasks' mean differences, and the ratio of means with its CI."""
    keys = sorted(k for k in run if k in base and run[k]["success"] and base[k]["success"]
                  and (tasks is None or k[0] in tasks))
    out = {"units": len(keys), "tasks": len({k[0] for k in keys}), "metrics": {}}
    for m in METRICS:
        pairs = [(k[0], run[k][m], base[k][m]) for k in keys
                 if run[k][m] is not None and base[k][m] is not None]
        if not pairs:
            out["metrics"][m] = None
            continue
        a, b = [x for _, x, _ in pairs], [y for _, _, y in pairs]
        by: dict[str, list[tuple[float, float]]] = {}
        for t, x, y in pairs:
            by.setdefault(t, []).append((x, y))
        per_task = [sum(x - y for x, y in v) / len(v) for v in by.values()]
        sums = [(sum(x for x, _ in v), sum(y for _, y in v), len(v)) for v in by.values()]
        rng = random.Random(seed)  # nosec B311
        diffs, ratios = [], []
        for _ in range(boot):
            pick = [rng.choice(sums) for _ in sums]
            sa, sb, n = (sum(g[i] for g in pick) for i in range(3))
            diffs.append((sa - sb) / n)
            ratios.append(sa / sb if sb else math.nan)
        diffs.sort()
        ratios = sorted(r for r in ratios if not math.isnan(r))

        def ci(xs):
            return [xs[int(0.025 * len(xs))], xs[max(0, int(0.975 * len(xs)) - 1)]] if xs else None

        out["metrics"][m] = {
            "pairs": len(pairs), "run_median": statistics.median(a),
            "base_median": statistics.median(b), "run_mean": statistics.fmean(a),
            "base_mean": statistics.fmean(b),
            "median_diff": statistics.median(x - y for x, y in zip(a, b)),
            "mean_diff": statistics.fmean(x - y for x, y in zip(a, b)), "mean_diff_ci95": ci(diffs),
            "ratio_of_means": sum(a) / sum(b) if sum(b) else None, "ratio_ci95": ci(ratios),
            "sign_flip_p": sign_flip_p(per_task)}
    return out


def success_map(m: dict) -> dict:
    return {k: v["success"] for k, v in m.items()}


def rows_of(spec: str, pool) -> list[dict]:
    """A reference's unit rows: an amp eval run folder of the study's arm, or job ids under the
    runs folder, comma separated (one eval split over several jobs)."""
    p = Path(spec).expanduser()
    if p.is_dir():
        units = p / pool.arm / "agentic" / "units"
        if not units.is_dir():
            raise FileNotFoundError(
                f"{p}: no {pool.arm}/agentic/units: no units yet, or another arm's run")
        return [{**json.loads(u.read_text()), "job": p.name} for u in sorted(units.glob("*.json"))]
    if "/" in spec:
        raise FileNotFoundError(f"{spec}: no such run folder")
    out = []
    for job in spec.split(","):
        out += [{**r, "job": r.get("job_id") or job} for r in pool.units(job)]
    return out


def debrief_counts(L: dict) -> str:
    return (f"{L['changed']} changed the notebook, {L['unchanged']} left it unchanged by choice, "
            f"{L['ended_without_done']} ended without done() (unchanged), {L['refused_by_gate']} "
            f"refused by the gate, {L['skipped_infra']} skipped (trace did not rebuild), "
            f"{L['failed']} failed")
