#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Few-shot practice (notebook v3), per RoboDojo task: practice batches from one workspace
version, a debrief after every batch ending in keep, change or stop, a budget of practice
episodes, then the frozen workspace's evaluation and results (docs/practice.md).

  amp practice fewshot init STATE --study STUDY.json
  amp practice fewshot run STATE [--every 60] [--shard I/N] [--eval]   until every task is frozen:
        collect finished jobs, run due debriefs, submit planned batches (--eval: and each frozen
        task's evaluation job, no debrief, no result returned; a shard: tasks I, I+N, ... of the
        study)
  amp practice fewshot results STATE [--none RUN] [--check-delivery]   workspace minus the
        no-memory baseline run: per-config differences, exact sign-flip p, task-clustered
        bootstrap 95% CI, practice used; the leaderboard success rate and, with --none, the
        model-call ratio (a folder of --round evaluations: the k curve)
  amp practice fewshot eval RUN ... | select RUN RUN ...   evaluations of frozen runs and the
        best-of-N selection (amp.practice.fewshot_eval)

State: STATE/config.json; per task STATE/tasks/<task>/state.json (rounds, current version,
status), versions/<sha>/ (every saved workspace, content-addressed), versions.jsonl (sha, parent,
round, summary), episodes/<key>/ (copied traces), history/ (the debrief's read-only view) and
debriefs/r<n>/ (each debrief's directory and trace). Jobs run on this machine under the study's
runs folder (amp.practice.local). Every command is safe to repeat.

Rules: a batch runs `batch` episodes (4; 1 or an even number) in parallel from one version, on
practice layouts that are never evaluation layouts; a Generalization task runs both settings of
each layout (one setting a batch at batch 1). After every batch one debrief continues one of its episodes, sees every version and every earlier
episode, and ends with keep, change or stop. Practice stops at a stop or at the budget (48
episodes), and the workspace freezes as left.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from amp.fs.atomic import write_bytes_atomic
from amp.practice import debrief as D
from amp.practice import history as H
from amp.practice import local
from amp.practice import workspace as W

EMPTY = W.sha_of({})
UNDELIVERED = "undelivered"
TERMINAL = local.TERMINAL
DEFAULTS = {"batch": 4, "budget": 48, "workspace_max_bytes": 128 << 10, "prompt": "amp1",
            "gpus": "auto", "debrief": {"max_turns": 100, "max_cost_usd": 20.0}}
NOTEBOOK = 3
POINTER = 2
WORKSPACE_FORMATS = ("code", "notes")
TAG = re.compile(r"[A-Za-z0-9._-]{1,32}")
DEBRIEF_ATTEMPTS = 3
DEBRIEF_BACKOFF_S = (600, 1800)
RESUBMITS = 3
TOPUPS = 3

def now() -> float:
    return time.time()


def config(state: Path) -> dict:
    return json.loads((state / "config.json").read_text())


def task_dir(state: Path, task: str) -> Path:
    return state / "tasks" / task


def load(state: Path, task: str) -> dict:
    p = task_dir(state, task) / "state.json"
    if p.is_file():
        return json.loads(p.read_text())
    return {"task": task, "rounds": [], "version": EMPTY, "status": "practising", "frozen": None,
            "cursor": 0, "eval": None}


def save(state: Path, task: str, st: dict) -> None:
    d = task_dir(state, task)
    d.mkdir(parents=True, exist_ok=True)
    tmp = d / "state.json.tmp"
    tmp.write_text(json.dumps(st, indent=1))
    os.replace(tmp, d / "state.json")


def store_version(state: Path, task: str, files: dict[str, bytes], parent: str | None,
                  meta: dict) -> str:
    """Save a workspace under versions/<sha>/ (once) and log it; its sha."""
    sha = W.sha_of(files)
    d = task_dir(state, task) / "versions" / sha
    if not d.is_dir():
        tmp = d.with_name(sha + ".tmp")
        W.write_dir(files, tmp)
        tmp.mkdir(parents=True, exist_ok=True)
        os.replace(tmp, d)
    with (task_dir(state, task) / "versions.jsonl").open("a") as f:
        f.write(json.dumps({"sha": sha, "parent": parent, "bytes": W.size(W.encode(files)),
                            "files": len(files), **meta}) + "\n")
    return sha


def version_files(state: Path, task: str, sha: str) -> dict[str, bytes]:
    if sha == EMPTY:
        return {}
    return W.read_dir(task_dir(state, task) / "versions" / sha)


def task_configs(corpus: str, task: str) -> list[str]:
    from amp import benchmarks

    return sorted({u.task for u in benchmarks.expand(corpus, None, (task,), (0,))})


def scored_layouts(corpus: str, task: str) -> int:
    """Scored layouts per seed of every config of a base task (25 for a Generalization task)."""
    from amp.benchmarks import robodojo as RB

    return min(RB._lookup(c).eval_nums for c in task_configs(corpus, task))


def practice_queue(cfg: dict, task: str) -> list[tuple[str, int]]:
    """(corpus, layout) in practice order: every scored layout of each practice corpus (never
    the eval corpus's, init refuses a shared one)."""
    return [(corpus, k) for corpus in cfg["practice"]["corpora"]
            for k in range(scored_layouts(corpus, task))]


def episodes_per_scene(cfg: dict, task: str) -> int:
    """Episodes one layout gives: a Generalization task runs both configs."""
    return len(task_configs(cfg["practice"]["corpora"][0], task))


def units_of(corpus: str, task: str, layouts=None, episodes=None) -> int:
    from amp import benchmarks

    return len(benchmarks.expand(corpus, episodes, (task,),
                                 None if layouts is None else tuple(layouts)))


def study_task_names(cfg: dict, task: str) -> list[str]:
    """Every task or config name of the study's corpora: the debrief's saves may hold none."""
    from amp import benchmarks
    from amp.benchmarks import robodojo as RB

    names = {task, *cfg["tasks"]}
    for c in {*cfg["practice"]["corpora"], cfg["eval"]["corpus"]}:
        names.update(benchmarks.corpus_tasks(c))
    names.update(RB.load().tasks)
    return sorted(names)


def episode_key(cfg: dict, rnd: int, row: dict) -> str:
    """A collected episode's folder name, without its task name: r01_e3 or r01_e3_random by
    config."""
    random = str(row["task"]).endswith("_random")
    return f"r{rnd:02d}_e{row['episode']}" + ("_random" if random else "")


def scene_of(cfg: dict, row: dict) -> int:
    from amp.benchmarks import robodojo as RB

    return RB.address(row["task"], row["episode"])[1]


def outcomes(state: Path, task: str) -> list[dict]:
    """Every collected episode's outcome.json, in round order."""
    h = task_dir(state, task) / "history"
    return [json.loads(p.read_text()) for p in sorted(h.glob("*/outcome.json"))]


def record(state: Path, task: str, sha: str) -> tuple[int, int]:
    """(passes, fails) of episodes that started from exactly this version."""
    rows = [o for o in outcomes(state, task) if (o.get("workspace_sha") or EMPTY) == sha]
    p = sum(1 for o in rows if o["success"])
    return p, len(rows) - p


def used(state: Path, task: str) -> int:
    return len(outcomes(state, task))


def round_outcomes(state: Path, task: str, rnd: dict) -> list[dict]:
    h = task_dir(state, task) / "history"
    return [json.loads((h / k / "outcome.json").read_text()) for k in rnd.get("episodes") or []]


def one_setting(cfg: dict, task: str) -> bool:
    """A batch smaller than one layout's episodes (batch 1 on a Generalization task): each
    batch is one setting of one layout."""
    return cfg["batch"] < episodes_per_scene(cfg, task)


def batch_slots(cfg: dict, task: str) -> list[tuple]:
    """What batches take, in order: the practice layouts, or with one_setting each layout's
    settings in turn, standard then random, as (corpus, layout, config)."""
    q = practice_queue(cfg, task)
    if not one_setting(cfg, task):
        return q
    return [(corpus, k, c) for corpus, k in q
            for c in sorted(task_configs(corpus, task), key=lambda c: c.endswith("_random"))]


def layouts_per_batch(cfg: dict, task: str) -> int:
    """Slots one batch takes: the batch's episodes over the episodes one layout gives, at
    least one (a single setting, one_setting)."""
    return max(1, cfg["batch"] // episodes_per_scene(cfg, task))


def batches_total(cfg: dict, task: str) -> int:
    """The batches of a task: the budget in whole batches, as many as its practice layouts
    allow; a last partial batch is never run."""
    return min(cfg["budget"] // cfg["batch"],
               len(batch_slots(cfg, task)) // layouts_per_batch(cfg, task))


def setting_job(corpus: str, task: str, only: list[tuple[str, int]]) -> dict:
    """A job running only these (config, layout) episodes: amp eval runs every setting of a
    layout, so the others run too ("runs") and are never collected."""
    ks = sorted({k for _, k in only})
    return {"corpus": corpus, "layouts": ks, "want": len(only), "only": [list(x) for x in only],
            "runs": units_of(corpus, task, ks)}


def plan_batch(cfg: dict, st: dict, task: str) -> dict:
    """The next batch: the next layouts of the queue, in order; on a Generalization task both
    settings of each, the standard first in odd batches and the random first in even ones (with
    one_setting, the next setting of the queue)."""
    n, k = len(st["rounds"]) + 1, layouts_per_batch(cfg, task)
    take = batch_slots(cfg, task)[st["cursor"]:st["cursor"] + k]
    if len(take) != k:
        raise RuntimeError("a batch past the practice queue")
    st["cursor"] += k
    rnd = {"n": n, "kind": "batch", "version": st["version"], "status": "planned", "jobs": [],
           "episodes": [], "random_first": episodes_per_scene(cfg, task) == 2 and n % 2 == 0}
    if one_setting(cfg, task):
        (corpus, lay, conf), = take
        rnd["random_first"] = conf.endswith("_random")
        rnd["jobs"].append(setting_job(corpus, task, [(conf, lay)]))
        return rnd
    by: dict[str, list[int]] = {}
    for corpus, lay in take:
        by.setdefault(corpus, []).append(lay)
    for corpus, ks in by.items():
        rnd["jobs"].append({"corpus": corpus, "layouts": ks, "want": units_of(corpus, task, ks)})
    return rnd


def batch_order(rnd: dict, results: list[dict]) -> list[dict]:
    """The batch's outcomes in batch order: by layout, and within a layout the setting that comes
    first in this batch (rnd["random_first"])."""
    rf = bool(rnd.get("random_first"))

    def key(o):
        lay = o.get("layout") if isinstance(o.get("layout"), int) else -1
        return lay, str(o["episode"]).endswith("_random") != rf

    return sorted(results, key=key)


def freeze_v3(st: dict, status: str, why: str) -> None:
    st["status"], st["frozen"], st["frozen_why"] = status, st["version"], why


def budget_episodes(cfg: dict, task: str) -> int:
    return batches_total(cfg, task) * cfg["batch"]


def advance(cfg: dict, state: Path, task: str, st: dict) -> None:
    """Move a task on: a debrief after every batch, then stop (the debrief's decision), a
    top-up batch for lost episodes, the budget of scored episodes, or the next batch."""
    if st["status"] != "practising":
        return
    last = st["rounds"][-1] if st["rounds"] else None
    n_used, budget = used(state, task), budget_episodes(cfg, task)
    if last is not None:
        if last["status"] in ("planned", "submitted", "needs_debrief"):
            return
        if last["status"] == "collected":
            if round_outcomes(state, task, last):
                last["status"] = "needs_debrief"
                return
            last["debrief"] = {"skipped": "no scored episode in this round", "decision": None,
                               "applied": "keep"}
            last["status"] = "debriefed"
        if (last.get("debrief") or {}).get("applied") == "stop":
            freeze_v3(st, "stopped", f"stop: the round {last['n']} debrief stopped practice after "
                                     f"{n_used} of {budget} practice episodes")
            return
    lost = [r for r in st["rounds"] if r.get("lost") and "rerun_in" not in r]
    if lost and n_used < budget:
        if sum(1 for r in st["rounds"] if r["kind"] == "topup") < TOPUPS:
            st["rounds"].append(plan_topup(cfg, st, task, lost))
            print(f"PROBLEM {task}: round {st['rounds'][-1]['n']} is a top-up batch running again "
                  f"the lost layouts of round(s) {', '.join(str(r['n']) for r in lost)}",
                  flush=True)
            return
        for r in lost:
            r["rerun_in"] = None
        print(f"PROBLEM {task}: {TOPUPS} top-up batches used: the lost layouts of round(s) "
              f"{', '.join(str(r['n']) for r in lost)} are not run again", flush=True)
    k = layouts_per_batch(cfg, task)
    if n_used + cfg["batch"] <= budget and st["cursor"] + k <= len(batch_slots(cfg, task)):
        st["rounds"].append(plan_batch(cfg, st, task))
        return
    short = "" if n_used >= budget else " (no whole batch is left)"
    freeze_v3(st, "budget", f"budget: {n_used} of {budget} practice episodes used{short}; frozen "
                            f"as left after the round {len(st['rounds'])} debrief")


def plan_topup(cfg: dict, st: dict, task: str, lost: list[dict]) -> dict:
    """A top-up batch: the lost layouts of earlier rounds again, from the current version,
    composed as a batch (every setting of each layout, the first setting alternating)."""
    n = len(st["rounds"]) + 1
    rnd = {"n": n, "kind": "topup", "topup_of": [r["n"] for r in lost], "version": st["version"],
           "status": "planned", "jobs": [], "episodes": [],
           "random_first": episodes_per_scene(cfg, task) == 2 and n % 2 == 0}
    if one_setting(cfg, task):
        only: dict[str, list[tuple[str, int]]] = {}
        for r in lost:
            r["rerun_in"] = n
            for corpus, conf, lay in r["lost_units"]:
                only.setdefault(corpus, []).append((conf, lay))
        for corpus, us in only.items():
            rnd["jobs"].append(setting_job(corpus, task, sorted(us, key=lambda u: (u[1], u[0]))))
        rnd["random_first"] = rnd["jobs"][0]["only"][0][0].endswith("_random")
        return rnd
    by: dict[str, list[int]] = {}
    for r in lost:
        r["rerun_in"] = n
        for corpus, lay in r["lost"]:
            if lay not in by.setdefault(corpus, []):
                by[corpus].append(lay)
    for corpus, ks in by.items():
        ks.sort()
        rnd["jobs"].append({"corpus": corpus, "layouts": ks, "want": units_of(corpus, task, ks)})
    return rnd


def referenced(st: dict) -> set[str]:
    """The versions the saved state refers to: a version logged but never saved with the state
    (a crash in between) is no version."""
    out = {st["version"]}
    for r in st["rounds"]:
        out.add(r["version"])
        out.add((r.get("debrief") or {}).get("version_after") or r["version"])
    return out


def version_order(state: Path, task: str, st: dict | None = None) -> list[str]:
    """Every workspace version of the task in the order it was first made; v0 is the empty one."""
    keep = None if st is None else referenced(st)
    out = [EMPTY]
    p = task_dir(state, task) / "versions.jsonl"
    for line in p.read_text().splitlines() if p.is_file() else ():
        sha = json.loads(line)["sha"]
        if sha not in out and (keep is None or sha in keep):
            out.append(sha)
    return out


def version_folder(i: int, sha: str) -> str:
    return "v0_no_notes" if i == 0 else f"v{i}_{sha[:12]}"


def version_rows(cfg: dict, state: Path, task: str, st: dict) -> list[dict]:
    """Per version: its number, folder, parent, the round whose debrief made it, the rounds and
    episodes that ran from it, and its passes and attempts per setting."""
    made = {}
    p = task_dir(state, task) / "versions.jsonl"
    for line in p.read_text().splitlines() if p.is_file() else ():
        r = json.loads(line)
        made.setdefault(r["sha"], r)
    order = version_order(state, task, st)
    rows = []
    for i, sha in enumerate(order):
        eps = [o for o in outcomes(state, task) if (o.get("workspace_sha") or EMPTY) == sha]
        split = {}
        for name, rand in (("standard", False), ("random", True)):
            mine = [o for o in eps if str(o["episode"]).endswith("_random") == rand]
            split[name] = (sum(1 for o in mine if o["success"]), len(mine))
        parent = (made.get(sha) or {}).get("parent")
        rows.append({"n": i, "sha": sha, "folder": version_folder(i, sha),
                     "parent": None if i == 0 or parent not in order else order.index(parent),
                     "made_after_round": None if i == 0 else (made.get(sha) or {}).get("round"),
                     "rounds": [r["n"] for r in st["rounds"]
                                if r["version"] == sha
                                and r["status"] not in ("planned", "submitted")],
                     "episodes": [o["episode"] for o in eps],
                     "passes": sum(1 for o in eps if o["success"]), "attempts": len(eps), **split})
    return rows


def write_versions(cfg: dict, state: Path, task: str, st: dict) -> Path:
    """history/versions/: every version's files under v<n>_<sha12>/ (v0_no_notes/ empty) and
    index.md with each version's parent and the rounds and episodes that ran from it."""
    import shutil

    root = task_dir(state, task) / "history" / "versions"
    if root.exists():
        shutil.rmtree(root)
    root.mkdir(parents=True)
    rows = version_rows(cfg, state, task, st)
    cur = next(r["n"] for r in rows if r["sha"] == st["version"])
    lines = ["# Workspace versions", "",
             (f"One folder per version of this task's workspace, with its files. ./workspace now "
              f"holds v{cur}."), ""]
    for r in rows:
        (root / r["folder"]).mkdir()
        W.write_dir(version_files(state, task, r["sha"]), root / r["folder"])
        how = ("the empty workspace (no notes)" if r["n"] == 0 else
               f"made by the round {r['made_after_round']} debrief from "
               f"{'v' + str(r['parent']) if r['parent'] is not None else 'an earlier version'}")
        lines.append(f"- v{r['n']} ({r['folder']}/): {how}; rounds run from it: "
                     f"{', '.join(map(str, r['rounds'])) or 'none'}; episodes: "
                     f"{', '.join(r['episodes']) or 'none'}.")
    (root / "index.md").write_text("\n".join(lines) + "\n")
    return root


def v3_facts(cfg: dict, state: Path, task: str, st: dict, rnd: dict) -> str:
    """The debrief's facts: the per-version pass table, the practice episodes used and what the
    decision does."""
    gen = episodes_per_scene(cfg, task) == 2
    rows = version_rows(cfg, state, task, st)
    lines = [("Official practice results by workspace version (episodes run from exactly that "
              "version; each version's files are in history/versions/):"), ""]
    if gen:
        lines += [("| version | rounds | standard setting: passes/attempts | random setting: "
                   "passes/attempts |"), "|---|---|---|---|"]
    else:
        lines += ["| version | rounds | passes/attempts |", "|---|---|---|"]
    for r in rows:
        name = "v0 (no notes)" if r["n"] == 0 else f"v{r['n']}"
        rounds = ", ".join(map(str, r["rounds"])) or "-"
        cells = ([f"{r['standard'][0]}/{r['standard'][1]}", f"{r['random'][0]}/{r['random'][1]}"]
                 if gen else [f"{r['passes']}/{r['attempts']}"])
        lines.append(f"| {name} | {rounds} | " + " | ".join(cells) + " |")
    budget = budget_episodes(cfg, task)
    lines += ["", f"You have used {used(state, task)} of the {budget} practice episodes.", "",
              ("After each batch you decide: keep the workspace as it is and continue practising, "
               "change it and continue, or stop. When you stop, or when the "
               f"{budget} practice episodes are used up, the workspace as you leave it is frozen "
               "and evaluated once, with no further changes, on new layouts of this task that you "
               "have not seen. That evaluation is what counts.")]
    return "\n".join(lines)


def version_label(state: Path, task: str, sha: str, st: dict | None = None) -> str:
    order = version_order(state, task, st)
    i = order.index(sha) if sha in order else len(order)
    return "v0 (the empty workspace, no notes)" if i == 0 else f"v{i} ({sha[:12]})"


def debrief_batch(cfg: dict, state: Path, task: str, st: dict, rnd: dict, *, profile=None,
               **kw) -> dict:
    """The debrief of a batch: it continues the batch's first episode (in batch order) that got the
    batch's version and whose trace rebuilds, sees every version in history/versions/, and ends
    with keep, change or stop."""
    d = task_dir(state, task)
    start = rnd["version"]
    results = batch_order(rnd, round_outcomes(state, task, rnd))
    off = [o["episode"] for o in results if (o.get("workspace_sha") or EMPTY) != start]
    cands = ([o for o in results if o["episode"] not in off]
             + [o for o in results if o["episode"] in off])
    conv, passed_over = conversation_of(cands, d / "episodes", prefer_failure=False)
    for ep, why in passed_over:
        print(f"PROBLEM {task}: round {rnd['n']}'s debrief cannot continue {ep}: {why}", flush=True)
    if conv is None:
        rnd["debrief"] = {"skipped": "no episode's trace of this round rebuilds",
                          "passed_over": [list(x) for x in passed_over], "decision": None,
                          "applied": "keep"}
        rnd["status"] = "debriefed"
        advance(cfg, state, task, st)
        save(state, task, st)
        print(f"{task} round {rnd['n']} debrief skipped: no usable conversation, version "
              f"{st['version'][:12]} unchanged", flush=True)
        return rnd["debrief"]
    if profile is None:
        profile = local.profile(cfg)
    write_versions(cfg, state, task, st)
    attempt = len(rnd.get("debrief_failures") or []) + 1
    try:
        rec = D.run(d / "debriefs" / f"r{rnd['n']:02d}", trace=d / "episodes" / conv,
                    results=results, conversation=conv,
                    workspace_files=version_files(state, task, start),
                    workspace_sha=None if start == EMPTY else start, history=d / "history",
                    profile=profile, names=study_task_names(cfg, task),
                    notebook=NOTEBOOK, facts=v3_facts(cfg, state, task, st, rnd),
                    max_bytes=cfg["workspace_max_bytes"],
                    workspace_format=cfg.get("workspace_format"),
                    version_label=version_label(state, task, start, st), undelivered=off,
                    max_turns=cfg["debrief"]["max_turns"],
                    max_cost_usd=cfg["debrief"]["max_cost_usd"], private=[state], **kw)
    except Exception as e:  # noqa: BLE001
        rec = {"end_reason": "exception", "error": f"{type(e).__name__}: {e}", "turns": 0,
               "cost_usd": 0.0, "saved": False, "sha": None, "summary": None, "decision": None}
    said = rec.get("decision")
    transient = said is None and (rec["end_reason"] in ("api_error", "exception")
                                  or not rec.get("turns"))
    if transient:
        fail = {**{k: v for k, v in rec.items() if k != "files"}, "attempt": attempt, "at": now()}
        rnd.setdefault("debrief_failures", []).append(fail)
        if attempt < DEBRIEF_ATTEMPTS:
            wait = DEBRIEF_BACKOFF_S[min(attempt, len(DEBRIEF_BACKOFF_S)) - 1]
            rnd["debrief_retry_at"] = now() + wait
            save(state, task, st)
            print(f"PROBLEM {task}: round {rnd['n']} debrief ended {rec['end_reason']} with no "
                  f"decision (attempt {attempt} of {DEBRIEF_ATTEMPTS}): not applied, tried again "
                  f"in {wait // 60} min", flush=True)
            return fail
        print(f"PROBLEM {task}: round {rnd['n']} debrief ended {rec['end_reason']} with no "
              f"decision {DEBRIEF_ATTEMPTS} times: kept the workspace (forced_keep)", flush=True)
    deb = {k: v for k, v in rec.items() if k != "files"}
    if said is None:
        deb["forced_keep" if transient else "no_decision"] = rec["end_reason"]
        if rec.get("saved") and rec.get("sha") != start:
            deb["unapplied_sha"] = rec["sha"]
    elif rec["saved"] and rec["sha"] != start:
        st["version"] = store_version(state, task, rec["files"], start, {
            "round": rnd["n"], "summary": rec["summary"], "cost_usd": rec["cost_usd"]})
    edited = st["version"] != start
    applied = "stop" if said == "stop" else "change" if edited else "keep"
    deb.update(decision=said, applied=applied, edited=edited, version_after=st["version"])
    if said == "keep" and edited:
        deb["keep_with_edits"] = True
    if said == "change" and not edited:
        deb["change_without_edits"] = True
    rnd.pop("debrief_retry_at", None)
    rnd["debrief"] = deb
    rnd["status"] = "debriefed"
    advance(cfg, state, task, st)
    save(state, task, st)
    print(f"{task} round {rnd['n']} debrief: {rec['end_reason']}, decision {said} ({applied}), "
          f"version {st['version'][:12]}, ${rec['cost_usd']}", flush=True)
    return deb


def decisions_of(st: dict) -> dict:
    """A task's debrief decisions: applied keep/change/stop counts of the debriefs that
    decided, those that did not (no_decision, forced_keep), keeps with edits, and whether its
    stop came right after an edit."""
    debs = [r["debrief"] for r in st["rounds"]
            if r.get("debrief") and not r["debrief"].get("skipped")]
    real = [x for x in debs if not x.get("no_decision") and not x.get("forced_keep")]
    out = {k: sum(1 for x in real if x.get("applied") == k) for k in ("keep", "change", "stop")}
    out["no_decision"] = sum(1 for x in debs if x.get("no_decision"))
    out["forced_keep"] = sum(1 for x in debs if x.get("forced_keep"))
    out["keep_with_edits"] = sum(1 for x in debs if x.get("keep_with_edits"))
    out["skipped"] = sum(1 for r in st["rounds"] if (r.get("debrief") or {}).get("skipped"))
    out["stop_after_edit"] = any(x.get("applied") == "stop" and x.get("edited") for x in debs)
    out["rounds_at_freeze"] = len(st["rounds"]) if st["frozen"] is not None else None
    return out


def eval_frozen(cfg: dict, state: Path, tasks: list[str], runner=None) -> list[str]:
    """Submit the eval job of every frozen task that has none yet (run --eval); the tasks."""
    out = []
    for t in tasks:
        st = load(state, t)
        ev = (st.get("eval") or {}).get("jobs")
        if st["frozen"] is None or (ev and all(j.get("job") for j in ev)):
            continue
        try:
            eval_submit(cfg, state, t, runner=runner)
            out.append(t)
        except Exception as e:  # noqa: BLE001
            print(f"EVAL FAILED {t}: {type(e).__name__}: {e}", flush=True)
    return out


def eval_pending(state: Path, tasks: list[str]) -> list[str]:
    """The frozen tasks with an eval job not yet submitted (or none planned)."""
    out = []
    for t in tasks:
        st = load(state, t)
        ev = (st.get("eval") or {}).get("jobs")
        if st["frozen"] is not None and not (ev and all(j.get("job") for j in ev)):
            out.append(t)
    return out


def submit_argv(cfg: dict, state: Path, task: str, job: dict, version: str, tag: str) -> list[str]:
    """The job's amp eval robodojo argv (amp.practice.local runs it)."""
    ws = None if version == EMPTY else task_dir(state, task) / "versions" / version
    return local.eval_argv(cfg, tasks=[task], corpus=job["corpus"], exp_tag=tag,
                           layouts=job.get("layouts"),
                           episodes=None if job.get("layouts") is not None else job["episodes"],
                           workspace=ws, notebook=POINTER if ws else None,
                           max_bytes=cfg["workspace_max_bytes"] if ws else None)


def tag_for(cfg: dict, task: str, what: str) -> str:
    t = f"{cfg['tag']}_{cfg['tasks'].index(task):02d}{what}"
    if not TAG.fullmatch(t):
        raise ValueError(f"exp tag {t!r}: 1-32 of letters, digits, . _ -")
    return t


def submit(cfg: dict, state: Path, task: str, runner=None) -> None:
    runner = runner or local.LocalRunner(cfg["runs"])
    st = load(state, task)
    for rnd in st["rounds"]:
        if rnd["status"] != "planned":
            continue
        for i, job in enumerate(rnd["jobs"]):
            if job.get("job"):
                continue
            argv = submit_argv(cfg, state, task, job, rnd["version"],
                               tag_for(cfg, task, f"r{rnd['n']}{'abcdef'[i] if i else ''}"))
            job["job"] = runner(argv)
            save(state, task, st)
            print(f"{task} round {rnd['n']}: {job['job']}", flush=True)
        rnd["status"] = "submitted"
        save(state, task, st)


def check_delivery(ep_dir: Path, version: str) -> str | None:
    """Why the episode's agent did not get this version (agent.workspace), or None."""
    p = ep_dir / "trace" / "agent.jsonl"
    return trace_delivery(p.read_text() if p.is_file() else None, version)


def trace_delivery(text: str | None, version: str) -> str | None:
    """check_delivery on an agent trace's text (None: the trace could not be read)."""
    if text is None:
        return "no agent trace"
    got = [json.loads(x)["data"] for x in text.splitlines() if '"agent.workspace"' in x]
    got = [d for d in got if isinstance(d, dict) and "sha" in d]
    if version == EMPTY:
        return (f"a workspace {got[0]['sha'][:12]} reached an episode that should have none"
                if got else None)
    if not got:
        return f"no agent.workspace event: version {version[:12]} did not reach the agent"
    return None if got[0]["sha"] == version else f"workspace {got[0]['sha'][:12]} != {version[:12]}"


def delivered_sha(ep_dir: Path) -> str:
    """The workspace sha the episode's agent recorded (agent.workspace), else UNDELIVERED."""
    p = ep_dir / "trace" / "agent.jsonl"
    if p.is_file():
        for d in workspace_events(p.read_text()):
            if isinstance(d.get("sha"), str) and d.get("found", True):
                return d["sha"]
    return UNDELIVERED


def job_ended_short(pool, job: dict) -> str | None:
    """The terminal state of a job that ended with fewer unit files than it wants, or None while
    it may still write them (a view that cannot say, as a test fake, keeps waiting)."""
    state = getattr(pool, "job_state", lambda _: None)(job["job"])
    return state if state in TERMINAL else None


def collect(cfg: dict, state: Path, task: str, pool) -> bool:
    """Collect the task's submitted round once all its jobs finished; whether it did."""
    st = load(state, task)
    rnd = next((r for r in st["rounds"] if r["status"] == "submitted"), None)
    if rnd is None:
        return False
    keys, problems = [], []
    for j in rnd["jobs"]:
        n, runs = pool.count(j["job"]), j.get("runs", j["want"])
        if n < runs:
            ended = job_ended_short(pool, j)
            if ended is None:
                return False
            problems.append(f"job {j['job']} ended {ended} with {n} of {runs} unit files: "
                            "collecting what exists")
    d = task_dir(state, task)
    got = set()
    for j in rnd["jobs"]:
        for row in sorted(pool.units(j["job"]), key=lambda r: (r["task"], r["episode"])):
            if "only" in j and [row["task"], scene_of(cfg, row)] not in j["only"]:
                continue
            if (row.get("status") != "ok" or row.get("attempted") is False
                    or row.get("success") is None):
                continue
            key = episode_key(cfg, rnd["n"], row)
            ep = d / "episodes" / key
            missing = pool.fetch(row, ep)
            if missing and not (ep / H.ROBOT_RESULT).is_file():
                problems.append(f"{key}: not fetched ({missing[:2]})")
                continue
            why = check_delivery(ep, rnd["version"])
            sha = rnd["version"]
            if why:
                sha = delivered_sha(ep)
                problems.append(f"{key}: {why}; counted toward the budget, recorded as "
                                f"{sha[:12]}, not as version {rnd['version'][:12]}")
            H.build(ep, d / "history" / key, episode=key, layout=scene_of(cfg, row),
                    workspace_sha=sha)
            keys.append(key)
            got.add((j["corpus"], row["task"], scene_of(cfg, row)))
    if not keys and rnd.get("resubmits", 0) < RESUBMITS:
        return resubmit_v3(state, task, st, rnd, problems)
    lost_v3(task, rnd, got, problems)
    H.write_outcomes(d / "history", outcomes(state, task))
    rnd["episodes"], rnd["problems"] = keys, rnd.get("problems", []) + problems
    rnd["status"] = "collected"
    for p in problems:
        print(f"PROBLEM {task}: {p}", flush=True)
    advance(cfg, state, task, st)
    save(state, task, st)
    return True


def resubmit_v3(state: Path, task: str, st: dict, rnd: dict, problems: list[str]) -> bool:
    """A batch with no scored episode: the same layouts again from the same version (no one
    saw them), with no debrief and nothing counted toward the budget."""
    import shutil

    rnd["resubmits"] = rnd.get("resubmits", 0) + 1
    for j in rnd["jobs"]:
        j.setdefault("lost_jobs", []).append(j.pop("job"))
    for p in (task_dir(state, task) / "episodes").glob(f"r{rnd['n']:02d}_e*"):
        shutil.rmtree(p, ignore_errors=True)
    problems.append(f"round {rnd['n']} has no scored episode: submitted again ({rnd['resubmits']} "
                    f"of {RESUBMITS}), nothing counted toward the budget")
    rnd["problems"] = rnd.get("problems", []) + problems
    rnd["status"] = "planned"
    for p in problems:
        print(f"PROBLEM {task}: {p}", flush=True)
    save(state, task, st)
    return True


def lost_v3(task: str, rnd: dict, got: set, problems: list[str]) -> None:
    """Record the batch's units that were not scored; rnd["lost"] holds their layouts, which a
    top-up batch runs again."""
    lost = sorted((j["corpus"], c, k) for j in rnd["jobs"]
                  for c, k in unit_keys(j["corpus"], task, j["layouts"])
                  if (j["corpus"], c, k) not in got and ("only" not in j or [c, k] in j["only"]))
    if not lost:
        return
    rnd["lost_units"] = [list(x) for x in lost]
    rnd["lost"] = [list(x) for x in sorted({(corpus, k) for corpus, _, k in lost})]
    problems.append(f"round {rnd['n']} lost {len(lost)} of {sum(j['want'] for j in rnd['jobs'])} "
                    f"episodes ({', '.join(f'{c} layout {k}' for _, c, k in lost)}): not counted "
                    "toward the budget; their layouts go to a top-up batch")


def trace_problem(ep_dir: Path) -> str | None:
    """Why the debrief cannot continue this episode's context (no trace, no model request, a body
    that does not rebuild), or None."""
    if not (ep_dir / "trace" / "agent.jsonl").is_file():
        return "no agent trace"
    try:
        D.rebuild(ep_dir)
    except Exception as e:  # noqa: BLE001
        return f"{type(e).__name__}: {e}"
    return None


def conversation_of(results: list[dict], episodes: Path | None = None,
                    prefer_failure: bool = True) -> tuple[str | None, list]:
    """(episode, passed_over): the episode whose context the debrief continues, the round's first
    failure whose trace rebuilds (its first episode when none failed), or None when none does;
    passed_over lists the candidates skipped and why."""
    failed = [r for r in results if not r["success"]]
    passed_over = []
    for r in (failed or results[:1]) if prefer_failure else results:
        why = None if episodes is None else trace_problem(episodes / r["episode"])
        if why is None:
            return r["episode"], passed_over
        passed_over.append((r["episode"], why))
    return None, passed_over


def debrief(cfg: dict, state: Path, task: str, *, profile=None, **kw) -> dict | None:
    """Run the task's due debrief and save its workspace as a version; its record."""
    st = load(state, task)
    rnd = next((r for r in st["rounds"] if r["status"] == "needs_debrief"), None)
    if rnd is None:
        return None
    if (rnd.get("debrief_retry_at") or 0) > now():
        return None
    return debrief_batch(cfg, state, task, st, rnd, profile=profile, **kw)


def guarded(what: str, task: str, fn, *args):
    """fn(*args), or None after printing STEP FAILED: one task's error (a job or file
    hiccup) must not stop the loop for the others; the next poll tries that task again."""
    try:
        return fn(*args)
    except Exception as e:  # noqa: BLE001
        print(f"STEP FAILED {task} ({what}): {type(e).__name__}: {e}", flush=True)
        return None


def advance_task(cfg: dict, state: Path, task: str) -> None:
    st = load(state, task)
    advance(cfg, state, task, st)
    save(state, task, st)


def step(cfg: dict, state: Path, tasks: list[str], *, pool=None, runner=None,
         parallel: int = 8, **debrief_kw) -> None:
    pool = pool or local.LocalPool(cfg)
    runner = runner or local.LocalRunner(cfg["runs"])
    for t in tasks:
        guarded("advance", t, advance_task, cfg, state, t)
        guarded("collect", t, collect, cfg, state, t, pool)
    due = [t for t in tasks
           if any(r["status"] == "needs_debrief" for r in load(state, t)["rounds"])]
    if due:
        def one(t):
            try:
                return debrief(cfg, state, t, **debrief_kw)
            except Exception as e:  # noqa: BLE001
                print(f"DEBRIEF FAILED {t}: {type(e).__name__}: {e}", flush=True)
                return None
        with ThreadPoolExecutor(max(1, min(parallel, len(due)))) as ex:
            list(ex.map(one, due))
    for t in tasks:
        guarded("submit", t, submit, cfg, state, t, runner)


def unit_keys(corpus: str, task: str, layouts: list[int]) -> set[tuple[str, int]]:
    """(config, layout) of the units amp eval runs for this task on these layouts."""
    from amp import benchmarks
    from amp.benchmarks import robodojo as RB

    return {(u.task, RB.address(u.task, u.episode)[1])
            for u in benchmarks.expand(corpus, None, (task,), tuple(layouts))}


def eval_jobs(cfg: dict, state: Path, task: str) -> list[dict]:
    e = cfg["eval"]
    return [{"corpus": e["corpus"], "episodes": e["episodes"], "want": e["episodes"]}]


def eval_submit(cfg: dict, state: Path, task: str, runner=None) -> None:
    runner = runner or local.LocalRunner(cfg["runs"])
    st = load(state, task)
    if st["frozen"] is None:
        return
    st["eval"] = st.get("eval") or {"version": st["frozen"], "jobs": eval_jobs(cfg, state, task)}
    if st["frozen"] != EMPTY:
        held = W.sha_of(version_files(state, task, st["frozen"]))
        if held != st["frozen"]:
            raise RuntimeError(f"{task}: versions/{st['frozen'][:12]} holds {held[:12]}, not the "
                               "frozen version: refusing to evaluate it")
    for job in st["eval"]["jobs"]:
        if job.get("job"):
            continue
        job["job"] = runner(submit_argv(cfg, state, task, job, st["frozen"],
                                        tag_for(cfg, task, "ev")))
        save(state, task, st)
        print(f"{task} eval: {job['job']}", flush=True)


def eval_submit_all(cfg: dict, state: Path, tasks: list[str], runner=None,
                    dry: bool = False) -> str | None:
    """One eval job for these frozen tasks, each episode getting its own; None if all submitted."""
    runner = runner or local.LocalRunner(cfg["runs"])
    todo, entries = [], []
    for t in tasks:
        st = load(state, t)
        if st["frozen"] is None:
            raise RuntimeError(f"{t}: not frozen yet: finish practice first")
        st["eval"] = st.get("eval") or {"version": st["frozen"], "jobs": eval_jobs(cfg, state, t)}
        if all(j.get("job") for j in st["eval"]["jobs"]):
            continue
        v = st["frozen"]
        if v != EMPTY:
            held = W.sha_of(version_files(state, t, v))
            if held != v:
                raise RuntimeError(f"{t}: versions/{v[:12]} holds {held[:12]}, not the frozen "
                                   "version: refusing to evaluate it")
        folder = None if v == EMPTY else (task_dir(state, t) / "versions" / v).relative_to(state)
        entries.append({"task": t, "folder": None if folder is None else folder.as_posix(),
                        "sha": None if v == EMPTY else v})
        todo.append((t, st))
    if not todo:
        return None
    tag = f"{cfg['tag']}_ev"
    if not TAG.fullmatch(tag):
        raise ValueError(f"exp tag {tag!r}: 1-32 of letters, digits, . _ -")
    specs = {(j["corpus"], j["episodes"]) for _, st in todo for j in st["eval"]["jobs"]}
    if len(specs) != 1 or any(len(st["eval"]["jobs"]) != 1 for _, st in todo):
        raise RuntimeError("the tasks' evals differ in corpus or episodes, so one job cannot run "
                           f"them: {sorted(specs, key=str)}")
    spec = todo[0][1]["eval"]["jobs"][0]
    body = (json.dumps(entries, indent=1) + "\n").encode()
    manifest = state / f"{tag}_notebooks_{hashlib.sha256(body).hexdigest()[:16]}.json"
    if not manifest.exists():
        write_bytes_atomic(manifest, body)
    elif manifest.read_bytes() != body:
        raise RuntimeError(f"{manifest} exists with other content: refusing to overwrite the "
                           "manifest an earlier job's argv names")
    argv = local.eval_argv(cfg, tasks=[t for t, _ in todo], corpus=spec["corpus"], exp_tag=tag,
                           episodes=spec["episodes"], workspaces=manifest, notebook=POINTER,
                           max_bytes=cfg["workspace_max_bytes"])
    if dry:
        print(" ".join(map(str, argv)))
        return None
    job = runner(argv)
    for t, st in todo:
        st["eval"]["jobs"][0]["job"] = job
        st["eval"]["manifest"] = manifest.name
        save(state, t, st)
    print(f"eval of {len(todo)} tasks: {job}", flush=True)
    return job


def task_rows(rows: list[dict], task: str) -> list[dict]:
    """The rows of task's configs: one eval job may run many tasks."""
    from amp.benchmarks import robodojo as RB

    return [r for r in rows if RB._lookup(r["task"]).base == task]


def scored(rows: list[dict]) -> dict[tuple[str, int], bool]:
    """{(config, layout): success} of the rows the benchmark scored (infra failures out)."""
    from amp.benchmarks import robodojo as RB

    out = {}
    for r in rows:
        if r.get("status") != "ok" or r.get("attempted") is False or r.get("success") is None:
            continue
        out[(r["task"], RB.address(r["task"], r["episode"])[1])] = bool(r["success"])
    return out


def sign_flip_p(diffs: list[float]) -> float:
    """The exact sign-flip p, by counting the 2^n signed sums instead of listing them,
    so 54 configs or 120 tasks take milliseconds (sums kept in units of 1e-9)."""
    ks = [round(d * 1e9) for d in diffs]
    dist = {0: 1}
    for k in ks:
        nxt: dict[int, int] = {}
        for s, c in dist.items():
            nxt[s + k] = nxt.get(s + k, 0) + c
            nxt[s - k] = nxt.get(s - k, 0) + c
        dist = nxt
    obs = abs(sum(ks)) - len(ks)
    return sum(c for s, c in dist.items() if abs(s) >= obs) / 2 ** len(ks)


def compare(ws: dict, base: dict, units: list[str], *, boot: int = 10_000, seed: int = 0) -> dict:
    """Workspace against one baseline: per reporting unit (config or task) both success rates and
    their difference, the mean difference with its exact sign-flip p and a task-clustered
    bootstrap 95% CI, and the discordant scene pairs."""
    per = {}
    for u in units:
        a = [v for (t, _), v in ws.items() if t == u]
        b = [v for (t, _), v in base.items() if t == u]
        if not a or not b:
            continue
        pairs = [k for k in ws if k[0] == u and k in base]
        per[u] = {"workspace": f"{sum(a)}/{len(a)}", "baseline": f"{sum(b)}/{len(b)}",
                  "diff": sum(a) / len(a) - sum(b) / len(b), "paired": len(pairs),
                  "only_workspace": sum(ws[k] and not base[k] for k in pairs),
                  "only_baseline": sum(base[k] and not ws[k] for k in pairs)}
    diffs = [v["diff"] for v in per.values()]
    rng = random.Random(seed)  # nosec B311
    means = (sorted(sum(rng.choice(diffs) for _ in diffs) / len(diffs) for _ in range(boot))
             if diffs else [])
    ci = [means[int(0.025 * boot)], means[int(0.975 * boot) - 1]] if means else None
    return {"units": per, "mean_diff": sum(diffs) / len(diffs) if diffs else None,
            "sign_flip_p": sign_flip_p(diffs) if diffs else None, "ci95": ci}


def baseline_rows(arg: str, pool) -> list[dict]:
    """A baseline's unit rows: an amp eval run folder of the study's arm."""
    units = Path(arg).expanduser() / str(pool.arm) / "agentic" / "units"
    if not units.is_dir():
        raise SystemExit(f"{arg}: not an amp eval run folder of this study's arm ({pool.arm})")
    return [json.loads(u.read_text()) for u in sorted(units.glob("*.json"))]


def results(cfg: dict, state: Path, baselines: dict[str, list[str]], pool=None,
            check: bool | None = None) -> dict:
    """The study's eval against the baselines."""
    pool = pool or local.LocalPool(cfg)
    check = True if check is None else check
    rows, practice, problems, bad = [], {}, [], set()
    units_of: dict[str, list[dict]] = {}
    for t in cfg["tasks"]:
        st = load(state, t)
        version = (st.get("eval") or {}).get("version") or st["frozen"]
        for j in (st.get("eval") or {}).get("jobs", []):
            if j["job"] not in units_of:
                units_of[j["job"]] = pool.units(j["job"])
            got = task_rows(units_of[j["job"]], t)
            rows += got
            for r in got if check else ():
                text = pool.agent_trace(r)
                why = delivery_problem(text, version, one_task=True)
                if why:
                    problems.append(f"{t}: {r['task']} e{r['episode']}: {why}")
                    bad.add((r["task"], r["episode"]))
        p, f = record(state, t, st["frozen"] or st["version"])
        practice[t] = {"practice_episodes": used(state, t), "status": st["status"],
                       "frozen": st["frozen"], "frozen_record": f"{p}/{p + f}",
                       "debrief_cost_usd": round(sum((r.get("debrief") or {}).get("cost_usd") or 0
                                                     for r in st["rounds"]), 2)}
        practice[t]["decisions"] = decisions_of(st)
    raw = scored(rows)
    delivered = scored([r for r in rows if (r["task"], r["episode"]) not in bad])
    ws = delivered if check else raw
    units = sorted({k[0] for k in raw})

    def rate(got):
        return f"{sum(got[k] for k in got if k[0] in units)}/{sum(1 for k in got if k[0] in units)}"

    out = {"study": cfg["name"], "label": cfg.get("label"), "units": units, "practice": practice,
           "practice_total": sum(v["practice_episodes"] for v in practice.values()),
           "workspace_success": rate(ws), "vs": {}}
    out.update(workspace_success_raw=rate(raw), delivery_checked=check,
               delivery_problems=problems, undelivered=sorted(
                   f"{t} {k}" for t, k in raw.keys() - delivered.keys()) if check else [])
    for name, args in baselines.items():
        base = scored([r for a in args for r in baseline_rows(a, pool)])
        out["vs"][name] = compare(ws, base, units)
        out["vs"][name]["baseline_success"] = f"{sum(base.values())}/{len(base)}"
    return out


def results_md(res: dict) -> str:
    lines = [f"# {res['study']}" + (f" ({res['label']})" if res.get("label") else ""), "",
             (f"Workspace success on the reported units: {res['workspace_success']}; practice "
              f"episodes used: {res['practice_total']}."), ""]
    if res.get("delivery_checked"):
        lines += [(f"Counted on the units whose agent got the frozen version; all scored units: "
                   f"{res['workspace_success_raw']}, undelivered: {len(res['undelivered'])}."), ""]
        lines += [f"- DELIVERY PROBLEM {p}" for p in res["delivery_problems"]]
        lines += [""] if res["delivery_problems"] else []
    else:
        lines += [f"- DELIVERY PROBLEM {p}" for p in res.get("delivery_problems") or []]
    lines += ["| task | status | practice episodes | frozen record | debrief cost |",
              "|---|---|---|---|---|"]
    for t, v in res["practice"].items():
        lines.append(f"| {t} | {v['status']} | {v['practice_episodes']} | {v['frozen_record']} | "
                     f"${v['debrief_cost_usd']} |")
    for name, c in res["vs"].items():
        lines += ["", f"## workspace minus {name}"
                  + (f" (baseline {c['baseline_success']})" if c.get("baseline_success")
                     else ""), "",
                  "| unit | workspace | baseline | difference | only workspace | only baseline |",
                  "|---|---|---|---|---|---|"]
        for u, v in c["units"].items():
            lines.append(f"| {u} | {v['workspace']} | {v['baseline']} | {100 * v['diff']:+.0f}pp | "
                         f"{v['only_workspace']} | {v['only_baseline']} |")
        md = c["mean_diff"]
        ci = c["ci95"]
        lines += ["", "Mean per-unit difference: " + ("n/a" if md is None else
                  f"{100 * md:+.1f}pp, exact sign-flip p = {c['sign_flip_p']:.3g}, task-clustered "
                  f"bootstrap 95% CI [{100 * ci[0]:+.1f}, {100 * ci[1]:+.1f}]pp.")]
    return "\n".join(lines) + "\n"


def workspace_events(text: str) -> list[dict]:
    """The agent.workspace events' data of an agent trace's text."""
    out = []
    for line in text.splitlines():
        if '"agent.workspace"' in line:
            e = json.loads(line)
            if e.get("kind") == "agent.workspace" and isinstance(e.get("data"), dict):
                out.append(e["data"])
    return out


def delivery_problem(text: str | None, version: str, one_task: bool = False) -> str | None:
    """Why an episode's agent did not get its task's version, or None."""
    if text is None:
        return "no agent trace"
    got = workspace_events(text)
    if one_task and version == EMPTY:
        shas = [d["sha"] for d in got if isinstance(d.get("sha"), str)]
        return (f"a workspace {shas[0][:12]} reached an episode that should have none"
                if shas else None)
    if not got:
        return "no agent.workspace event"
    d = got[0]
    if not d.get("found", one_task):
        return f"no workspace for its instruction {d.get('key')!r}"
    if d.get("sha") == version:
        return None
    return f"workspace {str(d.get('sha'))[:12]} != {version[:12]}"


def init(state: Path, study: Path) -> dict:
    """The study's config.json in STATE: the study file over DEFAULTS, checked; "tasks": "all" is
    every task of the practice corpus; the runs folder resolves against the working directory."""
    from amp.eval.robodojo_official import WORKSPACE_MAX_BYTES_LIMIT

    raw = json.loads(study.read_text())
    cfg = {**DEFAULTS, **raw}
    cfg["debrief"] = {**DEFAULTS["debrief"], **raw.get("debrief", {})}
    cfg["study_file"] = str(study.resolve())
    cfg.setdefault("tag", cfg["name"][:20])
    for k in ("harness", "model", "practice", "eval"):
        if k not in cfg:
            raise SystemExit(f"{study}: no {k!r}")
    if not {"corpus", "episodes"} <= cfg["eval"].keys():
        raise SystemExit(f"{study}: eval needs a corpus and episodes")
    if cfg.get("tasks", "all") != "all":
        raise SystemExit(f"{study}: tasks is \"all\" (every task of the practice corpus)")
    from amp import benchmarks

    cfg["runs"] = str(Path(cfg.get("runs") or "runs/practice").expanduser().resolve())
    cfg["tasks"] = benchmarks.corpus_tasks(cfg["practice"]["corpora"][0])
    if "workspace_format" in cfg and cfg["workspace_format"] not in WORKSPACE_FORMATS:
        raise SystemExit(f"workspace_format {cfg['workspace_format']!r}: code or notes")
    if cfg["batch"] < 1 or cfg["batch"] > 1 and cfg["batch"] % 2 or cfg["budget"] < cfg["batch"]:
        raise SystemExit(f"batch {cfg['batch']} must be even (or 1) and at most the budget "
                         f"{cfg['budget']}")
    if not W.MAX_BYTES <= cfg["workspace_max_bytes"] <= WORKSPACE_MAX_BYTES_LIMIT:
        raise SystemExit(f"workspace_max_bytes {cfg['workspace_max_bytes']}: from "
                         f"{W.MAX_BYTES} to {WORKSPACE_MAX_BYTES_LIMIT}")
    clash = set(cfg["practice"]["corpora"]) & {cfg["eval"]["corpus"]}
    if clash:
        raise SystemExit(f"practice and eval share a seed corpus: {sorted(clash)}")
    for t in cfg["tasks"]:
        practice_queue(cfg, t)
        if batches_total(cfg, t) < 1:
            raise SystemExit(f"{t}: no whole batch of {cfg['batch']} in its practice layouts")
    tag_for(cfg, cfg["tasks"][-1], "r99abc")
    local.arm_id(cfg)
    state.mkdir(parents=True, exist_ok=True)
    (state / "config.json").write_text(json.dumps(cfg, indent=1))
    return cfg


def wait_jobs(pool, jobs: list[str], every: float) -> None:
    """Return once every job ended (one-shot commands: their jobs run in this process)."""
    while True:
        left = [j for j in jobs if pool.job_state(j) not in TERMINAL]
        if not left:
            return
        print(f"waiting for {len(left)} job(s): {', '.join(left)}", flush=True)
        time.sleep(every)


def main(argv: list[str] | None = None) -> int:
    from amp.practice import fewshot_eval as FE

    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] in (["eval"], ["select"]):
        return FE.main(argv)
    ap = argparse.ArgumentParser(prog="amp practice fewshot", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["init", "run", "results"])
    ap.add_argument("state", type=Path, help="the study's state folder")
    ap.add_argument("--study", type=Path, help="init: the study file")
    ap.add_argument("--task", help="run: only this task")
    ap.add_argument("--every", type=int, default=60, help="run: seconds between polls")
    ap.add_argument("--none", help="results: the no-memory baseline (an amp eval run folder)")
    ap.add_argument("--check-delivery", action=argparse.BooleanOptionalAction, default=None,
                    help="results: read every unit's agent trace and count only the units whose "
                    "agent got its task's frozen version (default on)")
    ap.add_argument("--shard", help="run: I/N, only tasks I, I+N, ... of the study (0-based)")
    ap.add_argument("--eval", action="store_true",
                    help="run: also submit each task's eval job as soon as it is frozen")
    a = ap.parse_args(argv)
    a.state = a.state.resolve()
    if a.cmd == "init":
        if not a.study:
            ap.error("init needs --study")
        init(a.state, a.study)
        print(a.state / "config.json")
        return 0
    if a.cmd == "results" and not (a.state / "config.json").is_file():
        FE.curve_results(a.state, a.none, a.check_delivery)
        return 0
    cfg = config(a.state)
    tasks = [a.task] if a.task else cfg["tasks"]
    if a.shard:
        i, _, n = a.shard.partition("/")
        if not (i.isdigit() and n.isdigit() and int(i) < int(n)):
            ap.error(f"--shard {a.shard!r}: I/N with 0 <= I < N")
        tasks = tasks[int(i)::int(n)]
    runner, pool = local.LocalRunner(cfg["runs"]), local.LocalPool(cfg)
    if a.cmd == "run":
        while True:
            step(cfg, a.state, tasks, pool=pool, runner=runner)
            if a.eval:
                eval_frozen(cfg, a.state, tasks, runner=runner)
            sts = [load(a.state, t)["status"] for t in tasks]
            print(f"{time.strftime('%H:%M:%S')} {sts.count('practising')} of {len(tasks)} tasks "
                  "practising", flush=True)
            if all(load(a.state, t)["status"] != "practising" for t in tasks):
                pending = eval_pending(a.state, tasks) if a.eval else []
                if not pending:
                    if a.eval:
                        wait_jobs(pool, [j["job"] for t in tasks
                                         for j in (load(a.state, t).get("eval") or {}).get(
                                             "jobs", []) if j.get("job")], a.every)
                    n = sum(1 for t in tasks if load(a.state, t)["frozen"] is not None)
                    print("every task is frozen" + (f"; eval jobs done for all {n} frozen "
                                                    "tasks" if a.eval else ": run again with --eval"))
                    return 0
                print(f"EVAL NOT SUBMITTED yet: {', '.join(pending)}; trying again in "
                      f"{a.every} s", flush=True)
            time.sleep(a.every)
    elif a.cmd == "results":
        FE.results_command(cfg, a.state, a.none, pool, a.check_delivery)
    return 0
