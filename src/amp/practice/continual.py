#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Continual shared notebook: one notebook learned over every RoboDojo task one task at a time,
over several passes, each in a new random order, with checkpoint evals along the way
(docs/practice.md).

  amp practice continual init STATE --study STUDY.json [--chains 1]
  amp practice continual step STATE   one pass: submit due checkpoint evals, collect a finished
        round, run its debriefs one after another, submit the next round (or freeze after the
        last)
  amp practice continual run STATE [--every 120] [--wait]   step until learning is over and every
        checkpoint eval is submitted; with --wait, wait for the evals and write the results too.
        Resumable.
  amp practice continual results STATE [--none RUN] [--no-check-delivery]
  amp practice continual full-eval STATE --steps 10,20,30,42 --corpora robodojo-seed2
        [--episodes 50] [--dry-run]   the full evaluation: each listed
        checkpoint's notebook version on every task of each corpus, one job per (checkpoint,
        corpus), recorded under the checkpoint (results reports it against the study's
        full_baselines, on every unit as the benchmark does); with a RoboDojo full evaluation,
        results also writes Figure 4 I-K's data as the paper computes it (paper_scores.py):
        memory_curve_seedN.csv, efficiency_seedN.csv and matrix_seedN.csv

Learning: the study's tasks run in `epochs` passes, each pass in its own order (the sorted tasks
shuffled by random.Random("<seed>:<pass>")). A place is one task of one pass; rounds of
`round_size` consecutive places of a pass run as one job from the current notebook version, on
that pass's layout (`layouts`, amp eval robodojo --layouts) or episode 0. Every episode is
debriefed in order: its agent, continued from its own conversation, gets its official result, its
efficiency, the goal, the embodiment fact and the read-only history of every learning episode so
far, and edits the notebook as the previous debrief left it. A Generalization task runs both its
configs in its place; one debrief speaks for both.

Checkpoints: every `eval.every` places learned, and once more when learning ends, the version
of that moment is evaluated on every task learned so far (eval.corpus x eval.episodes, one job,
not waited for). With `eval.epoch_end`, every pass's end is a checkpoint too. With `eval.unseen`,
a second job evaluates the same version on the tasks not learned yet, so results report seen and
unseen tasks apart. Results pair each checkpoint with a no-memory baseline run on the same units
(the study's `baseline`, or `results --none`): success (primary) and efficiency on units
successful in both; with no baseline, or one that does not exist yet, each checkpoint's own
success.

State: STATE/config.json (the study, seed, orders, configs), state.json (rounds, version, frozen,
checkpoints), versions/, episodes/<key>/ (raw, never shown), history/ (the debriefs' read-only
view; key e<place>[b] with no task name, and outcomes.csv), debriefs/<key>/, history_view/,
driver.lock and results.{json,md}. Jobs run under the study's runs folder (amp.practice.local).
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import shutil
import sys
import time
from pathlib import Path

from amp.practice import fewshot as LP
from amp.practice import history as H
from amp.practice import local
from amp.practice import paper_scores as PS
from amp.practice import shared as S
from amp.practice import workspace as W

EMPTY = LP.EMPTY
log = S.log
DEFAULTS = {"round_size": 1, "epochs": 1, "prompt": "amp1", "gpus": "auto",
            "max_bytes": W.MAX_BYTES, "eval": {"episodes": 5, "every": 10},
            "debrief": {"max_turns": 60, "max_cost_usd": 20.0}}
FIELDS = ("episode", "instruction", "workspace_sha", "success", "end_reason", "steps_used",
          "step_budget", "turns", "cost_usd", "wall_s", "result")
GOAL = "Later episodes of any task should succeed."
LEARN = ("Whether this episode succeeded or failed, it can teach later episodes: what made it work "
         "is as worth keeping as what went wrong.")


def epoch_order(tasks: list[str], seed: int, e: int) -> list[str]:
    """Pass e's order: the sorted tasks shuffled by random.Random("<seed>:<e>")."""
    order = sorted(tasks)
    random.Random(f"{seed}:{e}").shuffle(order)  # nosec B311
    return order


def configs_of(corpus: str, tasks: list[str]) -> dict[str, list[str]]:
    """Each task's configs, its own first: a Generalization task has two."""
    from amp.benchmarks import robodojo as RB

    out: dict[str, list[str]] = {}
    for t in RB.tasks_of(corpus):
        if t.base in tasks:
            out.setdefault(t.base, []).append(t.name)
    return {b: sorted(cs, key=lambda c: c != b) for b, cs in out.items()}


def tag(cfg: dict, what: str) -> str:
    return f"{cfg['tag']}_{what}"


def n_places(cfg: dict) -> int:
    return len(cfg["order"]) * len(cfg["tasks"])


def epoch_of(cfg: dict, p: int) -> int:
    """The pass (0-based) of place p (1-based)."""
    return (p - 1) // len(cfg["tasks"])


def task_at(cfg: dict, p: int) -> str:
    return cfg["order"][epoch_of(cfg, p)][(p - 1) % len(cfg["tasks"])]


def key_of(cfg: dict, p: int, config: str) -> str:
    """A learning episode's folder name: its place and, for a task's second config, b."""
    i = cfg["configs"][task_at(cfg, p)].index(config)
    return f"e{p:03d}" + ("" if i == 0 else chr(ord("a") + i))


def place_of(key: str) -> int:
    return int(key[1:4])


def run_path(here: Path, spec: str) -> str:
    """A reference run as the study names it: an amp eval run folder or rows file, made absolute;
    a job id under the runs folder as it is."""
    p = here / Path(spec).expanduser()
    return str(p.resolve()) if "/" in spec or p.exists() else spec


def init(state: Path, study: Path, **override) -> dict:
    """Write STATE/config.json: the study (with override's fields), its seed, every pass's order,
    each task's configs, and its runs folder and reference runs made absolute (relative paths are
    the working directory's)."""
    from amp import benchmarks

    raw = {**json.loads(study.read_text()), **override}
    cfg = {**DEFAULTS, **raw, "eval": {**DEFAULTS["eval"], **raw.get("eval", {})},
           "debrief": {**DEFAULTS["debrief"], **raw.get("debrief", {})}}
    missing = [k for k in ("name", "corpus", "harness", "model") if not cfg.get(k)]
    if missing:
        raise SystemExit(f"the study names no {', '.join(missing)}")
    if not str(cfg["corpus"]).startswith("robodojo"):
        raise SystemExit(f"corpus {cfg['corpus']!r}: the continual study runs on RoboDojo")
    corpus_tasks = benchmarks.corpus_tasks(cfg["corpus"])
    tasks = corpus_tasks if cfg.get("tasks") in (None, "all") else list(cfg["tasks"])
    unknown = sorted(set(tasks) - set(corpus_tasks))
    if unknown:
        raise SystemExit(f"not tasks of {cfg['corpus']}: {', '.join(unknown[:5])}")
    if int(cfg["round_size"]) < 1 or int(cfg["epochs"]) < 1:
        raise SystemExit("round_size and epochs must be at least 1")
    if cfg.get("layouts") is not None and len(cfg["layouts"]) != int(cfg["epochs"]):
        raise SystemExit(f"layouts names {len(cfg['layouts'])} layouts for {cfg['epochs']} epochs")
    if cfg.get("seed") is None:
        cfg["seed"] = random.SystemRandom().randrange(2 ** 32)
    tasks = sorted(tasks)
    cfg.update(tasks=tasks, study_file=str(study.resolve()),
               runs=str(Path(cfg.get("runs", "runs/practice")).expanduser().resolve()),
               configs=configs_of(cfg["corpus"], tasks),
               order=[epoch_order(tasks, int(cfg["seed"]), e) for e in range(int(cfg["epochs"]))])
    cfg.setdefault("tag", cfg["name"][:20])
    here = Path.cwd()
    cfg["baseline"] = run_path(here, cfg["baseline"]) if cfg.get("baseline") else None
    cfg["full_baselines"] = {n: run_path(here, v) for n, v in
                             (cfg.get("full_baselines") or {}).items()}
    worst = n_places(cfg) + S.MAX_TOPUP_TASKS
    for t in (tag(cfg, f"r{worst:03d}a{S.MAX_ROUND_ATTEMPTS}"), tag(cfg, f"ck{n_places(cfg):03d}")):
        if not LP.TAG.fullmatch(t):
            raise SystemExit(f"exp tag {t!r}: 1-32 of letters, digits, . _ -")
    old = state / "config.json"
    if old.is_file() and json.loads(old.read_text())["order"] != cfg["order"]:
        raise SystemExit(f"{old} holds other learning orders: use a new state dir")
    state.mkdir(parents=True, exist_ok=True)
    old.write_text(json.dumps(cfg, indent=1))
    return cfg


def init_chains(root: Path, study: Path, chains: int = 1) -> dict:
    """root/chain_00/: the paper's chain, its seed derived from the study's (or a drawn one),
    recorded in root/chains.json."""
    if chains != 1:
        raise SystemExit("--chains 1: one chain (chain_00, the paper's), not several")
    raw = json.loads(study.read_text())
    old = root / "chains.json"
    prev = json.loads(old.read_text()) if old.is_file() else None
    base = raw.get("seed")
    base = prev["base_seed"] if prev else (
        random.SystemRandom().randrange(2 ** 32) if base is None else int(base))
    seeds = S.chain_seeds(base, 1)
    if prev is not None and seeds != prev["seeds"]:
        raise SystemExit(f"{old}: its seeds are not this sequence's")
    d = root / "chain_00"
    if prev is None:
        init(d, study, seed=seeds[0], name=f"{raw['name']}_c00",
             tag=f"{raw.get('tag') or raw['name'][:16]}c00")
    root.mkdir(parents=True, exist_ok=True)
    out = {"study": str(study), "base_seed": base, "chains": 1, "seeds": seeds, "dirs": [d.name]}
    old.write_text(json.dumps(out, indent=1))
    return out


def load(state: Path) -> dict:
    st = S.load(state)
    st.setdefault("checkpoints", [])
    return st


def units_of(cfg: dict, tasks: list[str], *, episodes: int | None = None,
             layouts: list[int] | None = None) -> int:
    """Units a job of these tasks holds: episodes per task, or every layout on every config."""
    if layouts is not None:
        return sum(len(cfg["configs"][t]) for t in tasks) * len(layouts)
    return len(tasks) * int(episodes)


def submit_argv(cfg: dict, state: Path, tasks: list[str], version: str, exp_tag: str,
                corpus: str, *, episodes: int | None = None, layouts: list[int] | None = None,
                workspace: Path | None = None) -> list[str]:
    """amp eval robodojo's argv (local.eval_argv); workspace: the notebook folder of version, in
    place of versions/<sha>."""
    ws = None if version == EMPTY else (workspace or S.version_dir(state, version))
    return local.eval_argv(cfg, tasks=tasks, corpus=corpus, episodes=episodes, layouts=layouts,
                           workspace=ws, sha=None if ws is None else version, shared=ws is not None,
                           exp_tag=exp_tag)


def round_layouts(cfg: dict, rnd: dict) -> list[int] | None:
    return None if cfg.get("layouts") is None else [cfg["layouts"][rnd["epoch"] - 1]]


def round_argv(cfg: dict, state: Path, rnd: dict) -> list[str]:
    a = int(rnd.get("attempt", 1))
    t = tag(cfg, f"r{rnd['n']:03d}" + (f"a{a}" if a > 1 else ""))
    lay = round_layouts(cfg, rnd)
    return submit_argv(cfg, state, rnd["tasks"], rnd["version"], t, cfg["corpus"],
                       episodes=None if lay else 1, layouts=lay)


def ck_tag(cfg: dict, ck: dict) -> str:
    return tag(cfg, f"ck{ck['step']:03d}")


def ck_argv(cfg: dict, state: Path, ck: dict) -> list[str]:
    e = cfg["eval"]
    return submit_argv(cfg, state, ck["tasks"], ck["version"], ck_tag(cfg, ck), e["corpus"],
                       episodes=e["episodes"])


def submit_job(state: Path, st: dict, rec: dict, argv: list[str], exp_tag: str, pool,
               runner) -> str:
    """Submit once: a submit left unfinished (rec["submitting"]) is looked up by its tag first."""
    find = getattr(pool, "find_job", None)
    job = find(exp_tag, rec["submitting"] - 300) if rec.get("submitting") and find else None
    if job:
        log(f"{exp_tag}: adopted {job}, already submitted", flush=True)
    else:
        rec["submitting"] = S.now()
        S.save(state, st)
        job = runner(argv)
    rec.update(job=job, submitted=S.now())
    S.save(state, st)
    return job


def lost_pending(cfg: dict, st: dict) -> tuple[list[int], list[tuple[int, str]]]:
    """Places lost to infra a top-up round should run again, and those that may not (why)."""
    have = {place_of(k) for r in st["rounds"] for k in r["episodes"]}
    tries: dict[int, int] = {}
    for r in st["rounds"]:
        for p in r["places"]:
            tries[p] = tries.get(p, 0) + 1
    lost = list(dict.fromkeys(p for r in st["rounds"] for p in r.get("lost", []) if p not in have))
    room = S.MAX_TOPUP_TASKS - sum(len(r["places"]) for r in st["rounds"] if r.get("topup"))
    take, drop = [], []
    for p in lost:
        if tries[p] >= S.MAX_TASK_ATTEMPTS:
            drop.append((p, f"lost to infra in {tries[p]} rounds"))
        elif len(take) >= max(room, 0):
            drop.append((p, f"the chain's {S.MAX_TOPUP_TASKS} top-up units are used"))
        else:
            take.append(p)
    return take, drop


def plan(cfg: dict, st: dict) -> dict | None:
    """The next round from the current version: a top-up of places lost to infra (one pass's),
    else the next places of the current pass; None after the last."""
    size = int(cfg["round_size"])
    again, drop = lost_pending(cfg, st)
    dropped = {d["place"] for d in st.setdefault("dropped", [])}
    for p, why in drop:
        if p not in dropped:
            st["dropped"].append({"place": p, "task": task_at(cfg, p), "why": why})
            log(f"PROBLEM: place {p} dropped from learning: {why}", flush=True)
    if again:
        e = epoch_of(cfg, again[0])
        places = [p for p in again if epoch_of(cfg, p) == e][:size]
        log(f"PROBLEM: top-up round {len(st['rounds']) + 1} runs places {places} again (lost "
            f"to infra) from version {st['version'][:12]}", flush=True)
    else:
        done = {p for r in st["rounds"] if not r.get("topup") for p in r["places"]}
        nxt = next((p for p in range(1, n_places(cfg) + 1) if p not in done), None)
        if nxt is None:
            return None
        e = epoch_of(cfg, nxt)
        places = list(range(nxt, min(nxt + size, (e + 1) * len(cfg["tasks"]) + 1)))
    rnd = {"n": len(st["rounds"]) + 1, "epoch": e + 1, "places": places,
           "tasks": [task_at(cfg, p) for p in places], "version": st["version"],
           "status": "planned", "job": None, "episodes": [], "missing": [], "lost": [],
           "problems": [], "debriefs": {}, "timing": {"planned": S.now()}}
    if again:
        rnd["topup"] = True
    st["rounds"].append(rnd)
    return rnd


def submit(cfg: dict, state: Path, st: dict, rnd: dict, pool, runner) -> None:
    argv = round_argv(cfg, state, rnd)
    rnd["status"] = "submitting"
    rnd["timing"]["submitting"] = S.now()
    S.save(state, st)
    rnd["job"] = runner(argv)
    rnd["status"] = "submitted"
    rnd["timing"]["submitted"] = S.now()
    S.save(state, st)
    log(f"round {rnd['n']} (pass {rnd['epoch']}, places {rnd['places']}"
        + (", top-up" if rnd.get("topup") else "") + f", version {rnd['version'][:12]}, attempt "
        f"{rnd.get('attempt', 1)}): {rnd['job']}", flush=True)


def adopt(cfg: dict, state: Path, st: dict, rnd: dict, pool) -> None:
    """A round left "submitting": adopt the job its tag already names, else plan it again."""
    t = local.split_tag(round_argv(cfg, state, rnd))[1]
    find = getattr(pool, "find_job", None)
    job = find(t, rnd["timing"]["submitting"] - 300) if find else None
    if job:
        rnd.update(job=job, status="submitted")
        rnd["timing"]["submitted"] = S.now()
        log(f"round {rnd['n']}: adopted {job}, already submitted as {t}", flush=True)
    else:
        rnd["status"] = "planned"
        log(f"round {rnd['n']}: no job tagged {t}: submitting it", flush=True)
    S.save(state, st)


def collect(cfg: dict, state: Path, st: dict, rnd: dict, pool) -> None:
    """Collect the round once its job has finished (shared.collect, per config of each place): a
    place with no scored episode is lost to infra, one with some is kept as it is."""
    job = rnd["job"]
    want = units_of(cfg, rnd["tasks"], episodes=1, layouts=round_layouts(cfg, rnd))
    n = pool.count(job)
    ended = None
    if n < want:
        ended = LP.job_ended_short(pool, {"job": job})
        if ended is None:
            return
    try:
        rows = {r["task"]: r for r in pool.units(job)}
    except (FileNotFoundError, SystemExit):
        if ended is None:
            raise
        rows = {}
    expected = [(p, c, key_of(cfg, p, c)) for p in rnd["places"]
                for c in cfg["configs"][task_at(cfg, p)]]
    fetched, short = {}, []
    for _, c, key in expected:
        if not S.scored(rows.get(c)):
            continue
        ep = state / "episodes" / key
        S.claim(ep, job)
        fetched[key] = pool.fetch(rows[c], ep)
        if fetched[key]:
            short.append(key)
    polls = rnd.setdefault("fetch_polls", {})
    if short:
        for k in short:
            polls[k] = polls.get(k, 0) + 1
        if any(polls[k] < S.FETCH_POLLS for k in short):
            S.save(state, st)
            log(f"round {rnd['n']}: fetch incomplete for {', '.join(short)} (poll "
                f"{max(polls[k] for k in short)} of {S.FETCH_POLLS}): trying again", flush=True)
            return
    rnd["episodes"], rnd["missing"], rnd["lost"], rnd["problems"] = [], [], [], []
    rnd["debrief_keys"], rnd["siblings"] = [], {}
    if ended:
        rnd["problems"].append(f"job {job} ended {ended} with {n} of {want} unit files: "
                               "collecting what exists")
    for p in rnd["places"]:
        got, why = [], []
        for pp, c, key in expected:
            if pp != p:
                continue
            row = rows.get(c)
            if not S.scored(row):
                why.append((row or {}).get("status", "no unit"))
                continue
            ep = state / "episodes" / key
            missing = fetched[key]
            if missing and not (ep / H.ROBOT_RESULT).is_file():
                why.append("not fetched")
                continue
            if missing:
                rnd["problems"].append(f"{key}: {', '.join(missing[:2])} not fetched after "
                                       f"{S.FETCH_POLLS} polls: taken as it is")
            sha = rnd["version"]
            bad = LP.check_delivery(ep, sha)
            if bad:
                sha = LP.delivered_sha(ep)
                rnd["problems"].append(f"{key}: {bad}; recorded as {sha[:12]}")
            o = H.build(ep, state / "history" / key, episode=key, workspace_sha=sha)
            robot = json.loads((ep / H.ROBOT_RESULT).read_text())
            eff = {k: v for k, v in S.efficiency(row, None, robot).items() if k != "steps"}
            if any(v is None for v in eff.values()):
                rnd["problems"].append(f"{key}: efficiency {S.UNKNOWN} for "
                                       f"{', '.join(k for k, v in eff.items() if v is None)}")
            o.update(instruction=S.instruction_of(ep), **eff)
            (state / "history" / key / "outcome.json").write_text(json.dumps(o, indent=1))
            got.append(key)
        if not got:
            rnd["missing"].append(p)
            if all(w in S.NOT_RERUN for w in why):
                rnd["problems"].append(f"place {p}: no scored learning episode ({', '.join(why)}, "
                                       "not run again); no debrief")
            else:
                rnd["lost"].append(p)
                rnd["problems"].append(f"place {p}: no scored learning episode ({', '.join(why)}): "
                                       "lost to infra, to be run again")
        elif why:
            rnd["problems"].append(f"place {p}: {len(why)} of its configs not scored "
                                   f"({', '.join(why)}): the others are kept, not run again")
        rnd["episodes"] += got
        if got:
            pick, others = debriefed(state, got)
            rnd["debrief_keys"].append(pick)
            rnd["siblings"][pick] = others
            for k in others:
                transcript_only(state / "history" / k)
    H.write_outcomes(state / "history", [S.shown(o) for o in S.outcomes(state)], FIELDS)
    for x in rnd["problems"]:
        log(f"PROBLEM round {rnd['n']}: {x}", flush=True)
    if not rnd.get("topup") and not rnd["episodes"] and len(rnd["lost"]) == len(rnd["places"]):
        attempt = int(rnd.get("attempt", 1))
        rnd.setdefault("prior_jobs", []).append({"job": job, "attempt": attempt,
                                                 "problems": rnd["problems"]})
        rnd.update(status="planned", job=None, attempt=attempt + 1, fetch_polls={}, missing=[],
                   lost=[], problems=[])
        if attempt >= S.MAX_ROUND_ATTEMPTS:
            rnd["halted"] = (f"round {rnd['n']} lost every place to infra in {attempt} "
                             f"submissions (last {job})")
            S.save(state, st)
            raise S.ChainFailed(rnd["halted"] + ": fix the cause, then run this chain again")
        S.save(state, st)
        log(f"PROBLEM round {rnd['n']}: every place lost to infra: submitting the same places "
            f"from the same version again (attempt {attempt + 1} of {S.MAX_ROUND_ATTEMPTS})",
            flush=True)
        return
    rnd["status"] = "collected"
    rnd["timing"]["collected"] = S.now()
    S.save(state, st)


def outcome(state: Path, key: str) -> dict:
    return json.loads((state / "history" / key / "outcome.json").read_text())


def debriefed(state: Path, keys: list[str]) -> tuple[str, list[str]]:
    """The one episode of a place whose agent is debriefed, and its siblings: a failed one before
    a successful one, then more agent turns, then more simulator steps; one whose trace does not
    rebuild only if none does."""
    def rank(k):
        o = outcome(state, k)
        return (bool(o["success"]), -(o.get("turns") or 0), -(o.get("steps_used") or 0))

    ok = [k for k in keys if LP.trace_problem(state / "episodes" / k) is None] or keys
    pick = min(ok, key=rank)
    return pick, [k for k in keys if k != pick]


def transcript_only(ep: Path) -> None:
    """A sibling's history folder keeps only its transcript and outcome."""
    for p in ep.iterdir():
        if p.name not in ("transcript.md", "outcome.json"):
            shutil.rmtree(p) if p.is_dir() else p.unlink()


SHARED_FACT = ("This notebook is shared by every task of the benchmark. Later episodes, and the "
               "evaluations, may be of tasks you have seen or of tasks you have not; what you keep "
               "here should help with any task, so they succeed.")


def message(cfg: dict, o: dict, current: str, siblings=()) -> str:
    """The debrief's one user message: the episode's facts, the embodiment and learning from
    success and failure alike; success is its only goal, and nothing about the study's plan."""
    started = o.get("workspace_sha") or EMPTY
    ver = ("an empty notebook" if started == EMPTY else "no notebook (it did not reach the episode)"
           if started == LP.UNDELIVERED else f"notebook version {started[:12]}")
    now_ = "empty" if current == EMPTY else f"version {current[:12]}"
    wall = o.get("wall_s")
    lines = [
        f"Official benchmark result of this episode: {o['result']}",
        (f"Efficiency of this episode, from its trace: {S.known(o.get('turns'))} agent turns "
         f"(model calls), {S.known(o.get('steps_used'))} simulator steps, "
         f"{S.money(o.get('cost_usd'))} model cost, "
         f"{S.UNKNOWN if wall is None else f'{round(wall)} s'} wall-clock time."), "",
        ("This is a debrief, outside any episode: nothing you do now moves the robot or changes "
         f"this result. This episode started from {ver}. ./workspace is the notebook as the "
         f"previous debrief left it ({now_}), and it is yours: notes, code, data, in any form and "
         "organisation. Every later episode, of any task, and every evaluation start with the "
         "notebook exactly as you leave it, as files in ./workspace, with one line in their first "
         f"message pointing to it. About {int(cfg['max_bytes']) // 1000} KB fits; a file you "
         "delete is gone from later episodes."), "",
        f"{GOAL} {SHARED_FACT}"]
    if cfg.get("embodiment"):
        lines += ["", cfg["embodiment"]]
    for sib in siblings:
        lines += ["", ("The same task also ran once more, at the same time and from the same "
                       "notebook, on a variant of this scene: its official result was "
                       f"{sib['result']}, and history/{sib['episode']}/transcript.md is its "
                       "transcript. This one debrief speaks for both episodes.")]
    lines += [
        "", LEARN, "",
        ("./history holds every learning episode so far, of every task, read-only: "
         "history/outcomes.csv (one row per episode: its instruction, the "
         "notebook version it started from, its official result, and its agent turns, simulator "
         "steps, model cost and wall-clock time) and, per episode, history/<episode>/transcript.md "
         "(every turn's text, code, output and errors), frames/ (camera frames by step), images/ "
         "(images the agent was shown), report.json (its report) and outcome.json. This episode "
         f"is history/{o['episode']}/."), "",
        ("Tools now: python runs code in a fresh Python with no robot, in a directory holding "
         "workspace/ (writable), history/ (read-only), tmp/ and images/ (scratch); view_image "
         "shows an image file under it; done(summary) ends the debrief and saves the notebook as "
         "it stands, changed or not. The notebook may not name a benchmark task: done() checks "
         "this and says what to fix.")]
    return "\n".join(lines)


def task_names(cfg: dict) -> list[str]:
    """Every task and config name of the study's corpora: no saved notebook may hold one."""
    from amp import benchmarks

    names = {*cfg["tasks"], *(c for cs in cfg["configs"].values() for c in cs)}
    for corpus in (cfg["corpus"], cfg["eval"]["corpus"]):
        names |= set(benchmarks.corpus_tasks(corpus))
        names |= {c for cs in configs_of(corpus, benchmarks.corpus_tasks(corpus)).values()
                  for c in cs}
    return sorted(names)


def debrief_one(cfg: dict, state: Path, st: dict, rnd: dict, key: str, *, profile=None,
                **kw) -> dict:
    """One debrief with this study's message, names and size limit, recorded in the round."""
    from amp.practice import debrief as D

    t0 = S.now()
    o = json.loads((state / "history" / key / "outcome.json").read_text())
    why = LP.trace_problem(state / "episodes" / key)
    if why is not None:
        rec = {"skipped": f"the conversation does not rebuild: {why}", "version": st["version"]}
        log(f"PROBLEM round {rnd['n']}: {key}'s debrief skipped: {why}", flush=True)
    else:
        profile = profile or local.profile(cfg)
        before = st["version"]
        common = dict(trace=state / "episodes" / key, conversation=key, profile=profile,
                      names=task_names(cfg),
                      max_turns=cfg["debrief"]["max_turns"],
                      max_cost_usd=cfg["debrief"]["max_cost_usd"], private=[state],
                      max_bytes=int(cfg["max_bytes"]), **kw)
        sibs = [outcome(state, k) for k in (rnd.get("siblings") or {}).get(key, [])]
        got = D.run(state / "debriefs" / key, results=[o, *sibs],
                    workspace_files=S.version_files(state, before),
                    workspace_sha=None if before == EMPTY else before,
                    history=state / "history", message=message(cfg, o, before, sibs),
                    done_tool=S.DONE_TOOL, history_mirror=state / "history_view", **common)
        if got["end_reason"] == "api_error" or not got["turns"]:
            raise S.DebriefFailed(f"the debrief ended {got['end_reason']} after {got['turns']} "
                                  "turns: nothing to record", got.get("cost_usd") or 0.0)
        rec = {k: v for k, v in got.items() if k != "files"}
        meta = {"round": rnd["n"], "episode": key, "summary": got["summary"],
                "cost_usd": got["cost_usd"]}
        rec["changed"] = bool(got["saved"]) and got["sha"] != before
        if rec["changed"]:
            st["version"] = S.store_version(state, got["files"], before, meta)
        rec["version"] = st["version"]
    rec["timing"] = {"start": t0, "end": S.now()}
    rnd["debriefs"][key] = rec
    S.save(state, st)
    log(f"round {rnd['n']} debrief {key}: {rec.get('end_reason', 'skipped')}, saved "
        f"{rec.get('saved')}, changed {rec.get('changed')}, version {st['version'][:12]}, "
        f"{S.money(rec.get('cost_usd'))}, {round(rec['timing']['end'] - t0)} s", flush=True)
    return rec


def run_debriefs(cfg: dict, state: Path, st: dict, rnd: dict, debrief_kw: dict) -> str | None:
    """The round's debriefs in order (shared.run_debriefs): what it waits for, or None."""
    for key in rnd.get("debrief_keys", rnd["episodes"]):
        if key in rnd["debriefs"]:
            continue
        fail = (rnd.get("debrief_failures") or {}).get(key) or {}
        if fail.get("retry_at", 0) > S.now():
            return (f"debrief {key} (attempt {fail['n'] + 1} of {S.DEBRIEF_ATTEMPTS} after "
                    f"{time.strftime('%H:%M:%S', time.localtime(fail['retry_at']))})")
        try:
            debrief_one(cfg, state, st, rnd, key, **debrief_kw)
        except Exception as e:  # noqa: BLE001
            n_ = rnd["n"]
            st.clear()
            st.update(load(state))
            rnd = next(r for r in st["rounds"] if r["n"] == n_)
            n = fail.get("n", 0) + 1
            why = f"{type(e).__name__}: {e}"[:1000]
            cost = (fail.get("cost_usd") or 0.0) + getattr(e, "cost_usd", 0.0)
            log(f"DEBRIEF FAILED {key} (attempt {n} of {S.DEBRIEF_ATTEMPTS}): {why}", flush=True)
            rnd.setdefault("debrief_failures", {})[key] = {
                "n": n, "last": why, "cost_usd": cost,
                "retry_at": S.now() + S.DEBRIEF_BACKOFF_S * 2 ** (n - 1)}
            if n < S.DEBRIEF_ATTEMPTS:
                S.save(state, st)
                return f"debrief {key} (failed {n} of {S.DEBRIEF_ATTEMPTS}: run again later)"
            t = S.now()
            rnd["debriefs"][key] = {"failed": f"{n} attempts, last {why}", "attempts": n,
                                    "cost_usd": cost, "version": st["version"],
                                    "timing": {"start": t, "end": t}}
            S.save(state, st)
            log(f"PROBLEM round {n_}: {key}'s debrief failed {n} times: recorded as failed, "
                "the notebook stays as it was", flush=True)
    return None


def learned(st: dict) -> list[int]:
    """Places with a scored, debriefed episode, in place order."""
    return sorted({place_of(k) for r in st["rounds"] if r["status"] == "done"
                   for k in r["episodes"]})


def version_at(st: dict, done: list[int], m: int) -> str:
    """The notebook version right after the m-th place learned: the version its round's last
    debrief left (a checkpoint taken late still evaluates the notebook of that moment)."""
    p = done[m - 1]
    rnd = next((r for r in st["rounds"] if r["status"] == "done"
                and any(place_of(k) == p for k in r["episodes"])), None)
    recs = [] if rnd is None else [d for d in rnd["debriefs"].values() if d.get("version")]
    return recs[-1]["version"] if recs else st["version"]


def add_checkpoints(cfg: dict, st: dict, final: bool = False) -> list[dict]:
    """A checkpoint at every eval.every places learned, at every pass's end with eval.epoch_end
    (and, final, at the end) not yet taken: the version of that moment, on every task learned
    so far."""
    done, every = learned(st), cfg["eval"].get("every")
    steps = list(range(every, len(done) + 1, every)) if every else []
    if cfg["eval"].get("epoch_end"):
        steps += list(range(len(cfg["tasks"]), len(done) + 1, len(cfg["tasks"])))
    if final and done:
        steps.append(len(done))
    have = {c["step"] for c in st["checkpoints"]}
    for c in st["checkpoints"]:
        if final and c["step"] == len(done):
            c["final"] = True
    new = []
    for m in dict.fromkeys(steps):
        if m in have:
            continue
        ck = {"step": m, "pass": epoch_of(cfg, done[m - 1]) + 1,
              "version": version_at(st, done, m),
              "tasks": sorted({task_at(cfg, p) for p in done[:m]}), "job": None,
              "final": final and m == len(done)}
        ck["unseen"] = {"tasks": sorted(set(cfg["tasks"]) - set(ck["tasks"])), "job": None}
        if m % len(cfg["tasks"]) == 0:
            ck["epoch_end"] = True
        st["checkpoints"].append(ck)
        new.append(ck)
        log(f"checkpoint at {m} places learned: version {ck['version'][:12]} on "
            f"{len(ck['tasks'])} tasks", flush=True)
    st["checkpoints"].sort(key=lambda c: c["step"])
    return new


def unseen_of(cfg: dict, ck: dict) -> dict | None:
    """The checkpoint's unseen-task eval ({tasks, job}), set up when missing (a checkpoint taken
    before eval.unseen was on); None when eval.unseen is off or every task was learned."""
    if not cfg["eval"].get("unseen"):
        return None
    un = ck.setdefault("unseen", {"tasks": sorted(set(cfg["tasks"]) - set(ck["tasks"])),
                                  "job": None})
    return un if un["tasks"] else None


def unseen_argv(cfg: dict, state: Path, ck: dict) -> list[str]:
    e = cfg["eval"]
    return submit_argv(cfg, state, ck["unseen"]["tasks"], ck["version"], ck_tag(cfg, ck) + "u",
                       e["corpus"], episodes=e["episodes"])


def eval_jobs(cfg: dict, ck: dict) -> list[tuple[str, list[str]]]:
    """(job, tasks) of the checkpoint's evals: seen, and unseen when it has them."""
    un = unseen_of(cfg, ck)
    return [(ck["job"], ck["tasks"])] + ([(un["job"], un["tasks"])] if un else [])


def submit_checkpoints(cfg: dict, state: Path, st: dict, pool, runner) -> None:
    for ck in st["checkpoints"]:
        if not ck.get("job"):
            job = submit_job(state, st, ck, ck_argv(cfg, state, ck), ck_tag(cfg, ck), pool,
                             runner)
            log(f"checkpoint {ck['step']} eval: {job} ({len(ck['tasks'])} tasks x "
                f"{cfg['eval']['episodes']}, version {ck['version'][:12]})", flush=True)
        un = unseen_of(cfg, ck)
        if un is not None and not un.get("job"):
            job = submit_job(state, st, un, unseen_argv(cfg, state, ck), ck_tag(cfg, ck) + "u",
                             pool, runner)
            log(f"checkpoint {ck['step']} unseen eval: {job} ({len(un['tasks'])} tasks x "
                f"{cfg['eval']['episodes']}, version {ck['version'][:12]})", flush=True)


def step(cfg: dict, state: Path, *, pool=None, runner=None, **debrief_kw) -> str:
    """Move the study on as far as it can without waiting; what it waits for now ("done" once
    learning is over and every checkpoint eval is submitted)."""
    pool = pool or local.LocalPool(cfg)
    runner = runner or local.LocalRunner(cfg["runs"])
    st = load(state)
    if add_checkpoints(cfg, st):
        S.save(state, st)
    while True:
        submit_checkpoints(cfg, state, st, pool, runner)
        if st["frozen"] is not None:
            return "done"
        rnd = next((r for r in st["rounds"] if r["status"] != "done"), None)
        if rnd is None:
            rnd = plan(cfg, st)
            if rnd is None:
                st["frozen"] = st["version"]
                st["frozen_at"] = S.now()
                add_checkpoints(cfg, st, final=True)
                S.save(state, st)
                log(f"learning done: notebook frozen at {st['frozen'][:12]}", flush=True)
                continue
            S.save(state, st)
        if rnd.get("halted"):
            raise S.ChainFailed(rnd["halted"])
        if rnd["status"] == "submitting":
            adopt(cfg, state, st, rnd, pool)
        if rnd["status"] == "planned":
            submit(cfg, state, st, rnd, pool, runner)
        if rnd["status"] == "submitted":
            collect(cfg, state, st, rnd, pool)
            if rnd["status"] == "submitted":
                return f"round {rnd['n']}'s job {rnd['job']}"
            continue
        if rnd["status"] == "collected":
            waiting = run_debriefs(cfg, state, st, rnd, debrief_kw)
            if waiting:
                return waiting
            rnd = next(r for r in st["rounds"] if r["n"] == rnd["n"])
            rnd["status"] = "done"
            rnd["timing"]["done"] = S.now()
            add_checkpoints(cfg, st)
            S.save(state, st)
            t = S.round_timing(rnd)
            log(f"round {rnd['n']} timing: job {S.hms(t['pool_s'])}, {t['debriefs']} debriefs "
                f"{S.hms(t['debriefs_s'])}, total {S.hms(t['total_s'])}", flush=True)


def by_task(cfg: dict, m: dict) -> dict:
    """{(config, episode): v} -> {(task, "config:episode"): v}, so tasks are the clusters."""
    task = {c: t for t, cs in cfg["configs"].items() for c in cs}
    return {(task[c], f"{c}:{e}"): v for (c, e), v in m.items() if c in task}


def finished(pool, job: str, want: int) -> tuple[bool, int]:
    got = pool.count(job)
    return got >= want or LP.job_ended_short(pool, {"job": job}) is not None, got


def learning_curve(cfg: dict, state: Path, st: dict) -> dict:
    """Learning success per pass and, in each pass, per third of its order; each task's result
    per pass; debriefs by kind and the costs."""
    os_ = {o["episode"]: o for o in S.outcomes(state)}
    n = len(cfg["tasks"])
    per_pass, thirds, per_task = {}, {}, {}
    for k, o in sorted(os_.items()):
        p = place_of(k)
        e, i = epoch_of(cfg, p) + 1, (p - 1) % n
        per_pass.setdefault(e, []).append(o["success"])
        thirds.setdefault(e, {}).setdefault(min(2, 3 * i // n), []).append(o["success"])
        per_task.setdefault(task_at(cfg, p), {}).setdefault(e, []).append(o["success"])
    debs = [d for r in st["rounds"] for d in r["debriefs"].values()]
    kinds = {k: 0 for k in S.KINDS}
    for d in debs:
        kinds[S.debrief_kind(d)] += 1

    def frac(xs):
        return f"{sum(xs)}/{len(xs)}"

    return {"per_pass": {e: frac(v) for e, v in per_pass.items()},
            "thirds": {e: {t: frac(v) for t, v in sorted(d.items())} for e, d in thirds.items()},
            "per_task": {t: {e: frac(v) for e, v in d.items()} for t, d in per_task.items()},
            "debriefs": len(debs), **kinds,
            "episode_cost_usd": round(sum(o.get("cost_usd") or 0 for o in os_.values()), 2),
            "debrief_cost_usd": round(sum(d.get("cost_usd") or 0 for d in debs)
                                      + sum(S.failed_debriefs_cost(r) for r in st["rounds"]), 2)}


def eval_part(cfg: dict, tasks: list[str], job: str | None, version: str, base: dict | None,
              pool, check: bool) -> dict:
    """One checkpoint eval job against the baseline on units scored in both; alone without one."""
    rec = {"tasks": len(tasks), "job": job}
    want = units_of(cfg, tasks, episodes=cfg["eval"]["episodes"])
    if not job:
        return {**rec, "pending": "not submitted"}
    done, got = finished(pool, job, want)
    if not done:
        return {**rec, "pending": f"{got} of {want} units"}
    rows = [{**r, "job": job} for r in pool.units(job)]
    problems, bad = [], set()
    for r in rows if check else ():
        why = LP.delivery_problem(pool.agent_trace(r), version, one_task=True)
        if why:
            problems.append(f"{r['task']} e{r['episode']}: {why}")
            bad.add((r["task"], r["episode"]))
    raw = S.unit_metrics(rows)
    every = by_task(cfg, {k: v for k, v in raw.items() if k not in bad})
    if base is None:
        return {**rec, "units_planned": want, "units": len(every),
                "success": f"{sum(v['success'] for v in every.values())}/{len(every)}",
                "baseline_success": None, "unpaired": 0, "undelivered": len(bad),
                "delivery_problems": problems, "mean_diff": None, "sign_flip_p": None,
                "ci95": None, "efficiency": None}
    run = {k: v for k, v in every.items() if k in base}
    if every and not run:
        raise ValueError(f"checkpoint eval {job}: none of its {len(every)} scored units is in the "
                         "baseline (a baseline of another seed or episode count?); results would "
                         "pair it at 0/0")
    paired = {k: base[k] for k in run}
    g = LP.compare(S.success_map(run), S.success_map(paired), tasks)
    return {**rec, "units_planned": want, "units": len(run),
            "success": f"{sum(v['success'] for v in run.values())}/{len(run)}",
            "baseline_success": f"{sum(v['success'] for v in paired.values())}/{len(paired)}",
            "unpaired": len(every) - len(run), "undelivered": len(bad),
            "delivery_problems": problems, "mean_diff": g["mean_diff"],
            "sign_flip_p": g["sign_flip_p"], "ci95": g["ci95"],
            "efficiency": S.efficiency_vs(run, paired, set(tasks))}


def corpus_short(corpus: str) -> str:
    """robodojo-seed1 -> s1 (exp tags); any other corpus keeps its name."""
    return "s" + corpus.rsplit("seed", 1)[1] if "seed" in corpus else corpus


def all_tasks(cfg: dict) -> list[str]:
    return sorted(cfg["tasks"])


def full_tag(cfg: dict, step: int, corpus: str) -> str:
    return tag(cfg, f"full_ck{step:03d}_{corpus_short(corpus)}")


def full_argv(cfg: dict, state: Path, ck: dict, corpus: str, episodes: int) -> list[str]:
    return submit_argv(cfg, state, all_tasks(cfg), ck["version"],
                       full_tag(cfg, ck["step"], corpus), corpus, episodes=episodes)


def full_eval(cfg: dict, state: Path, steps: list[int], corpora: list[str], episodes: int, *,
              dry: bool = False, pool=None, runner=None) -> list[list[str]]:
    """Submit (once) each listed checkpoint's notebook on every task of each corpus; the jobs are
    recorded as ck["full"][corpus]."""
    pool = pool or local.LocalPool(cfg)
    runner = runner or local.LocalRunner(cfg["runs"])
    st = load(state)
    out = []
    for step_ in steps:
        ck = next((c for c in st["checkpoints"] if c["step"] == step_), None)
        if ck is None:
            log(f"full eval: no checkpoint at {step_} places yet: skipped")
            continue
        for corpus in corpora:
            rec = ck.setdefault("full", {}).setdefault(corpus, {"episodes": episodes, "job": None})
            if rec.get("job"):
                log(f"full eval ck{step_} {corpus}: already {rec['job']}")
                continue
            argv = full_argv(cfg, state, ck, corpus, episodes)
            out.append(argv)
            if dry:
                log(" ".join(argv))
                continue
            job = submit_job(state, st, rec, argv, full_tag(cfg, step_, corpus), pool, runner)
            log(f"full eval ck{step_} {corpus}: {job} ({len(all_tasks(cfg))} tasks x {episodes}, "
                f"version {ck['version'][:12]})", flush=True)
    return out


def success_by_task(cfg: dict, rows: list[dict]) -> dict:
    """{(task, "config:episode"): success} of the scored rows."""
    task = {c: t for t, cs in cfg["configs"].items() for c in cs}
    return {(task[r["task"]], f"{r['task']}:{r['episode']}"): bool(r["success"]) for r in rows
            if S.scored(r) and r["task"] in task}


def trial(r: dict) -> int | None:
    """A unit as RoboDojo's aggregation counts it: 1 or 0 for a trial (an attempted episode that
    ended unscored, e.g. a sim crash, is 0); None for a layout that was no trial (abandoned at
    reset, or failed before the agent ran)."""
    if S.scored(r):
        return int(bool(r["success"]))
    attempted = r.get("status") == "attempt_error" and str(r.get("attempted")) == "True"
    return 0 if attempted and "while loading" not in str(r.get("error") or "") else None


def robodojo_sr(cfg: dict, rows_by_corpus: dict[str, list[dict]]) -> dict | None:
    """RoboDojo's success rate, as amp.benchmarks.robodojo's SUBSET_AGGREGATION_METHOD aggregates a
    run without spare layouts: a (task, seed) cell is successes / trials over its layouts (a
    Generalization task pooling its two configs), a task's number the mean over the seeds, a
    capability dimension's the mean over its tasks, the overall the mean over the dimensions
    present."""
    from amp.benchmarks import robodojo as RB

    task_of = {c: t for t, cs in cfg["configs"].items() for c in cs}
    dim_of = {}
    for corpus in rows_by_corpus:
        if not corpus.startswith("robodojo"):
            return None
        meta = RB.corpus_meta(corpus, episodes=RB.STANDALONE_TRIALS)
        dim_of.update({task_of.get(c, v["base"]): v["dimension"]
                       for c, v in meta["task_dimensions"].items()})
    per_task: dict[str, list[float]] = {}
    for rows in rows_by_corpus.values():
        cell: dict[str, list[int]] = {}
        for r in rows:
            o = trial(r) if r["task"] in task_of else None
            if o is not None:
                a = cell.setdefault(task_of[r["task"]], [0, 0])
                a[0] += o
                a[1] += 1
        for t, (k, n) in cell.items():
            per_task.setdefault(t, []).append(k / n)
    dims: dict[str, list[float]] = {}
    for t, v in per_task.items():
        dims.setdefault(dim_of[t], []).append(sum(v) / len(v))
    if not dims:
        return None
    by_dim = {d: sum(v) / len(v) for d, v in sorted(dims.items())}
    return {"success_rate": sum(by_dim.values()) / len(by_dim), "dimensions": by_dim,
            "tasks": len(per_task)}


def full_compare(cfg: dict, run: dict, base: dict, tasks: list[str]) -> dict:
    """Success on the units scored in both, the per-task mean difference, its p and CI."""
    run = {k: v for k, v in run.items() if k in base and k[0] in tasks}
    paired = {k: base[k] for k in run}
    g = LP.compare(run, paired, tasks)
    return {"units": len(run), "success": f"{sum(run.values())}/{len(run)}",
            "baseline_success": f"{sum(paired.values())}/{len(paired)}",
            "mean_diff": g["mean_diff"], "sign_flip_p": g["sign_flip_p"], "ci95": g["ci95"]}


def full_results(cfg: dict, st: dict, pool, collected: dict | None = None) -> list[dict]:
    """Every full evaluation against the study's full_baselines (success, paired per task), per
    corpus and over all corpora, on every unit (the benchmark's protocol)."""
    bases, base_rows, missing = {}, {}, {}

    def base_of(corpus: str) -> dict | None:
        """The corpus's full baseline by task, read when a finished full evaluation needs it."""
        if corpus not in bases and corpus not in missing:
            job = (cfg.get("full_baselines") or {}).get(corpus)
            why = baseline_missing(job, pool) if job else "no full baseline for it"
            if why:
                missing[corpus] = why
                if job:
                    log(f"full baseline {corpus}: {why}: its full evaluations stay unpaired")
                return None
            base_rows[corpus] = S.rows_of(job, pool)
            bases[corpus] = success_by_task(cfg, base_rows[corpus])
            if not bases[corpus]:
                raise ValueError(f"full baseline {corpus} ({job}) has no scored unit of the "
                                 "study's tasks: it may still be running, or be another study's")
        return bases.get(corpus)

    tasks = all_tasks(cfg)
    out = []
    for ck in st["checkpoints"]:
        full = ck.get("full") or {}
        if not full:
            continue
        rec = {"step": ck["step"], "pass": ck["pass"], "version": ck["version"], "corpora": {}}
        runs, base_all, run_rows = {}, {}, {}
        for corpus, f in full.items():
            if not f.get("job"):
                rec["corpora"][corpus] = {"pending": "not submitted"}
                continue
            want = units_of(cfg, tasks, episodes=f["episodes"])
            done, got = finished(pool, f["job"], want)
            if not done:
                rec["corpora"][corpus] = {"job": f["job"], "pending": f"{got} of {want} units"}
                continue
            rows = [{**r, "job": f["job"]} for r in pool.units(f["job"])]
            run = success_by_task(cfg, rows)
            base = base_of(corpus)
            if base is None:
                rec["corpora"][corpus] = {"job": f["job"], "pending": missing[corpus]}
                continue
            if run and not any(k in base for k in run):
                raise ValueError(f"full evaluation {f['job']}: none of its {len(run)} scored units "
                                 f"is in the {corpus} full baseline (another seed?); it would pair "
                                 "at 0/0")
            rec["corpora"][corpus] = {"job": f["job"],
                                      **full_compare(cfg, run, base, tasks),
                                      "robodojo": robodojo_sr(cfg, {corpus: rows}),
                                      "baseline_robodojo": robodojo_sr(cfg, {corpus: base_rows[corpus]})}
            runs.update(run)
            base_all.update(base)
            run_rows[corpus] = rows
            if collected is not None:
                collected.setdefault("runs", {})[(ck["step"], corpus)] = rows
        if runs and all("pending" not in v for v in rec["corpora"].values()):
            rec["combined"] = {**full_compare(cfg, runs, base_all, tasks),
                               "robodojo": robodojo_sr(cfg, run_rows),
                               "baseline_robodojo": robodojo_sr(cfg, {c: base_rows[c] for c in run_rows})}
        out.append(rec)
    if collected is not None:
        collected["base"] = base_rows
    return out


def figure_data(cfg: dict, state: Path, collected: dict) -> dict:
    """Figure 4 I-K of each RoboDojo corpus with a finished full evaluation, as the paper's scripts
    compute them (paper_scores): written to STATE/memory_curve_<seed>.csv, efficiency_<seed>.csv
    and matrix_<seed>.csv in the paper CSVs' layout."""
    from amp.benchmarks import robodojo as RB

    task_of = {c: t for t, cs in cfg["configs"].items() for c in cs}
    order = cfg["order"][0]
    out = {}
    for corpus, base_rows in (collected.get("base") or {}).items():
        runs = sorted((step, rows) for (step, c), rows in (collected.get("runs") or {}).items()
                      if c == corpus)
        if not (corpus.startswith("robodojo") and runs):
            continue
        meta = RB.corpus_meta(corpus, episodes=RB.STANDALONE_TRIALS)
        dim_of = {task_of.get(c, v["base"]): v["dimension"]
                  for c, v in meta["task_dimensions"].items()}
        by_cat = PS.tasks_by_cat(dim_of)
        intended: dict[str, list[int]] = {}
        for r in base_rows:
            if r["task"] in task_of:
                intended.setdefault(r["task"], []).append(r["episode"])
        intended = {c: sorted(set(es)) for c, es in intended.items()}

        def unit(rows):
            m = S.unit_metrics([r for r in rows if r["task"] in task_of])
            return {u: (v["success"], v["turns"]) for u, v in m.items()}

        units = {step: unit(rows) for step, rows in runs}
        base = unit(base_rows)
        succ = {None: {u: v[0] for u, v in base.items()},
                **{step: {u: v[0] for u, v in m.items()} for step, m in units.items()}}
        rng = random.Random(0)  # nosec B311
        curve = {step: PS.with_ci(PS.success_rates(s, intended), by_cat, rng)
                 for step, s in succ.items()}
        calls = {}
        for step, m in units.items():
            per = PS.call_ratios(m, base)
            calls[step] = (*PS.with_ci(per, by_cat, rng, PS.gmean), len(per))
        matrix = {step: PS.matrix_row(s, intended, task_of, order) for step, s in succ.items()
                  if step is None or step <= len(order)}
        seed = corpus.rsplit("-", 1)[-1]
        names = {None: "no_memory"}
        with open(state / f"memory_curve_{seed}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["checkpoint", "mean", "ci95_low", "ci95_high"])
            for step, v in curve.items():
                w.writerow([names.get(step, step), *(round(x, 2) for x in v)])
        with open(state / f"efficiency_{seed}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["checkpoint", "metric", "mean", "ci95_low", "ci95_high", "common_units"])
            w.writerow(["no_memory", "model_calls", 1, 1, 1, ""])
            for step, (p, lo, hi, n) in calls.items():
                w.writerow([step, "model_calls", round(p, 4), round(lo, 4), round(hi, 4),
                            f"{n} configs"])
        with open(state / f"matrix_{seed}.csv", "w", newline="") as f:
            csv.writer(f).writerows([["" if x is None else x for x in row]
                                     for row in matrix.values()])
        out[corpus] = {
            "success": {str(names.get(k, k)): [round(x, 2) for x in v] for k, v in curve.items()},
            "model_calls": {str(k): [round(x, 4) for x in v[:3]] + [v[3]]
                            for k, v in calls.items()},
            "matrix": {str(names.get(k, k)): {
                "cells": row, "labels": None if k is None else
                [PS.label(b, k) for b in range(len(row))]} for k, row in matrix.items()},
            "files": [f"{n}_{seed}.csv" for n in ("memory_curve", "efficiency", "matrix")]}
    return out


def baseline_missing(spec: str, pool) -> str | None:
    """Why spec's run or jobs do not exist yet, or None."""
    p = Path(spec).expanduser()
    if "/" in spec or p.exists():
        return None if p.exists() else f"{spec} does not exist yet"
    if not hasattr(pool, "job_dir"):
        return None
    gone = [j for j in spec.split(",") if not pool.job_dir(j).exists()]
    return f"no job {', '.join(gone)} under {pool.runs} yet" if gone else None


def baseline_of(cfg: dict, pool, none: str | None = None) -> tuple[dict | None, str]:
    """The baseline the checkpoints pair with, by task, and its results.md name; None if absent."""
    spec = run_path(Path.cwd(), none) if none else cfg.get("baseline")
    if not spec:
        return None, "none (each checkpoint's own success)"
    why = baseline_missing(spec, pool)
    if why and none:
        raise ValueError(f"--none {none}: {why}")
    if why:
        log(f"baseline: {why}: results without it")
        return None, f"{spec} (not found yet: each checkpoint's own success)"
    base = by_task(cfg, S.unit_metrics(S.rows_of(spec, pool)))
    if not base:
        raise ValueError(f"baseline {spec} has no scored unit of the study's tasks: it may still "
                         "be running, or be another study's; rerun results once it has scored "
                         "units, or pass --none")
    return base, spec


def results(cfg: dict, state: Path, pool=None, check: bool = True,
            none: str | None = None) -> dict:
    """Each finished checkpoint eval against the baseline on shared units; the learning curve."""
    pool = pool or local.LocalPool(cfg)
    st = load(state)
    base, base_name = baseline_of(cfg, pool, none)
    collected: dict = {}
    full = full_results(cfg, st, pool, collected)
    cks = []
    for ck in st["checkpoints"]:
        rec = {k: ck[k] for k in ("step", "pass", "version", "job", "final")}
        rec["notebook_bytes"] = (0 if ck["version"] == EMPTY else
                                 W.size(W.encode(S.version_files(state, ck["version"]))))
        rec.update(eval_part(cfg, ck["tasks"], ck.get("job"), ck["version"], base, pool, check))
        un = unseen_of(cfg, ck)
        if un is not None:
            rec["unseen"] = eval_part(cfg, un["tasks"], un.get("job"), ck["version"], base, pool,
                                      check)
        cks.append(rec)
    return {"study": cfg["name"], "label": cfg.get("label"), "baseline": base_name,
            "full": full, "figures": figure_data(cfg, state, collected),
            "full_baselines": cfg.get("full_baselines") or {},
            "seed": cfg["seed"], "frozen": st["frozen"], "checkpoints": cks,
            "learning": learning_curve(cfg, state, st), "dropped": st.get("dropped", [])}


def ratio(e: dict | None, m: str) -> str:
    v = ((e or {}).get("metrics") or {}).get(m)
    if not v or v.get("ratio_of_means") is None:
        return "-"
    ci = v.get("ratio_ci95")
    return f"x{v['ratio_of_means']:.2f}" + (f" [{ci[0]:.2f}, {ci[1]:.2f}]" if ci else "")


def part_cells(c: dict) -> str:
    """The result cells of one checkpoint eval: success, baseline, diff, p and efficiency."""
    if "pending" in c:
        return f"{c['pending']} | | | | | | | |"
    md, ci, e = c["mean_diff"], c["ci95"], c["efficiency"]
    if c.get("baseline_success") is None:
        return f"{c['success']} | - | - | - | - | - | - | - |"
    return (f"{c['success']} | {c['baseline_success']} | "
            + ("-" if md is None else f"{100 * md:+.1f}pp [{100 * ci[0]:+.1f}, "
               f"{100 * ci[1]:+.1f}]") + " | "
            + ("-" if c["sign_flip_p"] is None else f"{c['sign_flip_p']:.3g}") + " | "
            + f"{e['units']} | {ratio(e, 'turns')} | {ratio(e, 'steps')} | "
            f"{ratio(e, 'cost_usd')} |")


def results_md(res: dict) -> str:
    L = res["learning"]
    lines = [f"# {res['study']}" + (f" ({res['label']})" if res.get("label") else ""), "",
             (f"Order seed {res['seed']}; baseline {res['baseline']}; frozen "
              f"{(res['frozen'] or '-')[:12]}."), "",
             ("## Checkpoints: the notebook of that moment on every task learned so far (and, "
              "rows marked unseen, on the tasks not learned yet)"), "",
             ("| places learned | pass | tasks | notebook | success | baseline (same units) | "
              "mean per-task diff [95% CI] | p | both ok | turns | steps | cost |"),
             "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in res["checkpoints"]:
        for part, label in ((c, str(c["tasks"])), (c.get("unseen"), "unseen")):
            if part is None:
                continue
            label = label if part is c else f"{part['tasks']} {label}"
            head = (f"| {c['step']}{' (final)' if c.get('final') else ''} | {c['pass']} | "
                    f"{label} | {c['notebook_bytes'] / 1000:.1f} KB | ")
            lines.append(head + part_cells(part))
    def pct(r):
        return "-" if not r else f"{100 * r['success_rate']:.1f}%"

    if res.get("full"):
        lines += ["", "## Full evaluation (every task, every layout of each corpus)", "",
                  ("| places learned | corpus | units | success | baseline | mean per-task diff "
                   "[95% CI] | p | RoboDojo SR (5 dims) | baseline |"),
                  "|---|---|---|---|---|---|---|---|---|"]
        for f in res["full"]:
            parts = [(c, v) for c, v in f["corpora"].items()]
            if "combined" in f:
                parts.append(("combined", f["combined"]))
            for corpus, v in parts:
                if "pending" in v:
                    lines.append(f"| {f['step']} | {corpus} | {v['pending']} | | | | | | |")
                    continue
                md = ("-" if v["mean_diff"] is None else
                      f"{100 * v['mean_diff']:+.1f}pp [{100 * v['ci95'][0]:+.1f}, "
                      f"{100 * v['ci95'][1]:+.1f}]")
                lines.append(f"| {f['step']} | {corpus} | {v['units']} | {v['success']} | "
                             f"{v['baseline_success']} | {md} | "
                             + ("-" if v["sign_flip_p"] is None else f"{v['sign_flip_p']:.3g}")
                             + f" | {pct(v.get('robodojo'))} | {pct(v.get('baseline_robodojo'))} |")
    lines += ["", ("Efficiency columns: ratio of means (notebook / baseline) on units successful "
                   "in both, task-clustered bootstrap 95% CI."), "",
              "## Learning", "",
              (f"{L['debriefs']} debriefs: {S.debrief_counts(L)}; cost "
               f"{S.money(L['episode_cost_usd'])} episodes + {S.money(L['debrief_cost_usd'])} "
               "debriefs."), "", "| pass | success | first third | second third | last third |",
              "|---|---|---|---|---|"]
    for e, s in L["per_pass"].items():
        t = L["thirds"].get(e, {})
        lines.append(f"| {e} | {s} | " + " | ".join(t.get(i, "-") for i in range(3)) + " |")
    passes = sorted({e for d in L["per_task"].values() for e in d})
    lines += ["", "| task | " + " | ".join(f"pass {e}" for e in passes) + " |",
              "|---|" + "---|" * len(passes)]
    for t in sorted(L["per_task"]):
        lines.append(f"| {t} | " + " | ".join(L["per_task"][t].get(e, "-") for e in passes) + " |")
    for corpus, fig in (res.get("figures") or {}).items():
        lines += ["", f"## Figure 4 I-K ({corpus})", "",
                  ("The paper's formulas (paper_scores.py): unscored episodes count as failures, "
                   "categories weigh equally, 95% CI by bootstrap over tasks. Files: "
                   + ", ".join(fig["files"]) + "."), "",
                  "| places learned | success % [95% CI] | model calls vs no memory [95% CI] |",
                  "|---|---|---|"]
        for k, (p, lo, hi) in fig["success"].items():
            c = fig["model_calls"].get(k)
            lines.append(f"| {k} | {p:.2f} [{lo:.2f}, {hi:.2f}] | "
                         + ("1" if c is None else f"{c[0]:.4f} [{c[1]:.4f}, {c[2]:.4f}] ({c[3]} configs)")
                         + " |")
        lines += ["", ("| matrix (task mean %) | block 1 | block 2 | block 3 | block 4 |"),
                  "|---|---|---|---|---|"]
        for k, m in fig["matrix"].items():
            lines.append(f"| {k} | " + " | ".join(
                "-" if x is None else f"{x}" + (f" ({m['labels'][i]})" if m["labels"] else "")
                for i, x in enumerate(m["cells"])) + " |")
    for d in res["dropped"]:
        lines.append(f"\nDROPPED place {d['place']} ({d['task']}): {d['why']}")
    for c in res["checkpoints"]:
        for p in (c.get("delivery_problems") or []) + ((c.get("unseen") or {}).get(
                "delivery_problems") or []):
            lines.append(f"- checkpoint {c['step']} DELIVERY PROBLEM {p}")
    return "\n".join(lines) + "\n"


def write_results(cfg: dict, state: Path, pool=None, check: bool = True,
                  none: str | None = None) -> dict:
    res = results(cfg, state, pool, check, none)
    (state / "results.json").write_text(json.dumps(res, indent=1, default=str))
    (state / "results.md").write_text(results_md(res))
    log(results_md(res))
    return res


def run_chain(state: Path, *, every: int, wait: bool, check: bool, pool=None, runner=None,
              **debrief_kw) -> str:
    """Step one chain, holding its lock, until learning is over and every checkpoint eval is
    submitted; with wait, until they finished and the results are written."""
    with S.chain_lock(state):
        cfg = S.config(state)
        pool = pool or local.LocalPool(cfg)
        runner = runner or local.LocalRunner(cfg["runs"])
        st = load(state)
        halted = [r for r in st["rounds"] if r.pop("halted", None)]
        if halted:
            S.save(state, st)
            log(f"round {halted[0]['n']} was halted after losing every place to infra: run again "
                "by its operator, submitting it once more")
        fails = 0
        while True:
            try:
                waiting = step(cfg, state, pool=pool, runner=runner, **debrief_kw)
                if waiting == "done" and wait:
                    st = load(state)
                    left = [c for c in st["checkpoints"] for job, tasks in eval_jobs(cfg, c)
                            if not finished(pool, job, units_of(
                                cfg, tasks, episodes=cfg["eval"]["episodes"]))[0]]
                    if not left:
                        write_results(cfg, state, pool, check)
                        return "results"
                    waiting = f"checkpoint evals {', '.join(str(c['step']) for c in left)}"
            except S.ChainFailed:
                raise
            except (Exception, SystemExit) as e:
                fails += 1
                log(f"STEP FAILED ({fails} of {S.MAX_STEP_FAILS} in a row): "
                    f"{type(e).__name__}: {e}")
                if fails >= S.MAX_STEP_FAILS:
                    raise S.ChainFailed(f"{fails} passes in a row failed, the last with "
                                        f"{type(e).__name__}: {e}") from e
                waiting = "a retry after the error"
            else:
                fails = 0
            if waiting == "done":
                log("learning done and every checkpoint eval submitted; amp practice continual "
                    "results STATE once they finished")
                return waiting
            log(time.strftime("%H:%M:%S"), f"waiting for: {waiting}")
            time.sleep(max(every, min(every * fails, 900)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="amp practice continual", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["init", "step", "run", "results", "full-eval"])
    ap.add_argument("state", type=Path, help="a state dir, or the root init --chains made")
    ap.add_argument("--study", type=Path)
    ap.add_argument("--chains", type=int, help="init: 1, the paper's chain: STATE/chain_00 with "
                    "a seed derived from the study's")
    ap.add_argument("--dry-run", action="store_true", help="full-eval: print the jobs, submit "
                    "nothing")
    ap.add_argument("--every", type=int, default=120, help="run: seconds between polls")
    ap.add_argument("--wait", action="store_true", help="run: wait for the checkpoint evals and "
                    "write the results")
    ap.add_argument("--check-delivery", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--none", help="results: the no-memory run the checkpoints pair with, in place "
                    "of the study's baseline (an amp eval run folder of the study's arm)")
    ap.add_argument("--steps", help="full-eval: checkpoints (places learned), e.g. 10,20,30,42")
    ap.add_argument("--corpora", default="robodojo-seed2",
                    help="full-eval: corpora, each run on every task")
    ap.add_argument("--episodes", type=int, default=50, help="full-eval: episodes per task")
    a = ap.parse_args(argv)
    path = a.state
    if a.cmd == "init":
        if not a.study:
            ap.error("init needs --study")
        if a.chains:
            c = init_chains(path, a.study, a.chains)
            log(f"{path / 'chains.json'}: {c['chains']} chain, seed {c['seeds'][0]}")
            return 0
        cfg = init(path, a.study)
        log(f"{path / 'config.json'}: seed {cfg['seed']}, {n_places(cfg)} places")
        return 0
    if a.cmd == "full-eval" and not a.steps:
        ap.error("full-eval needs --steps")
    s = S.chain_dirs(path)[0]
    cfg = S.config(s)
    try:
        if a.cmd == "run":
            return 0 if run_chain(s, every=a.every, wait=a.wait, check=a.check_delivery) in (
                "done", "results") else 1
        if a.cmd == "results":
            write_results(cfg, s, check=a.check_delivery, none=a.none)
        elif a.cmd == "step":
            with S.chain_lock(s):
                log(f"waiting for: {step(cfg, s)}")
        elif a.dry_run:
            full_eval(cfg, s, [int(x) for x in a.steps.split(",")], a.corpora.split(","),
                      a.episodes, dry=True)
        else:
            with S.chain_lock(s):
                full_eval(cfg, s, [int(x) for x in a.steps.split(",")], a.corpora.split(","),
                          a.episodes)
    except S.ChainLocked as e:
        log(f"REFUSED: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
