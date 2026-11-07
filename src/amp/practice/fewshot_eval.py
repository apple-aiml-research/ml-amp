#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Evaluations of frozen few-shot runs (Figure 4F-H) and the paper's scoring (docs/practice.md).

  amp practice fewshot eval {RUN | --notebooks NB} --corpus C --episodes K --output DIR
        [--model M] [--reasoning R] [--cut code|notes] [--round K[,K...]] [--reuse DIR[,DIR...]]
        [--dry-run]
        every task's frozen workspace (or the version after round K, or cut to its .py files or to
        the rest) evaluated on C in one job, each episode with its own task's; DIR is a run
        folder that eval and results accept, a --round series one per round under DIR/k<K>/.
        NB is a folder of per-task notebooks with a checkpoints.json (the paper's,
        src/amp/practice/notebooks/fewshot_best5), checked against its shas and evaluated with
        the default study's agent
  amp practice fewshot select RUN RUN ... --corpus robodojo-seed1 --episodes K --output SEL
        [--seed1 DIR,...] [--dry-run]   each run's frozen workspaces evaluated on C (or reused),
        per task the run with the highest success, SEL a run folder with selection.json

An evaluation folder has a practice run's layout (config.json, tasks/<task>/state.json and
versions/) with the evaluated version as each task's frozen one, so its eval jobs are fewshot's
own (eval_submit). Scoring follows the paper (leaderboard, model_calls).
"""
from __future__ import annotations

import argparse
import json
import random
import re
import shutil
import statistics
import sys
from pathlib import Path

from amp.practice import fewshot as LP
from amp.practice import local
from amp.practice import paper_scores as PS
from amp.practice import shared as S
from amp.practice import workspace as W

EMPTY = LP.EMPTY
STUDY = Path(__file__).parent / "studies" / "fewshot.json"
CUTS = {"code": lambda p: p.endswith(".py"), "notes": lambda p: not p.endswith(".py")}


def intended(corpus: str, episodes: int, tasks: list[str]) -> dict[str, set[int]]:
    """{config: layouts} of every unit the evaluation asks for."""
    from amp import benchmarks
    from amp.benchmarks import robodojo as RB

    out: dict[str, set[int]] = {}
    for u in benchmarks.expand(corpus, episodes, tuple(tasks)):
        out.setdefault(u.task, set()).add(RB.address(u.task, u.episode)[1])
    return out


def _by_cat(configs) -> dict[str, list[str]]:
    """{category: sorted base tasks} of these configs, the categories that have any."""
    from amp.benchmarks import robodojo as RB

    dim_of = {t.base: t.dimension for t in map(RB._lookup, configs)}
    return {c: ts for c, ts in PS.tasks_by_cat(dim_of).items() if ts}


def _ci(per: dict, by: dict, agg=statistics.fmean) -> list[float] | None:
    """The CI with a fresh Random(0), as recompute_all.py seeds each CI."""
    rng = random.Random(0)  # nosec B311
    bounds = PS.ci(per, by, rng, agg)
    return None if bounds is None else list(bounds)


def leaderboard(succ: dict, want: dict[str, set[int]]) -> dict:
    """The RoboDojo leaderboard success rate (%) over the intended units, per category, its CI."""
    per = PS.success_rates(succ, want)
    by = _by_cat(want)
    dims = {d: PS.formula(per, {d: ts}) for d, ts in by.items()}
    return {"success_rate": PS.formula(per, by), "ci95": _ci(per, by),
            "categories": dims, "intended": sum(map(len, want.values())),
            "successes": sum(bool(succ.get((c, k))) for c, ks in want.items() for k in ks)}


def call_ratio(run: dict, base: dict, want: dict[str, set[int]]) -> dict:
    """Model calls with memory over without: per setting the mean over the units successful in
    both, combined with geometric means in the leaderboard's structure; run and base map
    (config, layout) to (success, turns)."""
    num, den = PS.paired({k: v for k, v in run.items() if k[0] in want and k[1] in want[k[0]]},
                         base)
    per = PS.ratios(num, den)
    by = _by_cat(want)
    return {"ratio": PS.formula(per, by, PS.gmean) if per else None,
            "ci95": _ci(per, by, PS.gmean) if per else None,
            "pairs": sum(map(len, num.values())), "settings": len(per)}


def unit_map(rows: list[dict], bad=frozenset()) -> dict:
    """{(config, layout): (success, turns)} of the scored rows; a unit in bad is left out."""
    from amp.benchmarks import robodojo as RB

    out = {}
    for r in rows:
        if S.scored(r):
            k = (r["task"], RB.address(r["task"], r["episode"])[1])
            if k not in bad:
                out[k] = (bool(r["success"]), S.efficiency(r)["turns"])
    return out


def eval_rows(cfg: dict, state: Path, pool) -> list[dict]:
    rows = []
    for t in cfg["tasks"]:
        for j in (LP.load(state, t).get("eval") or {}).get("jobs", []):
            if j.get("job"):
                rows += LP.task_rows(pool.units(j["job"]), t)
    return rows


def undelivered(res: dict) -> set:
    """fewshot results' undelivered units as (config, layout)."""
    return {(c, int(k)) for c, k in (s.rsplit(" ", 1) for s in res.get("undelivered") or [])}


def scores(cfg: dict, state: Path, res: dict, pool, none: str | None) -> dict:
    """results.json's paper numbers: the leaderboard success rate on the intended units, and with
    a no-memory run its success rate and the model-call ratio."""
    want = intended(cfg["eval"]["corpus"], cfg["eval"]["episodes"], cfg["tasks"])
    run = unit_map(eval_rows(cfg, state, pool), undelivered(res))
    out = {"leaderboard": leaderboard({k: v[0] for k, v in run.items()}, want)}
    if none:
        base = unit_map(LP.baseline_rows(none, pool))
        out["none_leaderboard"] = leaderboard({k: v[0] for k, v in base.items()}, want)
        out["model_calls"] = call_ratio(run, base, want)
    return out


def pct(x) -> str:
    return "n/a" if x is None else f"{x:.1f}%"


def ci_text(ci, fmt="{:.1f}") -> str:
    return "n/a" if not ci else f"[{fmt.format(ci[0])}, {fmt.format(ci[1])}]"


def scores_md(res: dict) -> str:
    lb = res["leaderboard"]
    lines = ["", "## Paper scoring (RoboDojo leaderboard formula)", "",
             (f"Success rate {pct(lb['success_rate'])}, 95% CI {ci_text(lb['ci95'])} "
              f"({lb['successes']}/{lb['intended']} intended units succeeded; an unscored unit "
              "counts as a failure)."), "",
             "| category | success rate |", "|---|---|"]
    lines += [f"| {d} | {pct(v)} |" for d, v in lb["categories"].items()]
    if "none_leaderboard" in res:
        nb, mc = res["none_leaderboard"], res["model_calls"]
        ratio = "n/a" if mc["ratio"] is None else f"{mc['ratio']:.3f}"
        lines += ["", (f"No memory: {pct(nb['success_rate'])}, 95% CI {ci_text(nb['ci95'])}. "
                       f"Model calls relative to no memory: {ratio}, 95% CI "
                       f"{ci_text(mc['ci95'], '{:.3f}')} ({mc['pairs']} units successful in both, "
                       f"{mc['settings']} settings).")]
    return "\n".join(lines) + "\n"


def results_command(cfg: dict, state: Path, none: str | None, pool, check) -> dict:
    """fewshot results: results.{json,md} with the paper's numbers added."""
    res = LP.results(cfg, state, {"none": [none]} if none else {}, pool=pool, check=check)
    res.update(scores(cfg, state, res, pool, none))
    md = LP.results_md(res) + scores_md(res)
    (state / "results.json").write_text(json.dumps(res, indent=1))
    (state / "results.md").write_text(md)
    print(md)
    return res


def curve_folders(root: Path) -> list[tuple[int, Path]]:
    """The k<K>/ evaluation folders of a --round series, by K."""
    out = []
    for p in root.glob("k*/config.json"):
        rnd = (json.loads(p.read_text()).get("eval_of") or {}).get("round")
        if isinstance(rnd, int):
            out.append((rnd, p.parent))
    return sorted(out)


def curve_results(root: Path, none: str | None, check) -> dict:
    """The k curve of a --round series: each round's results, and k = 0 from the no-memory run."""
    folders = curve_folders(root)
    if not folders:
        raise SystemExit(f"{root}: no config.json and no k<K>/ evaluation folders")
    rows = []
    for k, d in folders:
        cfg = LP.config(d)
        res = results_command(cfg, d, none, local.LocalPool(cfg), check)
        rows.append({"k": k, "folder": str(d), "success_rate": res["leaderboard"]["success_rate"],
                     "ci95": res["leaderboard"]["ci95"],
                     "model_calls": (res.get("model_calls") or {}).get("ratio"),
                     "model_calls_ci95": (res.get("model_calls") or {}).get("ci95")})
        if none and not any(r["k"] == 0 for r in rows):
            nb = res["none_leaderboard"]
            rows.insert(0, {"k": 0, "folder": none, "success_rate": nb["success_rate"],
                            "ci95": nb["ci95"], "model_calls": 1.0, "model_calls_ci95": None})
    lines = ["# k curve", "", "| k | success rate | 95% CI | model calls vs none | 95% CI |",
             "|---|---|---|---|---|"]
    for r in rows:
        mc = "n/a" if r["model_calls"] is None else f"{r['model_calls']:.3f}"
        lines.append(f"| {r['k']} | {pct(r['success_rate'])} | {ci_text(r['ci95'])} | {mc} | "
                     f"{ci_text(r['model_calls_ci95'], '{:.3f}')} |")
    (root / "curve.json").write_text(json.dumps({"curve": rows}, indent=1))
    (root / "curve.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return {"curve": rows}


def round_version(st: dict, k: int) -> str:
    """The workspace after k debriefed batches: the version batch k+1 started from, or the frozen
    one for a task that practised fewer batches (AMP's k-curve rule; k = 0 is no workspace)."""
    batches = [r for r in st["rounds"] if r.get("kind") == "batch"]
    if len(batches) > k:
        return batches[k]["version"]
    return st["frozen"]


def clone_task(src: Path, dst: Path, task: str, keep_eval: bool) -> dict:
    """A task's state, versions and practice outcomes copied into another run folder; its state."""
    s, d = LP.task_dir(src, task), LP.task_dir(dst, task)
    if (s / "versions").is_dir():
        shutil.copytree(s / "versions", d / "versions", dirs_exist_ok=True)
    d.mkdir(parents=True, exist_ok=True)
    if (s / "versions.jsonl").is_file():
        shutil.copy2(s / "versions.jsonl", d / "versions.jsonl")
    for o in (s / "history").glob("*/outcome.json"):
        (d / "history" / o.parent.name).mkdir(parents=True, exist_ok=True)
        shutil.copy2(o, d / "history" / o.parent.name / "outcome.json")
    st = LP.load(src, task)
    if not keep_eval:
        st["eval"] = None
    return st


def same_jobs(a: dict, b: dict) -> bool:
    """Whether two folders' eval jobs are interchangeable: corpus, episodes, agent, workspace
    limit and runs folder."""
    return (a["eval"] == b["eval"] and local.arm_id(a) == local.arm_id(b)
            and a.get("workspace_max_bytes") == b.get("workspace_max_bytes")
            and Path(a["runs"]) == Path(b["runs"]))


def tag_of(cfg: dict, corpus: str, cut, rnd, model) -> str:
    seed = re.sub(r"\D", "", corpus)[-2:]
    parts = [f"e{seed}", f"c{cut[0]}" if cut else "", "" if rnd is None else f"k{rnd}",
             f"m{re.sub(r'[^A-Za-z0-9]', '', model)[:5]}" if model else ""]
    return (cfg["tag"][:8] + "".join(parts))[:26]


def notebooks_run(nb: Path, stage: Path) -> Path:
    """A frozen run folder (the default study's config) from a folder of per-task notebooks and its
    checkpoints.json, each notebook checked against its sha; stage."""
    if (stage / "ready").is_file():
        return stage
    try:
        entries = W.read_manifest(nb / "checkpoints.json")
    except W.WorkspaceError as e:
        raise SystemExit(str(e)) from None
    files = {}
    for task, (folder, sha) in entries.items():
        if folder is None or not folder.is_dir():
            raise SystemExit(f"{nb}: checkpoints.json names no folder for {task}")
        files[task] = W.read_dir(folder)
        if W.sha_of(files[task]) != sha:
            raise SystemExit(f"{folder} holds {W.sha_of(files[task])[:12]}, not the notebook "
                             f"{sha[:12]} of checkpoints.json: refusing to evaluate it")
    shutil.rmtree(stage, ignore_errors=True)
    cfg = LP.init(stage, STUDY)
    if sorted(files) != sorted(cfg["tasks"]) or len(files) != len(entries):
        raise SystemExit(f"{nb}: checkpoints.json is not one notebook for each of the study's "
                         f"{len(cfg['tasks'])} tasks")
    for t in cfg["tasks"]:
        sha = LP.store_version(stage, t, files[t], None, {"round": None, "summary": "notebook"})
        st = LP.load(stage, t)
        st["version"] = sha
        LP.freeze_v3(st, "frozen", f"the notebook of {nb.name}")
        LP.save(stage, t, st)
    (stage / "ready").write_text("")
    return stage


def prepare(src: Path, out: Path, *, corpus: str, episodes: int, model=None, reasoning=None,
            cut=None, rnd=None, reuse=(), run=None) -> dict:
    """The evaluation folder out (made once, then resumed): src's tasks, each with the version to
    evaluate as its frozen one; an equal earlier eval of that version is reused."""
    src_cfg = LP.config(src)
    spec = {"run": str(run or src), "corpus": corpus, "episodes": episodes, "cut": cut, "round": rnd,
            "model": model or src_cfg["model"],
            "reasoning": reasoning or src_cfg.get("reasoning")}
    if (out / "config.json").is_file():
        cfg = LP.config(out)
        if cfg.get("eval_of") != spec:
            raise SystemExit(f"{out} holds another evaluation ({cfg.get('eval_of')}): pick another "
                             "--output")
        return cfg
    if corpus in src_cfg["practice"]["corpora"]:
        raise SystemExit(f"{corpus} is a practice corpus of {src}: evaluate on another seed")
    open_ = [t for t in src_cfg["tasks"] if LP.load(src, t)["frozen"] is None]
    if open_:
        raise SystemExit(f"{src}: {len(open_)} task(s) not frozen yet ({', '.join(open_[:3])}): "
                         "finish practice first")
    cfg = {**src_cfg, "eval": {"corpus": corpus, "episodes": episodes}, "model": spec["model"],
           "reasoning": spec["reasoning"], "eval_of": spec,
           "tag": tag_of(src_cfg, corpus, cut, rnd, model)}
    LP.tag_for(cfg, cfg["tasks"][-1], "ev")
    local.arm_id(cfg)
    intended(corpus, episodes, cfg["tasks"])
    sources = [(c, LP.config(c)) for c in (src, *reuse)]
    for t in cfg["tasks"]:
        st = clone_task(src, out, t, keep_eval=False)
        version = st["frozen"] if rnd is None else round_version(st, rnd)
        files = LP.version_files(src, t, version)
        if cut:
            files = {p: b for p, b in files.items() if CUTS[cut](p)}
            version = LP.store_version(out, t, files, version, {
                "round": None, "summary": f"{cut} cut"}) if files else EMPTY
        st.update(source_frozen=st["frozen"], frozen=version)
        for c, ccfg in sources:
            ev = LP.load(c, t).get("eval") or {}
            if (same_jobs(ccfg, cfg) and ev.get("version") == version and ev.get("jobs")
                    and all(j.get("job") for j in ev["jobs"])):
                st["eval"] = {**ev, "reused_from": str(c)}
                break
        LP.save(out, t, st)
    (out / "config.json").write_text(json.dumps(cfg, indent=1))
    return cfg


def submit(cfg: dict, out: Path, runner, dry: bool) -> list[str]:
    """Submit one eval job for every task not yet submitted (dry: print its argv); the jobs."""
    for t in cfg["tasks"] if dry else ():
        ev = LP.load(out, t).get("eval") or {}
        if ev.get("jobs") and all(j.get("job") for j in ev["jobs"]):
            print(f"{t}: reuses {', '.join(j['job'] for j in ev['jobs'])}")
    LP.eval_submit_all(cfg, out, cfg["tasks"], runner=runner, dry=dry)
    if dry:
        return []
    return sorted({j["job"] for t in cfg["tasks"] for j in LP.load(out, t)["eval"]["jobs"]
                   if j.get("job")})


def evaluate(folders: list[tuple[dict, Path]], dry: bool, every: int) -> None:
    """Submit every folder's eval jobs and wait for them (they run in this process)."""
    waits = []
    for cfg, out in folders:
        jobs = submit(cfg, out, local.LocalRunner(cfg["runs"]), dry)
        waits.append((local.LocalPool(cfg), jobs))
    for pool, jobs in waits:
        LP.wait_jobs(pool, jobs, every)


def eval_success(cfg: dict, folder: Path, check) -> dict:
    """{(config, layout): success} of the folder's eval units whose agent got their version."""
    pool = local.LocalPool(cfg)
    res = LP.results(cfg, folder, {}, pool=pool, check=check)
    return {k: v[0] for k, v in unit_map(eval_rows(cfg, folder, pool), undelivered(res)).items()}


def pick(task: str, names: list[str], succ: dict, used: dict) -> dict:
    """AMP's best-of-N pick: the highest success on the units every run scored, ties to fewer
    practice episodes, then the earlier run."""
    from amp.benchmarks import robodojo as RB

    mine = {n: {k: v for k, v in succ[n].items() if RB._lookup(k[0]).base == task} for n in names}
    common = set.intersection(*(set(m) for m in mine.values()))
    basis = "common"
    if not common:
        basis = "own"
        print(f"PROBLEM {task}: no unit scored in every run; each run's rate is over its own "
              "scored units", flush=True)

    def rate(n):
        keys = common if common else set(mine[n])
        return sum(mine[n][k] for k in keys) / len(keys) if keys else -1.0

    rates = {n: rate(n) for n in names}
    best = min(names, key=lambda n: (-rates[n], used[n][task], names.index(n)))
    return {"pick": best, "rates": rates, "units": len(common), "basis": basis,
            "used": {n: used[n][task] for n in names}}


def run_names(runs: list[Path]) -> list[str]:
    out = []
    for r in runs:
        n, i = r.name, 2
        while n in out:
            n, i = f"{r.name}_{i}", i + 1
        out.append(n)
    return out


def select(runs: list[Path], out: Path, *, corpus: str, episodes: int, seed1=None, check=None,
           dry=False, every=60, pair: str | None = None) -> dict | None:
    """Best-of-N: each run's frozen workspaces on corpus, per task the best run, out a run folder
    holding the picked tasks and selection.json."""
    names = run_names(runs)
    cfgs = {n: LP.config(r) for n, r in zip(names, runs)}
    tasks = cfgs[names[0]]["tasks"]
    if any(c["tasks"] != tasks for c in cfgs.values()):
        raise SystemExit("the runs do not practise the same tasks")
    folders = {}
    for i, (n, r) in enumerate(zip(names, runs)):
        if seed1:
            folders[n] = seed1[i]
        elif cfgs[n]["eval"] == {"corpus": corpus, "episodes": episodes}:
            folders[n] = r
        else:
            folders[n] = out / "seed1" / n
            prepare(r, folders[n], corpus=corpus, episodes=episodes)
    for n, r in zip(names, runs):
        fc = LP.config(folders[n])
        for t in tasks:
            fz, got = LP.load(r, t)["frozen"], LP.load(folders[n], t)
            if (fc["eval"] != {"corpus": corpus, "episodes": episodes}
                    or fc["model"] != cfgs[n]["model"] or got["frozen"] != fz
                    or (got.get("eval") or {}).get("version", fz) != fz):
                raise SystemExit(f"{folders[n]} is not an eval of {r}'s frozen {t} on {corpus}")
    evaluate([(LP.config(f), f) for f in folders.values()], dry, every)
    if dry:
        return None
    succ = {n: eval_success(LP.config(folders[n]), folders[n], check) for n in names}
    if pair:
        kept = set(unit_map(LP.baseline_rows(pair, local.LocalPool(cfgs[names[0]]))))
        succ = {n: {k: v for k, v in s.items() if k in kept} for n, s in succ.items()}
    used = {n: {t: LP.used(r, t) for t in tasks} for n, r in zip(names, runs)}
    picks = {t: pick(t, names, succ, used) for t in tasks}
    first = cfgs[names[0]]
    cfg = {**first, "name": f"{first['name']}_best{len(names)}", "selection_of": [str(r) for r in runs]}
    for t, p in picks.items():
        n = p["pick"]
        src = runs[names.index(n)]
        st = clone_task(src, out, t, keep_eval=same_jobs(cfgs[n], cfg))
        st["selected_from"] = str(src)
        LP.save(out, t, st)
    sel = {"runs": dict(zip(names, map(str, runs))), "corpus": corpus, "episodes": episodes,
           "evals": {n: str(f) for n, f in folders.items()},
           "rule": ("per task the highest success on the units every run scored; ties to fewer "
                    "practice episodes, then the earlier run"),
           "pair": pair,
           "picks": {n: sum(p["pick"] == n for p in picks.values()) for n in names},
           "tasks": picks}
    (out / "selection.json").write_text(json.dumps(sel, indent=1))
    (out / "config.json").write_text(json.dumps(cfg, indent=1))
    print(f"{out}: picks " + ", ".join(f"{n} {c}" for n, c in sel["picks"].items()))
    return sel


def paths(s: str | None) -> list[Path]:
    return [Path(x).expanduser().resolve() for x in s.split(",")] if s else []


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="amp practice fewshot", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["eval", "select"])
    ap.add_argument("runs", nargs="*", type=Path, help="practice run folder(s)")
    ap.add_argument("--notebooks", type=Path, help="eval: instead of RUN, a folder of per-task "
                    "notebooks and its checkpoints.json, evaluated with the default study's agent")
    ap.add_argument("--corpus", required=True, help="the evaluation corpus (robodojo-seedN)")
    ap.add_argument("--episodes", type=int, required=True, help="episodes per task")
    ap.add_argument("--output", type=Path, required=True, help="the evaluation or selection folder")
    ap.add_argument("--model", help="eval: another model, with the run's memory")
    ap.add_argument("--reasoning", help="eval: that model's reasoning effort")
    ap.add_argument("--cut", choices=sorted(CUTS),
                    help="eval: each workspace cut to its .py files (code) or to the rest (notes)")
    ap.add_argument("--round", help="eval: K[,K...], the workspace after K rounds (0: none), "
                    "each in OUTPUT/k<K>/")
    ap.add_argument("--reuse", help="eval: folders whose eval of the same version is reused")
    ap.add_argument("--seed1", help="select: one completed eval folder per run, in order")
    ap.add_argument("--pair", help="select: a no-memory run on the same corpus; only the units "
                    "it scored count, as in the paper's selection")
    ap.add_argument("--check-delivery", action=argparse.BooleanOptionalAction, default=None)
    ap.add_argument("--every", type=int, default=60, help="seconds between job polls")
    ap.add_argument("--dry-run", action="store_true", help="prepare the folders, print each "
                    "job's argv, submit nothing; a dry run writes the manifest its argv names")
    a = ap.parse_args(argv)
    runs, out = [r.expanduser().resolve() for r in a.runs], a.output.expanduser().resolve()
    if a.cmd == "select" and (not runs or a.notebooks):
        ap.error("select takes practice run folders")
    if a.cmd == "select":
        seed1 = paths(a.seed1)
        if seed1 and len(seed1) != len(runs):
            ap.error("--seed1 names one eval folder per run")
        select(runs, out, corpus=a.corpus, episodes=a.episodes, seed1=seed1,
               check=a.check_delivery, dry=a.dry_run, every=a.every, pair=a.pair)
        return 0
    if len(runs) + bool(a.notebooks) != 1:
        ap.error("eval takes one run folder or --notebooks DIR")
    kw = {"corpus": a.corpus, "episodes": a.episodes, "model": a.model,
          "reasoning": a.reasoning, "cut": a.cut, "reuse": paths(a.reuse)}
    if a.notebooks:
        if a.round:
            ap.error("--round needs a practice run folder: a notebook folder holds frozen versions only")
        nb = a.notebooks.expanduser().resolve()
        runs = [notebooks_run(nb, out / ".notebooks")]
        kw["run"] = nb
    if a.round:
        ks = [int(k) for k in a.round.split(",")]
        folders = [prepare(runs[0], out / f"k{k}", rnd=k, **kw) for k in ks]
        todo = [(c, out / f"k{k}") for c, k in zip(folders, ks)]
    else:
        todo = [(prepare(runs[0], out, **kw), out)]
    evaluate(todo, a.dry_run, a.every)
    if not a.dry_run:
        print(f"done: amp practice fewshot results {out} --none <the no-memory run>")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
