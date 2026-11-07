#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RoboDojo: "robodojo-seed0", "robodojo-seed1" and "robodojo-seed2", one evaluation seed's first
N layouts (--episodes N, 1-50) of each of the 42 tasks, from one manifest (docs/robodojo.md)."""

from __future__ import annotations

import functools
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import NoReturn

from .base import Benchmark, CorpusError, Unit, UnknownCorpus

SCHEMA = 1
TASK_SET = "robodojo"
DEFAULT_MANIFEST = Path(__file__).with_name("robodojo_manifest.json")
GENERATOR = "AMP, from the RoboDojo checkout at robodojo_commit"
REGENERATE = "regenerate it from RoboDojo at the pinned commit"
CONTROL_DT = 0.04
ACTION_WALL_S = 0.106
SEEDS = (0, 1, 2)
START_UP_S = 95.0
ROW_WAIT_S = {TASK_SET: 1800.0 + 600.0 + 60.0}
EMBODIMENT = "robodojo_dual_x5"
DIMENSIONS = ("generalization", "memory", "precision", "long-horizon", "open")
PIN_KEYS = ("robodojo", "xpolicylab", "isaaclab", "curobo", "hf_revision")
SEED_CORPUS = "robodojo-seed{}"
TAKES_LAYOUTS = True
PAIRED_HALF = 25
STANDALONE_TRIALS = 50
SUMMARISER = "scripts/internal/summarize_result.py"
FIRST_LAYOUTS_DIFFERENCE = (
    "one seed and its first N layouts per task, not RoboDojo's 3 seeds x each task's trials, and "
    "no refill: an abandoned layout is not replaced, so it leaves its task short")
PROTOCOL_DIFFERENCES = (
    "num_envs 1 (one layout per scene)",
    "the policy is AMP's agent behind a bridge, not a model server",
    "sensing: depth and camera parameters added (env_cfg arx_x5_amp)",
    "seeds are evaluation seeds of one system, not training seeds",
    FIRST_LAYOUTS_DIFFERENCE,
    ("retries only before the robot's first motion: after it nothing is replayed, and an "
     "episode that breaks is a failure scored 0"),
    ("AMP's idle (1,800 s) and episode (21,600 s) limits and an agent's own failure end an "
     "episode as a failure scored 0"),
    ("agent input: RoboDojo's instruction (with the task recipe only under the robodojo_recipe "
     "overlay), plus the robot and camera description"),
    "the agent sees the live step budget (step_lim), as on every AMP benchmark",
    ("after the agent ends its episode the arms hold still until RoboDojo ends it, and "
     "success can still register then"))
AGGREGATION_TITLE = "RoboDojo aggregation"
SEED_SCORE_METHOD = (
    "the public leaderboard's rule on one seed: a task's success rate is its successes over its "
    "episodes (a Generalization task pooling its standard and _random configs, an unscored "
    "episode counting 0), a dimension's the mean over its tasks, the overall the mean over the "
    "dimensions")

BENCHMARK = Benchmark(name="robodojo", task_sets=(TASK_SET,),
                      default_corpus=SEED_CORPUS.format(2), embodiments={TASK_SET: EMBODIMENT})


@dataclass(frozen=True)
class Task:
    """One runnable RoboDojo config: its dimension, trials per seed and motion budget."""

    name: str
    base: str
    random: bool
    dimension: str
    eval_nums: int
    step_lim: int
    robot_config: str
    scene_config: str
    layouts: dict[int, int]
    data_source: str

    @property
    def episode_length_s(self) -> float:
        return self.step_lim * CONTROL_DT

    @property
    def est_s(self) -> float:
        return self.step_lim * ACTION_WALL_S + START_UP_S

    @property
    def attributes(self) -> list[str]:
        return [self.dimension] + (["random"] if self.random else [])


@dataclass(frozen=True)
class Manifest:
    path: str
    robodojo_commit: str
    pins: dict[str, str]
    seeds: tuple[int, ...]
    tasks: dict[str, Task]
    common: dict


def manifest_path() -> Path:
    return DEFAULT_MANIFEST


def _fail(path, why: str) -> NoReturn:
    raise CorpusError(f"bad robodojo manifest {path}: {why}")


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _sha(path, d: dict, key: str, where: str = "") -> str:
    v = d.get(key)
    if not isinstance(v, str) or not re.fullmatch(r"[0-9a-f]{40}", v):
        _fail(path, f"{where}{key} {v!r} is not a full commit sha")
    return v


def _task(path, d) -> Task:
    if not isinstance(d, dict):
        _fail(path, "a task is not an object")
    name = d.get("name")
    if not isinstance(name, str) or not re.fullmatch(r"\w+", name):
        _fail(path, f"task name {name!r} is not a RoboDojo config name")
    where = f"task {name}"
    if d.get("dimension") not in DIMENSIONS:
        _fail(path, f"{where}: dimension {d.get('dimension')!r} is not one of "
                    f"{', '.join(DIMENSIONS)}")
    for key in ("eval_nums", "step_lim"):
        if not _is_int(d.get(key)) or d[key] < 1:
            _fail(path, f"{where}: {key} must be a positive int, got {d.get(key)!r}")
    for key in ("base", "robot_config", "scene_config", "data_source"):
        if not isinstance(d.get(key), str) or not d[key].strip():
            _fail(path, f"{where}: {key} must be a non-empty string")
    if not isinstance(d.get("random"), bool):
        _fail(path, f"{where}: random must be a bool, got {d.get('random')!r}")
    if name != d["base"] + ("_random" if d["random"] else ""):
        _fail(path, f"{where}: a {'random' if d['random'] else 'base'} config of {d['base']!r} is "
                    f"named {d['base'] + ('_random' if d['random'] else '')}")
    raw = d.get("layouts")
    if not isinstance(raw, dict) or sorted(raw) != [str(s) for s in SEEDS]:
        _fail(path, f"{where}: layouts must count the files of every seed "
                    f"{', '.join(str(s) for s in SEEDS)}, got {raw!r}")
    layouts = {}
    for seed in SEEDS:
        n = raw[str(seed)]
        if not _is_int(n) or n < d["eval_nums"]:
            _fail(path, f"{where}: seed {seed} has {n!r} layouts, fewer than its "
                        f"{d['eval_nums']} trials (eval_nums)")
        layouts[seed] = n
    return Task(name=name, base=d["base"], random=d["random"], dimension=d["dimension"],
                eval_nums=d["eval_nums"], step_lim=d["step_lim"], robot_config=d["robot_config"],
                scene_config=d["scene_config"], layouts=layouts, data_source=d["data_source"])


def parse(d, path: str = "<manifest>") -> Manifest:
    """A schema-1 manifest dict, validated."""
    if not isinstance(d, dict):
        _fail(path, "expected an object")
    if d.get("schema") != SCHEMA:
        _fail(path, f"schema {d.get('schema')!r} is not {SCHEMA} (RoboDojo's official task "
                    f"inventory); {REGENERATE}")
    commit = _sha(path, d, "robodojo_commit")
    pins = d.get("pins")
    if not isinstance(pins, dict) or sorted(pins) != sorted(PIN_KEYS):
        _fail(path, f"pins must be exactly {', '.join(PIN_KEYS)}, got {pins!r}")
    pins = {k: _sha(path, pins, k, "pins: ") for k in PIN_KEYS}
    if pins["robodojo"] != commit:
        _fail(path, f"pins.robodojo {pins['robodojo']} is not robodojo_commit {commit}")
    if d.get("task_set") != TASK_SET:
        _fail(path, f"task_set {d.get('task_set')!r} is not {TASK_SET}")
    if d.get("seeds") != list(SEEDS):
        _fail(path, f"seeds {d.get('seeds')!r} are not RoboDojo's evaluation seeds {list(SEEDS)}")
    if not isinstance(d.get("common"), dict):
        _fail(path, "common must be _task.yml's common block")
    raw = d.get("tasks")
    if not isinstance(raw, list) or not raw:
        _fail(path, "expected a non-empty tasks list")
    tasks: dict[str, Task] = {}
    for body in raw:
        t = _task(path, body)
        if t.name in tasks:
            _fail(path, f"task {t.name} is listed twice")
        tasks[t.name] = t
    return Manifest(path=str(path), robodojo_commit=commit, pins=pins, seeds=SEEDS, tasks=tasks,
                    common=d["common"])


@functools.lru_cache(maxsize=8)
def _load(path: str, _mtime_ns: int) -> Manifest:
    try:
        text = Path(path).read_text()
    except OSError as e:
        _fail(path, f"cannot read it: {e}")
    try:
        d = json.loads(text)
    except ValueError as e:
        _fail(path, f"not JSON: {e}")
    return parse(d, path)


def load(path: str | Path | None = None) -> Manifest:
    """The manifest at `path` (default manifest_path()), re-read when the file changes."""
    p = Path(path) if path is not None else manifest_path()
    try:
        mtime = p.stat().st_mtime_ns
    except OSError:
        raise CorpusError(f"no robodojo manifest at {p} ({REGENERATE})") from None
    return _load(str(p), mtime)


def known_corpora() -> list[str]:
    """Every robodojo corpus name: one per evaluation seed."""
    return [SEED_CORPUS.format(s) for s in SEEDS]


def listed_corpora() -> list[str]:
    """The corpora listings show; every robodojo corpus is one."""
    return known_corpora()


def corpus_names() -> list[str]:
    """Every robodojo corpus the manifest can run; [] without a manifest."""
    try:
        load()
    except CorpusError:
        return []
    return known_corpora()


def _seed(name: str) -> int:
    """The evaluation seed a corpus name runs."""
    for s in SEEDS:
        if name == SEED_CORPUS.format(s):
            return s
    raise UnknownCorpus(name, listed_corpora())


def tasks_of(name: str) -> list[Task]:
    """The corpus's configs, in the manifest's order."""
    _seed(name)
    return list(load().tasks.values())


def task(name: str) -> Task:
    """One task of the manifest, by its RoboDojo config name."""
    return _lookup(name)


def _lookup(name: str) -> Task:
    m = load()
    t = m.tasks.get(name)
    if t is None:
        raise CorpusError(f"no robodojo task {name!r} in {m.path}")
    return t


def catalog_tasks() -> list[dict]:
    """Every task of the manifest, as the backend declares it; RoboDojo sends its own instruction."""
    return [{"name": t.name, "instruction": "", "attributes": t.attributes,
             "episode_length_s": t.episode_length_s} for t in load().tasks.values()]


def address(task: str, episode: int) -> tuple[int, int]:
    """(seed, layout) of unit (task, episode): seed index x eval_nums + layout, fixed before the run."""
    t = _lookup(task)
    n = t.eval_nums
    if not _is_int(episode) or not 0 <= episode < len(SEEDS) * n:
        raise CorpusError(f"{task} has {len(SEEDS) * n} units ({len(SEEDS)} seeds x {n} trials); "
                          f"episode {episode!r} is none")
    index, layout = divmod(episode, n)
    seed = SEEDS[index]
    if layout >= t.layouts[seed]:
        raise CorpusError(f"{task} seed {seed} has {t.layouts[seed]} layouts in the manifest; "
                          f"episode {episode} asks for layout {layout}")
    return seed, layout


def unit_spec(task: str, episode: int) -> dict:
    """What the launcher runs for unit (task, episode)."""
    t = _lookup(task)
    seed, layout = address(task, episode)
    return {"task": task, "seed": seed, "layout": layout, "trials_per_seed": t.eval_nums,
            "step_lim": t.step_lim, "robot_config": t.robot_config,
            "scene_config": t.scene_config, "dimension": t.dimension}


def task_set_of(name: str) -> str:
    """RoboDojo's one task set; an unknown name gets it too and fails the slot's catalog check."""
    return TASK_SET


def corpus_tasks(name: str) -> list[str]:
    """The task names --task takes: RoboDojo's 42 base tasks."""
    return list(_bases(tasks_of(name)))


def _bases(listed: list[Task]) -> dict[str, list[Task]]:
    """Base task name -> its configs; RoboDojo's 42 base tasks are these keys."""
    out: dict[str, list[Task]] = {}
    for t in listed:
        out.setdefault(t.base, []).append(t)
    return out


def _select(name: str, tasks: tuple[str, ...] | None) -> list[Task]:
    """The configs the run covers: every one of the corpus, or those of the base tasks --task names."""
    listed = tasks_of(name)
    if tasks is None:
        return listed
    wanted = set(tasks)
    halves = [t for t in listed if t.random and t.name in wanted]
    if halves:
        raise CorpusError(
            f"--task on {name} names base tasks, each of which runs both configs of a "
            f"Generalization task: pass {', '.join(sorted({t.base for t in halves})[:5])} "
            f"instead of {', '.join(sorted(t.name for t in halves)[:5])}")
    unknown = sorted(wanted - set(_bases(listed)))
    if unknown:
        raise CorpusError(f"tasks not in {name}: {unknown[:10]}")
    if not tasks:
        raise CorpusError("tasks, when given, must name at least one task")
    return [t for t in listed if t.base in wanted]


def _split(n: int, t: Task, configs: int) -> int:
    """How many of a base task's n episodes run on config `t`."""
    if configs < 2:
        return n
    return n // 2 if t.random else (n + 1) // 2


def _capacity(configs: list[Task]) -> int:
    """The most episodes of one base task a single seed holds."""
    if len(configs) < 2:
        return configs[0].eval_nums
    standard = min((t.eval_nums for t in configs if not t.random), default=0)
    random = min((t.eval_nums for t in configs if t.random), default=0)
    return min(2 * standard, 2 * random + 1)


def _chosen(listed: list[Task], seed: int, episodes: int | None,
            layouts: tuple[int, ...]) -> dict[str, list[int]]:
    """Episode ids per config for the layouts --layouts names, each of them on every config."""
    if episodes is not None:
        raise CorpusError("--layouts and --episodes are exclusive: the layouts are the episodes")
    if not layouts or min(layouts) < 0:
        raise CorpusError(f"--layouts must name one or more layout ids >= 0, got {list(layouts)}")
    over = sorted(f"{t.name} (0-{t.eval_nums - 1})" for t in listed if max(layouts) >= t.eval_nums)
    if over:
        raise CorpusError(f"layout {max(layouts)} is not a scored layout of "
                          f"{', '.join(over[:5])}{' ...' if len(over) > 5 else ''}: a seed scores "
                          f"{STANDALONE_TRIALS} layouts per task ({PAIRED_HALF} per config of a "
                          "Generalization task)")
    i = SEEDS.index(seed)
    return {t.name: [i * t.eval_nums + k for k in sorted(set(layouts))] for t in listed}


def _plan(name: str, listed: list[Task], episodes: int | None,
          layouts: tuple[int, ...] | None = None) -> dict[str, list[int]]:
    """Episode ids per config: the seed's first `episodes` layouts of each base task, or `layouts`."""
    if layouts is not None:
        return _chosen(listed, _seed(name), episodes, tuple(layouts))
    bases = _bases(listed)
    if episodes is None:
        cap = min(_capacity(cs) for cs in bases.values())
        raise CorpusError(f"{name} runs one seed's first N layouts of each task: pass --episodes "
                          f"1-{cap}")
    n = int(episodes)
    if n < 1:
        raise CorpusError(f"episodes must be >= 1, got {n}")
    over = sorted(f"{b} ({_capacity(cs)})" for b, cs in bases.items() if n > _capacity(cs))
    if over:
        raise CorpusError(f"{n} episodes per task exceed the trials one seed has of "
                          f"{', '.join(over[:5])}{' ...' if len(over) > 5 else ''}: a seed holds "
                          f"{STANDALONE_TRIALS} per task ({PAIRED_HALF} + {PAIRED_HALF} for a "
                          "Generalization task)")
    i = SEEDS.index(_seed(name))
    return {t.name: [i * t.eval_nums + k for k in range(_split(n, t, len(bases[t.base])))]
            for t in listed}


def expand(name: str, episodes: int | None = None, tasks: tuple[str, ...] | None = None,
           layouts: tuple[int, ...] | None = None) -> list[Unit]:
    """The corpus's units: the first `episodes` layouts of each base task (or `layouts`), longest
    horizon first."""
    listed = _select(name, tasks)
    plan = _plan(name, listed, episodes, layouts)
    units = [Unit(t.name, k, TASK_SET, t.est_s) for t in listed for k in plan[t.name]]
    return sorted(units, key=lambda u: (-u.est_s, u.task, u.episode))


def _one_or_map(values: dict[str, int]) -> int | dict[str, int]:
    distinct = set(values.values())
    return next(iter(distinct)) if len(distinct) == 1 else dict(values)


def corpus_meta(name: str, episodes: int | None = None, tasks: tuple[str, ...] | None = None,
                layouts: tuple[int, ...] | None = None) -> dict:
    """What a run of the corpus depends on: its tasks, seed, trials, pins and deviations."""
    m, seed = load(), _seed(name)
    listed = _select(name, tasks)
    plan = _plan(name, listed, episodes, layouts)
    per_task = (f"{int(episodes)} per task" if layouts is None
                else f"layouts {','.join(str(k) for k in sorted(set(layouts)))}")
    shown = [t for t in listed if plan[t.name]]
    dimensions: dict[str, int] = {}
    for configs in _bases(shown).values():
        dim = configs[0].dimension
        dimensions[dim] = dimensions.get(dim, 0) + 1
    return {"name": name, "task_set": TASK_SET, "tasks": len(_bases(shown)),
            "episodes_per_task": int(episodes) if layouts is None else None,
            **({} if layouts is None else {"layouts": sorted(set(layouts))}),
            "all_tasks": len(listed) == len(m.tasks),
            "seeds": [seed],
            "trials_per_seed": _one_or_map({t.name: len(plan[t.name]) for t in shown}),
            "units": sum(len(ks) for ks in plan.values()),
            "dimensions": dimensions, "all_dimensions": list(DIMENSIONS),
            "task_dimensions": {t.name: {"dimension": t.dimension, "base": t.base}
                                for t in shown},
            "aggregation": {"title": f"{AGGREGATION_TITLE} (seed {seed}, {per_task})",
                            "method": SEED_SCORE_METHOD,
                            "reference": f"robodojo@{m.robodojo_commit} {SUMMARISER}",
                            "unit_block": TASK_SET,
                            "paired_half": PAIRED_HALF, "standalone_trials": STANDALONE_TRIALS},
            "robodojo_commit": m.robodojo_commit, "pins": dict(m.pins),
            "protocol_differences": differences(layouts)}


def differences(layouts: tuple[int, ...] | None = None) -> list[str]:
    """PROTOCOL_DIFFERENCES as a run has them: a --layouts run names the layouts it chose."""
    if layouts is None:
        return list(PROTOCOL_DIFFERENCES)
    chosen = (f"one seed and its layouts {layout_span(sorted(set(layouts)))} per task, not "
              "RoboDojo's 3 seeds x each task's trials, and no refill: an abandoned layout is "
              "not replaced, so it leaves its task short")
    return [chosen if d == FIRST_LAYOUTS_DIFFERENCE else d for d in PROTOCOL_DIFFERENCES]


def layout_span(layouts) -> str:
    """Layout ids as text: 3, 3-5 or 1, 4, 7-9."""
    runs: list[list[int]] = []
    for k in layouts:
        if runs and k == runs[-1][1] + 1:
            runs[-1][1] = k
        else:
            runs.append([k, k])
    return ", ".join(str(a) if a == b else f"{a}-{b}" for a, b in runs)


def seed_score(tallies: Mapping[str, Mapping]) -> dict:
    """RoboDojo's leaderboard numbers for a one-seed run, from summary.json's per-config tallies."""
    m = load()
    per_task: dict[str, list[int]] = {}
    for name, t in tallies.items():
        cfg = m.tasks.get(name)
        if cfg is None:
            continue
        acc = per_task.setdefault(cfg.base, [0, 0])
        acc[0] += int(t.get("succeeded") or 0)
        acc[1] += int(t.get("units") or 0) - int(t.get("missing") or 0)
    tasks = {b: {"dimension": m.tasks[b].dimension, "succeeded": s, "episodes": n,
                 "success_rate": s / n} for b, (s, n) in sorted(per_task.items()) if n}
    dims = {}
    for d in DIMENSIONS:
        rates = [t["success_rate"] for t in tasks.values() if t["dimension"] == d]
        if rates:
            dims[d] = {"tasks": len(rates), "success_rate": sum(rates) / len(rates)}
    overall = sum(v["success_rate"] for v in dims.values()) / len(dims) if dims else None
    return {"success_rate": overall, "dimensions_present": len(dims),
            "dimensions_total": len(DIMENSIONS), "dimensions": dims, "tasks": tasks}
