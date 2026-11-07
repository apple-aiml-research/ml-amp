#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CaP-X ("capgym"): its official LIBERO, robosuite and BEHAVIOR protocols, from a schema-3 manifest."""

from __future__ import annotations

import functools
import json
import os
import re
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import NoReturn

from .base import Benchmark, CorpusError, Unit, UnknownCorpus

SCHEMA = 3
TASK_SET = "capgym_libero"
MANIFEST_ENV = "AMP_CAPGYM_MANIFEST"
DEFAULT_MANIFEST = Path(__file__).with_name("capgym_manifest.json")
GENERATOR = "AMP, from the CaP-X checkout at capx_commit"
REGENERATE = "regenerate it from CaP-X at the pinned commit"
FIRST_TRIAL = 1
TAKES_LAYOUTS = True

BATCH_SOURCE = "capx/envs/scripts/run_libero_batch.py"
BATCH_SUITES = ("libero_object_swap", "libero_object_task", "libero_goal_swap",
                "libero_goal_task", "libero_spatial_swap", "libero_spatial_task")
PRO_SUITES = BATCH_SUITES
KNOWN_SUITES = frozenset(PRO_SUITES)
LIBERO_PRO_DATASET = "zhouxueyang/LIBERO-Pro"


ROBOSUITE_TASK_SETS = {"cube_lifting": "capgym_robosuite_cube", "cube_stack": "capgym_robosuite_cube",
                       "cube_restack": "capgym_robosuite_cube",
                       "nut_assembly": "capgym_robosuite_nut",
                       "spill_wipe": "capgym_robosuite_wipe",
                       "two_arm_lift": "capgym_robosuite_two_arm_lift",
                       "two_arm_handover": "capgym_robosuite_two_arm_handover"}
ROBOSUITE_SOURCES = {
    "cube_lifting": "env_configs/cube_lifting/franka_robosuite_cube_lifting.yaml",
    "cube_stack": "env_configs/cube_stack/franka_robosuite_cube_stack.yaml",
    "cube_restack": "env_configs/cube_restack/franka_robosuite_cube_restack.yaml",
    "nut_assembly": "env_configs/nut_assembly/franka_robosuite_nut_assembly.yaml",
    "spill_wipe": "env_configs/spill_wipe/franka_robosuite_spill_wipe.yaml",
    "two_arm_lift": "env_configs/two_arm_lift/franka_robosuite_two_arm_lift.yaml",
    "two_arm_handover": "env_configs/two_arm_handover/two_arm_handover.yaml"}
BEHAVIOR_TASK_SET = "capgym_b1k"
BEHAVIOR_SOURCE = "env_configs/r1pro"
BEHAVIOR_ENV_CLASS = "capx.envs.simulators.r1pro_b1k.R1ProBehaviourLowLevel"
BEHAVIOR_VARIANTS = ("_oracle", "_multiturn_vdm")
BEHAVIOR_PREFIXES = ("r1pro_",)
BEHAVIOR_PAPER_TRIALS = {"r1pro_pick_up_radio": 25, "r1pro_pick_up_trash": 25}
BEHAVIOR_PAPER_NOTE = ("25 trials per task, as the CaP-X paper reports (Table 3); the shipped "
                       "radio config's trials: 20 is a CLI default")
BEHAVIOR_EXECUTION = {"torchdynamo_disabled": False, "torch_compile": "enabled"}
PROCESS_PER_TASK = (BEHAVIOR_TASK_SET,)
LOAD_TIMEOUT_S = {BEHAVIOR_TASK_SET: 1800.0}
NO_ESTIMATE = 0.0
BEHAVIOR_CONSTRUCTION = "constructs"
BEHAVIOR_SCORING = "capx_lift"

EMBODIMENTS = {"capgym_libero": "capx_libero_panda",
               "capgym_robosuite_cube": "capx_robosuite_panda",
               "capgym_robosuite_nut": "capx_robosuite_panda_nut",
               "capgym_robosuite_wipe": "capx_robosuite_panda_wipe",
               "capgym_robosuite_two_arm_lift": "capx_robosuite_two_panda_lift",
               "capgym_robosuite_two_arm_handover": "capx_robosuite_two_panda_handover",
               BEHAVIOR_TASK_SET: "capx_b1k_r1pro"}
SUPPORTED_TASK_SETS = ("capgym_libero", "capgym_robosuite_cube", "capgym_robosuite_nut",
                       "capgym_robosuite_two_arm_lift", "capgym_robosuite_two_arm_handover",
                       "capgym_robosuite_wipe", BEHAVIOR_TASK_SET)


@dataclass(frozen=True)
class Row:
    """A capgym corpus: one official CaP-Gym protocol."""

    name: str
    source: str
    batch: bool
    configs: tuple[str, ...] = ()
    robosuite: bool = False
    behavior: bool = False
    tasks: tuple[str, ...] = ()
    trials: tuple[tuple[str, int], ...] = ()
    trials_note: str = ""


ROWS = (
    Row("libero-pro", BATCH_SOURCE, batch=True),
    Row("robosuite", "env_configs", batch=False, configs=tuple(ROBOSUITE_SOURCES.values()),
        robosuite=True),
    Row("behavior-1k", BEHAVIOR_SOURCE, batch=False, behavior=True,
        tasks=tuple(BEHAVIOR_PAPER_TRIALS), trials=tuple(BEHAVIOR_PAPER_TRIALS.items()),
        trials_note=BEHAVIOR_PAPER_NOTE),
)
_ROWS = {r.name: r for r in ROWS}
BENCHMARK = Benchmark(name="capgym", task_sets=SUPPORTED_TASK_SETS,
                      default_corpus="libero-pro", embodiments=EMBODIMENTS)


@dataclass(frozen=True)
class Task:
    """A LIBERO task as CaP-X loads it."""

    name: str
    suite: str
    task_id: int
    bddl_file: str
    instruction: str
    init_states: int


@dataclass(frozen=True)
class Setting:
    """How an official protocol runs a task: its horizon, rate, trials and API flags."""

    source: str
    max_steps: int
    control_freq: int
    trials: int | None
    privileged: bool
    apis: tuple[str, ...]
    base_config: str | None = None

    @property
    def episode_length_s(self) -> float:
        return self.max_steps / self.control_freq


@dataclass(frozen=True)
class Entry:
    """A unit-level task: a LIBERO task under one protocol's setting."""

    name: str
    task: Task
    setting: Setting

    @property
    def trials(self) -> int:
        return self.task.init_states if self.setting.trials is None else self.setting.trials

    @property
    def task_set(self) -> str:
        return TASK_SET

    @property
    def episode_length_s(self) -> float:
        return self.setting.episode_length_s


@dataclass(frozen=True)
class RobosuiteTask:
    """A robosuite task as its official base config builds it; units are (task, trial 1..trials)."""

    name: str
    task_set: str
    source: str
    low_level: str
    env_class: str
    privileged: bool
    enable_render: bool
    viser_debug: bool
    max_steps: int
    control_freq: int
    trials: int
    instruction: str
    instruction_source: str

    @property
    def episode_length_s(self) -> float:
        return self.max_steps / self.control_freq


@dataclass(frozen=True)
class BehaviorTask:
    """A BEHAVIOR-1K base config as CaP-X builds it; units are (task, trial 1..trials)."""

    name: str
    task_set: str
    source: str
    low_level: str
    env_class: str
    low_level_kwargs: dict
    controller_cfg: str
    activity_name: str
    scene_model: str
    official_scene: str
    trials: int
    instruction: str
    instruction_source: str
    scoring: str
    construction: str

    @property
    def episode_length_s(self) -> None:
        return None


@dataclass(frozen=True)
class Manifest:
    path: str
    capx_commit: str
    libero_pro_commit: str
    libero_pro_dataset: str
    suites: dict[str, tuple[Task, ...]]
    batch: Setting
    robosuite: dict[str, RobosuiteTask] = field(default_factory=dict)
    robosuite_provenance: dict[str, dict] = field(default_factory=dict)
    behavior: dict[str, BehaviorTask] = field(default_factory=dict)
    b1k_commit: str | None = None
    behavior_excluded: tuple[str, ...] = ()

    def entries(self) -> dict[str, Entry]:
        return {t.name: Entry(t.name, t, self.batch)
                for s in PRO_SUITES for t in self.suites.get(s, ())}


def manifest_path() -> Path:
    return Path(os.environ.get(MANIFEST_ENV) or DEFAULT_MANIFEST)


def _fail(path, why: str) -> NoReturn:
    raise CorpusError(f"bad capgym manifest {path}: {why}")


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _sha(path, d: dict, key: str) -> str:
    v = d.get(key)
    if not isinstance(v, str) or not re.fullmatch(r"[0-9a-f]{40}", v):
        _fail(path, f"{key} {v!r} is not a full commit sha")
    return v


def _task(path, suite: str, d) -> Task:
    if not isinstance(d, dict):
        _fail(path, f"suite {suite}: a task is not an object")
    where = f"suite {suite}: task {d.get('name')!r}"
    for key in ("task_id", "init_states"):
        if not _is_int(d.get(key)):
            _fail(path, f"{where}: {key} must be an int, got {d.get(key)!r}")
    tid = d["task_id"]
    if d.get("name") != f"{suite}.{tid:02d}":
        _fail(path, f"{where}: task {tid} of {suite} is named {suite}.{tid:02d}")
    bddl = d.get("bddl_file")
    if not isinstance(bddl, str) or not re.fullmatch(r"[\w.-]+/[\w.-]+\.bddl", bddl):
        _fail(path, f"{where}: bddl_file must be <problem folder>/<file>.bddl, got {bddl!r}")
    if not isinstance(d.get("instruction"), str) or not d["instruction"].strip() \
            or d["instruction"] != d["instruction"].strip() or "\n" in d["instruction"]:
        _fail(path, f"{where}: instruction must be the non-empty BDDL :language text")
    if d["init_states"] < 1:
        _fail(path, f"{where}: init_states must be >= 1")
    return Task(name=d["name"], suite=suite, task_id=tid, bddl_file=bddl,
                instruction=d["instruction"], init_states=d["init_states"])


def _setting(path, where: str, d, *, batch: bool) -> Setting:
    if not isinstance(d, dict):
        _fail(path, f"{where}: expected an object")
    for key in ("max_steps", "control_freq"):
        if not _is_int(d.get(key)) or d[key] < 1:
            _fail(path, f"{where}: {key} must be a positive int, got {d.get(key)!r}")
    trials = d.get("trials")
    if batch:
        if trials != "init_states":
            _fail(path, f"{where}: the batch protocol runs one trial per init state "
                        f"(trials: \"init_states\"), got {trials!r}")
        trials = None
    elif not _is_int(trials) or trials < 1:
        _fail(path, f"{where}: trials must be a positive int, got {trials!r}")
    if d.get("privileged") is not False:
        _fail(path, f"{where}: privileged must be false, got {d.get('privileged')!r}")
    apis = d.get("apis")
    if not isinstance(apis, list) or not apis or not all(isinstance(a, str) and a for a in apis):
        _fail(path, f"{where}: apis must be a non-empty list of API class names")
    config = d.get("base_config")
    if batch and not (isinstance(config, str) and config.startswith("env_configs/")):
        _fail(path, f"{where}: base_config must be the env_configs path the env is built from")
    return Setting(source=d.get("source"), max_steps=d["max_steps"],
                   control_freq=d["control_freq"], trials=trials, privileged=d["privileged"],
                   apis=tuple(apis), base_config=config if batch else None)


def _suites(path, raw) -> dict[str, tuple[Task, ...]]:
    if not isinstance(raw, dict):
        _fail(path, "expected a suites object")
    suites = {}
    for suite, body in raw.items():
        if suite not in KNOWN_SUITES:
            _fail(path, f"suite {suite!r} is in no capgym protocol; known: "
                        f"{', '.join(sorted(KNOWN_SUITES))}")
        if not isinstance(body, dict) or not isinstance(body.get("tasks"), list):
            _fail(path, f"suite {suite}: expected an object with a tasks list")
        if body.get("task_set", TASK_SET) != TASK_SET:
            _fail(path, f"suite {suite}: task_set {body.get('task_set')!r} is not {TASK_SET}")
        tasks = tuple(_task(path, suite, t) for t in body["tasks"])
        if not tasks or [t.task_id for t in tasks] != list(range(len(tasks))):
            _fail(path, f"suite {suite}: tasks must be every task_id 0..n-1, in order")
        suites[suite] = tasks
    return suites


ROBOSUITE_INT_FIELDS = ("max_steps", "control_freq", "trials")
ROBOSUITE_BOOL_FIELDS = ("enable_render", "viser_debug")
_ENV_CLASS = r"capx\.envs\.simulators\.\w+\.\w+"
_CODE_ENV = r"capx\.envs\.tasks\.franka\.\w+\.\w+"
_SOURCE_LINES = r"capx/envs/(?:tasks/franka|simulators)/\w+\.py:\d+(?:-\d+)?"
_GOAL_LINE = r"capx/envs/tasks/franka/\w+\.py:\d+"


def _robosuite_task(path, name: str, d) -> tuple[RobosuiteTask, dict]:
    if name not in ROBOSUITE_TASK_SETS:
        _fail(path, f"robosuite task {name!r} is in no capgym protocol; known: "
                    f"{', '.join(ROBOSUITE_TASK_SETS)}")
    where = f"robosuite task {name}"
    if not isinstance(d, dict):
        _fail(path, f"{where}: expected an object")
    if d.get("task_set") not in ROBOSUITE_TASK_SETS.values():
        _fail(path, f"{where}: task_set {d.get('task_set')!r} is not one of "
                    f"{', '.join(sorted(set(ROBOSUITE_TASK_SETS.values())))}")
    if d["task_set"] != ROBOSUITE_TASK_SETS[name]:
        _fail(path, f"{where}: task_set {d['task_set']!r} is not {ROBOSUITE_TASK_SETS[name]}")
    if d.get("source") != ROBOSUITE_SOURCES[name]:
        _fail(path, f"{where}: source {d.get('source')!r} is not {ROBOSUITE_SOURCES[name]}")
    for key in ROBOSUITE_INT_FIELDS:
        if not _is_int(d.get(key)) or d[key] < 1:
            _fail(path, f"{where}: {key} must be a positive int, got {d.get(key)!r}")
    for key in ROBOSUITE_BOOL_FIELDS:
        if not isinstance(d.get(key), bool):
            _fail(path, f"{where}: {key} must be a bool, got {d.get(key)!r}")
    if d.get("privileged") is not False:
        _fail(path, f"{where}: privileged must be false, got {d.get('privileged')!r}")
    for key in ("low_level", "instruction"):
        if not isinstance(d.get(key), str) or not d[key].strip():
            _fail(path, f"{where}: {key} must be a non-empty string")
    if d["instruction"] != d["instruction"].strip() or "\n" in d["instruction"]:
        _fail(path, f"{where}: instruction must be the one goal sentence, got {d['instruction']!r}")
    for key, pattern in (("env_class", _ENV_CLASS), ("code_env", _CODE_ENV),
                         ("instruction_source", _GOAL_LINE),
                         ("code_env_source", _SOURCE_LINES),
                         ("max_steps_source", _SOURCE_LINES)):
        if not isinstance(d.get(key), str) or not re.fullmatch(pattern, d[key]):
            _fail(path, f"{where}: {key} must match {pattern}, got {d.get(key)!r}")
    task = RobosuiteTask(name=name, **{k: d[k] for k in (
        "task_set", "source", "low_level", "env_class", "privileged", "enable_render",
        "viser_debug", "max_steps", "control_freq", "trials", "instruction",
        "instruction_source")})
    return task, {k: d[k] for k in ("code_env", "code_env_source", "max_steps_source")}


def _robosuite(path, raw) -> tuple[dict[str, RobosuiteTask], dict[str, dict]]:
    if not isinstance(raw, dict) or not isinstance(raw.get("tasks"), dict):
        _fail(path, "expected a robosuite object with a tasks object")
    tasks, extra = {}, {}
    for name, body in raw["tasks"].items():
        tasks[name], extra[name] = _robosuite_task(path, name, body)
    if not tasks:
        _fail(path, "robosuite has no tasks")
    return tasks, extra


BEHAVIOR_STR_FIELDS = ("low_level", "controller_cfg", "activity_name", "scene_model",
                       "official_scene", "instruction", "instruction_source")
BEHAVIOR_KEYS = frozenset((
    "task_set", "source", "low_level", "env_class", "low_level_kwargs", "controller_cfg",
    "activity_name", "scene_model", "official_scene", "trials", "instruction",
    "instruction_source", "scoring", "construction", "construction_note"))
BEHAVIOR_SECTION_KEYS = frozenset(("b1k_commit", "tasks", "excluded_variants"))
_BEHAVIOR_GOAL_SOURCE = r"(?:env_configs/r1pro/\w+\.yaml|capx/envs/tasks/r1pro/\w+\.py):\d+"


def _behavior_task(path, name: str, d) -> BehaviorTask:
    where = f"behavior task {name}"
    if not isinstance(d, dict):
        _fail(path, f"{where}: expected an object")
    if not name.startswith(BEHAVIOR_PREFIXES) or name.endswith(BEHAVIOR_VARIANTS):
        _fail(path, f"{where}: not a base config stem of {BEHAVIOR_SOURCE}")
    if d.get("task_set") != BEHAVIOR_TASK_SET:
        _fail(path, f"{where}: task_set {d.get('task_set')!r} is not {BEHAVIOR_TASK_SET}")
    if d.get("source") != f"{BEHAVIOR_SOURCE}/{name}.yaml":
        _fail(path, f"{where}: source {d.get('source')!r} is not {BEHAVIOR_SOURCE}/{name}.yaml")
    if d.get("env_class") != BEHAVIOR_ENV_CLASS:
        _fail(path, f"{where}: env_class {d.get('env_class')!r} is not {BEHAVIOR_ENV_CLASS}")
    extra = sorted(set(d) - BEHAVIOR_KEYS)
    if extra:
        _fail(path, f"{where}: {', '.join(extra)} not allowed (a CaP-Bench field, or none of "
                    "env construction and scoring reads it)")
    if not isinstance(d.get("low_level_kwargs"), dict):
        _fail(path, f"{where}: low_level_kwargs must be an object")
    for key in BEHAVIOR_STR_FIELDS:
        if not isinstance(d.get(key), str) or not d[key].strip():
            _fail(path, f"{where}: {key} must be a non-empty string")
    if not _is_int(d.get("trials")) or d["trials"] < 1:
        _fail(path, f"{where}: trials must be a positive int, got {d.get('trials')!r}")
    if "\n" in d["instruction"].strip() or d["instruction"] != d["instruction"].strip():
        _fail(path, f"{where}: instruction must be the one-line task goal")
    if not re.fullmatch(_BEHAVIOR_GOAL_SOURCE, d["instruction_source"]):
        _fail(path, f"{where}: instruction_source must match {_BEHAVIOR_GOAL_SOURCE}, got "
                    f"{d['instruction_source']!r}")
    for key, want in (("scoring", BEHAVIOR_SCORING), ("construction", BEHAVIOR_CONSTRUCTION)):
        if d.get(key) != want:
            _fail(path, f"{where}: {key} {d.get(key)!r} is not {want!r}")
    return BehaviorTask(name=name, **{k: d[k] for k in (
        "task_set", "source", "low_level", "env_class", "low_level_kwargs", "controller_cfg",
        "activity_name", "scene_model", "official_scene", "trials", "instruction",
        "instruction_source", "scoring", "construction")})


def _behavior(path, raw) -> tuple[dict[str, BehaviorTask], str | None, tuple[str, ...]]:
    if raw is None:
        return {}, None, ()
    if not isinstance(raw, dict) or not isinstance(raw.get("tasks"), dict) or not raw["tasks"]:
        _fail(path, "expected a behavior object with a non-empty tasks object")
    if set(raw) - BEHAVIOR_SECTION_KEYS:
        _fail(path, f"behavior: {sorted(set(raw) - BEHAVIOR_SECTION_KEYS)} not allowed")
    excluded = raw.get("excluded_variants", [])
    if not isinstance(excluded, list) or not all(
            isinstance(k, str) and k.endswith(BEHAVIOR_VARIANTS) for k in excluded):
        _fail(path, "behavior: excluded_variants lists the variant stems skipped by name")
    tasks = {n: _behavior_task(path, n, b) for n, b in raw["tasks"].items()}
    return tasks, _sha(path, raw, "b1k_commit"), tuple(excluded)


def parse(d, path: str = "<manifest>") -> Manifest:
    """A schema-3 manifest dict, validated."""
    if not isinstance(d, dict):
        _fail(path, "expected an object")
    if d.get("schema") != SCHEMA:
        _fail(path, f"schema {d.get('schema')!r} is not {SCHEMA} (CaP-Gym's official "
                    f"LIBERO-PRO and robosuite protocols); {REGENERATE}")
    commit, pro = _sha(path, d, "capx_commit"), _sha(path, d, "libero_pro_commit")
    ds = d.get("libero_pro_dataset")
    if not isinstance(ds, dict) or ds.get("repo") != LIBERO_PRO_DATASET:
        _fail(path, f"libero_pro_dataset must be {{repo: {LIBERO_PRO_DATASET!r}, revision}}, "
                    f"got {ds!r}")
    dataset = _sha(path, ds, "revision")
    suites = _suites(path, d.get("suites"))
    batch = _setting(path, "batch", d.get("batch"), batch=True)
    if batch.source != BATCH_SOURCE:
        _fail(path, f"batch: source {batch.source!r} is not {BATCH_SOURCE}")
    if d["batch"].get("suites") != list(BATCH_SUITES):
        _fail(path, f"batch: suites {d['batch'].get('suites')!r} are not {BATCH_SOURCE}'s "
                    f"{list(BATCH_SUITES)}")
    if "added_suites" in d["batch"]:
        _fail(path, "batch: added_suites not allowed, libero-pro is the six suites of "
                    f"{BATCH_SOURCE}")
    if d.get("configs"):
        _fail(path, "configs: the env_configs/libero configs are not run here; "
                    f"{REGENERATE}")
    rs, provenance = _robosuite(path, d.get("robosuite"))
    bh, b1k, excluded = _behavior(path, d.get("behavior"))
    return Manifest(path=str(path), capx_commit=commit, libero_pro_commit=pro,
                    libero_pro_dataset=dataset, suites=suites, batch=batch,
                    robosuite=rs, robosuite_provenance=provenance, behavior=bh, b1k_commit=b1k,
                    behavior_excluded=excluded)


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
        raise CorpusError(f"no capgym manifest at {p} (it is generated from CaP-X at the pinned commit)") from None
    return _load(str(p), mtime)


def known_corpora() -> list[str]:
    """Every capgym corpus name, whether or not this checkout's manifest can run it."""
    return [r.name for r in ROWS]


def corpus_names() -> list[str]:
    """Every capgym corpus the manifest can run; [] without a manifest."""
    out = []
    for r in ROWS:
        try:
            entries_of(r.name)
        except CorpusError:
            continue
        out.append(r.name)
    return out


def _row(name: str) -> Row:
    r = _ROWS.get(name)
    if r is None:
        raise UnknownCorpus(name, known_corpora())
    return r


def _batch_suites(row: Row, m: Manifest) -> list[str]:
    """The batch suites the corpus runs: all of them, or a CorpusError naming the missing ones."""
    missing = [s for s in PRO_SUITES if s not in m.suites]
    if missing:
        raise CorpusError(
            f"{row.name} is all {len(PRO_SUITES)} suites of {BATCH_SOURCE}, but {m.path} lacks "
            f"{', '.join(missing)} (LIBERO-PRO ships their BDDL and init files on Hugging Face, "
            f"its README:165-173; regenerate the manifest from CaP-X at the pinned commit once "
            "they are installed)")
    return list(PRO_SUITES)


def _robosuite_entries(row: Row, m: Manifest) -> list[RobosuiteTask]:
    missing = [n for n in ROBOSUITE_TASK_SETS if n not in m.robosuite]
    if missing:
        raise CorpusError(f"{row.name} is the {len(ROBOSUITE_TASK_SETS)} robosuite base configs, "
                          f"but {m.path} lacks {', '.join(missing)}; {REGENERATE}")
    every = list(m.robosuite.values())
    todo = sorted({t.task_set for t in every} - set(SUPPORTED_TASK_SETS))
    if todo:
        raise CorpusError(
            f"{row.name} is all {len(every)} robosuite base configs, but the task sets "
            f"{', '.join(todo)} have no adapter yet (supported: {', '.join(SUPPORTED_TASK_SETS)})")
    return every


def _behavior_entries(row: Row, m: Manifest) -> list[BehaviorTask]:
    if not m.behavior:
        raise CorpusError(f"{row.name}: {m.path} has no behavior section; "
                          f"{REGENERATE}")
    absent = [n for n in row.tasks if n not in m.behavior]
    if absent:
        raise CorpusError(f"{row.name} is {', '.join(row.tasks)}, but {m.path} lacks "
                          f"{', '.join(absent)}; {REGENERATE}")
    listed = [m.behavior[n] for n in row.tasks]
    trials = dict(row.trials)
    return [replace(t, trials=trials[t.name]) if t.name in trials else t
            for t in listed]


def entries_of(name: str) -> list[Entry | RobosuiteTask | BehaviorTask]:
    """The corpus's unit-level tasks, in protocol order."""
    row, m = _row(name), load()
    if row.behavior:
        return _behavior_entries(row, m)
    if row.robosuite:
        return _robosuite_entries(row, m)
    every = m.entries()
    return [every[t.name] for s in _batch_suites(row, m) for t in m.suites[s]]


def entry(name: str) -> Entry:
    """One unit-level task of the manifest, by its unit task name."""
    m = load()
    e = m.entries().get(name)
    if e is None:
        raise CorpusError(f"no capgym task {name!r} in {m.path}")
    return e


def task(name: str) -> Task:
    """The LIBERO task behind a unit task name."""
    return entry(name).task


def robosuite_tasks(manifest: Manifest | None = None) -> dict[str, RobosuiteTask]:
    """Every robosuite task of the manifest (default: the loaded one), by name."""
    return dict((manifest or load()).robosuite)


def _robosuite_spec(t: RobosuiteTask, task: str, episode: int) -> dict:
    if not _is_int(episode) or not 0 <= episode < t.trials:
        raise CorpusError(f"{task} has {t.trials} official episodes; episode {episode!r} is none")
    trial = episode + FIRST_TRIAL
    return {"task": task, "episode": episode, "trial": trial, "seed": trial,
            "task_set": t.task_set, "family": "robosuite", "source": t.source,
            "low_level": t.low_level, "env_class": t.env_class, "privileged": t.privileged,
            "enable_render": t.enable_render, "viser_debug": t.viser_debug,
            "max_steps": t.max_steps, "control_freq": t.control_freq,
            "episode_length_s": t.episode_length_s, "instruction": t.instruction,
            "instruction_source": t.instruction_source}


def behavior_tasks(manifest: Manifest | None = None) -> dict[str, BehaviorTask]:
    """Every BEHAVIOR task of the manifest (default: the loaded one), by name."""
    return dict((manifest or load()).behavior)


def _behavior_spec(t: BehaviorTask, task: str, episode: int) -> dict:
    if not _is_int(episode) or not 0 <= episode < t.trials:
        raise CorpusError(f"{task} has {t.trials} official episodes; episode {episode!r} is none")
    trial = episode + FIRST_TRIAL
    return {"task": task, "episode": episode, "trial": trial, "seed": trial,
            "options": {"trial": trial}, "task_set": t.task_set, "family": "behavior",
            "source": t.source, "low_level": t.low_level, "env_class": t.env_class,
            "low_level_kwargs": dict(t.low_level_kwargs), "controller_cfg": t.controller_cfg,
            "activity_name": t.activity_name, "scene_model": t.scene_model,
            "scoring": t.scoring, "construction": t.construction,
            "max_steps": None, "episode_length_s": None, "instruction": t.instruction,
            "instruction_source": t.instruction_source}


def unit_spec(task: str, episode: int) -> dict:
    """What the backend runs for unit (task, episode): CaP-X trial episode + 1, seeded with it."""
    m = load()
    if task in m.behavior:
        return _behavior_spec(m.behavior[task], task, episode)
    rs = m.robosuite.get(task)
    if rs is not None:
        return _robosuite_spec(rs, task, episode)
    e = entry(task)
    if not _is_int(episode) or not 0 <= episode < e.trials:
        raise CorpusError(f"{task} has {e.trials} official episodes; episode {episode!r} is none")
    trial = episode + FIRST_TRIAL
    return {"task": task, "episode": episode, "trial": trial, "seed": trial,
            "init_state": (trial - 1) % e.task.init_states, "suite": e.task.suite,
            "task_id": e.task.task_id, "bddl_file": e.task.bddl_file,
            "instruction": e.task.instruction, "max_steps": e.setting.max_steps,
            "control_freq": e.setting.control_freq,
            "episode_length_s": e.setting.episode_length_s, "source": e.setting.source,
            "privileged": e.setting.privileged}


def task_set_of(name: str) -> str:
    """The task set that registers unit task `name`: a robosuite task's (the manifest checks the
    map), a BEHAVIOR config stem's, else LIBERO's. An unknown name gets LIBERO's and fails at the
    slot's catalog check."""
    if name.startswith(BEHAVIOR_PREFIXES):
        return BEHAVIOR_TASK_SET
    return ROBOSUITE_TASK_SETS.get(name, TASK_SET)


def corpus_tasks(name: str) -> list[str]:
    return [e.name for e in entries_of(name)]


def _select(name: str, tasks: tuple[str, ...] | None) -> list[Entry | RobosuiteTask]:
    listed = entries_of(name)
    if tasks is None:
        return listed
    unknown = sorted(set(tasks) - {e.name for e in listed})
    if unknown:
        raise CorpusError(f"tasks not in {name}: {unknown[:10]}")
    if not tasks:
        raise CorpusError("tasks, when given, must name at least one task")
    wanted = set(tasks)
    return [e for e in listed if e.name in wanted]


def _trials(listed: list[Entry | RobosuiteTask], episodes: int | None) -> dict[str, int]:
    """Trials per task: the official count, or the user's --episodes (at most the official)."""
    if episodes is None:
        return {e.name: e.trials for e in listed}
    n = int(episodes)
    if n < 1:
        raise CorpusError(f"episodes must be >= 1, got {n}")
    over = [f"{e.name} ({e.trials})" for e in listed if n > e.trials]
    if over:
        raise CorpusError(f"{n} episodes per task exceed the official episodes of "
                          f"{', '.join(over[:5])}{' ...' if len(over) > 5 else ''}")
    return {e.name: n for e in listed}


def _chosen(listed: list[Entry | RobosuiteTask], episodes: int | None,
            layouts: tuple[int, ...]) -> dict[str, list[int]]:
    """Episode ids per task for --layouts: those ids of every chosen task (trial id + 1)."""
    if episodes is not None:
        raise CorpusError("--layouts and --episodes are exclusive: the layouts are the episodes")
    if not layouts or min(layouts) < 0:
        raise CorpusError(f"--layouts must name one or more trial ids >= 0, got {list(layouts)}")
    unseeded = sorted(e.name for e in listed if isinstance(e, RobosuiteTask))
    if unseeded:
        raise CorpusError(f"--layouts cannot choose robosuite scenes ({', '.join(unseeded[:3])}"
                          f"{' ...' if len(unseeded) > 3 else ''}): its reset ignores the seed, so "
                          "every run is a new random scene; use --episodes")
    over = sorted(f"{e.name} (0-{e.trials - 1})" for e in listed if max(layouts) >= e.trials)
    if over:
        raise CorpusError(f"episode {max(layouts)} is not an official episode of "
                          f"{', '.join(over[:5])}{' ...' if len(over) > 5 else ''}")
    return {e.name: list(layouts) for e in listed}


def _episodes(listed: list[Entry | RobosuiteTask], episodes: int | None,
              layouts: tuple[int, ...] | None) -> dict[str, list[int]]:
    """Episode ids per task: the chosen layouts, or 0..trials-1."""
    if layouts is not None:
        return _chosen(listed, episodes, tuple(layouts))
    return {name: list(range(n)) for name, n in _trials(listed, episodes).items()}


def expand(name: str, episodes: int | None = None, tasks: tuple[str, ...] | None = None,
           layouts: tuple[int, ...] | None = None) -> list[Unit]:
    """Every official episode of the corpus (or the first `episodes`, or the `layouts`), longest
    horizon first."""
    _row(name)
    listed = _select(name, tasks)
    ids = _episodes(listed, episodes, layouts)
    units = [Unit(e.name, k, e.task_set, NO_ESTIMATE if e.episode_length_s is None
                  else e.episode_length_s)
             for e in listed for k in ids[e.name]]
    return sorted(units, key=lambda u: (-u.est_s, u.task, u.episode))


def _one_or_map(values: dict[str, int]) -> int | dict[str, int]:
    distinct = set(values.values())
    return next(iter(distinct)) if len(distinct) == 1 else dict(values)


def _robosuite_meta(row: Row, m: Manifest, listed: list[RobosuiteTask],
                    trials: dict[str, int]) -> dict:
    official = {t.name: t.trials for t in listed}
    run = _one_or_map(trials)
    whole = len(listed) == len(ROBOSUITE_TASK_SETS)
    task_sets = {t.name: t.task_set for t in listed}
    sets = sorted(set(task_sets.values()))
    meta = {"name": row.name, "task_set": sets[0] if len(sets) == 1 else None,
            "task_sets": task_sets, "tasks": len(listed),
            "episodes_per_task": run if isinstance(run, int) else None, "all_tasks": whole,
            "source": row.source, "trials": run, "official_trials": _one_or_map(official),
            "partial": not whole or trials != official, "first_trial": FIRST_TRIAL,
            "capx_commit": m.capx_commit,
            "config_sources": [t.source for t in listed],
            "configs": {t.name: {"source": t.source, "task_set": t.task_set,
                                 "low_level": t.low_level, "env_class": t.env_class,
                                 "privileged": t.privileged, "enable_render": t.enable_render,
                                 "viser_debug": t.viser_debug, "max_steps": t.max_steps,
                                 "control_freq": t.control_freq, "trials": t.trials}
                        for t in listed}}
    return meta


def _behavior_meta(row: Row, m: Manifest, listed: list[BehaviorTask],
                   trials: dict[str, int]) -> dict:
    official = {t.name: t.trials for t in listed}
    run = _one_or_map(trials)
    protocol = [m.behavior[n] for n in row.tasks]
    left_out = {t.name: t.construction for t in protocol if t.name not in {x.name for x in listed}}
    meta = {"name": row.name, "task_set": BEHAVIOR_TASK_SET, "tasks": len(listed),
            "episodes_per_task": run if isinstance(run, int) else None,
            "all_tasks": not left_out, "source": row.source, "trials": run,
            "official_trials": _one_or_map(official),
            "partial": bool(left_out) or trials != official,
            "first_trial": FIRST_TRIAL,
            "capx_commit": m.capx_commit, "b1k_commit": m.b1k_commit,
            "max_steps": None, "episode_length_s": None, "execution": dict(BEHAVIOR_EXECUTION),
            "left_out": left_out,
            "left_out_reasons": {},
            "excluded_variants": list(m.behavior_excluded),
            "config_sources": [t.source for t in listed],
            "configs": {t.name: {"source": t.source, "activity_name": t.activity_name,
                                 "controller_cfg": t.controller_cfg, "scene_model": t.scene_model,
                                 "official_scene": t.official_scene, "scoring": t.scoring,
                                 "trials": t.trials, "construction": t.construction}
                        for t in listed}}
    if row.trials_note:
        meta["trials_note"] = row.trials_note
    return meta


LIBERO_TRIALS_NOTE = ("trials are seeded LIBERO placements (np.random.seed(t)); CaP-X #23/#27 "
                      "inherited as published")


def corpus_meta(name: str, episodes: int | None = None, tasks: tuple[str, ...] | None = None,
                layouts: tuple[int, ...] | None = None) -> dict:
    """What a run of the corpus depends on: its protocol, suites, trials and the pins."""
    row, m = _row(name), load()
    listed = _select(name, tasks)
    trials = {k: len(v) for k, v in _episodes(listed, episodes, layouts).items()}
    if row.behavior:
        meta = _behavior_meta(row, m, listed, trials)
    elif row.robosuite:
        meta = _robosuite_meta(row, m, listed, trials)
    else:
        meta = _libero_meta(row, m, name, listed, trials)
    if layouts is not None:
        meta["layouts"] = sorted(layouts)
    return meta


def _libero_meta(row: Row, m: Manifest, name: str, listed: list[Entry],
                 trials: dict[str, int]) -> dict:
    official = {e.name: e.trials for e in listed}
    whole = len(listed) == len(entries_of(name))
    run = _one_or_map(trials)
    suites: dict[str, int] = {}
    for e in listed:
        suites[e.task.suite] = suites.get(e.task.suite, 0) + 1
    meta = {"name": row.name, "task_set": TASK_SET, "tasks": len(listed),
            "episodes_per_task": run if isinstance(run, int) else None, "all_tasks": whole,
            "source": row.source,
            "suites": suites,
            "trials": run, "official_trials": _one_or_map(official),
            "partial": not whole or trials != official, "first_trial": FIRST_TRIAL,
            "capx_commit": m.capx_commit, "libero_pro_commit": m.libero_pro_commit,
            "libero_pro_dataset": {"repo": LIBERO_PRO_DATASET, "revision": m.libero_pro_dataset},
            "trials_note": LIBERO_TRIALS_NOTE}
    b = m.batch
    meta.update(base_config=b.base_config, max_steps=b.max_steps,
                control_freq=b.control_freq, privileged=b.privileged)
    return meta
