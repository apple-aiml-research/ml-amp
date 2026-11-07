#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp eval robodojo: RoboDojo units on local GPUs, through RoboDojo's own unmodified runner.

    amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --model opus-5.5 \\
        --corpus robodojo-seed2 --episodes 50 [--task T ...] \\
        --gpus auto|0,1 --output runs/<name>

One gateway is shared; each GPU has one slot: a robot server and a RoboDojo launcher, which runs
eval_policy.sh once per unit (task, seed, layout). Rerunning with the same --output skips the
units already final (docs/robodojo.md).

With --workspaces, one run evaluates many tasks' notebooks: each episode's agent gets its own
task's notebook, as a one-task --workspace run gives it (docs/practice.md).
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import os
import shutil
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

from amp.benchmarks import robodojo
from amp.contracts.eval import Arm, EvalProtocol
from amp.contracts.robot import SlotSpec

from . import capgym_official, gpus
from .capgym_official import Launcher
from .capgym_slot import CapgymSlot
from .provenance import ROBODOJO_ROOT_ENV
from .robolab_official import make_profile, stop_on_signals
from .store import write_json
from .study import Study, StudyError

BENCHMARK = "robodojo"
PROG = "amp eval robodojo"
SEED_SCORE_FILE = "robodojo_summary.json"
SLOTS_PER_GPU = 1
WORKSPACE_MAX_BYTES_LIMIT = 128 << 10
SHARED_WORKSPACE_MAX_BYTES = 256_000


def log(msg: str) -> None:
    capgym_official.log(msg, PROG)


class RoboDojoSlot(CapgymSlot):
    """A slot for RoboDojo's task set: Isaac Sim 5.1 on the slot's one physical GPU."""

    client_label = "the RoboDojo launcher"

    def gpu_mode(self, task_set: str | None = None) -> str:
        return "cuda"

    def spec(self, info_mode: str = "standard", cameras: str = "droid") -> SlotSpec:
        env = {"CUDA_VISIBLE_DEVICES": str(self.gpu_index), "CUDA_DEVICE_ORDER": "PCI_BUS_ID"}
        return SlotSpec(slot_id=self.slot_id, gpu=self.gpu_index, index=0,
                        server_port=self.server_port, cameras=cameras, info_mode=info_mode,
                        results_dir=str(self.results_dir), env=env, task_set=self.task_set)


@dataclass
class RoboDojoLauncher(Launcher):
    """capgym's launcher with RoboDojo's slots, gateway exemption and plan."""

    slot_factory: object = RoboDojoSlot
    prog = PROG

    def run(self) -> int:
        rc = super().run()
        self.write_seed_score()
        return rc

    def write_seed_score(self) -> dict:
        """RoboDojo's own numbers for a one-seed run, beside summary.json."""
        from .finalize import SUMMARY_FILE

        try:
            summary = json.loads((self.output / SUMMARY_FILE).read_text())
        except (OSError, ValueError) as e:
            self.log(f"no RoboDojo success rate: {SUMMARY_FILE} unreadable ({e})")
            return {}
        out = {"job_id": summary.get("job_id"), "corpus": self.study.corpus,
               "method": robodojo.SEED_SCORE_METHOD, "arms": {}}
        for arm, s in summary["arms"].items():
            score = out["arms"][arm] = robodojo.seed_score(s["tasks"])
            if score["success_rate"] is not None:
                self.log(f"{arm}: RoboDojo success rate {score['success_rate']:.1%} "
                         f"({score['dimensions_present']} of {score['dimensions_total']} "
                         "dimensions)")
        write_json(self.output / SEED_SCORE_FILE, out)
        return out

    def gateway_argv(self, config: Path) -> tuple[str, ...]:
        """The gateway's command: several slots may run episodes of one RoboDojo task at once."""
        from amp import benchmarks

        argv = (self.python, "-m", "amp.gateway", "--config", str(config), "--poll-s", "1.0",
                "--concurrent-task-sets", ",".join(benchmarks.get(BENCHMARK).task_sets),
                "--slot-task-sets", str(self.slot_task_sets_path))
        floors = benchmarks.gateway_load_timeouts()
        if floors:
            argv += ("--load-timeouts", ",".join(f"{k}={v:g}" for k, v in sorted(floors.items())))
        return argv

    def write_plan(self) -> None:
        self.output.mkdir(parents=True, exist_ok=True)
        write_json(self.node / "plan.json", {
            "job_id": self.study.job_id, "corpus": self.study.corpus,
            "episodes": self.study.episodes,
            **({"layouts": list(self.study.layouts)} if self.study.layouts is not None else {}),
            "units": len(self.study.units) * len(self.study.arms),
            "gpus": [g.uuid for g in self.gpu_list], "slots_per_gpu": self.slots_per_gpu,
            "busy_gpus": [g.uuid for g in self.gpu_list if g.busy],
            "gateway_port": self.gateway_port,
            "slots": [{"slot": s.slot_id, "gpu": s.gpu_uuid, "server_port": s.server_port,
                       "task_set": s.task_set} for s in self.slots]})


def make_backend(a: argparse.Namespace):
    from amp.robots.robodojo.backend import RoboDojoBackend

    return RoboDojoBackend(python=a.robodojo_python, xpl_python=a.xpl_python,
                           robodojo_root=a.robodojo_root)


def check_backend(backend) -> list[str]:
    """What stops the RoboDojo launcher from starting here: a missing checkout or interpreter."""
    from amp.robots.robodojo.backend import XPL_PYTHON_ENV, interpreter

    problems = []
    root = backend.root
    if not (root / "scripts" / "eval_policy.sh").is_file():
        problems.append(f"no RoboDojo checkout at {root} (make robodojo, or set $AMP_ROBODOJO_ROOT "
                        "or --robodojo-root)")
    elif not (root / "Assets" / "Eval_Layout").is_dir():
        problems.append(f"{root} has no Assets: download them (docs/robodojo.md, section 1)")
    for what, get in (("RoboDojo's", lambda: backend.python_for(BENCHMARK)),
                      ("XPolicyLab's", lambda: interpreter(backend.xpl_python, XPL_PYTHON_ENV))):
        try:
            py = get()
        except ValueError as e:
            problems.append(f"{what} interpreter: {e}")
            continue
        if not (os.path.isfile(py) or shutil.which(py)):
            problems.append(f"{what} interpreter {py} not found")
    return problems


def parser() -> argparse.ArgumentParser:
    from amp import benchmarks

    ap = argparse.ArgumentParser(prog=PROG, description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--harness", required=True,
                    help="comma-separated harness ids, one harness configuration each: amp1_basic, "
                         "amp1_assistive, or an overlay harness such as "
                         "amp1_basic_rgb_only_nocal_rdrecipe_rdnotes (the paper's RoboDojo input)")
    ap.add_argument("--prompt", default="amp1",
                    help="the system prompt id (default amp1, the AMP1 system prompt)")
    ap.add_argument("--model", required=True, help="a model key (docs/models.md)")
    ap.add_argument("--reasoning", default=None, help="reasoning effort (default: the model's)")
    ap.add_argument("--corpus", default=None,
                    help=f"default: {benchmarks.get(BENCHMARK).default_corpus}; "
                    + benchmarks.listing(BENCHMARK))
    ap.add_argument("--task", action="append", default=[],
                    help="run only these base tasks of the corpus (repeat or comma-separate)")
    ap.add_argument("--episodes", type=int, default=None,
                    help="the first N layouts of each base task at the corpus's seed (1-50)")
    ap.add_argument("--layouts", default=None,
                    help="instead of --episodes: these layout ids (comma-separated) of every config "
                         "of each base task at the corpus's seed; practice runs off the eval layouts")
    ap.add_argument("--gpus", default="auto",
                    help="auto (every NVIDIA GPU that is not busy), or GPU indices or UUIDs")
    ap.add_argument("--output", required=True, type=Path, help="the run folder; reuse to resume")
    ap.add_argument("--job-id", default=None, help="default: the --output folder's name")
    ap.add_argument("--sandbox", default="auto", choices=("auto", "bwrap"))
    ap.add_argument("--idle-timeout-s", type=float, default=1800.0)
    ap.add_argument("--episode-timeout-s", type=float, default=21600.0)
    ap.add_argument("--load-timeout-s", type=float, default=900.0)
    ap.add_argument("--robodojo-root", default=None,
                    help="the RoboDojo checkout (default $AMP_ROBODOJO_ROOT, else external/robodojo)")
    ap.add_argument("--robodojo-python", default=None,
                    help="RoboDojo's venv's python (default $AMP_ROBODOJO_PYTHON)")
    ap.add_argument("--xpl-python", default=None,
                    help="XPolicyLab's venv's python (default $AMP_ROBODOJO_XPL_PYTHON)")
    ap.add_argument("--max-retries", type=int, default=None,
                    help="re-runs of a unit that ended infra_error (default 2)")
    ws = ap.add_argument_group("workspace memory (docs/practice.md)")
    ws.add_argument("--workspace", metavar="DIR", type=Path,
                    help="a notebook folder: every episode's agent finds its files in ./workspace; "
                         "the run must cover one base task unless --shared-workspace")
    ws.add_argument("--workspace-sha", metavar="SHA",
                    help="with --workspace: the version the folder must hold "
                         "(amp.practice.workspace.sha_of)")
    ws.add_argument("--shared-workspace", action="store_true",
                    help=f"with --workspace: one notebook for every task of the run (up to "
                         f"{SHARED_WORKSPACE_MAX_BYTES:,} bytes)")
    ws.add_argument("--workspace-notebook", type=int, choices=[1, 2], default=None,
                    help="with --workspace or --workspaces: 2 tells the agent its edits are not "
                         "kept (practice)")
    ws.add_argument("--workspace-max-bytes", type=int, metavar="N", default=None,
                    help="with --workspace or --workspaces: the bytes one task's notebook may "
                         f"take, from 60,000 (the default) to {WORKSPACE_MAX_BYTES_LIMIT:,}")
    ws.add_argument("--workspaces", metavar="FILE", type=Path,
                    help="a notebooks manifest, a JSON list of {task, folder, sha} as the paper's "
                         "checkpoints.json: one run for many tasks, each episode's agent finding "
                         "its own task's notebook in ./workspace (a randomized variant its base "
                         "task's)")
    return ap


def notebook_limit(a: argparse.Namespace) -> int:
    """--workspace-max-bytes, checked: the bytes one task's notebook may take."""
    from amp.practice import workspace as W

    limit = W.MAX_BYTES if a.workspace_max_bytes is None else a.workspace_max_bytes
    if not W.MAX_BYTES <= limit <= WORKSPACE_MAX_BYTES_LIMIT:
        raise ValueError(f"--workspace-max-bytes {limit:,}: from {W.MAX_BYTES:,} (the default) to "
                         f"{WORKSPACE_MAX_BYTES_LIMIT:,}")
    return limit


def read_notebook(folder: Path, who: str, a: argparse.Namespace, sha: str | None, limit: int,
                  may_take: str, no_file: str) -> dict:
    """folder's notebook as a workspace: present, non-empty, sha and size checked, or ValueError."""
    from amp.practice import workspace as W

    if not folder.exists():
        raise ValueError(f"{who}: {folder} does not exist")
    if not folder.is_dir():
        raise ValueError(f"{who}: {folder} is not a directory")
    try:
        ws = W.with_notebook(W.encode(W.read_dir(folder)), a.workspace_notebook or 1)
    except (W.WorkspaceError, OSError) as e:
        raise ValueError(f"{who}: {e}") from None
    if not ws["files"]:
        raise ValueError(f"{who}: {folder} holds no file; {no_file}")
    if sha is not None and sha != ws["sha"]:
        raise ValueError(f"{who}: {folder} holds version {ws['sha'][:12]}, not the {sha[:12]} "
                         "its sha names")
    if W.size(ws) > limit:
        raise ValueError(f"{who}: {W.size(ws):,} bytes, past the {limit:,} {may_take} may take")
    return ws


def with_workspace(agent, a: argparse.Namespace, tasks: Sequence[str]):
    """The agent profile carrying --workspace's notebook; ValueError when the flags or the folder
    are refused."""
    from amp.practice import workspace as W

    if a.workspace is None:
        if a.workspace_sha or a.shared_workspace or (a.workspaces is None and (
                a.workspace_notebook is not None or a.workspace_max_bytes is not None)):
            raise ValueError("--workspace-sha, --shared-workspace, --workspace-notebook and "
                             "--workspace-max-bytes need --workspace")
        return agent
    if a.workspaces is not None:
        raise ValueError("--workspace and --workspaces are exclusive")
    if not a.workspace.is_dir():
        raise ValueError(f"--workspace {a.workspace}: not a directory")
    if a.shared_workspace:
        limit, may_take = SHARED_WORKSPACE_MAX_BYTES, "a shared workspace"
    else:
        limit, may_take = notebook_limit(a), "one task's notebook"
    ws = read_notebook(a.workspace, "--workspace", a, a.workspace_sha, limit, may_take,
                       "run without --workspace")
    if not a.shared_workspace and len(tasks) != 1:
        covers = f"{len(tasks)} tasks" if tasks else "every task"
        raise ValueError(f"--workspace is one task's: the run covers {covers}; pass one --task, "
                         "or add --shared-workspace")
    return W.with_workspace(agent, ws)


def with_workspaces(agent, a: argparse.Namespace, configs: Sequence[str]):
    """The agent profile carrying --workspaces' notebooks for the run's configs; ValueError if
    refused."""
    from amp.practice import workspace as W

    limit = notebook_limit(a)
    try:
        entries = W.read_manifest(a.workspaces)
    except W.WorkspaceError as e:
        raise ValueError(f"--workspaces {e}") from None
    base_of = {c: robodojo._lookup(c).base for c in configs}
    by_task = {}
    for task in sorted(set(base_of.values())):
        if task not in entries:
            raise ValueError(f"--workspaces {a.workspaces}: no entry for {task}")
        folder, sha = entries[task]
        if folder is not None:
            by_task[task] = read_notebook(folder, f"--workspaces {a.workspaces}: {task}", a, sha,
                                          limit, "one task's notebook", f"give {task} folder null")
    return W.with_table(agent, by_task,
                        {c: b if b in by_task else None for c, b in base_of.items()})


def build_study(a: argparse.Namespace, tasks: Sequence[str]) -> Study:
    from amp import benchmarks

    agent = with_workspace(make_profile(a.model, a.reasoning, a.prompt), a, tasks)
    arms = tuple(Arm(h.strip(), agent) for h in a.harness.split(",") if h.strip())
    proto = EvalProtocol(sandbox=a.sandbox, idle_timeout_s=a.idle_timeout_s,
                         episode_timeout_s=a.episode_timeout_s, load_timeout_s=a.load_timeout_s)
    _, corpus = benchmarks.resolve(BENCHMARK, a.corpus)
    job = a.job_id or Path(a.output).resolve().name
    layouts = (tuple(int(k) for k in a.layouts.split(",") if k.strip())
                if a.layouts is not None else None)
    study = Study(arms=arms, corpus=corpus, tasks=tuple(tasks) or None, episodes=a.episodes,
                  layouts=layouts, protocol=proto, job_id=job, code_ref="local",
                  benchmark=BENCHMARK)
    if a.workspaces is None:
        return study
    agent = with_workspaces(agent, a, sorted({u.task for u in study.units}))
    return dataclasses.replace(study, arms=tuple(Arm(x.harness, agent) for x in arms))


def main(argv: list[str] | None = None) -> int:
    ap = parser()
    a = ap.parse_args(argv)
    if a.episodes is not None and a.episodes < 1:
        ap.error("--episodes must be >= 1")
    tasks = [t.strip() for arg in a.task for t in arg.split(",") if t.strip()]
    try:
        study = build_study(a, tasks)
        units = study.units
    except (KeyError, ValueError, StudyError) as e:
        ap.error(str(e).strip("'\""))
    backend = make_backend(a)
    problems = check_backend(backend)
    if problems:
        ap.error("; ".join(problems))
    if not (os.environ.get("OMNI_KIT_ACCEPT_EULA") or "").strip():
        log("warning: OMNI_KIT_ACCEPT_EULA is not set; unless NVIDIA's Omniverse licence was "
            "accepted on this machine before, Isaac Sim stops at its licence prompt and every "
            "episode fails (docs/robodojo.md, section 1)")
    os.environ[ROBODOJO_ROOT_ENV] = str(backend.root)
    try:
        chosen, refused = gpus.select(a.gpus, gpus.query(), warn=log)
        for g, why in refused:
            log(f"refused {g.describe()}: {why}")
    except gpus.GpuError as e:
        ap.error(str(e))
    n_units = len(units) * len(study.arms)
    log(f"{len(study.arms)} harness configurations x {len(units)} units ({study.corpus}) on "
        f"{len(chosen)} GPUs x {SLOTS_PER_GPU} slot: {n_units} units")
    launcher = RoboDojoLauncher(study=study, backend=backend, output=a.output, gpu_list=chosen,
                                slots_per_gpu=SLOTS_PER_GPU, retries=a.max_retries)
    stop_on_signals()
    try:
        return launcher.run()
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
