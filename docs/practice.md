# Learning from practice

AMP1+ is AMP1 with a **workspace**, the paper's memory M: a folder of notes, code and data that it
writes in debriefs after practice episodes and finds as `./workspace` in later episodes (a task's
stored workspace is its notebook). The paper learns one on RoboDojo ([robodojo.md](robodojo.md)):

- **Few-shot** (`amp practice fewshot`, Figure 4F–H): one workspace per task, practised in
  batches with a debrief after each, then frozen and evaluated.
- **Continual** (`amp practice continual`, Figure 4I–K): one workspace shared by every task,
  learned one task at a time, evaluated at checkpoints.

Practice runs on seed-0 layouts and its debriefs see only each episode's official success, how it
ended and its steps. Evaluations never write to the workspace; the reported numbers are on seed 2.

## 1. How the workspace reaches the agent

`amp eval robodojo --workspace DIR` gives every episode the folder as `./workspace`. Related flags:

- `--workspace-sha SHA` refuses a folder holding any other version.
- `--workspace-notebook 2` tells the agent its edits during an episode are not kept.
- `--workspace-max-bytes N`: one task's limit, from 60,000 (the default) to 131,072.
- `--shared-workspace`: one workspace for every task of the run, up to 256,000 bytes.
- `--workspaces FILE`: each task gets its own notebook. FILE is a JSON list of `{task, folder, sha}`
  as in the paper's `checkpoints.json`, folders relative to FILE's directory (null for no notebook).
  The agent is not told its task.
- `--layouts 5,6,7` runs those layout ids instead of the first N.

## 2. The debrief

After practice, the same agent continues one practice episode's conversation with the official
results and its earlier episodes under `history/`, edits `./workspace` with `python` and
`view_image`, and saves it with `done` (at most `debrief.max_turns` turns, 100 in the paper). Its
`python` kernel has no robot and only a guard against writes outside `workspace/`, subprocesses and
network: run practice in a container or VM if you do not trust the model.

## 3. Few-shot practice

```bash
uv run amp practice fewshot init runs/practice/fewshot --study src/amp/practice/studies/fewshot.json
uv run amp practice fewshot run runs/practice/fewshot --eval # practice, then each frozen task's evaluation
uv run amp practice fewshot results runs/practice/fewshot --none runs/paper/fig4-zero-shot
```

The protocol ([`src/amp/practice/studies/fewshot.json`](../src/amp/practice/studies/fewshot.json)):
batches of 4 practice episodes on seed 0 (a generalization task runs both configs of each layout),
a debrief after each ending in keep, change or stop; at a stop or after 48 episodes the workspace
freezes and is evaluated on seed 2, 50 episodes per task.

Optional study keys: `batch` (1 or even), `budget` (practice episodes per task),
`workspace_max_bytes`, `workspace_format` (`"code"`: only `.py` files; `"notes"`: no `.py`), `gpus`,
`runs`, `model` and `reasoning`.

`run` is resumable.
`results --none BASE` pairs each task's evaluation with a no-memory run of the same harness
configuration (the zero-shot run of [reproducing-the-paper.md](reproducing-the-paper.md)) on the
units both scored: paired difference, exact sign-flip p and task-clustered bootstrap 95% CI.

### Variations of few-shot learning

The default study with one setting changed, in [`src/amp/practice/studies/`](../src/amp/practice/studies/):

| Study file | Change | Experiment |
|---|---|---|
| `fewshot_code.json`, `fewshot_notes.json` | `workspace_format`: code only, notes only | memory restricted during practice |
| `fewshot_haiku.json`, `fewshot_qwen.json` | `model`: `haiku-5.5` (reasoning `high`), `qwen3.8-flash-next` (reasoning `max`) | a model's own practice |
| `fewshot_b1.json`, `fewshot_b8.json` | `batch`: 1, 8 (default 4) | batch size |

The Qwen study needs a self-hosted server ([models.md](models.md)). Two related evaluations
use the default study's frozen workspaces with `eval --cut code|notes` (memory restricted only
at evaluation), or with `eval --model` and `--reasoning` set to Haiku's or Qwen's values from the
table (their model with Opus's memory).

### Evaluating frozen runs: eval and select

`eval` evaluates a run's frozen workspaces in one `amp eval robodojo --workspaces` job, into a
folder that `results` reads like a run folder:

```bash
uv run amp practice fewshot eval {RUN | --notebooks NB} --corpus robodojo-seed2 --episodes 50 \
    --output DIR [--model M --reasoning R] [--cut code|notes] [--round K[,K...]] [--reuse DIR,...]
```

- `--notebooks NB`: a notebooks folder with its `checkpoints.json`
  ([the paper's](#the-papers-few-shot-notebooks)) instead of RUN; not with `--round`.
- `--model`, `--reasoning`: another model with the run's workspaces.
- `--cut code` keeps each workspace's `.py` files, `--cut notes` the rest; a task left with no file
  runs without a workspace.
- `--round K`: each task's workspace after K practice rounds (the frozen one if it stopped earlier;
  K = 0 is none), into `DIR/k<K>/`; `results DIR` gives the curve.
- `--reuse DIR,...` reuses matching earlier evaluations; `--dry-run` prints each job's command.

`select` is the paper's best of several runs: per task, the run with the highest seed-1 success on
the units every run scored, ties going to fewer practice episodes, then to the run named first.
Evaluations go under `SEL/seed1/` (or come from `--seed1 DIR,...`). `SEL` is a run folder for
`eval` and `results`; `SEL/selection.json` records the picks.

```bash
uv run amp practice fewshot select RUN1 RUN2 ... --corpus robodojo-seed1 --episodes 50 --output SEL \
    [--pair NO_MEMORY_RUN]
```

`--pair` counts only the units the no-memory run also scored, as the paper did (2,091 of 2,100).

`results` reports RoboDojo's leaderboard success rate (mean over the five categories, a category
the mean over its tasks, generalization the mean of standard and random, an unscored episode a
failure) with a 95% bootstrap CI over tasks within categories. With `--none` it also gives model
calls relative to the no-memory run: per setting, the mean over episodes successful in both,
combined by geometric means in the same structure.

### The paper's few-shot notebooks

The paper's frozen few-shot notebooks (per task, the best of five runs) are in
[`src/amp/practice/notebooks/fewshot_best5/`](../src/amp/practice/notebooks/fewshot_best5/).
Evaluate and score all 42 without practice:

```bash
uv run amp practice fewshot eval --notebooks src/amp/practice/notebooks/fewshot_best5 \
    --corpus robodojo-seed2 --episodes 50 --output DIR
uv run amp practice fewshot results DIR --none ZERO_SHOT_RUN
```

The agent settings come from the default study; `--model`, `--reasoning` and `--cut` apply. The
folder's README has the one-task `amp eval robodojo --workspace` form. Subfolders `k2/`, `k4/` and
`k8/` hold the notebooks after 2, 4 and 8 rounds (Figure 4F's earlier points):
`eval --notebooks src/amp/practice/notebooks/fewshot_best5/k4` gives k = 4.

## 4. Continual practice

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed1 --episodes 5 \
    --gpus auto --output runs/practice/k0-seed1-x5    # optional: the seed-1 checkpoints' baseline
uv run amp practice continual init runs/practice/continual --chains 1 \
    --study src/amp/practice/studies/continual.json
uv run amp practice continual run runs/practice/continual --wait
uv run amp practice continual full-eval runs/practice/continual --steps 10,20,30,42 --corpora robodojo-seed2
uv run amp practice continual results runs/practice/continual
```

The protocol ([`src/amp/practice/studies/continual.json`](../src/amp/practice/studies/continual.json)):

- **Places.** One workspace (up to 120,000 bytes) is learned over the 42 tasks in one pass, in a
  random order. A place is one task: one episode on seed-0 layout 5 (both configs for a
  generalization task), then one debrief.
- **Checkpoints.** Every 10 places and at the end, the workspace is evaluated on the tasks learned
  so far and, as a second job, on the rest: seed 1, 5 episodes per task, paired with the study's
  `baseline`. The baseline is optional; `results --none RUN` pairs the checkpoints with a
  no-memory run given at results time. These seed-1 evaluations are the study's own check; the
  paper does not report them.
- **Full evaluation.** `full-eval` evaluates the chosen checkpoints (`--steps`) on every task of
  `--corpora`, 50 episodes per task. `results` reports it against `full_baselines` with RoboDojo's
  success rate and writes Figure 4I–K's data to `memory_curve_seed2.csv`, `efficiency_seed2.csv`
  and `matrix_seed2.csv` in the chain folder. The paper reports steps 10, 20, 30 and 42 on seed 2.

`init --chains 1` makes the paper's chain, `chain_00`. `run` is resumable.

### The paper's checkpoints

The paper's chain's notebooks after 10, 20, 30 and 42 tasks are in
[`src/amp/practice/notebooks/continual_chain00/`](../src/amp/practice/notebooks/continual_chain00/).
Its run went on into a second pass, which the paper does not report.
Evaluate one with `amp eval robodojo --workspace` and `--shared-workspace`; the folder's README has
the command and the paper's seed-2 numbers.

## 5. Where things go

The state folder (`runs/practice/fewshot`, `runs/practice/continual`) holds `config.json`,
`state.json`, every workspace version under `versions/<sha>/`, the debriefs and
`results.{json,md}`. Every job is an `amp eval robodojo` run in `<runs>/<tag>-<time>-<id>/` (study
key `runs`, default `runs/practice`).
