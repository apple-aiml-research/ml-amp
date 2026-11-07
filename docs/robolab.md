# RoboLab

This repository runs AMP1 on [RoboLab](https://github.com/NVlabs/RoboLab) v0.3.1 by RoboLab's own
rules.

## Running

- RoboLab is the v0.3.1 checkout in `external/robolab` (commit `9db0aaf09d9f`; commands in
  [reproducibility.md](reproducibility.md), "Benchmark versions"), with its `isaac50` extra
  (Isaac Sim 5.0.0, Isaac Lab 2.2.0).
- The tasks are RoboLab's 120 benchmark tasks, RoboLab-120 in the paper (`--task-set benchmark`,
  the default), posed with `default`, `vague` or `specific` instructions (`--instruction-type`,
  default `default`), 10 episodes per task (`--num-runs 10`, the default).

## Where results go

`--output` is the run folder, `<out>` below. `run_id` is `<job_id>__<config>`: `job_id` defaults to
the `--output` folder's name, and `<config>` is `<harness>__<model>[-<reasoning>]__<prompt>`, plus
`__<vision>-<vdm_model>` for a text-only run (see Sensory ablations).

RoboLab's own output, one folder per Isaac process (`slot<i>`), merged per harness configuration
at the end:

```text
<out>/robolab/slot<i>/<run_id>/
    episode_results.jsonl            RoboLab's rows for the tasks this folder ran
    <Task>/env_cfg.json              once per task
    <Task>/run_<k>.hdf5              the recording of episode k
    <Task>/log_<k>_env0.json         the event log
    <Task>/<instruction>_<k>.mp4     the sensor video, and ..._<k>_viewport.mp4 the viewport video
<out>/robolab/merged/<run_id>/       the merged results, plus results.json
```

Per-episode records and traces of this repository, per harness configuration:

```text
<out>/<config>/agentic/run_config.json, status.json  the configuration's settings and a tally of its units
<out>/<config>/agentic/units/<Task>__<k:04d>.json    one record per episode: status, score, commits
<out>/<config>/agentic/episodes/<Task>__<episode_id>/trace/{agent,eval,kernel,robot__robolab}.jsonl
```

`<out>/robolab/merged/<run_id>/` holds only RoboLab's own content: it is the folder to compare and
share.

When all tasks are done, the launcher merges each configuration's folders with RoboLab's
`analysis/compile_results.py` into `<out>/robolab/merged/<run_id>/`, prints
`analysis/read_results.py`'s tables, and exits non-zero if a task is missing episodes.

## Resume

Rerun the same command with the same `--output` and `--gpus`: finished tasks are skipped, and
RoboLab's own resume skips the episodes of a started task that already have rows. A run with
different arguments needs a different `--output`. A task hit by an infrastructure fault is
restarted up to `--max-restarts` times (default 3).

## Sensory ablations (Figures 5C and 5D)

The ablations change only what the agent senses. The harness is the basic one and the prompt is
`amp1`; each removal is a perception overlay. A harness named `<harness>_<tag>_<tag>...` is
generated on first use from `src/amp/harnesses/_overlays/`, applying the overlays in the order
written.

| Tag | Removes |
|---|---|
| `noperc` | the perception tools (T): the scene model, segmentation, object edits, object queries and base motion are refused, and observations carry no objects |
| `rgb_only` | depth (D): the robot server drops every camera's depth on arrival, so nothing is perceived from depth |
| `nocal` | calibration (C): observations carry no camera intrinsics or extrinsics |
| `scenecam` | the wrist camera, leaving the scene cameras |
| `wrist` | the scene cameras, leaving the wrist camera |

The harness for each condition, by camera set:

| Condition | All cameras | Scene cameras only | Wrist camera only |
|---|---|---|---|
| RGB+C+D+T | `amp1_basic` | `amp1_basic_scenecam` | `amp1_basic_wrist` |
| RGB+C+D | `amp1_basic_noperc` | `amp1_basic_noperc_scenecam` | `amp1_basic_noperc_wrist` |
| RGB+C | `amp1_basic_rgb_only` | `amp1_basic_scenecam_rgb_only` | `amp1_basic_wrist_rgb_only` |
| RGB | `amp1_basic_noperc_rgb_only_nocal` | `amp1_basic_noperc_rgb_only_nocal_scenecam` | `amp1_basic_noperc_rgb_only_nocal_wrist` |
| Text | `amp1_basic_noperc_rgb_only_nocal` with `--vision vdm_strict` | `amp1_basic_noperc_rgb_only_nocal_scenecam` with `--vision vdm_strict` | `amp1_basic_noperc_rgb_only_nocal_wrist` with `--vision vdm_strict` |

RGB+C uses `rgb_only` alone: the perception tools stay, but perceive no objects without depth. In
Text, the column's RGB harness is run by the text-only agent (`--vision vdm_strict`): a vision
model (`--vdm-model`, a second instance of the agent's own model) describes the cameras in
words, after every motion and on `show(camera=<name>, caption=<question>)`, and the agent's code is
refused the images.

Figure 5C is the all-cameras column for Claude Opus 5.5, Sonnet 5.5, Haiku 5.5 and Opus 4.5;
Figure 5D is the three columns for Opus 5.5. Both run at `--reasoning high` with `--num-runs 10`.
The four image rungs of Figure 5C for Opus 5.5, then its Text rung:

```bash
uv run amp eval robolab \
    --harness amp1_basic,amp1_basic_noperc,amp1_basic_rgb_only,amp1_basic_noperc_rgb_only_nocal \
    --prompt amp1 --model opus-5.5 --reasoning high --num-runs 10 --output runs/fig5c-opus-5.5
uv run amp eval robolab --harness amp1_basic_noperc_rgb_only_nocal --vision vdm_strict \
    --vdm-model opus-5.5 --prompt amp1 --model opus-5.5 --reasoning high --num-runs 10 \
    --output runs/fig5c-text-opus-5.5
```

`--vision` applies to every harness of a run, so a Text run is its own `amp eval robolab`.
[reproducing-the-paper.md](reproducing-the-paper.md) has every run of both figures.
