# Reproducing the paper

The command for each result in the paper, by figure. Every run uses the AMP1 system prompt
(`--prompt amp1`) with one attempt per episode, and is zero-shot except Figure 4's runs after
practice.

## Before you start

- Follow the README's [Setup](../README.md#setup), with RoboLab in `external/robolab` and CaP-X in
  `external/cap-x` at the commits in [reproducibility.md](reproducibility.md), "Benchmark versions"
  (`make robolab`, `make capx`).
- RoboLab: its `isaac50` environment in `external/robolab/.venv` and `OMNI_KIT_ACCEPT_EULA=YES`
  ([quickstart.md](quickstart.md), sections 2 and 3); `uv run amp doctor` checks the environment.
- CaP-Gym: CaP-X's environments, the `AMP_CAPX_*` variables and, for Figure 2E, BEHAVIOR-1K
  ([capgym.md](capgym.md), section 1).
- RoboDojo: `make robodojo`, its two venvs, its Assets and the `AMP_ROBODOJO_*` variables
  ([robodojo.md](robodojo.md), section 1); `uv run amp doctor --robodojo` checks them.
- Model keys, API keys and self-hosted servers: [models.md](models.md).

Run the release commit unmodified, from this repository's root. Rerunning a command with the same
`--output` resumes it. Figure 5A runs each model at both of its paper settings, Figure 5B at the
higher one; every other run uses `--reasoning high`, except Qwen3.8-Flash-Next in the
few-shot variations (`--reasoning max`).

## Figure 2B: RoboLab-120, default, vague and specific instructions

```bash
for model in gemini-4-argon opus-5.5; do
  for type in default vague specific; do
    uv run amp eval robolab --harness amp1_basic --prompt amp1 --model $model --reasoning high \
        --task-set benchmark --instruction-type $type --num-runs 10 \
        --gpus auto --output runs/paper/fig2b-$model-$type
  done
done
```

## Figures 2C–2E: CaP-Gym LIBERO-PRO, robosuite, BEHAVIOR-1K

```bash
for model in gemini-4-argon opus-5.5; do
  for corpus in libero-pro robosuite behavior-1k; do
    uv run amp eval capgym --corpus $corpus --harness amp1_basic --prompt amp1 \
        --model $model --reasoning high --gpus auto --output runs/paper/fig2-$corpus-$model
  done
done
```

## Figure 5A: model and reasoning effort

```bash
while read -r model reasoning; do
  uv run amp eval robolab --harness amp1_basic --prompt amp1 --model $model --reasoning $reasoning \
      --task-set benchmark --num-runs 10 --gpus auto --output runs/paper/fig5a-$model-$reasoning
done <<'EOF'
opus-5.5 low
opus-5.5 high
opus-5 low
opus-5 high
opus-4.5 low
opus-4.5 high
sonnet-5.5 low
sonnet-5.5 high
sonnet-5 low
sonnet-5 high
haiku-5.5 low
haiku-5.5 high
gemini-4-argon low
gemini-4-argon high
qwen3.6-35b-a3b off
qwen3.6-35b-a3b max
qwen3.8-27b low
qwen3.8-27b max
qwen3.8-flash-next low
qwen3.8-flash-next max
glm-5.3-flash low
glm-5.3-flash high
deepseek-v4.1-flash low
deepseek-v4.1-flash high
kimi-k3 low
kimi-k3 high
EOF
```

## Figure 5B: basic and assistive harnesses

```bash
while read -r model level; do
  uv run amp eval robolab --harness amp1_basic,amp1_assistive --prompt amp1 --model $model \
      --reasoning $level --task-set benchmark --num-runs 10 --gpus auto --output runs/paper/fig5b-$model
done <<'EOF'
opus-5.5 high
opus-5 high
opus-4.5 high
sonnet-5.5 high
sonnet-5 high
haiku-5.5 high
gemini-4-argon high
qwen3.6-35b-a3b max
qwen3.8-27b max
qwen3.8-flash-next max
glm-5.3-flash high
deepseek-v4.1-flash high
kimi-k3 high
EOF
```

## Figure 5C: sensory ablations, all cameras

RGB+C+D+T, RGB+C+D, RGB+C and RGB, one harness each
([robolab.md](robolab.md#sensory-ablations-figures-5c-and-5d)):

```bash
for model in opus-5.5 sonnet-5.5 haiku-5.5 opus-4.5; do
  uv run amp eval robolab \
      --harness amp1_basic,amp1_basic_noperc,amp1_basic_rgb_only,amp1_basic_noperc_rgb_only_nocal \
      --prompt amp1 --model $model --reasoning high --task-set benchmark --num-runs 10 \
      --gpus auto --output runs/paper/fig5c-$model
done
```

Text: the same model describes every camera in text (`--vision vdm_strict`, which applies to the
whole run, so it is a run of its own):

```bash
for model in opus-5.5 sonnet-5.5 haiku-5.5 opus-4.5; do
  uv run amp eval robolab --harness amp1_basic_noperc_rgb_only_nocal --vision vdm_strict \
      --vdm-model $model --prompt amp1 --model $model --reasoning high --task-set benchmark \
      --num-runs 10 --gpus auto --output runs/paper/fig5c-text-$model
done
```

## Figure 5D: camera sets (Opus 5.5)

The all-cameras column is Figure 5C's `opus-5.5` runs. Scene cameras only and wrist camera only:

```bash
uv run amp eval robolab \
    --harness amp1_basic_scenecam,amp1_basic_noperc_scenecam,amp1_basic_scenecam_rgb_only,amp1_basic_noperc_rgb_only_nocal_scenecam \
    --prompt amp1 --model opus-5.5 --reasoning high --task-set benchmark --num-runs 10 \
    --gpus auto --output runs/paper/fig5d-scenecam
uv run amp eval robolab \
    --harness amp1_basic_wrist,amp1_basic_noperc_wrist,amp1_basic_wrist_rgb_only,amp1_basic_noperc_rgb_only_nocal_wrist \
    --prompt amp1 --model opus-5.5 --reasoning high --task-set benchmark --num-runs 10 \
    --gpus auto --output runs/paper/fig5d-wrist
```

Text, scene cameras and wrist camera:

```bash
uv run amp eval robolab --harness amp1_basic_noperc_rgb_only_nocal_scenecam --vision vdm_strict \
    --vdm-model opus-5.5 --prompt amp1 --model opus-5.5 --reasoning high --task-set benchmark \
    --num-runs 10 --gpus auto --output runs/paper/fig5d-text-scenecam
uv run amp eval robolab --harness amp1_basic_noperc_rgb_only_nocal_wrist --vision vdm_strict \
    --vdm-model opus-5.5 --prompt amp1 --model opus-5.5 --reasoning high --task-set benchmark \
    --num-runs 10 --gpus auto --output runs/paper/fig5d-text-wrist
```

## Figure 4: RoboDojo zero-shot

The k = 0 point of Figures 4F and 4I, on seed 2 (RGB only, no calibration, each task's recipe;
[robodojo.md](robodojo.md), section 4):

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --gpus auto --output runs/paper/fig4-zero-shot
```

The paper's 26.4% is the no-memory point that `amp practice fewshot results --none` reports below,
which counts unscored episodes as failures.

## Figure 4: RoboDojo after practice

The paper's AMP1+ is AMP1 with a persistent memory, which these docs call the workspace (the
notebook, once stored); see [practice.md](practice.md).

### Figures 4F–4H: few-shot learning

Five practice runs; per task, the run whose frozen workspace succeeds most often on seed 1 is
selected and evaluated on seed 2. Run 1 (`--eval`) is also the reference of the few-shot
variations below.

```bash
for i in 1 2 3 4 5; do
  uv run amp practice fewshot init runs/paper/fig4-fewshot-r$i \
      --study src/amp/practice/studies/fewshot.json
done
uv run amp practice fewshot run runs/paper/fig4-fewshot-r1 --eval
for i in 2 3 4 5; do uv run amp practice fewshot run runs/paper/fig4-fewshot-r$i; done
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed1 --episodes 50 \
    --gpus auto --output runs/practice/k0-seed1      # the no-memory run the selection pairs with
uv run amp practice fewshot select runs/paper/fig4-fewshot-r1 runs/paper/fig4-fewshot-r2 \
    runs/paper/fig4-fewshot-r3 runs/paper/fig4-fewshot-r4 runs/paper/fig4-fewshot-r5 \
    --corpus robodojo-seed1 --episodes 50 --pair runs/practice/k0-seed1 \
    --output runs/paper/fig4-fewshot-best5
uv run amp practice fewshot eval runs/paper/fig4-fewshot-best5 --corpus robodojo-seed2 \
    --episodes 50 --output runs/paper/fig4-fewshot-best5-seed2
uv run amp practice fewshot results runs/paper/fig4-fewshot-best5-seed2 --none runs/paper/fig4-zero-shot
```

Its `results.md` gives 4F's last point (84.1%) and 4G (model calls relative to zero-shot). 4F's
curve, after k = 2, 4, 8 and 12 rounds, is in `runs/paper/fig4-fewshot-kcurve/curve.md`:

```bash
uv run amp practice fewshot eval runs/paper/fig4-fewshot-best5 --corpus robodojo-seed2 \
    --episodes 50 --round 2,4,8,12 --reuse runs/paper/fig4-fewshot-best5-seed2 \
    --output runs/paper/fig4-fewshot-kcurve
uv run amp practice fewshot results runs/paper/fig4-fewshot-kcurve --none runs/paper/fig4-zero-shot
```

4H sets 4F's end points against the RoboDojo leaderboard.

Without practising, evaluate the paper's 42 selected notebooks
([`src/amp/practice/notebooks/fewshot_best5/`](../src/amp/practice/notebooks/fewshot_best5/)) for
4F's last point, and those at k = 2, 4 and 8 for its earlier points:

```bash
uv run amp practice fewshot eval --notebooks src/amp/practice/notebooks/fewshot_best5 \
    --corpus robodojo-seed2 --episodes 50 --output runs/paper/fig4-fewshot-notebooks
uv run amp practice fewshot results runs/paper/fig4-fewshot-notebooks --none runs/paper/fig4-zero-shot
for k in 2 4 8; do
  uv run amp practice fewshot eval --notebooks src/amp/practice/notebooks/fewshot_best5/k$k \
      --reuse runs/paper/fig4-fewshot-notebooks \
      --corpus robodojo-seed2 --episodes 50 --output runs/paper/fig4-fewshot-notebooks-k$k
  uv run amp practice fewshot results runs/paper/fig4-fewshot-notebooks-k$k --none runs/paper/fig4-zero-shot
done
```

### Figures 4I–4K: continual learning

One chain of the continual study: one workspace learned over the 42 tasks in one pass, evaluated
on seed 2 after 10, 20, 30 and 42 tasks:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed1 --episodes 5 \
    --gpus auto --output runs/practice/k0-seed1-x5    # optional: the seed-1 checkpoints' baseline
uv run amp practice continual init runs/paper/fig4-continual --chains 1 \
    --study src/amp/practice/studies/continual.json
uv run amp practice continual run runs/paper/fig4-continual --wait
uv run amp practice continual full-eval runs/paper/fig4-continual --steps 10,20,30,42 \
    --corpora robodojo-seed2
uv run amp practice continual results runs/paper/fig4-continual
```

`--chains 1` gives `chain_00` the paper's task orders and layouts. In
`runs/paper/fig4-continual/chain_00/`: `memory_curve_seed2.csv` (4I), `efficiency_seed2.csv` (4J)
and `matrix_seed2.csv` (4K); `results.md` has the same numbers.

Without practising, each checkpoint's notebook is in
`src/amp/practice/notebooks/continual_chain00/ck<N>/`. The 42-task point:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --workspace src/amp/practice/notebooks/continual_chain00/ck42 --shared-workspace \
    --gpus auto --output runs/paper/fig4-continual-ck42
```

### Variations of few-shot learning

Memory restricted to code (`.py` files) or notes, at evaluation only:

```bash
for cut in code notes; do
  uv run amp practice fewshot eval runs/paper/fig4-fewshot-r1 --corpus robodojo-seed2 \
      --episodes 50 --cut $cut --output runs/paper/fig4-fewshot-cut-$cut
  uv run amp practice fewshot results runs/paper/fig4-fewshot-cut-$cut --none runs/paper/fig4-zero-shot
done
```

Memory restricted during practice: the studies `src/amp/practice/studies/fewshot_code.json` and
`fewshot_notes.json`, each run like the default study (`init`, `run --eval`, `results`).

Haiku 5.5 with no memory and with Opus 5.5's memory from run 1:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model haiku-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --gpus auto --output runs/paper/fig4-zero-shot-haiku
uv run amp practice fewshot eval runs/paper/fig4-fewshot-r1 --corpus robodojo-seed2 --episodes 50 \
    --model haiku-5.5 --reasoning high --output runs/paper/fig4-fewshot-opus-memory-haiku
uv run amp practice fewshot results runs/paper/fig4-fewshot-opus-memory-haiku \
    --none runs/paper/fig4-zero-shot-haiku
```

For Qwen3.8-Flash-Next, use `--model qwen3.8-flash-next --reasoning max`. With their own memory:
the studies `fewshot_haiku.json` and `fewshot_qwen.json`, run like the default study.

Batch size: the studies `fewshot_b1.json` (B = 1) and `fewshot_b8.json` (B = 8), against run 1
(B = 4).

Where results go: [robolab.md](robolab.md), [capgym.md](capgym.md), [robodojo.md](robodojo.md)
and [practice.md](practice.md).
