# The paper's few-shot notebooks, frozen after practice

The paper's few-shot result (Figure 4F's last point, 84.1% on seed 2; [docs/practice.md](../../../../../docs/practice.md),
section 3) evaluates one frozen notebook per task. Practice ran five times per task, and for each task the run that
succeeded most often on seed 1 was kept. Each folder here is that run's notebook for one task, exactly as practice
left it, with the files the agent wrote (notes, Python helpers, and in three tasks an image or a NumPy array it saved).
`checkpoints.json` gives each folder's version (`amp.practice.workspace.sha_of`), the practice run it came from, and its
seed-2 successes in the paper.

To evaluate one task with its notebook:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 --task make_kong \
    --workspace src/amp/practice/notebooks/fewshot_best5/make_kong \
    --workspace-notebook 2 --workspace-max-bytes 131072 \
    --gpus auto --output runs/paper/fig4-fewshot-make_kong
```

To evaluate all 42 in one run, each episode's agent getting its own task's notebook:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --workspaces src/amp/practice/notebooks/fewshot_best5/checkpoints.json \
    --workspace-notebook 2 --workspace-max-bytes 131072 \
    --gpus auto --output runs/paper/fig4-fewshot-all
```

To evaluate all 42 in one new run with the practice driver and score it with the RoboDojo leaderboard formula, which
gives Figure 4F's point at k = 12 (this starts its own job, not the run above):

```bash
uv run amp practice fewshot eval --notebooks src/amp/practice/notebooks/fewshot_best5 \
    --corpus robodojo-seed2 --episodes 50 --output runs/paper/fig4-fewshot-notebooks
uv run amp practice fewshot results runs/paper/fig4-fewshot-notebooks --none runs/paper/fig4-zero-shot
```

`amp practice fewshot eval` checks every folder against its sha in `checkpoints.json` before it runs anything.

## The notebooks at k = 2, 4 and 8

`k2/`, `k4/` and `k8/` hold the notebooks behind Figure 4F's earlier points: for each task, the notebook of the same
selected run after 2, 4 and 8 practice rounds, the version its next round started from, or its final notebook when
practice stopped before that round (the `same_as_final` field). Each folder has its own `checkpoints.json` in the
format above, with the round and the notebook's seed-2 successes in the paper, so each point is evaluated the same
way, reusing the k = 12 evaluation above for the tasks whose notebook is the final one, for example the one at
k = 4:

```bash
uv run amp practice fewshot eval --notebooks src/amp/practice/notebooks/fewshot_best5/k4 \
    --reuse runs/paper/fig4-fewshot-notebooks \
    --corpus robodojo-seed2 --episodes 50 --output runs/paper/fig4-fewshot-notebooks-k4
uv run amp practice fewshot results runs/paper/fig4-fewshot-notebooks-k4 --none runs/paper/fig4-zero-shot
```

The practice runs themselves are not included.
