# The paper's continual notebook, at its checkpoints

The paper's continual-learning chain (Figure 4I–K; [docs/practice.md](../../../../../docs/practice.md),
section 4) learned one shared notebook over RoboDojo's 42 tasks, one task at a time. Each folder
here is the notebook exactly as the chain had it after that many tasks of its first pass, with the
files the agent wrote (`NOTES.md` and `helpers.py`). `checkpoints.json` gives each folder's
version (`amp.practice.workspace.sha_of`). The files are agent-written and are covered by the
repository's [LICENSE](../../../../../LICENSE); they carry no header of their own, since that would
change their sha.

| folder | tasks learned | seed-2 RoboDojo success rate in the paper (%) |
|---|---|---|
| (none) | 0 | 26.4 |
| `ck10` | 10 | 50.3 |
| `ck20` | 20 | 49.9 |
| `ck30` | 30 | 54.1 |
| `ck42` | 42 | 57.6 |

The table uses the paper's formula (unscored episodes count as failures, a generalization task is
the mean of its standard and randomized settings), which `amp practice continual results` writes
as `memory_curve_seed2.csv`.

To evaluate one, pass its folder as the shared workspace of every task:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --workspace src/amp/practice/notebooks/continual_chain00/ck42 --shared-workspace \
    --gpus auto --output runs/paper/fig4-continual-ck42
```
