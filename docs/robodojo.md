# RoboDojo

This repository runs AMP1 on [RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo) through
RoboDojo's own evaluation runner: 42 bimanual tasks for two ARX X5 arms in Isaac Sim 5.1, in five
dimensions: generalization (12 tasks), memory (6), long-horizon (8), precision (8) and open-ended
(8). The paper's RoboDojo results (Figure 4) are on evaluation seed 2; seed 1 selects among
practice runs, and practice ([practice.md](practice.md)) uses seed 0.

## 1. Setup

### Requirements

- Linux x86_64, glibc 2.35+ (Ubuntu 22.04 or 24.04), an NVIDIA GPU with 16 GB or more (the paper
  used H100 80GB), driver 570+. One episode uses one GPU.
- `git`, [uv](https://docs.astral.sh/uv/getting-started/installation/), CMake, a C/C++ toolchain
  and `ffmpeg` with libx264 (`sudo apt install git cmake build-essential ffmpeg`). Without a system
  `ffmpeg`, after step 2 link the venv's:
  `ln -s "$($R/venv/bin/python -c 'import imageio_ffmpeg as f; print(f.get_ffmpeg_exe())')" $R/venv/bin/ffmpeg`.
- Isaac Sim's system libraries, as in [quickstart.md](quickstart.md):
  `sudo apt install libgl1 libglu1-mesa libegl1 libxt6 libvulkan1 libglib2.0-0 libgomp1`.
- About 100 GB of disk, and internet access while running.

### Install

This is RoboDojo's `scripts/install.sh` and `scripts/init_assets.sh` with a uv venv, submodules at
their pinned commits, the Assets at a fixed revision, and the paper's exact packages (`sim.txt`;
`pip check` reports a few conflicts, used as is). `R` is any directory outside the checkout; run
from this repository's root:

```bash
export R=~/robodojo-env X=$PWD/external/robodojo C=$PWD/src/amp/robots/robodojo/constraints

# 1. RoboDojo at the pin, with its three submodules (XPolicyLab, IsaacLab, cuRobo) at their pins
make robodojo

# 2. RoboDojo's venv: CPython 3.11 with the paper's packages (sim.txt: install.sh's steps 2 and
#    4, with torch and Isaac Sim 5.1), then Isaac Lab and cuRobo from the submodules (steps 5-6)
uv venv $R/venv --managed-python --python 3.11 --seed
source $R/venv/bin/activate
export OMNI_KIT_ACCEPT_EULA=YES
python -m pip install --no-deps -r $C/sim.txt \
    --extra-index-url https://download.pytorch.org/whl/cu128 --extra-index-url https://pypi.nvidia.com
export PIP_CONSTRAINT=$C/sim.txt UV_CONSTRAINT=$C/sim.txt
(cd $X/third_party/IsaacLab && TERM=xterm-256color UV_PYTHON=$R/venv/bin/python \
    ./isaaclab.sh --install none)
(cd $X/third_party/curobo && python -m pip install -e ".[cu12]" --no-build-isolation --no-deps)
deactivate
unset PIP_CONSTRAINT UV_CONSTRAINT
$R/venv/bin/python -c "import isaaclab, curobo"   # isaaclab.sh exits 0 even when its installs fail

# 3. The Assets (39 GB) at the pinned dataset revision, where init_assets.sh puts them, then the
#    robots' configs
$R/venv/bin/python -c "from huggingface_hub import snapshot_download as d; \
d('RoboDojo-Benchmark/RoboDojo', repo_type='dataset', revision='35efbc7dedfdbeeb6e95fb749bd885d73d483e41', \
allow_patterns=['Assets/**'], local_dir='$X/.cache/robodojo_assets_repo')"
ln -s .cache/robodojo_assets_repo/Assets $X/Assets
(cd $X && $R/venv/bin/python utils/update_embodiment_config_path.py < /dev/null)

# 4. XPolicyLab's policy-server venv: XPolicyLab needs websockets 14 or newer, RoboDojo's venv 12.0
uv venv $R/xpl-venv --managed-python --python 3.11
uv pip install --python $R/xpl-venv/bin/python -c $C/xpl.txt numpy pyyaml \
    opencv-python-headless h5py websockets msgpack msgpack-numpy pydantic
```

`make robodojo` clones RoboDojo into `external/robodojo` at
`09bd5f5e762b949f5858ece31347ce62057b3a5e`, the commit the paper's runs used, with its submodules.
`isaaclab.sh` installs Isaac Lab with `uv pip install` into whichever environment uv picks, so
`UV_PYTHON` names RoboDojo's venv: a uv setting such as `[pip] system = true` would otherwise send
it to another Python.

`find $X/Assets/ -type f | wc -l` prints 15365 after the download, 15367 after the configs
command. Hugging Face rate-limits anonymous downloads by address, with an HTTP 429 error or
silently, so log in first on a shared address (`$R/venv/bin/hf auth login` or `HF_TOKEN`). If it
prints less, log in, rerun the download (it resumes), then the configs command.

For every run (put these in a file and `source` it):

```bash
export AMP_ROBODOJO_PYTHON=~/robodojo-env/venv/bin/python        # RoboDojo's venv
export AMP_ROBODOJO_XPL_PYTHON=~/robodojo-env/xpl-venv/bin/python # XPolicyLab's venv
export OMNI_KIT_ACCEPT_EULA=YES      # accepts NVIDIA's Omniverse licence; without it episodes stop at its prompt
```

`AMP_ROBODOJO_ROOT` (or `--robodojo-root`) points at a checkout other than `external/robodojo`.
`uv run amp doctor --robodojo` checks the checkout, submodules, Assets, both venvs, the system
libraries, `ffmpeg` and the licence variable (add `--skip robolab` on a machine without RoboLab).

## 2. Corpora

One corpus per seed: `robodojo-seed0`, `robodojo-seed1`, `robodojo-seed2` (the default).
`--episodes N` (1 to 50) runs layouts 0 to N-1 of each task, split between a generalization
task's two configs; `--task` names base tasks; `--layouts 5,6,7` runs those layout ids instead.

## 3. Checking the setup

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --model opus-5.5 \
    --corpus robodojo-seed2 --task align_blocks --episodes 2 --gpus 0 --output runs/robodojo-check
```

## 4. Running it

The paper's harness, `amp1_basic_rgb_only_nocal_rdrecipe_rdnotes`, gives RGB images only, no
depth or calibration, and the instruction followed by the task's recipe from RoboDojo's public
task descriptions, with strategy notes on four tasks (overlays `rgb_only`, `no_calibration`,
`robodojo_recipe`, `robodojo_notes`). The paper's zero-shot evaluation, 2,100 episodes:

```bash
uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --prompt amp1 \
    --model opus-5.5 --reasoning high --corpus robodojo-seed2 --episodes 50 \
    --gpus auto --output runs/paper/fig4-zero-shot
```

`--gpus auto` uses every GPU with under 4 GiB in use (`--gpus 0` names one). Rerunning with the
same `--output` resumes.

## 5. Where results go

- `<output>/robodojo_summary.json` (also printed): the numbers the paper and RoboDojo's leaderboard
  report. Each task's success rate over its episodes (a generalization task pooling both configs,
  an unscored episode counting 0), each dimension's mean over its tasks, and the overall mean over
  the five dimensions.
- `<output>/summary.json`: successes per config and in total, pooled over episodes.
- `<output>/<harness configuration>/`: per episode a record (`agentic/units/`), a trace
  (`agentic/episodes/`), RoboDojo's `_result.json` as `robodojo_result_<k>.json`, and the videos.

## 6. Troubleshooting

- **No first observation.** Read the Kit log, in a `kit_portable/` directory inside `<output>`.
- **Every episode fails at reset.** Check that `Assets/Robots/x5/curobo.yml` exists (step 3's last
  command) and that `ffmpeg -encoders` lists libx264.
