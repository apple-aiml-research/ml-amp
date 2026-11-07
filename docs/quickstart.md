# Quick start

From a fresh Ubuntu machine to a first RoboLab evaluation.

## 1. What you need

| | |
|---|---|
| OS | Linux on x86_64 with glibc 2.35 or newer (Ubuntu 22.04 or 24.04) |
| GPU | NVIDIA GPUs (the paper used H100 80GB) with a 580-series driver |
| Memory | about 9 GB of VRAM and 15 GB of RAM for RoboLab |
| Disk | about 40 GB for RoboLab |
| Model | an Anthropic or Gemini API key, or an OpenAI-compatible server ([models.md](models.md)) |

```bash
sudo apt update
sudo apt install curl git git-lfs bubblewrap   # git-lfs before cloning RoboLab, or its assets stay LFS pointers
curl -LsSf https://astral.sh/uv/install.sh | sh && export PATH="$HOME/.local/bin:$PATH"   # uv
```

## 2. Get the code and RoboLab

Clone RoboLab into `external/robolab` at v0.3.1 ([reproducibility.md](reproducibility.md), "Benchmark
versions"; `make robolab` does the same):

```bash
git clone https://github.com/apple-aiml-research/ml-amp AMP && cd AMP
git clone https://github.com/NVlabs/RoboLab.git external/robolab
git -C external/robolab checkout 9db0aaf09d9fe5d4f37b168320788258c7012463
git -C external/robolab lfs pull
```

## 3. RoboLab's Isaac Sim environment

RoboLab's `isaac50` extra: Python 3.11, Isaac Sim 5.0.0, Isaac Lab 2.2.0, torch 2.7.0+cu128.

```bash
sudo apt install libgl1 libglu1-mesa libegl1 libxt6 libvulkan1 libglib2.0-0 libgomp1
cd external/robolab
uv venv --python 3.11
uv sync --extra isaac50
uv pip freeze > ../../robolab-freeze.txt           # keep it with your results
cd ../..
export OMNI_KIT_ACCEPT_EULA=YES                    # accepts NVIDIA's Omniverse EULA (RoboLab, RoboDojo)
```

## 4. This repository's environment

```bash
uv sync --frozen
```

## 5. The kernel sandbox

The agent's code runs in a bubblewrap sandbox. Where AppArmor restricts unprivileged user namespaces
(the default on Ubuntu 24.04), install the profile that lets `/usr/bin/bwrap` create them:

```bash
if [ "$(cat /proc/sys/kernel/apparmor_restrict_unprivileged_userns 2>/dev/null)" = 1 ]; then
  sudo install -m 644 docs/apparmor/bwrap /etc/apparmor.d/bwrap
  sudo apparmor_parser -r /etc/apparmor.d/bwrap
fi
```

## 6. Models

```bash
export ANTHROPIC_API_KEY=...        # opus-5.5
export GEMINI_API_KEY=...           # gemini-4-argon
```

## 7. Check the machine

```bash
uv run amp doctor
```

It checks the GPUs, glibc, bubblewrap and the sandbox, the RoboLab checkout and its pin, and the model
(one short real call to it; `--skip model-call` leaves it out), and exits non-zero on a failure.

## 8. Run

A smoke test, one task for two episodes per harness:

```bash
uv run amp eval robolab \
    --harness amp1_basic,amp1_assistive --prompt amp1 \
    --model opus-5.5 --task BananaInBowlTask --num-runs 2 \
    --gpus auto --output runs/smoke
```

Rerunning a command with the same `--output` resumes it. RoboLab's results are merged into
`<output>/robolab/merged/<run_id>/` ([robolab.md](robolab.md), "Where results go"). The paper's runs are in
[reproducing-the-paper.md](reproducing-the-paper.md).

## 9. CaP-Gym

For LIBERO-PRO and robosuite ([capgym.md](capgym.md), section 1, has the dataset, the LIBERO config and
BEHAVIOR-1K):

```bash
sudo apt install gcc g++ make cmake linux-libc-dev
make capx-sim
uv python install 3.12 3.10
uv venv ~/capx/venv-libero --managed-python --python 3.12
(cd external/cap-x && VIRTUAL_ENV=~/capx/venv-libero uv sync --frozen --active --extra libero)
uv venv ~/capx/venv-robosuite --managed-python --python 3.10
(cd external/cap-x && VIRTUAL_ENV=~/capx/venv-robosuite uv sync --frozen --active --extra robosuite)
export AMP_CAPX_ROOT=$PWD/external/cap-x
export AMP_CAPX_LIBERO_PYTHON=~/capx/venv-libero/bin/python
export AMP_CAPX_ROBOSUITE_PYTHON=~/capx/venv-robosuite/bin/python
export LIBERO_CONFIG_PATH=~/capx/libero
```

## 10. RoboDojo

Install RoboDojo's two venvs and its Assets as [robodojo.md](robodojo.md), section 1, shows, then:

```bash
export AMP_ROBODOJO_PYTHON=~/robodojo-env/venv/bin/python
export AMP_ROBODOJO_XPL_PYTHON=~/robodojo-env/xpl-venv/bin/python
export OMNI_KIT_ACCEPT_EULA=YES
uv run amp doctor --robodojo
```

## Troubleshooting

| Symptom | Fix |
|---|---|
| `libGL.so.1: cannot open shared object file` (or `libX11.so.6`, `libXt.so.6`) in an Isaac Sim log | install the libraries in section 3 |
| `bwrap: loopback: Failed RTM_NEWADDR: Operation not permitted` | install the AppArmor profile (section 5) |
