# Reproducibility

To match the paper, run this repository's release commit unmodified (`amp --version` prints the commit
and whether tracked files are modified), with the pins, environment and seeds below. Record the commit,
the model key and the reasoning effort with your results. Each episode's record
(`<output>/<config>/agentic/units/`) stores the commits its run used, and a run warns when a checkout is
off its pin.

## Benchmark versions

RoboLab, CaP-X and RoboDojo are third-party projects under their own licences. Clone them into
`external/` at these commits, the ones the paper's results were produced at:

| Benchmark | Repository | Commit | Tag |
|---|---|---|---|
| RoboLab | `https://github.com/NVlabs/RoboLab.git` | `9db0aaf09d9fe5d4f37b168320788258c7012463` | `v0.3.1` |
| CaP-X (CaP-Gym) | `https://github.com/capgym/cap-x.git` | `53e9966d7a8e2fa7494676772bccc35280f5c0ed` | upstream main, 2026-10-07 |
| RoboDojo | `https://github.com/RoboDojo-Benchmark/RoboDojo.git` | `09bd5f5e762b949f5858ece31347ce62057b3a5e` | upstream main, 2026-10-12, with the submodules it pins |

`make robolab`, `make capx` and `make robodojo` run these commands:

```bash
git clone https://github.com/NVlabs/RoboLab.git external/robolab
git -C external/robolab checkout 9db0aaf09d9fe5d4f37b168320788258c7012463
git clone https://github.com/capgym/cap-x.git external/cap-x
git -C external/cap-x checkout 53e9966d7a8e2fa7494676772bccc35280f5c0ed
git clone https://github.com/RoboDojo-Benchmark/RoboDojo.git external/robodojo
git -C external/robodojo checkout 09bd5f5e762b949f5858ece31347ce62057b3a5e
git -C external/robodojo submodule update --init --recursive
```

Install RoboLab as [quickstart.md](quickstart.md), sections 2 and 3, shows, CaP-X as
[capgym.md](capgym.md), section 1, and RoboDojo as [robodojo.md](robodojo.md), section 1. The command for each result is in [reproducing-the-paper.md](reproducing-the-paper.md).

## Environment

| | The paper's runs |
|---|---|
| Hardware | NVIDIA H100 80GB GPUs, driver 580.159.03 |
| OS | Debian 13, glibc 2.41 |
| This repository | Python 3.11, dependencies locked in `uv.lock` |
| RoboLab | its `isaac50` extra: Python 3.11, Isaac Sim 5.0.0, Isaac Lab 2.2.0, torch 2.7.0+cu128 |
| CaP-X | LIBERO-PRO: Python 3.12; robosuite: Python 3.10; both `uv sync --frozen`. BEHAVIOR-1K: Python 3.10, Isaac Sim 4.5, OmniGibson 3.7.2 |
| RoboDojo | Python 3.11, Isaac Sim 5.1 |
| Models | provider defaults for temperature and top-p (none set) |
| Agent | `--prompt amp1`; at most 600 turns and 21,600 s of wall time per episode |

RoboLab has no lock file, so its environment resolves at install time. Save it, with the GPU model and
driver version (`nvidia-smi`), beside your results:

```bash
cd external/robolab && uv pip freeze > ../../robolab-freeze.txt
```

## Seeds

| Benchmark | Seeding |
|---|---|
| RoboLab | Python, NumPy, torch, CUDA and Replicator seeded with 0 at every environment creation |
| CaP-Gym | trial t runs `env.reset(seed=t)`, t = 1..N, as CaP-X does; robosuite's scenes are unseeded, as published |
| RoboDojo | seed 0 for practice, seed 1 to select few-shot runs, seed 2 reported (`--corpus robodojo-seed2`) |
