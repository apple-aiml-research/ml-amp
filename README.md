# On Agentic Manipulation Policies

[![Project Page](https://img.shields.io/badge/Project-Page-green)](https://apple-aiml-research.github.io/ml-amp/)
[![arXiv](https://img.shields.io/badge/arXiv-2611.09892-b31b1b.svg)](https://arxiv.org/abs/2611.09892)

*Chen Chen&dagger;, Omid Bagherieh&dagger;, Naveen Nikamanth Appiah Balaji&dagger;, Jesse T. Gonzalez&dagger;, Negin Heravi&dagger;, Fahad Islam&dagger;,
Xueting Li&dagger;, Jean-Baptiste Regli&dagger;, Kejia Ren&dagger;, Petr &Scaron;vec&dagger;, David J. Yoon&dagger;, Matthias M&uuml;ller and Vladlen Koltun*
(2026). &dagger; Equal contribution.

This repository contains AMP1, the agentic manipulation policy from the paper, and the code to reproduce
the paper's results on RoboLab-120, on CaP-Gym's LIBERO-PRO, robosuite and BEHAVIOR-1K suites, and on
RoboDojo.

## Setup

You need Linux on x86_64 with NVIDIA GPUs, [uv](https://docs.astral.sh/uv/) and an API key
for the model you evaluate. RoboLab, CaP-X and RoboDojo are third-party projects under their own licences: clone
them at the commits in [docs/reproducibility.md](docs/reproducibility.md). The full setup is in
[docs/quickstart.md](docs/quickstart.md).

The paper's simulation results were run on NVIDIA H100 80GB GPUs with driver 580.159.03, on Debian 13 Linux
(glibc 2.41). Results depend on the GPU, its driver and the system, so on other hardware or versions you may get
numbers slightly different from those reported in the paper.

```bash
git clone https://github.com/apple-aiml-research/ml-amp AMP && cd AMP
git clone https://github.com/NVlabs/RoboLab.git external/robolab
git -C external/robolab checkout 9db0aaf09d9fe5d4f37b168320788258c7012463   # v0.3.1; install it as docs/quickstart.md shows
git -C external/robolab lfs pull                                             # its assets (needs git-lfs; else they stay LFS pointers)
uv sync --frozen
export GEMINI_API_KEY=...   # or ANTHROPIC_API_KEY, or your own server (docs/models.md)
uv run amp doctor
```

## Running evaluations

The command for every result in the paper is in [docs/reproducing-the-paper.md](docs/reproducing-the-paper.md).

- **RoboLab-120** ([docs/robolab.md](docs/robolab.md)):

  ```bash
  uv run amp eval robolab --harness amp1_basic --model gemini-4-argon --reasoning high \
      --task-set benchmark --gpus auto --output runs/robolab
  ```

- **CaP-Gym** ([docs/capgym.md](docs/capgym.md), section 1 to install): `--corpus libero-pro`, `robosuite`
  or `behavior-1k`; results in `<output>/summary.json`.

  ```bash
  uv run amp eval capgym --corpus libero-pro --harness amp1_basic --model gemini-4-argon \
      --reasoning high --gpus auto --output runs/capgym
  ```

- **RoboDojo** ([docs/robodojo.md](docs/robodojo.md), section 1 to install); results in `<output>/summary.json`:

  ```bash
  uv run amp eval robodojo --harness amp1_basic_rgb_only_nocal_rdrecipe_rdnotes --model opus-5.5 \
      --reasoning high --corpus robodojo-seed2 --episodes 50 --gpus auto --output runs/robodojo
  ```

## Citation

If you find this work useful, please cite:

```bibtex
@article{2026amp,
  title   = {On Agentic Manipulation Policies},
  author  = {Chen, Chen and
             Bagherieh, Omid and
             Appiah Balaji, Naveen Nikamanth and
             Gonzalez, Jesse T. and
             Heravi, Negin and
             Islam, Fahad and
             Li, Xueting and
             Regli, Jean-Baptiste and
             Ren, Kejia and
             {\v{S}}vec, Petr and
             Yoon, David J. and
             M{\"u}ller, Matthias and
             Koltun, Vladlen},
  journal = {arXiv preprint arXiv:2611.09892},
  url     = {https://arxiv.org/abs/2611.09892},
  year    = {2026}
}
```

## License

Please check out the repository [LICENSE](LICENSE) before using the provided code. Third-party material in
this repository and its licences are listed in [ACKNOWLEDGEMENTS](ACKNOWLEDGEMENTS).
