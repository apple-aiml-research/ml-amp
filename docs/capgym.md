# CaP-Gym

The paper evaluates AMP1 on three suites of CaP-Gym, the benchmark of
[CaP-X](https://github.com/capgym/cap-x) (Figures 2C-E). Each is a corpus of `amp eval capgym`:

| Corpus | Tasks | Trials per task | Simulator |
|---|---|---|---|
| `libero-pro` (default) | `libero_{object,goal,spatial}_{swap,task}`: 60 | 50 | LIBERO-PRO, robosuite 1.4 fork, MuJoCo |
| `robosuite` | `cube_lifting`, `cube_stack`, `cube_restack`, `nut_assembly`, `spill_wipe`, `two_arm_lift`, `two_arm_handover` | 100 | robosuite 1.5.1, MuJoCo |
| `behavior-1k` | `r1pro_pick_up_radio`, `r1pro_pick_up_trash` (Pick up Soda Can) | 25 | BEHAVIOR-1K: OmniGibson 3.7.2 on Isaac Sim 4.5 |

CaP-X's extras conflict, so each simulator family runs CaP-X's own code in its own venv:

| Family | Interpreter variable | Python |
|---|---|---|
| LIBERO-PRO | `AMP_CAPX_LIBERO_PYTHON` | 3.12 |
| robosuite (single-arm, two-arm, wipe) | `AMP_CAPX_ROBOSUITE_PYTHON` | 3.10 |
| BEHAVIOR-1K | `AMP_CAPX_BEHAVIOR_PYTHON` | 3.10 |

Each variable must be set to the `bin/python` of that family's CaP-X venv, or the matching flag
(`--libero-python`, `--robosuite-python`, `--behavior-python`) given; a run whose
interpreter does not exist refuses to start.

## 1. Setup

CaP-X is a third-party project under its own licence, not part of this repository. Clone it,
unmodified, into `external/cap-x` at its pin:

```bash
git clone https://github.com/capgym/cap-x.git external/cap-x
git -C external/cap-x checkout 53e9966d7a8e2fa7494676772bccc35280f5c0ed
```

`make capx` does the same; `make capx-sim` also checks out the four nested repos the LIBERO-PRO and
robosuite families install. CaP-X's nested repos at the pin (`capx/third_party/`):

| Path | Upstream | Commit | Used by |
|---|---|---|---|
| `LIBERO-PRO` | `github.com/uynitsuj/LIBERO-PRO` | `5368540790fd` | LIBERO-PRO |
| `libero_dependencies/robosuite` | `github.com/Max-Fu/robosuite` | `a498b087d4bc` | LIBERO-PRO (robosuite 1.4) |
| `sam3` | `github.com/Max-Fu/sam3` | `6fe87d64a5be` | installed by CaP-X's extras, never called |
| `robosuite` | `github.com/uynitsuj/robosuite` | `97292732ed90` | robosuite (1.5.1) |
| `b1k` | `github.com/qingh097/b1k` | `272ec5ca9936` | BEHAVIOR-1K (OmniGibson, BDDL) |

### What you need

- Linux on x86_64 and an NVIDIA driver with EGL (`libEGL_nvidia`). The MuJoCo suites need no CUDA
  toolkit.
- `git`, [uv](https://docs.astral.sh/uv/getting-started/installation/) (validated with 0.12.5 and
  0.12.23) and a C/C++ toolchain; on Ubuntu or Debian:
  `sudo apt install gcc g++ make cmake git linux-libc-dev`.
- About 23 GB for the two MuJoCo venvs and the checkout, plus a 14 GB uv cache you can delete
  afterwards.

### The install: LIBERO-PRO and robosuite

This follows CaP-X's README and its `docs/libero-tasks.md`. `C` is any directory for the venvs and
data; run from the checkout of this repository:

```bash
export C=~/capx UV_CACHE_DIR=~/capx/uv-cache UV_LINK_MODE=copy
mkdir -p $C
X=$PWD/external/cap-x

# 1. CaP-X at the pin, with the four nested repos the two families install
make capx-sim

# 2. LIBERO-PRO family: uv's own CPython 3.12, the libero extra, the committed lock as is
uv python install 3.12 3.10
uv venv $C/venv-libero --managed-python --python 3.12
(cd $X && VIRTUAL_ENV=$C/venv-libero uv sync --frozen --active --extra libero)

# 3. robosuite family: CPython 3.10, the robosuite extra; after step 2, never beside it
uv venv $C/venv-robosuite --managed-python --python 3.10
(cd $X && VIRTUAL_ENV=$C/venv-robosuite uv sync --frozen --active --extra robosuite)

# 4. LIBERO-PRO dataset, moved into libero/libero as LIBERO-PRO's README says
#    (on HTTP 429 from Hugging Face, retry later)
$C/venv-libero/bin/python -c "from huggingface_hub import snapshot_download as d; \
d('zhouxueyang/LIBERO-Pro', repo_type='dataset', \
revision='c86fc3b8293185a6f373677018ff3e37f8391602', local_dir='$C/hf/libero_data')"
(cd $C/hf/libero_data && tr -d '\r' < SHA256SUMS.txt | sha256sum -c --quiet -)   # its lines end in CRLF
cd $X/capx/third_party/LIBERO-PRO
mv $C/hf/libero_data/bddl_files/* libero/libero/bddl_files/   # refuses 2 existing dirs: expected
mv $C/hf/libero_data/init_files/* libero/libero/init_files/   # refuses 6 existing dirs: expected
# the dirs mv refused must equal LIBERO-PRO's own copies, and no tracked file may change
for t in bddl_files init_files; do for d in $C/hf/libero_data/$t/*; do
  [ ! -e "$d" ] || diff -r "$d" libero/libero/$t/$(basename $d); done; done
git diff --quiet HEAD && test -z "$(git ls-files --deleted)" && rm -rf $C/hf
cd -

# 5. LIBERO config (CaP-X's docs/libero-tasks.md, "Headless / non-interactive setup")
mkdir -p $C/libero
B=$X/capx/third_party/LIBERO-PRO/libero/libero
printf 'assets: %s/assets\nbddl_files: %s/bddl_files\nbenchmark_root: %s\ndatasets: %s/../datasets\ninit_states: %s/init_files\n' \
  $B $B $B $B $B > $C/libero/config.yaml
```

- `--frozen` installs CaP-X's committed `uv.lock` as is; without it uv re-resolves and fails.
  Leave `CUDA_HOME` unset: neither extra builds cuRobo.
- Build the two venvs one after the other: both editable-install into the same checkout. Never run
  robosuite's `setup_macros.py`, which writes into the checkout.
- The dataset (CC BY 4.0, 676 files) adds only untracked files; the checkout stays clean
  (`git diff --quiet HEAD`).
- Expected: the LIBERO-PRO venv has CPython 3.12, torch 2.9.1+cu128, mujoco 3.5.0, robosuite 1.4.0
  and numpy 1.26.4; the robosuite venv CPython 3.10, robosuite 1.5.1 and mujoco 3.5.0.

Then, for every run (put it in a file and `source` it):

```bash
export AMP_CAPX_ROOT=$PWD/external/cap-x                  # the CaP-X checkout
export AMP_CAPX_LIBERO_PYTHON=~/capx/venv-libero/bin/python
export AMP_CAPX_ROBOSUITE_PYTHON=~/capx/venv-robosuite/bin/python
export LIBERO_CONFIG_PATH=~/capx/libero                     # the directory holding config.yaml
```

### The install: BEHAVIOR-1K

A third venv, Python 3.10, from public sources only. It follows CaP-X's `docs/behavior-tasks.md`
and the b1k submodule's `uv_install.sh`, pinning the base dependencies to CaP-X's `uv.lock` and
building cuRobo ahead of time with CUDA 12.8. Run it after the install above (it reuses `X`).

- **Needs:** an NVIDIA GPU with more than 16 GB, Vulkan and CUDA (validated with driver 580), the
  toolchain above, `curl` and `unzip`. About 19 GB for the venv, 37 GB for the dataset and 5.5 GB
  for the CUDA toolkit used only to build cuRobo.
- **cuRobo** is installed only because OmniGibson imports it. Its build needs a CUDA 12.x `nvcc`
  (step 3) and `TORCH_CUDA_ARCH_LIST` for your GPU: set `ARCH=12.0+PTX` on Blackwell, `ARCH=9.0+PTX`
  on H100.
- **Licences.** `OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's Omniverse terms, and
  `accept_license=True` (step 5) the BEHAVIOR Data Bundle licence (non-commercial academic research
  only; no extraction or redistribution); set them only if they apply to you. Kit's first launch
  fetches two extensions from NVIDIA's public registry.

```bash
export C=~/capx UV_CACHE_DIR=~/capx/uv-cache UV_LINK_MODE=copy OMNI_KIT_ACCEPT_EULA=YES
V=$C/venv-b1k B=$X/capx/third_party/b1k D=$C/b1k-data     # X as above
mkdir -p $C/b1k-build $C/cache/isaac-wheels

# 1. the b1k submodule, then CaP-X's override-dependencies
git -C $X submodule update --init -- capx/third_party/b1k
printf '%s\n' 'transformers<5.0' PyOpenGL==3.1.6 PyOpenGL-accelerate==3.1.6 numpy==1.26.4 \
  'opencv-python-headless<4.13' "opencv-python ; sys_platform == 'never'" 'jax<0.4.30' 'jaxlib<0.4.30' \
  > $C/b1k-build/overrides.txt
pipi() { VIRTUAL_ENV=$V uv pip install --override $C/b1k-build/overrides.txt "$@"; }

# 2. the venv: CaP-X's locked base dependencies, then OmniGibson and BDDL, then Isaac Sim 4.5
uv venv $V --managed-python --python 3.10
(cd $X && VIRTUAL_ENV=$V uv sync --frozen --active)
(cd $B && pipi -e bddl3 -e OmniGibson)
# the 26 wheels uv_install.sh lines 98-125 name, from NVIDIA's index
for pkg in omniverse_kit-106.5.0.162521 isaacsim-4.5.0.0 $(for n in kernel app core gui utils storage \
    asset sensor robot_motion robot benchmark code_editor ros1 ros2 cortex example replicator rl \
    robot_setup template test extscache_physics extscache_kit extscache_kit_sdk; do
    echo isaacsim_$n-4.5.0.0; done); do
  name=${pkg%-*}; w=$pkg-cp310-none-manylinux_2_34_x86_64.whl
  [ -s $C/cache/isaac-wheels/$w ] || curl -fsSL "https://pypi.nvidia.com/${name//_/-}/$w" \
    -o $C/cache/isaac-wheels/$w
done
pipi $C/cache/isaac-wheels/*.whl
find "$($V/bin/python -c 'import isaacsim,os;print(os.environ["ISAAC_PATH"])')/extscache" \
  -type d -path '*/pip_prebundle/websockets' -exec rm -rf {} +          # uv_install.sh lines 155-167

# 3. cuRobo for its import. First nvcc and the CUDA libraries, from NVIDIA's CUDA 12.8.1
#    redistributables (the redist JSON lists each tarball and its sha256), unpacked in $C/cuda-12.8
mkdir -p $C/cuda-12.8
curl -fsSL https://developer.download.nvidia.com/compute/cuda/redist/redistrib_12.8.1.json \
  -o $C/b1k-build/redistrib_12.8.1.json
CU=$C/cuda-12.8 M=$C/b1k-build/redistrib_12.8.1.json $V/bin/python - <<'PY'
import hashlib, json, os, shutil, tarfile, tempfile, urllib.request
base = "https://developer.download.nvidia.com/compute/cuda/redist/"
home = os.environ["CU"]; m = json.load(open(os.environ["M"]))
for p in ("cuda_nvcc", "cuda_cudart", "cuda_cccl", "cuda_nvrtc", "libcublas", "libcusparse",
          "libcusolver", "libcurand", "libcufft", "libnvjitlink", "cuda_nvtx", "cuda_nvml_dev",
          "cuda_profiler_api"):
    e = m[p]["linux-x86_64"]; tmp = tempfile.mkdtemp(); path, h = f"{tmp}/{p}.tar.xz", hashlib.sha256()
    with urllib.request.urlopen(base + e["relative_path"]) as r, open(path, "wb") as f:
        while chunk := r.read(1 << 20):
            h.update(chunk); f.write(chunk)
    assert h.hexdigest() == e["sha256"], p
    with tarfile.open(path) as t:
        top = t.getnames()[0].split("/")[0]; t.extractall(tmp)
    os.system(f"cp -a {tmp}/{top}/. {home}/"); shutil.rmtree(tmp)
os.symlink("lib", f"{home}/lib64")
PY
$C/cuda-12.8/bin/nvcc --version | grep "release 12.8"
pipi setuptools setuptools_scm wheel ninja                               # ARCH as above
CUDA_HOME=$C/cuda-12.8 PATH=$C/cuda-12.8/bin:$V/bin:$PATH TORCH_CUDA_ARCH_LIST="$ARCH" \
  GIT_LFS_SKIP_SMUDGE=1 pipi --no-build-isolation \
  "nvidia_curobo@git+https://github.com/StanfordVL/curobo@cbaf7d32436160956dad190a9465360fad6aba73"
pipi iopath einops timm "ftfy==6.1.1" decord pycocotools                 # uv_install.sh lines 309-321

# 4. the C++ toolchain OmniGibson's torch.compile needs (micromamba 2.3.3, a single binary)
curl -fsSL https://github.com/mamba-org/micromamba-releases/releases/download/2.3.3-0/micromamba-linux-64 \
  -o $C/cache/micromamba && chmod +x $C/cache/micromamba
MAMBA_ROOT_PREFIX=$C/cache/mamba $C/cache/micromamba create -y -p $C/toolchain \
  --override-channels -c conda-forge "gxx_linux-64=13.3.0"

# 5. the official dataset, into OMNIGIBSON_DATA_PATH: uv_install.sh --dataset's own steps
export OMNIGIBSON_DATA_PATH=$D TMPDIR=$D/.tmp; mkdir -p $TMPDIR
$V/bin/python -c "from omnigibson.utils.asset_utils import download_omnigibson_robot_assets as f; f()"
cp -a $B/assets/r1pro_ik.urdf $D/omnigibson-robot-assets/models/r1pro/urdf/
$V/bin/python -c "from omnigibson.utils.asset_utils import download_behavior_1k_assets as f; f(accept_license=True)"
$V/bin/python -c "from omnigibson.utils.asset_utils import download_2025_challenge_task_instances as f; f()"
rm -rf $D/.tmp
chmod -R a-w $D
```

- Expected: CPython 3.10, omnigibson 3.7.2 and bddl 3.7.0 (editable, from `b1k`), isaacsim 4.5.0.0,
  torch 2.9.1+cu128 and numpy 1.26.4. `$D` holds `omnigibson-robot-assets`, `behavior-1k-assets`
  (its `VERSION` reads `3.7.2rc1`), `2025-challenge-task-instances` and `omnigibson.key`.
- The dataset's three zips come from the Hugging Face dataset `behavior-1k/zipped-datasets`. To
  match the validated tree exactly, fetch `omnigibson-robot-assets.zip`,
  `behavior-1k-assets-3.7.2rc1.zip` and `2025-challenge-task-instances.zip` at revision
  `9f0d57d465726976ed98138d3f8b8ca3e2186775` and extract them into `$D` instead (sha256
  `5e9fe726172c8b4b...`, `3bd7cdc1a8c56217...` and `6262a496c82a534f...`). The installed
  `r1pro.urdf` hashes to `b0d76760cbedfbcb...`. Run the instances download once only.

Then, for every BEHAVIOR run (alongside `AMP_CAPX_ROOT`):

```bash
export AMP_CAPX_BEHAVIOR_PYTHON=~/capx/venv-b1k/bin/python
export AMP_CAPX_BEHAVIOR_DATA=~/capx/b1k-data              # set as OMNIGIBSON_DATA_PATH in the sim
export AMP_B1K_TOOLCHAIN=~/capx/toolchain                  # the sim's CXX and CC
```

## 2. Running

The commands for the paper's CaP-Gym results (Figures 2C-2E) are in
[reproducing-the-paper.md](reproducing-the-paper.md).

For `behavior-1k`, set `AMP_CAPX_BEHAVIOR_PYTHON`, `AMP_CAPX_BEHAVIOR_DATA` and `AMP_B1K_TOOLCHAIN`
(section 1) and run it in its own `amp eval capgym`. Keep the data read-only and outside this
repository, the runs folder and the system directories: BEHAVIOR refuses to run when its data lies
under a directory the kernel sandbox binds.

## 3. Results

`<output>` is the run folder and `<config>` the harness configuration,
`<harness>__<model>[-<reasoning>]__<prompt>`:

```text
<output>/summary.json                                  per <config>: total and per task {units, scored, succeeded, success_rate, missing}
<output>/<config>/agentic/run_config.json, status.json the configuration's settings, and a tally of its units
<output>/<config>/agentic/units/<Task>__<k:04d>.json   the final record of trial k+1
<output>/<config>/agentic/official/                    the official-mode record of each trial
<output>/<config>/agentic/episodes/                    each episode's traces and robot_result.json
```

The paper's success rate for a corpus is `summary.json`'s `total.success_rate` for the
configuration: successes over scored episodes (3,000 for LIBERO-PRO, 700 for robosuite, 50 for
BEHAVIOR-1K). The launcher prints the same count at the end.
