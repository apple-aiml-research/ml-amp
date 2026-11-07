#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""amp doctor: check GPUs, glibc, bwrap, the sandbox, RoboLab and the model for amp eval robolab,
and with --robodojo, RoboDojo for amp eval robodojo."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

PROFILE = Path("/etc/apparmor.d/bwrap")
USERNS_SYSCTL = Path("/proc/sys/kernel/apparmor_restrict_unprivileged_userns")
REPO_PROFILE = "docs/apparmor/bwrap"
BWRAP_PROBE = ("--unshare-user", "--unshare-net", "--unshare-pid", "--unshare-ipc",
               "--die-with-parent", "--ro-bind", "/", "/", "--proc", "/proc", "--dev", "/dev",
               "--tmpfs", "/tmp", "true")  # nosec B108
BWRAP_PROBE_NO_PROC = tuple(a for a in BWRAP_PROBE if a not in ("--proc", "/proc"))
BWRAP_REQUIRED = ("--ro-bind", "--bind", "--tmpfs", "--uid", "--gid", "--symlink", "--proc",
                  "--dev", "--hostname", "--chdir", "--cap-drop", "--unshare-user",
                  "--unshare-all", "--die-with-parent")
BWRAP_OPTIONAL = ("--size", "--disable-userns")
BWRAP_INSTALL = [("install bubblewrap: `sudo apt install bubblewrap` on Debian or Ubuntu, "
                  "otherwise the distribution's bubblewrap package (e.g. `sudo dnf install "
                  "bubblewrap`); then rerun amp doctor, which says whether AppArmor needs a "
                  "profile")]
NEWER_BWRAP = ("install a newer bubblewrap (0.4.0 is the oldest validated): the distribution's "
               "backports, or a build from https://github.com/containers/bubblewrap")
ISAAC_GLIBC = (2, 35)
ROBOLAB_FILES = ("robolab", "analysis/compile_results.py", "analysis/read_results.py",
                 "robolab/tasks/_metadata/task_metadata.json")
ROBOLAB_ROOT_ENV = "AMP_ROBOLAB_ROOT"
ROBOLAB_CLONE = ("clone RoboLab into external/robolab at its pinned commit (make robolab, or the "
                 "commands in docs/reproducibility.md), or point $AMP_ROBOLAB_ROOT at a checkout")
ROBOLAB_PYTHON_ENV = "AMP_ROBOLAB_PYTHON"
ROBODOJO_SETUP = "set RoboDojo up as docs/robodojo.md, section 1, says"
ROBODOJO_SUBMODULES = {"xpolicylab": "XPolicyLab", "isaaclab": "third_party/IsaacLab",
                       "curobo": "third_party/curobo"}
ROBODOJO_VENV = ("isaacsim", "isaaclab", "curobo", "websockets")
XPL_VENV = ("websockets", "msgpack", "msgpack_numpy", "yaml", "numpy", "cv2", "h5py", "pydantic")
ROBODOJO_SYSTEM_LIBS = {"GLU": "libglu1-mesa", "vulkan": "libvulkan1"}


@dataclass
class Check:
    name: str
    status: str
    detail: str = ""
    fix: list[str] = field(default_factory=list)


def check_gpus(spec: str, query: Callable | None = None) -> list[Check]:
    from amp.eval import gpus

    try:
        found = (query or gpus.query)()
    except gpus.GpuError as e:
        return [Check("gpus", "fail", str(e))]
    out = []
    for g in found:
        why = g.refusal()
        small = g.small_warning().removeprefix("warning: ") if g.small else ""
        out.append(Check(f"gpu {g.index}", "ok" if why is None and not small else "warn",
                         g.describe() + (f": refused, {why}" if why else ": usable")
                         + (f"; {small}" if small else "")))
    try:
        chosen, _ = gpus.select(spec, found, warn=lambda m: None)
        out.append(Check("gpus", "ok", f"--gpus {spec} selects "
                         + ", ".join(f"{g.index} ({g.uuid})" for g in chosen)))
    except gpus.GpuError as e:
        out.append(Check("gpus", "fail", str(e)))
    return out


def bwrap_fix(profile_present: bool) -> list[str]:
    if profile_present:
        return [("the AppArmor profile is installed but bwrap still fails: check "
                 "`sudo aa-status | grep bwrap` and `dmesg | grep -i apparmor`")]
    return [(f"{USERNS_SYSCTL} reads 1: install the AppArmor profile that lets bwrap create "
             f"user namespaces ({REPO_PROFILE} in this repo):"),
            f"  sudo install -m 644 {REPO_PROFILE} /etc/apparmor.d/bwrap",
            "  sudo apparmor_parser -r /etc/apparmor.d/bwrap",
            "only on a single-user machine, the system-wide alternative is:",
            "  sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0"]


def userns_restricted(sysctl: Path = USERNS_SYSCTL) -> bool:
    """True when AppArmor's unprivileged-userns restriction exists and is on."""
    try:
        return sysctl.read_text().strip() == "1"
    except OSError:
        return False


def _out(run: Callable, argv: list[str]) -> str:
    try:
        p = run(argv, capture_output=True, text=True, timeout=30, check=False)
        return f"{p.stdout or ''}{p.stderr or ''}"
    except (OSError, subprocess.TimeoutExpired):
        return ""


def _probe(run: Callable, argv: list[str]) -> tuple[int, str]:
    try:
        p = run(argv, capture_output=True, text=True, timeout=30, check=False)
        return p.returncode, (p.stderr or "").strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, f"{type(e).__name__}: {e}"


def check_bwrap(*, which: Callable[[str], str | None] = shutil.which,
                run: Callable = subprocess.run, profile: Path = PROFILE,
                platform: str = sys.platform, sysctl: Path = USERNS_SYSCTL) -> Check:
    if not platform.startswith("linux"):
        return Check("bwrap", "fail", f"AMP runs on Linux only, not {platform}")
    exe = which("bwrap")
    if not exe:
        return Check("bwrap", "fail", "bwrap is not installed", BWRAP_INSTALL)
    version = _out(run, [exe, "--version"]).strip() or "bubblewrap of unknown version"
    help_text = _out(run, [exe, "--help"])
    flags = set(help_text.split())
    missing = [f for f in BWRAP_REQUIRED if f not in flags]
    if "[--]" not in flags:
        missing.append("the -- separator")
    if missing:
        return Check("bwrap", "fail", f"{exe} ({version}) lacks {', '.join(missing)}, which the "
                     "sandbox needs", [NEWER_BWRAP])
    optional = [f for f in BWRAP_OPTIONAL if f in flags]
    rc, err = _probe(run, [exe, *BWRAP_PROBE])
    no_proc = ""
    if rc != 0:
        rc2, _ = _probe(run, [exe, *BWRAP_PROBE_NO_PROC])
        if rc2 == 0:
            rc, no_proc = 0, " without a fresh /proc (refused here, as in many containers)"
    present = profile.exists()
    if rc == 0:
        return Check("bwrap", "ok", f"{exe} ({version}) creates user, net, pid and ipc namespaces"
                     f"{no_proc}; optional {', '.join(optional) or 'none'}"
                     + (f" (AppArmor profile {profile})" if present else ""))
    why = f"{exe} ({version}) cannot create its namespaces: {err[:300]}"
    if userns_restricted(sysctl):
        return Check("bwrap", "fail", why, bwrap_fix(present))
    return Check("bwrap", "fail", why, [(
        "AppArmor does not restrict user namespaces here, so the profile is not the fix: check "
        "`sysctl user.max_user_namespaces` (must be > 0) and, where it exists, "
        "kernel.unprivileged_userns_clone (must be 1); a container needs its seccomp and "
        "AppArmor confinement lifted (or --privileged) for bwrap to create namespaces")])


def glibc_version(confstr: Callable[[str], str | None] = os.confstr) -> tuple[int, ...] | None:
    """The C library's glibc version, or None when it is not glibc."""
    try:
        name, _, ver = (confstr("CS_GNU_LIBC_VERSION") or "").partition(" ")
        return tuple(int(x) for x in ver.split(".")[:2]) if name == "glibc" else None
    except (ValueError, OSError):
        return None


def check_glibc(*, confstr: Callable[[str], str | None] = os.confstr,
                platform: str = sys.platform) -> Check:
    if not platform.startswith("linux"):
        return Check("glibc", "ok", f"not checked on {platform}: RoboLab runs on Linux")
    ver = glibc_version(confstr)
    need = ".".join(map(str, ISAAC_GLIBC))
    if ver is not None and ver >= ISAAC_GLIBC:
        return Check("glibc", "ok", f"glibc {'.'.join(map(str, ver))} >= {need} (Isaac Sim 5.0)")
    found = f"glibc {'.'.join(map(str, ver))}" if ver else "no glibc"
    return Check("glibc", "warn", f"{found}: RoboLab's Isaac Sim 5.0 needs glibc {need} or newer "
                 "(e.g. Ubuntu 22.04+); AMP's own side (gateway, agents, kernel sandbox) still "
                 "works here")


def check_sandbox(python: str) -> Check:
    try:
        from amp.kernel.sandbox import require_ok, self_test

        report = self_test("auto", python=python)
        require_ok(report)
    except ModuleNotFoundError as e:
        return Check("sandbox", "warn", f"self-test unavailable in this build: {e}")
    except Exception as e:  # noqa: BLE001
        return Check("sandbox", "fail", f"{type(e).__name__}: {e}")
    return Check("sandbox", "ok", f"{report.backend} self-test passed "
                 f"({sum(bool(v) for v in report.checks.values())} checks)")


def check_robolab(root: str | None, python: str | None,
                  run: Callable = subprocess.run) -> list[Check]:
    from amp.eval.provenance import PINS, git_head

    if not root:
        return [Check("robolab", "fail", f"no RoboLab checkout: set ${ROBOLAB_ROOT_ENV} or "
                      "pass --robolab-root", [ROBOLAB_CLONE])]
    missing = [f for f in ROBOLAB_FILES if not (Path(root) / f).exists()]
    if missing:
        out = [Check("robolab checkout", "fail", f"{root}: missing {', '.join(missing)}",
                     [ROBOLAB_CLONE])]
    else:
        pin, head = PINS["robolab_commit"][1], git_head(root)
        at = ("at the pinned v0.3.1" if head == pin else
              f"at {head}, not the pinned {pin}" if head else "not a git checkout, commit unknown")
        out = [Check("robolab checkout", "ok" if head == pin else "warn", f"{root}: {at}",
                     [] if head == pin else [ROBOLAB_CLONE])]
    if not python:
        out.append(Check("robolab venv", "fail", f"no RoboLab python: set ${ROBOLAB_PYTHON_ENV} "
                         "or pass --robolab-python"))
        return out
    code = ("import importlib.util as u, sys; m=[n for n in ('isaaclab', 'robolab') "
            "if u.find_spec(n) is None]; print(sys.version.split()[0], *m); sys.exit(1 if m else 0)")
    try:
        p = run([python, "-c", code], capture_output=True, text=True, timeout=120, check=False,
                cwd=root)
        ok, text = p.returncode == 0, (p.stdout or p.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        ok, text = False, f"{type(e).__name__}: {e}"
    out.append(Check("robolab venv", "ok" if ok else "fail",
                     f"{python}: Python {text}" if ok else f"{python}: missing {text}"))
    return out


def _imports(python: str, modules: tuple[str, ...], cwd: str | None, run: Callable,
             versions: tuple[str, ...] = ()) -> tuple[bool, str]:
    """(every module importable, Python version and the versions asked for, or what is missing)."""
    code = ("import importlib.util as u, importlib.metadata as md, sys; "
            f"m=[n for n in {modules!r} if u.find_spec(n) is None]; "
            f"v=[f'{{d}} {{md.version(d)}}' for d in {versions!r} if not m]; "
            "print(sys.version.split()[0], *(m or v)); sys.exit(1 if m else 0)")
    try:
        p = run([python, "-c", code], capture_output=True, text=True, timeout=120, check=False,
                cwd=cwd)
        return p.returncode == 0, (p.stdout or p.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, f"{type(e).__name__}: {e}"


def check_robodojo(root: Path, python: str | None, xpl_python: str | None,
                   run: Callable = subprocess.run, environ=None,
                   find_library: Callable[[str], str | None] | None = None) -> list[Check]:
    """RoboDojo's checkout, Assets, both venvs, system libraries, ffmpeg and the EULA."""
    from amp.benchmarks import robodojo as bench
    from amp.eval.provenance import PINS, git_head

    env = os.environ if environ is None else environ
    if not (root / "scripts" / "eval_policy.sh").is_file():
        return [Check("robodojo checkout", "fail", f"no RoboDojo checkout at {root}",
                      ["make robodojo, or point $AMP_ROBODOJO_ROOT at a checkout"])]
    pins = bench.load().pins
    want = PINS["robodojo_commit"][1]
    off = [f"RoboDojo at {git_head(root)}"] if git_head(root) != want else []
    off += [f"{d} at {git_head(root / d)}" for k, d in ROBODOJO_SUBMODULES.items()
            if git_head(root / d) != pins[k]]
    out = [Check("robodojo checkout", "warn" if off else "ok",
                 f"{root}: " + (", ".join(off) + ", not the pins" if off else
                                f"at the pinned {want[:12]}, submodules at their pins"),
                 ["make robodojo (it checks out the submodules RoboDojo pins)"] if off else [])]
    layouts = root / "Assets" / "Eval_Layout" / "RoboDojo" / "arx_x5"
    missing = [str(s) for s in bench.SEEDS if not (layouts / str(s)).is_dir()]
    if missing or not (root / "Assets" / "Robots" / "x5" / "curobo.yml").is_file():
        why = (f"no layouts for seeds {', '.join(missing)} in {layouts}" if missing else
               "Assets/Robots/x5/curobo.yml is missing (run utils/update_embodiment_config_path.py)")
        out.append(Check("robodojo assets", "fail", why, [ROBODOJO_SETUP]))
    else:
        n = sum(1 for _ in (layouts / "0").glob("*.json"))
        out.append(Check("robodojo assets", "ok", f"{n} seed-0 layouts, embodiment config written"))
    for name, py, mods, vers in (("robodojo venv", python, ROBODOJO_VENV, ("isaacsim",)),
                                 ("robodojo policy-server venv", xpl_python, XPL_VENV,
                                  ("websockets",))):
        if not py:
            out.append(Check(name, "fail", "no interpreter: set $AMP_ROBODOJO_PYTHON and "
                             "$AMP_ROBODOJO_XPL_PYTHON", [ROBODOJO_SETUP]))
            continue
        ok, text = _imports(py, mods, str(root), run, vers)
        out.append(Check(name, "ok" if ok else "fail",
                         f"{py}: Python {text}" if ok else f"{py}: missing {text}",
                         [] if ok else [ROBODOJO_SETUP]))
    if find_library is None:
        import ctypes.util

        find_library = ctypes.util.find_library
    missing = {lib: pkg for lib, pkg in ROBODOJO_SYSTEM_LIBS.items() if not find_library(lib)}
    if missing:
        out.append(Check("system libraries", "fail",
                         f"{', '.join(f'lib{lib}' for lib in missing)} not found: Isaac Sim's "
                         "renderer would not start and episodes would hang at their reset",
                         [(f"sudo apt install {' '.join(missing.values())} "
                           "(docs/robodojo.md, section 1)")]))
    else:
        out.append(Check("system libraries", "ok",
                         ", ".join(f"lib{lib}" for lib in ROBODOJO_SYSTEM_LIBS)))
    venv_bin = str(Path(python).parent) if python else ""
    ffmpeg = shutil.which("ffmpeg", path=os.pathsep.join(p for p in (venv_bin, env.get("PATH", ""))
                                                       if p))
    if ffmpeg is None:
        out.append(Check("ffmpeg", "fail", "RoboDojo writes its videos with ffmpeg (libx264), "
                         "which is not on PATH", ["sudo apt install ffmpeg"]))
    else:
        try:
            enc = run([ffmpeg, "-hide_banner", "-encoders"], capture_output=True, text=True,
                      timeout=30, check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            enc = ""
        out.append(Check("ffmpeg", "ok" if "libx264" in enc else "fail",
                         f"{ffmpeg}" + ("" if "libx264" in enc else ": no libx264 encoder"),
                         [] if "libx264" in enc else ["install an ffmpeg built with libx264"]))
    if not (env.get("OMNI_KIT_ACCEPT_EULA") or "").strip():
        out.append(Check("omniverse eula", "warn", "OMNI_KIT_ACCEPT_EULA is not set: Isaac Sim "
                         "asks to accept NVIDIA's Omniverse licence and a headless run stops there",
                         ["export OMNI_KIT_ACCEPT_EULA=YES, if you accept NVIDIA's terms"]))
    return out


def check_model(model: str) -> Check:
    try:
        from amp.agents.models import get_model

        spec = get_model(model)
    except Exception as e:  # noqa: BLE001
        return Check("model", "fail", f"{model}: {type(e).__name__}: {e}")
    try:
        import amp.agents.modelconfig  # noqa: F401
        where = "model config"
    except ImportError:
        where = "built-in models"
    return Check("model", "ok", f"{model} resolves to {spec.provider} {spec.model_id} ({where})")


def check_model_call(model: str, environ=None) -> Check:
    """One short real call to the model, so a wrong URL, key or model id fails here."""
    from amp.agents.factory import build_llm
    from amp.agents.llm.base import TRANSIENT_ATTEMPTS_ENV
    from amp.agents.models import profile

    env = {**(os.environ if environ is None else environ), TRANSIENT_ATTEMPTS_ENV: "2"}
    try:
        llm = build_llm(profile(model), environ=env)
        llm.max_attempts = 2
        llm.deadline = time.monotonic() + 180
        turn = llm.create(system=[], tools=[], messages=[
            {"role": "user", "content": [{"type": "text", "text": "Reply with the word ok."}]}])
    except Exception as e:  # noqa: BLE001
        return Check("model call", "fail", f"{model}: {type(e).__name__}: {str(e)[:300]}")
    return Check("model call", "ok", f"{model} replied in {turn.latency_s:.1f} s")


def report(checks: list[Check], out=None) -> int:
    out = out or sys.stdout
    for c in checks:
        print(f"[{c.status:4}] {c.name}: {c.detail}", file=out)
        for line in c.fix:
            print(f"       {line}", file=out)
    failed = [c.name for c in checks if c.status == "fail"]
    print(f"amp doctor: {'FAIL (' + ', '.join(failed) + ')' if failed else 'ok'}", file=out)
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="amp doctor", description=__doc__)
    ap.add_argument("--gpus", default="auto", help="auto, or GPU indices or UUIDs")
    ap.add_argument("--robolab-root", default=None)
    ap.add_argument("--robolab-python", default=None)
    ap.add_argument("--robodojo", action="store_true",
                    help="also check RoboDojo, for amp eval robodojo (docs/robodojo.md)")
    ap.add_argument("--robodojo-root", default=None)
    ap.add_argument("--robodojo-python", default=None)
    ap.add_argument("--xpl-python", default=None)
    ap.add_argument("--model", default="opus-5.5")
    ap.add_argument("--kernel-python", default=sys.executable)
    ap.add_argument("--skip", action="append", default=[],
                    choices=("gpus", "glibc", "bwrap", "sandbox", "robolab", "model", "model-call"))
    a = ap.parse_args(argv)
    from amp.eval.robolab_official import robolab_paths

    a.robolab_root, a.robolab_python = robolab_paths(a.robolab_root, a.robolab_python)
    checks: list[Check] = []
    if "gpus" not in a.skip:
        checks += check_gpus(a.gpus)
    if "glibc" not in a.skip:
        checks.append(check_glibc())
    if "bwrap" not in a.skip:
        checks.append(check_bwrap())
    if "sandbox" not in a.skip:
        checks.append(check_sandbox(a.kernel_python))
    if "robolab" not in a.skip:
        checks += check_robolab(a.robolab_root, a.robolab_python)
    if a.robodojo:
        from amp.robots.robodojo.backend import PYTHON_ENV, XPL_PYTHON_ENV, default_root

        root = Path(a.robodojo_root).expanduser() if a.robodojo_root else default_root()
        checks += check_robodojo(root, a.robodojo_python or os.environ.get(PYTHON_ENV),
                                 a.xpl_python or os.environ.get(XPL_PYTHON_ENV))
    if "model" not in a.skip:
        checks.append(check_model(a.model))
        if "model-call" not in a.skip and checks[-1].status == "ok":
            checks.append(check_model_call(a.model))
    return report(checks)


if __name__ == "__main__":
    sys.exit(main())
