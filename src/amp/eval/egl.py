#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Each NVIDIA GPU's EGL device index, keyed by GPU UUID, for MuJoCo's MUJOCO_EGL_DEVICE_ID."""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass

MAP_ENV = "AMP_EGL_DEVICE_MAP"
ENUM_TIMEOUT_S = 60.0
SMI_QUERY = ("--query-gpu=index,uuid,pci.bus_id", "--format=csv,noheader")
HINT = (f"set {MAP_ENV} to each GPU's EGL device: by nvidia-smi index (\"0,1\") or by UUID "
        f"(\"GPU-...=0,GPU-...=1\")")

ENUM_SCRIPT = r'''
import ctypes, json
egl = ctypes.CDLL("libEGL.so.1")
egl.eglGetProcAddress.restype = ctypes.c_void_p
egl.eglGetProcAddress.argtypes = [ctypes.c_char_p]

def proc(name, restype, *argtypes):
    addr = egl.eglGetProcAddress(name)
    if not addr:
        raise SystemExit(f"libEGL has no {name.decode()}")
    return ctypes.CFUNCTYPE(restype, *argtypes)(addr)

query = proc(b"eglQueryDevicesEXT", ctypes.c_uint, ctypes.c_int,
             ctypes.POINTER(ctypes.c_void_p), ctypes.POINTER(ctypes.c_int))
string = proc(b"eglQueryDeviceStringEXT", ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int)
attrib = proc(b"eglQueryDeviceAttribEXT", ctypes.c_uint, ctypes.c_void_p, ctypes.c_int,
              ctypes.POINTER(ctypes.c_ssize_t))
devs, n = (ctypes.c_void_p * 32)(), ctypes.c_int()
if not query(32, devs, ctypes.byref(n)):
    raise SystemExit("eglQueryDevicesEXT failed")
out = []
for i in range(n.value):
    ext = (string(devs[i], 0x3055) or b"").decode()
    def text(code, name):
        value = string(devs[i], code) if name in ext else None
        return value.decode() if value else None
    cuda = ctypes.c_ssize_t(-1)
    has_cuda = "EGL_NV_device_cuda" in ext and attrib(devs[i], 0x323A, ctypes.byref(cuda))
    out.append({"index": i, "drm": text(0x3233, "EGL_EXT_device_drm"),
                "render": text(0x3377, "EGL_EXT_device_drm_render_node"),
                "cuda": cuda.value if has_cuda else None,
                "vendor": text(0x3053, "EGL_EXT_device_query_name")})
print(json.dumps(out))
'''


class EglError(RuntimeError):
    """A selected GPU has no known EGL device."""


@dataclass(frozen=True)
class SmiGpu:
    index: int
    uuid: str
    bus: str | None


def bus_id(text: str) -> str | None:
    """A PCI bus id as dddd:bb:dd.f (nvidia-smi prints 8 domain digits, sysfs 4)."""
    m = re.fullmatch(r"\s*([0-9a-fA-F]+):([0-9a-fA-F]{2}):([0-9a-fA-F]{2})\.([0-7])\s*", text or "")
    return f"{int(m[1], 16):04x}:{m[2].lower()}:{m[3].lower()}.{m[4]}" if m else None


def parse_smi(text: str) -> list[SmiGpu]:
    out = []
    for row in text.splitlines():
        cols = [c.strip() for c in row.split(",")]
        if len(cols) >= 3 and cols[0].isdigit():
            out.append(SmiGpu(int(cols[0]), cols[1], bus_id(cols[2])))
    return sorted(out, key=lambda g: g.index)


def query_smi(smi: str | None = None, timeout_s: float = 30.0) -> list[SmiGpu]:
    exe = smi or os.environ.get("AMP_NVIDIA_SMI") or shutil.which("nvidia-smi")
    if not exe:
        raise EglError("nvidia-smi not found")
    try:
        p = subprocess.run([exe, *SMI_QUERY], capture_output=True, text=True, timeout=timeout_s,  # nosemgrep: python.lang.security.audit.dangerous-subprocess-use-tainted-env-args.dangerous-subprocess-use-tainted-env-args
                           check=False)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise EglError(f"nvidia-smi: {type(e).__name__}: {e}") from e
    if p.returncode != 0:
        raise EglError(f"nvidia-smi exited {p.returncode}: {p.stderr.strip()[:300]}")
    return parse_smi(p.stdout)


def enumerate_devices(python: str | None = None, timeout_s: float = ENUM_TIMEOUT_S) -> list[dict]:
    """The EGL devices as ENUM_SCRIPT lists them, run in a child that sees every GPU."""
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "CUDA_VISIBLE_DEVICES")}
    env.update(PYOPENGL_PLATFORM="egl", CUDA_DEVICE_ORDER="PCI_BUS_ID")
    try:
        p = subprocess.run([python or sys.executable, "-c", ENUM_SCRIPT], capture_output=True,  # nosemgrep: python.lang.security.audit.dangerous-subprocess-use-tainted-env-args.dangerous-subprocess-use-tainted-env-args
                           text=True, timeout=timeout_s, check=False, env=env, cwd="/",
                           stdin=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise EglError(f"listing EGL devices: {type(e).__name__}: {e}") from e
    if p.returncode != 0:
        raise EglError(f"listing EGL devices failed: {(p.stderr or p.stdout).strip()[-300:]}")
    try:
        rows = json.loads(p.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError) as e:
        raise EglError(f"the EGL device list is not JSON: {p.stdout[-200:]!r}") from e
    return [r for r in rows if isinstance(r, dict) and isinstance(r.get("index"), int)]


def is_nvidia(dev: dict) -> bool:
    """Whether an EGL device is NVIDIA's: it has a CUDA ordinal, or says NVIDIA as its vendor."""
    return isinstance(dev.get("cuda"), int) or str(dev.get("vendor") or "").startswith("NVIDIA")


def by_pci(devices: Sequence[dict], sysfs: str = "/sys", devdir: str = "/dev/dri"
           ) -> dict[str, int]:
    """{PCI bus id: EGL index} through each device's DRM node in sysfs, the first device winning;
    a non-NVIDIA one only through a node this process can open."""
    out: dict[str, int] = {}
    for dev in devices:
        for node in (dev.get("render"), dev.get("drm")):
            if not node or (not is_nvidia(dev) and not os.access(
                    os.path.join(devdir, os.path.basename(node)), os.R_OK | os.W_OK)):
                continue
            link = os.path.join(sysfs, "class", "drm", os.path.basename(node), "device")
            pci = bus_id(os.path.basename(os.path.realpath(link))) if os.path.exists(link) else None
            if pci:
                out.setdefault(pci, int(dev["index"]))
                break
    return out


def by_cuda(gpus: Sequence[SmiGpu], devices: Sequence[dict]) -> dict[str, int]:
    """{uuid: EGL index} from EGL_NV_device_cuda's ordinal, enumerated with CUDA_DEVICE_ORDER
    PCI_BUS_ID: ordinal k is the GPU k-th in PCI bus order."""
    ranked = sorted((g for g in gpus if g.bus), key=lambda g: g.bus)
    if len(ranked) != len(gpus):
        return {}
    ords = {int(d["cuda"]): int(d["index"]) for d in devices if isinstance(d.get("cuda"), int)
            and d["cuda"] >= 0}
    if len(ords) != len([d for d in devices if isinstance(d.get("cuda"), int) and d["cuda"] >= 0]):
        return {}
    if sorted(ords) != list(range(len(ranked))):
        return {}
    return {g.uuid: ords[k] for k, g in enumerate(ranked)}


def discover(gpus: Sequence[SmiGpu], devices: Sequence[dict], sysfs: str = "/sys",
             devdir: str = "/dev/dri") -> tuple[dict[str, int], str]:
    """({uuid: EGL index} for every GPU it can match, how it matched them)."""
    nvidia = [d for d in devices if is_nvidia(d)]
    pci = by_pci(nvidia or devices, sysfs, devdir)
    out = {g.uuid: pci[g.bus] for g in gpus if g.bus in pci}
    if len(out) == len(gpus):
        return out, "pci"
    cuda = by_cuda(gpus, devices)
    if len(cuda) == len(gpus):
        return cuda, "cuda_ordinal"
    one = nvidia or devices
    if len(gpus) == 1 and len(one) == 1:
        return {gpus[0].uuid: int(one[0]["index"])}, "trivial"
    return out, "pci"


def parse_override(text: str, gpus: Sequence[SmiGpu]) -> dict[str, int]:
    """MAP_ENV as {uuid: EGL index}; ValueError when it is malformed or names no such GPU."""
    by_index = {g.index: g.uuid for g in gpus}
    known = {g.uuid for g in gpus}
    out: dict[str, int] = {}
    parts = [p.strip() for p in text.split(",") if p.strip()]
    for pos, part in enumerate(parts):
        key, sep, value = part.rpartition("=")
        try:
            dev = int(value)
        except ValueError:
            raise ValueError(f"{MAP_ENV}={text!r}: {part!r} is not an EGL index") from None
        if dev < 0:
            raise ValueError(f"{MAP_ENV}={text!r}: {part!r} is negative")
        if sep:
            if key not in known:
                raise ValueError(f"{MAP_ENV}={text!r}: no GPU {key!r}")
            out[key] = dev
        elif pos in by_index:
            out[by_index[pos]] = dev
    return out


def resolve(chosen: Sequence[str], *, environ: Mapping[str, str] | None = None,
            smi: Callable[[], list[SmiGpu]] = query_smi,
            devices: Callable[[], list[dict]] = enumerate_devices,
            sysfs: str = "/sys", devdir: str = "/dev/dri"
            ) -> tuple[dict[str, int], str, list[SmiGpu]]:
    """({uuid: EGL index} covering every chosen UUID, its source, nvidia-smi's GPUs); EglError
    naming the GPUs it cannot place."""
    env = os.environ if environ is None else environ
    gpus = smi()
    override = env.get(MAP_ENV, "").strip()
    if override:
        try:
            got, how = parse_override(override, gpus), "env"
        except ValueError as e:
            raise EglError(str(e)) from None
    else:
        got, how = discover(gpus, devices(), sysfs, devdir)
    missing = [u for u in chosen if u not in got]
    if missing:
        raise EglError(f"no EGL device known for GPU(s) {', '.join(missing)} ({how}); {HINT}")
    return got, how, gpus


def positional(mapping: Mapping[str, int], gpus: Sequence[SmiGpu]) -> str | None:
    """mapping as the "e0,e1,..." list by nvidia-smi index; None unless it covers every GPU."""
    if any(g.uuid not in mapping for g in gpus) or \
            [g.index for g in gpus] != list(range(len(gpus))):
        return None
    return ",".join(str(mapping[g.uuid]) for g in gpus)
