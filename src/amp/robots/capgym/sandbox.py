#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Run isolation for a capgym slot: its private dirs, port block and EGL device, and in-process
guards for the hard-coded /tmp writers that TMPDIR cannot redirect."""

from __future__ import annotations

import functools
import hashlib
import importlib.abc
import logging
import os
import secrets
import shutil
import sys
import threading
import uuid
from pathlib import Path

SANDBOX = "sandbox"
SANDBOX_ENV = "AMP_CAPGYM_SANDBOX"
FRESH = ("TMPDIR", "XDG_RUNTIME_DIR")
EGL_MAP_ENV = "AMP_EGL_DEVICE_MAP"
CAPX_COMMIT_ENV = "AMP_CAPX_COMMIT"
PORT_ERROR_ENV = "AMP_CAPGYM_PORT_ERROR"
VISER_PORT_ENV = "_VISER_PORT_OVERRIDE"
PORT_BASE, PORT_ORIGIN, PORT_BLOCK = 20000, 9600, 16


LOCAL_ORIGIN, LOCAL_BASE = 12000, 9700
LOCAL_BLOCKS = (LOCAL_ORIGIN - LOCAL_BASE) // PORT_BLOCK
VISER_OFFSET = 0

RESERVED_PORTS = ((8710, 8710), (9600, 9699), (8114, 8116), (8211, 8211), (49100, 49100))
ROBOSUITE_LOGGER = "robosuite_logs"
TEXTURE_DIR = "/tmp/robosuite_temp_tex"  # nosec B108
TEXTURE_MODULE = "robosuite.utils.mjcf_utils"

REWRITTEN_COMPONENTS = frozenset({"robosuite", "libero"})

B1K_DATA_ENV = "AMP_CAPX_BEHAVIOR_DATA"

TOOLCHAIN_ENV = "AMP_B1K_TOOLCHAIN"
TOOLCHAIN_TRIPLE = "x86_64-conda-linux-gnu"
COMPILE_CACHE_ENV = "AMP_B1K_COMPILE_CACHE"
COMPILE_THREADS_MAX = 4
VK_MAP_ENV = "AMP_B1K_VK_DEVICE_MAP"
EXP_PATH_ENV = "EXP_PATH"

ISAAC_LINKED = ("exts", "extscache", "extsPhysics", "extsUser", "extsDeprecated")

log = logging.getLogger("amp.robots.capgym")
_atomic_dirs: set[str] = set()
_installed = False
_texture = threading.local()



def port_block(server_port: int) -> range:
    """The ports a slot's sims may bind, by its server's port (RoboDojo's slots)."""
    port = int(server_port)
    local = port >= LOCAL_ORIGIN
    if local:
        start = LOCAL_BASE + PORT_BLOCK * (port % LOCAL_BLOCKS)
    else:
        start = PORT_BASE + PORT_BLOCK * (port - PORT_ORIGIN)
    block = range(start, start + PORT_BLOCK)
    where = f"server port {server_port}: its sim ports {start}-{block.stop - 1}"
    if start < 1024 or block.stop > 65536:
        raise ValueError(f"{where} are outside 1024-65535")

    taken = (*RESERVED_PORTS, (LOCAL_ORIGIN, 65535)) if local else RESERVED_PORTS
    hit = [str(lo) if lo == hi else f"{lo}-{hi}" for lo, hi in taken
           if lo < block.stop and hi >= start]
    if port in block:
        hit.append(f"{port} (the server's own)")
    if hit:
        raise ValueError(f"{where} overlap the reserved {', '.join(hit)}")
    return block


def port_env(server_port: int) -> dict[str, str]:
    """The sims' port variables for the slot, or PORT_ERROR_ENV saying why it has none."""
    try:
        block = port_block(server_port)
    except ValueError as exc:
        return {PORT_ERROR_ENV: str(exc)}
    return {VISER_PORT_ENV: str(block[VISER_OFFSET])}


def require_ports(what: str) -> None:
    """ValueError if this process's slot has no port block, before a sim binds one."""
    why = os.environ.get(PORT_ERROR_ENV)
    if why:
        raise ValueError(f"{what} binds a port, and this slot has none: {why}")


def sandbox_env(slot_dir: str | os.PathLike) -> dict[str, str]:
    """HOME, TMPDIR, XDG and caches under <slot dir>/sandbox."""
    sb = Path(os.path.abspath(slot_dir)) / SANDBOX
    home = sb / "home"
    return {"HOME": str(home), "TMPDIR": str(sb / "tmp"), SANDBOX_ENV: str(sb),
            "XDG_RUNTIME_DIR": str(sb / "run"),
            "XDG_CACHE_HOME": str(home / ".cache"),
            "XDG_DATA_HOME": str(home / ".local" / "share"),
            "XDG_CONFIG_HOME": str(home / ".config"),
            "PYTHONDONTWRITEBYTECODE": "1", "NUMBA_CACHE_DIR": str(sb / "numba")}


def parse_device_map(device_map: str | None) -> list[int]:
    """A device map ("0,1,2,3": position = nvidia-smi index) as ints; [] if absent
    or malformed."""
    try:
        return [int(x) for x in device_map.split(",")] if device_map else []
    except ValueError:
        return []


def vk_device(gpu: int, device_map: str | None) -> str:
    """The Vulkan index of GPU `gpu`: the identity (measured on H100) unless VK_MAP_ENV maps it."""
    pos = max(gpu, 0)
    if not device_map:
        return str(pos)
    indices = parse_device_map(device_map)
    if pos >= len(indices) or any(i < 0 for i in indices):
        raise ValueError(f"{VK_MAP_ENV}={device_map!r} has no Vulkan index for GPU {pos}")
    return str(indices[pos])


def cuda_env(slot_dir: str | os.PathLike, gpu: int, visible: str | None,
             environ=None) -> dict[str, str]:
    """OmniGibson's and Kit's variables for a cuda family slot, under its sandbox. HOME, TMPDIR,
    XDG and PYTHONDONTWRITEBYTECODE are sandbox_env's."""
    env = os.environ if environ is None else environ
    physical = str(max(gpu, 0))
    if visible and visible.strip() != physical:

        raise ValueError(f"a cuda slot's CUDA_VISIBLE_DEVICES is its one physical GPU {physical}, "
                         f"not {visible!r}: Kit's renderer takes the physical index too")
    sb = Path(os.path.abspath(slot_dir)) / SANDBOX
    out = {"OMNIGIBSON_HEADLESS": "1", "OMNI_KIT_ACCEPT_EULA": "YES",
           "OMNIGIBSON_APPDATA_PATH": str(sb / "omnigibson-appdata"),


           EXP_PATH_ENV: str(sb / "isaac" / "apps"),
           "OPTIX_CACHE_PATH": str(sb / "home" / ".cache" / "optix"),
           "OMNI_CACHE_PATH": str(sb / "tmp" / "omni_cache"),

           "TORCH_EXTENSIONS_DIR": str(sb / "torch_extensions"),




           "OMNIGIBSON_GPU_ID": vk_device(gpu, env.get(VK_MAP_ENV))}
    if visible:
        out["CUDA_VISIBLE_DEVICES"] = physical
        if out["OMNIGIBSON_GPU_ID"] != physical and not env.get(VK_MAP_ENV):
            raise ValueError("OMNIGIBSON_GPU_ID differs from the physical GPU with no "
                             f"{VK_MAP_ENV} entry saying so")
    if env.get(B1K_DATA_ENV):
        out["OMNIGIBSON_DATA_PATH"] = env[B1K_DATA_ENV]
    out.update(compile_env(sb, env))
    return out


def compile_threads(cpus: int | None = None) -> int:
    """Inductor's compile workers: COMPILE_THREADS_MAX at most, and no more than the CPUs we may use."""
    if cpus is None:
        affinity = getattr(os, "sched_getaffinity", None)
        cpus = len(affinity(0)) if affinity else (os.cpu_count() or 1)
    return max(1, min(COMPILE_THREADS_MAX, cpus))


def compile_env(sb: Path, environ=None) -> dict[str, str]:
    """torch.compile as official runs it, with the install's toolchain and a per-slot inductor cache."""
    env = os.environ if environ is None else environ
    tc = env.get(TOOLCHAIN_ENV)
    cache = env.get(COMPILE_CACHE_ENV) or str(sb / "torchinductor")
    return {**toolchain_vars(tc),

            "TORCHINDUCTOR_CACHE_DIR": cache, "TORCHINDUCTOR_FX_GRAPH_CACHE": "1",
            "TORCHINDUCTOR_COMPILE_THREADS": str(compile_threads())}


def isaac_path() -> Path:
    """The running python's isaacsim package dir (ISAAC_PATH), found without importing it."""
    import importlib.util

    spec = importlib.util.find_spec("isaacsim")
    where = list(spec.submodule_search_locations or ()) if spec else []
    if not where:
        raise ValueError("no isaacsim package in this python: not the BEHAVIOR venv")
    return Path(where[0])


def make_isaac_slot(exp_path: str | os.PathLike, isaac: str | os.PathLike) -> None:
    """EXP_PATH as a real empty dir whose parent links the venv's five extension dirs; idempotent.
    ValueError rather than a path into the venv, which OmniGibson would then write to."""
    apps, isaac = Path(exp_path), Path(isaac)
    if apps.is_symlink() or os.path.realpath(apps).startswith(os.path.realpath(isaac) + os.sep):
        raise ValueError(f"EXP_PATH {apps} is in the venv's isaacsim {isaac}: OmniGibson rewrites it")
    apps.mkdir(parents=True, exist_ok=True)
    for name in ISAAC_LINKED:
        link, target = apps.parent / name, isaac / name
        if link.is_symlink() and os.readlink(link) == str(target):
            continue
        if link.is_symlink():
            link.unlink()
        link.symlink_to(target)


def prepare_isaac_slot(environ=None) -> bool:
    """Build this slot's EXP_PATH from the running venv's isaacsim; False off a cuda slot.
    Resolved in-process (the client runs in the family venv), so no subprocess or cache."""
    env = os.environ if environ is None else environ
    exp, sb = env.get(EXP_PATH_ENV), env.get(SANDBOX_ENV)
    if not exp or not sb or not Path(os.path.normpath(exp)).is_relative_to(os.path.normpath(sb)):
        return False
    make_isaac_slot(exp, isaac_path())
    return True


def egl_device(gpu: int, device_map: str | None) -> str:
    """The EGL index of GPU `gpu` from EGL_MAP_ENV's map; ValueError where a map is required."""
    pos = max(gpu, 0)
    if not device_map:


        if os.environ.get(CAPX_COMMIT_ENV):
            raise ValueError(f"no {EGL_MAP_ENV} on a CaP-X host: GPU {pos}'s EGL device is "
                             "unknown, and its GPU index may name another GPU")
        if pos:
            log.warning("no %s: using GPU %d's index as its EGL device", EGL_MAP_ENV,
                        pos)
        return str(pos)
    indices = parse_device_map(device_map)
    if pos >= len(indices) or any(i < 0 for i in indices):
        raise ValueError(f"{EGL_MAP_ENV}={device_map!r} has no EGL index for GPU {pos}")
    return str(indices[pos])


def fresh_paths(slot_dir: str | os.PathLike) -> tuple[str, ...]:
    """The FRESH dirs of the sandbox under slot_dir: what a process leaves there is wiped when
    it ends, for any exit (the slot runner does it, SIGKILL included)."""
    env = sandbox_env(slot_dir)
    return tuple(env[k] for k in FRESH)


def wipe_fresh(environ=None) -> list[str]:
    """Empty the FRESH dirs now, so what a process decrypted or cached there (the BEHAVIOR USD
    temp files are licensed content) does not outlive it; those it did."""
    return fresh_sandbox(environ)


def wipe_at_exit() -> None:
    """Wipe the fresh dirs when this process ends, however it does: BEHAVIOR decrypts licensed
    USD assets into TMPDIR (502 files once outlived a slot's process). A SIGTERM, which
    is how the slot runner stops a backend, becomes an exit so atexit runs; a SIGKILL cannot be
    handled, and the next start still wipes."""
    import atexit
    import signal

    atexit.register(wipe_fresh)
    if threading.current_thread() is threading.main_thread():
        signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(128 + signum))


def quiet_file_log(logger: logging.Logger) -> None:
    """Drop the logger's FileHandlers, those it has and any added later."""
    for h in [h for h in logger.handlers if isinstance(h, logging.FileHandler)]:
        logger.removeHandler(h)
        h.close()
    if getattr(logger.addHandler, "_amp_quiet", False):
        return
    add = logger.addHandler

    def add_handler(h: logging.Handler) -> None:
        if isinstance(h, logging.FileHandler):
            h.close()
            return
        add(h)

    add_handler._amp_quiet = True  # type: ignore[attr-defined]
    logger.addHandler = add_handler  # type: ignore[method-assign]


def atomic_saves(directory: str | os.PathLike) -> bool:
    """PIL saves into `directory` write a temp file and os.replace it; False without PIL."""
    try:
        from PIL import Image
    except ImportError:
        return False
    _atomic_dirs.add(os.path.abspath(directory))
    save = Image.Image.save
    if getattr(save, "_amp_atomic", False):
        return True

    def atomic_save(self, fp, format=None, **params):
        path = os.path.abspath(fp) if isinstance(fp, (str, os.PathLike)) else None
        to = getattr(_texture, "to", None)
        if isinstance(path, str) and to and os.path.dirname(path) == TEXTURE_DIR:
            os.makedirs(to, exist_ok=True)
            path = fp = os.path.join(to, os.path.basename(path))
        if not isinstance(path, str) or os.path.dirname(path) not in _atomic_dirs:
            return save(self, fp, format, **params)
        head, name = os.path.split(path)
        stem, ext = os.path.splitext(name)
        tmp = os.path.join(head, f".{stem}.{os.getpid()}.{uuid.uuid4().hex[:12]}{ext}")
        try:
            save(self, tmp, format, **params)
            os.replace(tmp, path)
        except BaseException:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
        return None

    atomic_save._amp_atomic = True  # type: ignore[attr-defined]
    Image.Image.save = atomic_save
    return True


def fresh_sandbox(environ=None) -> list[str]:
    """Empty (and make) the process's FRESH dirs that lie in its slot's sandbox; those it did."""
    env = os.environ if environ is None else environ
    sb = env.get(SANDBOX_ENV)
    if not sb or not os.path.isabs(sb):
        return []
    root, done = Path(os.path.normpath(sb)), []
    for key in FRESH:
        v = env.get(key)
        d = Path(os.path.normpath(v)) if v and os.path.isabs(v) else None
        if d is None or d == root or not d.is_relative_to(root):
            continue

        for p in (root, *(root.joinpath(*d.relative_to(root).parts[:i + 1])
                          for i in range(len(d.relative_to(root).parts)))):
            if p.is_symlink():
                p.unlink()
        for old in d.parent.glob(f".{d.name}.old-*"):
            shutil.rmtree(old, ignore_errors=True)
        if d.exists():
            aside = d.with_name(f".{d.name}.old-{secrets.token_hex(4)}")
            os.replace(d, aside)
            shutil.rmtree(aside, ignore_errors=True)
        d.mkdir(parents=True, exist_ok=True)
        if key == "XDG_RUNTIME_DIR":
            os.chmod(d, 0o700)  # nosemgrep: python.lang.security.audit.insecure-file-permissions.insecure-file-permissions
        done.append(str(d))
    return done


def texture_dir(environ=None) -> str | None:
    """The process's stand-in for TEXTURE_DIR, in its slot's TMPDIR and per venv; None outside one."""
    env = os.environ if environ is None else environ
    sb, tmp = env.get(SANDBOX_ENV), env.get("TMPDIR")
    if not (sb and tmp and os.path.isabs(sb) and os.path.isabs(tmp)):
        return None
    tmp = os.path.normpath(tmp)
    if not Path(tmp).is_relative_to(os.path.normpath(sb)):
        return None
    venv = hashlib.sha256(os.path.realpath(sys.prefix).encode()).hexdigest()[:12]
    out = os.path.join(tmp, f"{os.path.basename(TEXTURE_DIR)}.{venv}")
    if REWRITTEN_COMPONENTS & set(out.split(os.sep)):
        log.warning("robosuite textures stay in %s: %s has a component robosuite or LIBERO "
                    "rewrites", TEXTURE_DIR, out)
        return None
    return out


def redirect_textures(directory: str | os.PathLike) -> bool:
    """Save robosuite's generated textures in `directory` and name them there in its MJCF."""
    directory = os.path.abspath(directory)
    if not atomic_saves(directory):
        return False
    when_imported(TEXTURE_MODULE, functools.partial(_redirect_materials, directory))
    return True


def _redirect_materials(directory: str, module) -> None:
    cls = getattr(module, "CustomMaterial", None)
    if cls is None or getattr(cls.__init__, "_amp_textures", None) == directory:
        return
    init = getattr(cls.__init__, "_amp_init", cls.__init__)

    @functools.wraps(init)
    def __init__(self, *args, **kwargs):
        prev, _texture.to = getattr(_texture, "to", None), directory
        try:
            init(self, *args, **kwargs)
        finally:
            _texture.to = prev
        attrib = getattr(self, "tex_attrib", None)
        f = attrib.get("file") if isinstance(attrib, dict) else None
        if isinstance(f, str) and os.path.dirname(f) == TEXTURE_DIR:
            attrib["file"] = os.path.join(directory, os.path.basename(f))

    __init__._amp_textures, __init__._amp_init = directory, init  # type: ignore[attr-defined]
    cls.__init__ = __init__


class _Then(importlib.abc.Loader):
    """A loader that runs `then(module)` once `inner` has executed the module."""

    def __init__(self, inner, then):
        self._inner, self._then = inner, then

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        self._then(module)

    def __getattr__(self, name):
        return getattr(self._inner, name)


class _PostImport(importlib.abc.MetaPathFinder):
    """Finds `name` with the other finders and has `then` run on it once it is executed."""

    def __init__(self, name, then):
        self.name, self.then = name, then

    def find_spec(self, fullname, path, target=None):
        if fullname != self.name:
            return None
        for finder in list(sys.meta_path):
            find = getattr(finder, "find_spec", None)
            if finder is self or find is None:
                continue
            spec = find(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        if spec.loader is None or not hasattr(spec.loader, "exec_module"):
            return spec
        if self in sys.meta_path:
            sys.meta_path.remove(self)
        spec.loader = _Then(spec.loader, self.then)
        return spec


def when_imported(name: str, then) -> None:
    """then(module) on module `name`: now if it is imported, else as soon as it is."""
    module = sys.modules.get(name)
    if module is not None:
        then(module)
        return
    sys.meta_path.insert(0, _PostImport(name, then))


def install() -> None:
    """The process's fresh dirs and /tmp guards, once, before an adapter imports robosuite."""
    global _installed
    if _installed:
        return
    _installed = True
    fresh_sandbox()
    if prepare_isaac_slot():
        wipe_at_exit()


    quiet_file_log(logging.getLogger(ROBOSUITE_LOGGER))
    atomic_saves(TEXTURE_DIR)
    textures = texture_dir()
    if textures:
        redirect_textures(textures)


def toolchain_vars(tc: str | None) -> dict[str, str]:
    """CXX and CC from the toolchain directory; empty when AMP_B1K_TOOLCHAIN is unset."""
    if not tc:
        return {}
    return {"CXX": str(Path(tc) / "bin" / f"{TOOLCHAIN_TRIPLE}-g++"),
            "CC": str(Path(tc) / "bin" / f"{TOOLCHAIN_TRIPLE}-gcc")}
