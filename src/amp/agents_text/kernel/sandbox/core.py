#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One episode's sandbox: the backend, the relay and the episode file the sandboxed SDK opens."""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

from amp.contracts.eval import AGENT_CONFIG_FILE, EPISODE_FILE, SANDBOX_BACKENDS
from amp.contracts.trace import KERNEL_STREAM, TRACE_DIR, TRACE_ENV, Tracer
from amp.fs.atomic import write_bytes_atomic
from amp.kernel.process import KERNEL_TAG_ENV
from amp.kernel.sandbox import bwrap
from amp.kernel.sandbox.limits import limit_argv, resolve_limits, rlimits_for
from amp.trace.kernel_writer import STREAM_ENV

from .relay import Relay, episode_view

HIDDEN = (EPISODE_FILE, "meta.json")
KERNEL_DIR = "kernel"
SANDBOX_EPISODE = f"{KERNEL_DIR}/sandbox_episode.json"
RELAY_LOG = f"{KERNEL_DIR}/relay.log"
CWD_BACKING, STREAM_FILE = KERNEL_STREAM.split("/")
HIDDEN_DIRS = (TRACE_DIR,)
READONLY_DIRS = (KERNEL_DIR,)
HOST_FILES = ("agent.log", AGENT_CONFIG_FILE)
ENV_ALLOW = frozenset({
    "PATH", "PWD", "HOME", "LANG", "LC_ALL", "LC_CTYPE", "TZ", "TMPDIR", "TERM",
    "MPLBACKEND", "MPLCONFIGDIR", "PYTHONPATH", "PYTHONDONTWRITEBYTECODE", "PYTHONNOUSERSITE",
    "PYTHONUNBUFFERED", "PYTHONHASHSEED", "OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "OPENCV_FOR_THREADS_NUM",
    TRACE_ENV, STREAM_ENV, KERNEL_TAG_ENV})
SANDBOX_IDS_ENV = "AMP_SANDBOX_UID"
DEFAULT_IDS = (65534, 65534)
OPT_OUT = ("off", "none", "0")
WRITABLE = ("tmp", "images", CWD_BACKING)
VIEW = ("tmp", "images", KERNEL_DIR)
CPU_SHIM = Path(bwrap.__file__).with_name("cpushim.py").read_text()


class SandboxError(RuntimeError):
    """The requested sandbox cannot be built here."""


def sandbox_ids(environ=os.environ) -> tuple[int, int] | None:
    """The host uid:gid to drop to: None unless root on Linux, or when $AMP_SANDBOX_UID=off."""
    if not sys.platform.startswith("linux") or os.geteuid() != 0:
        return None
    v = environ.get(SANDBOX_IDS_ENV, "").strip().lower()
    if v in OPT_OUT:
        return None
    if not v:
        return DEFAULT_IDS
    uid, _, gid = v.partition(":")
    return int(uid), int(gid or uid)


def host_is_root() -> bool:
    """Whether the uid switch is mandatory: root on Linux."""
    return sys.platform.startswith("linux") and os.geteuid() == 0


def kernel_ids(want: tuple[int, int] | None, dropped: bool, why: str
               ) -> tuple[int, int] | None:
    """The ids the kernel runs as; SandboxError when a root host could not switch to want."""
    if want is not None and not dropped:
        raise SandboxError(f"the host is root and bwrap could not run as uid {want[0]} "
                           f"({why or 'no reason given'}); refusing to run the kernel as root. "
                           f"Dev only: ${SANDBOX_IDS_ENV}=off runs it as root and fails the "
                           "self-test")
    return want if dropped else None


APT_ENV_ALLOW = ("PATH", "LANG", "http_proxy", "https_proxy", "no_proxy", "ftp_proxy", "all_proxy",
                 "HTTP_PROXY", "HTTPS_PROXY", "NO_PROXY", "FTP_PROXY", "ALL_PROXY",
                 "SSL_CERT_FILE", "SSL_CERT_DIR", "REQUESTS_CA_BUNDLE", "CURL_CA_BUNDLE")


def ensure_bwrap() -> str:
    """The bwrap path, installing bubblewrap with apt-get when running as root and it is missing."""
    found = bwrap.find_bwrap()
    if found:
        return found
    apt = shutil.which("apt-get")
    if os.geteuid() != 0 or apt is None:
        raise SandboxError("bwrap is not installed and cannot be installed here (needs root and "
                           "apt-get): apt-get install -y bubblewrap")
    env = {**{k: os.environ[k] for k in APT_ENV_ALLOW if k in os.environ},
           "DEBIAN_FRONTEND": "noninteractive"}
    tail = ""
    for cmds in ([[apt, "install", "-y", "-qq", "bubblewrap"]],
                 [[apt, "update", "-qq"], [apt, "install", "-y", "-qq", "bubblewrap"]]):
        for c in cmds:
            r = subprocess.run(c, env=env, capture_output=True, text=True, timeout=600,
                               check=False)
            tail = (r.stderr or r.stdout)[-500:]
            if r.returncode != 0:
                break
        found = bwrap.find_bwrap()
        if found:
            return found
    raise SandboxError(f"apt-get install bubblewrap failed: {tail}")


def filter_env(env: dict) -> dict:
    return {k: str(v) for k, v in env.items() if k in ENV_ALLOW}


def resolve_backend(name: str = "auto") -> str:
    """The concrete backend for name: auto is bwrap."""
    if name not in SANDBOX_BACKENDS:
        raise ValueError(f"sandbox {name!r} not in {SANDBOX_BACKENDS}")
    return "bwrap"


def _placeholder(p: Path, is_dir: bool) -> None:
    """An empty file or dir at p to mount over, replacing whatever of the other kind is there."""
    if p.is_symlink() or (p.exists() and p.is_dir() != is_dir):
        if p.is_dir() and not p.is_symlink():
            shutil.rmtree(p)
        else:
            p.unlink()
    if is_dir:
        p.mkdir(exist_ok=True)
    else:
        p.touch()


class Sandbox:
    """Build with open_sandbox(); wrap() the kernel's argv, interrupt() it, close() at the end."""

    def __init__(self, backend: str, episode_dir: str | Path, *, python: str | Path,
                 sdk_dir: str | Path, hidden: tuple[str, ...] = HIDDEN,
                 relay_log: str | Path | None = None, limits: dict | None = None,
                 protect: tuple[str, ...] = (), tracer: Tracer | None = None,
                 deadline: bool = False, refuse: tuple[str, ...] = (), refuse_reason: str = ""):
        self.backend = resolve_backend(backend)
        self.deadline = deadline
        self.refuse = {"refuse": tuple(refuse), "refuse_reason": refuse_reason} if refuse else {}
        self.episode_dir = Path(os.path.realpath(episode_dir))
        self.python = str(Path(os.path.abspath(python)))
        self.sdk_dir = Path(os.path.realpath(sdk_dir))
        self.hidden = tuple(hidden)
        self.protect = tuple(protect)
        self.relay_log = Path(relay_log) if relay_log else self.episode_dir / RELAY_LOG
        self.relay: Relay | None = None
        self.tracer = tracer
        self.episode_file = str(self.episode_dir / EPISODE_FILE)
        lim = resolve_limits(limits)
        self.rlimits, self.rlimits_skipped = rlimits_for(lim)
        self.cpus = lim.get("cpus")
        self._sock_dir: str | None = None
        self._bwrap: str | None = None
        self._proc_mount = False
        self._masks: dict[str, str] = {}
        self._refused: list[str] = []
        self._summary: dict | None = None
        self._ids: tuple[int, int] | None = None
        self._drop_why = ""
        self._flags: frozenset[str] = frozenset()
        self._site: str | None = None

    @property
    def uid_opted_out(self) -> bool:
        """Whether a root host runs the kernel as root by ${SANDBOX_IDS_ENV}=off."""
        return self._ids is None and host_is_root()

    @property
    def bwrap_flags(self) -> frozenset[str]:
        """The optional bwrap flags in use (bwrap.SIZE_FLAG, bwrap.NO_USERNS)."""
        return self._flags

    @property
    def kernel_uid(self) -> int | None:
        """The host uid sandboxed code runs as when it is dropped to a separate one, else None."""
        return self._ids[0] if self._ids is not None else None

    @property
    def kernel_stream(self) -> str:
        """The episode's KERNEL_STREAM as the command sees it ($AMP_KERNEL_STREAM)."""
        return str(self.episode_dir / STREAM_FILE)

    def start(self) -> Sandbox:
        self._bwrap = bwrap.find_bwrap()
        if self._bwrap is None:
            raise SandboxError("bwrap is not installed (apt-get install bubblewrap)")
        info = json.loads((self.episode_dir / EPISODE_FILE).read_text())
        for d in (KERNEL_DIR, *WRITABLE):
            p = self.episode_dir / d
            if p.is_symlink():
                raise SandboxError(f"{p} is a symlink")
            p.mkdir(exist_ok=True)
        self.relay_log.touch()
        inner_url = self._start_bwrap(info)
        view = self.episode_dir / SANDBOX_EPISODE
        write_bytes_atomic(view, (json.dumps(episode_view(info, self.relay, inner_url), indent=2)
                                  + "\n").encode())
        self.episode_file = str(view)
        self._refused = bwrap.read_only_paths(self.python)[1]
        return self

    def _start_bwrap(self, info: dict) -> str:
        """The relay on a Unix socket, the uid drop and the CPU-share site dir; the inner URL."""
        self._sock_dir = tempfile.mkdtemp(prefix="amp-sb-")
        sock = os.path.join(self._sock_dir, "relay.sock")
        self.relay = Relay(info["gateway"], info["token"], info.get("base"), unix_path=sock,
                           log_path=self.relay_log, tracer=self.tracer, **self.refuse).start()
        want = sandbox_ids()
        dropped, self._proc_mount, self._drop_why, self._flags = bwrap.launch_mode(
            str(self._bwrap), self.python, want)
        self._ids = kernel_ids(want, dropped, self._drop_why)
        if self._ids is not None:
            os.chmod(self._sock_dir, 0o711)  # nosec B103  # nosemgrep: python.lang.security.audit.insecure-file-permissions.insecure-file-permissions
            os.chown(sock, *self._ids)
            os.mkdir(os.path.join(self._sock_dir, "stage"), 0o711)
        if self.cpus is not None:
            self._site = os.path.join(self._sock_dir, "site")
            os.mkdir(self._site, 0o755)
            Path(self._site, "sitecustomize.py").write_text(
                f"_CPUS = {int(self.cpus)}\n" + CPU_SHIM)
            os.chmod(self._sock_dir, 0o711)  # nosec B103  # nosemgrep: python.lang.security.audit.insecure-file-permissions.insecure-file-permissions
        return f"http://127.0.0.1:{bwrap.INNER_PORT}"

    def _readonly(self) -> list[str]:
        """Files the command may read but not change: its episode view, the relay's log and
        `protect`."""
        paths = [self.episode_file, str(self.relay_log),
                 *(str(self.episode_dir / p) for p in self.protect)]
        return [p for p in paths if Path(p).is_relative_to(self.episode_dir)]

    def env(self, env: dict) -> dict:
        """The environment to start the command with: only ENV_ALLOW names survive."""
        out = filter_env(env)
        if self._site is not None:
            out["PYTHONPATH"] = ":".join(p for p in (bwrap.INNER_SITE, out.get("PYTHONPATH")) if p)
        return out

    @property
    def nproc_enforced(self) -> bool:
        """Whether RLIMIT_NPROC binds: never for a root host uid, per sandbox for any other."""
        if "nproc" not in self.rlimits:
            return False
        return self._ids is not None or os.geteuid() != 0

    def read_roots(self) -> list[str]:
        """Host paths the command can read besides the episode (for the self-test's audit)."""
        return sorted({*bwrap.read_only_paths(self.python)[0], *bwrap.SYSTEM_LIBS})

    def _bwrap_masks(self) -> dict[str, str]:
        masks = {p: p for p in self._readonly() if os.path.lexists(p)}
        for d in READONLY_DIRS:
            p = self.episode_dir / d
            if p.is_dir() and not p.is_symlink():
                masks[str(p)] = str(p)
        for name in self.hidden:
            masks[str(self.episode_dir / name)] = (self.episode_file if name == EPISODE_FILE
                                                   else "/dev/null")
        return masks

    def _bwrap_view(self) -> tuple[str, list[str]]:
        """The episode's backing dir and the entries bound into it: all but hidden and links."""
        back = self.episode_dir / CWD_BACKING
        back.mkdir(exist_ok=True)
        skip = {*self.hidden, *HIDDEN_DIRS, *HOST_FILES, CWD_BACKING}
        names = []
        for e in sorted(os.scandir(self.episode_dir), key=lambda e: e.name):
            if (e.name not in VIEW or e.name in skip or e.is_symlink()
                    or not (e.is_dir() or e.is_file())):
                continue
            _placeholder(back / e.name, e.is_dir())
            names.append(e.name)
        for name in self.hidden:
            _placeholder(back / name, False)
        if self.sdk_dir.is_relative_to(self.episode_dir) and self.sdk_dir != self.episode_dir:
            rel = self.sdk_dir.relative_to(self.episode_dir)
            if rel.parts[0] not in names:
                (back / rel).mkdir(parents=True, exist_ok=True)
        return str(back), names

    def wrap(self, cmd: list[str]) -> list[str]:
        """argv that runs cmd inside the sandbox, cwd the episode directory."""
        cmd = [str(c) for c in cmd]
        if self.relay is None:
            raise SandboxError("start() the sandbox before wrap()")
        cmd = limit_argv(self.python, self.rlimits, cmd)
        self._masks = self._bwrap_masks()
        view = self._bwrap_view()
        drop = None
        if self._ids is not None:
            for name in WRITABLE:
                p = self.episode_dir / name
                if p.exists() and not p.is_symlink():
                    os.chown(p, *self._ids)
            drop = {"uid": self._ids[0], "gid": self._ids[1],
                    "stage": os.path.join(str(self._sock_dir), "stage")}
        return bwrap.argv(cmd, bwrap=str(self._bwrap), episode_dir=str(self.episode_dir),
                          sdk_dir=str(self.sdk_dir), python=self.python,
                          relay_sock=self.relay.unix_path, masks=self._masks,
                          proc=self._proc_mount, view=view, drop=drop, site=self._site,
                          flags=self._flags, deadline=self.deadline)

    def interrupt(self, proc: subprocess.Popen, sig: int = signal.SIGINT) -> None:
        """Signal (default SIGINT) the sandboxed command; under bwrap, its group (proc is not it)."""
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def close(self) -> None:
        r, self.relay = self.relay, None
        if r is not None:
            self._summary = r.summary()
            r.close()
        if self._sock_dir is not None:
            shutil.rmtree(self._sock_dir, ignore_errors=True)
            self._sock_dir = None

    def describe(self) -> dict:
        """For EpisodeResult.sandbox: what was isolated and what the relay saw. No secrets."""
        d = {"backend": self.backend, "platform": sys.platform, "hidden": list(self.hidden),
             "episode_file": os.path.relpath(self.episode_file, self.episode_dir)}
        d["hidden_dirs"] = list(HIDDEN_DIRS)
        d["readonly_dirs"] = list(READONLY_DIRS)
        d["rlimits"] = dict(self.rlimits)
        d["rlimits_skipped"] = dict(self.rlimits_skipped)
        d["nproc_enforced"] = self.nproc_enforced
        d["cpu_count"] = self.cpus
        d["host_uid"] = self._ids[0] if self._ids is not None else os.geteuid()
        d["uid_opt_out"] = self.uid_opted_out
        if self.uid_opted_out:
            d["nproc_note"] = (f"root: RLIMIT_NPROC is not enforced (uid drop disabled by "
                               f"${SANDBOX_IDS_ENV})")
        d["bwrap_flags"] = sorted(self._flags)
        d["cwd_backing"] = CWD_BACKING
        d["proc_mounted"] = self._proc_mount
        d["masked"] = sorted(os.path.relpath(p, self.episode_dir) for p in self._masks)
        d["refused_sys_path"] = list(self._refused)
        relay = self.relay.summary() if self.relay is not None else self._summary
        if relay is not None:
            d["relay"] = relay
        return d

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()


def open_sandbox(backend: str, episode_dir: str | Path, *, python: str | Path,
                 sdk_dir: str | Path, **kw) -> Sandbox:
    """A started Sandbox for one episode (backend "auto" resolves per platform)."""
    return Sandbox(backend, episode_dir, python=python, sdk_dir=sdk_dir, **kw).start()
