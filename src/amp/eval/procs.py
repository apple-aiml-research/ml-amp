#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A process manager: one process group per process, log files, liveness, a graceful stop, and
restart with exponential backoff until a crash loop marks the process failed."""

from __future__ import annotations

import contextlib
import logging
import os
import secrets
import shutil
import signal
import stat
import subprocess
import threading
import time
from collections.abc import Callable, Iterator, Mapping
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

from amp.contracts.trace import NullTracer, Tracer
from amp.eval import envs

TOKEN_ENV = "AMP_PROCS_TOKEN"
PROC_ROOT = Path("/proc")
LEFTOVER_GRACE_S = 5.0
MAX_CHECKERS = 32

log = logging.getLogger("amp.eval.procs")


@dataclass(frozen=True)
class RestartPolicy:
    """Backoff doubles from backoff_min_s to backoff_max_s; max_failures short runs in a row mark
    the process failed."""

    backoff_min_s: float = 2.0
    backoff_max_s: float = 120.0
    reset_after_s: float = 300.0
    max_failures: int = 5

    def __post_init__(self):
        if not 0 < self.backoff_min_s <= self.backoff_max_s:
            raise ValueError("need 0 < backoff_min_s <= backoff_max_s")
        if self.max_failures < 1:
            raise ValueError("max_failures must be >= 1")

    def delay(self, failures: int) -> float:
        return min(self.backoff_max_s, self.backoff_min_s * 2 ** max(0, failures - 1))


@dataclass(frozen=True)
class ProcSpec:
    """One supervised command."""

    name: str
    argv: tuple[str, ...]
    log: str
    env: Mapping[str, str] = field(default_factory=dict)
    cwd: str | None = None
    grace_s: float = 10.0
    restart: RestartPolicy = RestartPolicy()
    health: Callable[[], bool] | None = field(default=None, compare=False)
    health_grace_s: float = 30.0
    health_failures: int = 3
    wipe_on_exit: tuple[str, ...] = ()
    wipe_root: str = ""
    pre_start: Callable[[], dict] | None = field(default=None, compare=False, repr=False)
    health_ready: Callable[[float], bool] | None = field(default=None, compare=False, repr=False)
    on_exit: Callable[[str, int | None], None] | None = field(default=None, compare=False,
                                                              repr=False)

    def __post_init__(self):
        if not self.argv:
            raise ValueError(f"{self.name}: empty argv")
        object.__setattr__(self, "argv", tuple(str(a) for a in self.argv))
        object.__setattr__(self, "wipe_on_exit", tuple(str(p) for p in self.wipe_on_exit))
        object.__setattr__(self, "env", {str(k): str(v) for k, v in self.env.items()})


def group_alive(pgid: int) -> bool:
    """True while a member of group pgid runs; a zombie (exited, awaiting reap) does not count."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return _running_members(pgid) != 0


def _running_members(pgid: int) -> int:
    """Non-zombie members of pgid from /proc, or -1 where there is no /proc."""
    proc = Path("/proc")
    if not proc.is_dir():
        return -1
    n = 0
    for d in proc.iterdir():
        if not d.name.isdigit():
            continue
        try:
            fields = (d / "stat").read_text().rsplit(")", 1)[1].split()
        except (OSError, IndexError):
            continue
        if len(fields) > 2 and fields[2] == str(pgid) and fields[0] != "Z":
            n += 1
    return n


def _signal_group(pgid: int, sig: int) -> None:
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(pgid, sig)


def terminate(p: subprocess.Popen, grace_s: float) -> int | None:
    """SIGTERM p's process group, SIGKILL whatever is left after grace_s; returns p's exit code."""
    pgid = p.pid
    if p.poll() is None or group_alive(pgid):
        _signal_group(pgid, signal.SIGTERM)
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        if p.poll() is not None and not group_alive(pgid):
            return p.returncode
        time.sleep(0.05)
    _signal_group(pgid, signal.SIGKILL)
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    deadline = time.monotonic() + 5
    while group_alive(pgid) and time.monotonic() < deadline:
        _signal_group(pgid, signal.SIGKILL)
        time.sleep(0.05)
    return p.returncode


WIPE_MIN_PARTS = 3
_OPEN_DIR = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)


def _open_below(base: Path, parts: tuple[str, ...]) -> list[int]:
    """Open base and then each of parts below it, every step with O_NOFOLLOW (a link anywhere on the
    way, even one swapped in a moment ago, fails the open); the fds, deepest last."""
    fds: list[int] = []
    try:
        fds.append(os.open(base, _OPEN_DIR))
        for part in parts:
            fds.append(os.open(part, _OPEN_DIR, dir_fd=fds[-1]))
    except BaseException:
        for fd in fds:
            os.close(fd)
        raise
    return fds


def _empty_fd(fd: int) -> bool:
    """Delete every child of the directory fd, by name relative to it: a directory (not a link)
    through rmtree's fd-based walk, anything else, links included, by unlink, which never follows."""
    ok = True
    for name in os.listdir(fd):
        try:
            if stat.S_ISDIR(os.lstat(name, dir_fd=fd).st_mode):
                shutil.rmtree(name, dir_fd=fd)
            else:
                os.unlink(name, dir_fd=fd)
        except FileNotFoundError:
            continue
        except OSError:
            ok = False
    return ok and not os.listdir(fd)


def _wipe_one(raw: str, base: Path) -> bool:
    """Empty raw (a dir below base) without following a link; False if refused or not emptied."""
    lexical = raw.rstrip("/")
    if not lexical or not os.path.isabs(lexical):
        return False
    name = os.path.basename(lexical)
    if name not in ("", ".", "..") and os.path.islink(lexical):
        parent = Path(os.path.realpath(os.path.dirname(lexical)))
        if not parent.is_relative_to(base) or len(parent.parts) < WIPE_MIN_PARTS:
            return False
        fds = _open_below(base, parent.relative_to(base).parts)
        try:
            if stat.S_ISLNK(os.lstat(name, dir_fd=fds[-1]).st_mode):
                os.unlink(name, dir_fd=fds[-1])
                os.mkdir(name, dir_fd=fds[-1])
            return True
        except FileNotFoundError:
            return True
        finally:
            for fd in fds:
                os.close(fd)
    real = Path(os.path.realpath(lexical))
    if real == base or not real.is_relative_to(base) or len(real.parts) - 1 < WIPE_MIN_PARTS:
        return False
    parts = real.relative_to(base).parts
    fds = _open_below(base, parts[:-1])
    try:
        try:
            mode = os.lstat(parts[-1], dir_fd=fds[-1]).st_mode
        except FileNotFoundError:
            return True
        if not stat.S_ISDIR(mode):
            os.unlink(parts[-1], dir_fd=fds[-1])
            os.mkdir(parts[-1], dir_fd=fds[-1])
            return True
        fds.append(os.open(parts[-1], _OPEN_DIR, dir_fd=fds[-1]))
        return _empty_fd(fds[-1])
    finally:
        for fd in fds:
            os.close(fd)


def wipe_dirs(paths: tuple[str, ...], root: str = "") -> list[str]:
    """Empty each dir of paths inside root without following a link; the paths refused or not emptied."""
    if not shutil.rmtree.avoids_symlink_attacks:
        log.error("shutil.rmtree is not symlink-attack safe here: wiping nothing")
        return list(paths)
    if not root or not os.path.isabs(root):
        return list(paths)
    base = Path(os.path.realpath(root))
    if len(base.parts) < 2:
        return list(paths)
    failed = []
    for raw in paths:
        try:
            done = _wipe_one(raw, base)
        except OSError:
            done = False
        if not done:
            failed.append(raw)
    return failed


def env_pids(name: str, value: str) -> list[int]:
    """Pids (not ours) whose environment carries name=value exactly."""
    me, out = os.getpid(), []
    raw = b"\0" + f"{name}={value}".encode() + b"\0"
    for d in PROC_ROOT.iterdir():
        if d.name.isdigit() and int(d.name) != me:
            with contextlib.suppress(OSError):
                if raw in b"\0" + (d / "environ").read_bytes() + b"\0":
                    out.append(int(d.name))
    return out


def token_pids(token: str) -> list[int]:
    """Pids (not ours) whose environment carries AMP_PROCS_TOKEN=token."""
    return env_pids(TOKEN_ENV, token)


def kill_token(token: str, grace_s: float = 3.0) -> list[int]:
    """SIGTERM, then SIGKILL, every process carrying the token; reaps any that are our zombies."""
    pids = token_pids(token)
    for sig, wait in ((signal.SIGTERM, grace_s), (signal.SIGKILL, 5.0)):
        for pid in pids:
            with contextlib.suppress(OSError):
                os.kill(pid, sig)
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            for pid in pids:
                with contextlib.suppress(ChildProcessError, OSError):
                    os.waitpid(pid, os.WNOHANG)
            if not set(token_pids(token)) & set(pids):
                break
            time.sleep(0.05)
        pids = [p for p in pids if p in set(token_pids(token))]
        if not pids:
            break
    return pids


class Proc:
    """One supervised process. check() is one supervision step; ProcManager calls it."""

    def __init__(self, spec: ProcSpec, *, tracer: Tracer | None = None,
                 clock: Callable[[], float] = time.monotonic, token: str | None = None):
        self.spec = spec
        self._tracer = tracer or NullTracer("node")
        self._clock = clock
        self._token = token
        self._lock = threading.RLock()
        self._popen: subprocess.Popen | None = None
        self.state = "idle"
        self.closed = False
        self.started_at: float | None = None
        self.starts = 0
        self.failures = 0
        self.last_exit: dict | None = None
        self.next_start: float | None = None
        self.failed_reason: str | None = None
        self._bad_checks = 0
        self._served = False
        self._gen = 0

    @property
    def name(self) -> str:
        return self.spec.name

    @property
    def pid(self) -> int | None:
        p = self._popen
        return p.pid if p is not None and p.poll() is None else None

    def alive(self) -> bool:
        p = self._popen
        return p is not None and p.poll() is None

    def _emit(self, kind: str, data: dict, lvl: str = "info") -> None:
        self._tracer.event(kind, {"proc": self.name, **data}, lvl=lvl)

    def start(self) -> None:
        """Start it now with a clean failure record; a no-op while it runs or once closed."""
        with self._lock:
            if self.alive() or self.closed:
                return
            self.failures, self.failed_reason = 0, None
            gen = self._gen
        self._spawn_checked("start", gen)

    def restart(self, spec: ProcSpec | None = None) -> None:
        """Stop it, take the new spec if given, and start it at once with a clean record."""
        self.stop()
        with self._lock:
            if self.closed:
                return
            if spec is not None:
                self.spec = spec
            self.failures, self.failed_reason = 0, None
            self._emit("node.proc.restart", {"requested": True, "delay_s": 0.0})
            gen = self._gen
        self._spawn_checked("requested restart", gen)

    def close(self, grace_s: float | None = None) -> int | None:
        """Stop it for good: nothing (monitor, restart, start) can spawn it again."""
        with self._lock:
            self.closed = True
        return self.stop(grace_s)

    def stop(self, grace_s: float | None = None) -> int | None:
        """SIGTERM its group, then SIGKILL after the grace period; it is not restarted."""
        with self._lock:
            p, started = self._popen, self.started_at
            self.state, self.next_start, self._popen = "stopped", None, None
            self._gen += 1
        if p is None:
            return None
        spec = self.spec
        rc = terminate(p, spec.grace_s if grace_s is None else grace_s)
        self._wipe(spec)
        with self._lock:
            ran = self._clock() - (started or self._clock())
            self.last_exit = {"rc": rc, "ran_s": round(ran, 3), "why": "stopped"}
        self._emit("node.proc.exit", {"pid": p.pid, "rc": rc, "ran_s": round(ran, 3),
                                      "requested": True})
        return rc

    def check(self) -> None:
        """Notice an exit, restart after the backoff, or restart it when its health check fails."""
        with self._lock:
            now = self._clock()
            due = self._gen if (self.state == "backoff" and self.next_start is not None
                                and now >= self.next_start) else None
        if due is not None:
            self._spawn_checked("restart", due)
            return
        with self._lock:
            now = self._clock()
            p = self._popen
            if self.state != "running" or p is None:
                return
            started, health = self.started_at or now, self.spec.health
            ready, served = self.spec.health_ready, self._served
            exited = p.poll() is not None
            if exited:
                self._popen, self.state = None, "exiting"
        if exited:
            self._finish_exit(p, now - started, "exit")
            return
        if health is None:
            return
        try:
            ok = bool(health())
        except Exception:  # noqa: BLE001
            ok = False
        if not served and ready is not None:
            try:
                served = bool(ready(now - started))
            except Exception:  # noqa: BLE001
                served = False
        with self._lock:
            if self._popen is not p or self.state != "running":
                return
            self._served = self._served or served
            if ok:
                self._bad_checks = 0
                return
            if not self._served and self._clock() - started < self.spec.health_grace_s:
                return
            self._bad_checks += 1
            if self._bad_checks < self.spec.health_failures:
                return
            self._popen, self.state = None, "exiting"
        self._emit("node.proc.unhealthy", {"pid": p.pid, "checks": self._bad_checks}, "warn")
        terminate(p, self.spec.grace_s)
        self._finish_exit(p, self._clock() - started, "unhealthy")

    def _finish_exit(self, p: subprocess.Popen, ran: float, why: str) -> None:
        """Outside the lock: clear the rest of p's group, then back off or fail, unless it was
        stopped meanwhile."""
        if group_alive(p.pid):
            terminate(p, min(self.spec.grace_s, LEFTOVER_GRACE_S))
        self._wipe(self.spec)
        with self._lock:
            lost = self.state == "exiting"
            if not lost:
                self.last_exit = {"rc": p.returncode, "ran_s": round(ran, 3), "why": why}
            else:
                self._failed_exit(p.pid, ran, why, p.returncode)
        if lost and self.spec.on_exit is not None:
            try:
                self.spec.on_exit(why, p.returncode)
            except Exception:
                log.exception("%s: on_exit failed", self.name)

    def _wipe(self, spec: ProcSpec) -> None:
        """Empty spec.wipe_on_exit; called outside the lock once the process group is gone."""
        if not spec.wipe_on_exit:
            return
        failed = wipe_dirs(spec.wipe_on_exit, spec.wipe_root)
        self._emit("node.proc.wipe", {"dirs": list(spec.wipe_on_exit), "failed": failed},
                   "error" if failed else "debug")

    def _spawn_checked(self, why: str, gen: int) -> None:
        """Run pre_start unlocked, then spawn unless a stop, close or another spawn came meanwhile."""
        spec = self.spec
        facts: dict = {}
        if spec.pre_start is not None:
            try:
                facts = dict(spec.pre_start() or {})
            except Exception as e:  # noqa: BLE001
                facts = {"refuse": f"the pre-start check failed: {e!r}"}
        with self._lock:
            if self.closed or self._gen != gen or self._popen is not None:
                return
            if facts.get("refuse"):
                self.state, self.next_start, self.failed_reason = "failed", None, facts["refuse"]
                self._emit("node.proc.refused", facts, "error")
                return
            self._spawn(why, facts)

    def _spawn(self, why: str, facts: dict | None = None) -> None:
        """Start the process; called with the lock held, after the pre-start check passed."""
        if self.closed:
            return
        spec = self.spec
        facts = facts or {}
        env = dict(spec.env)
        if self._token:
            env[TOKEN_ENV] = self._token
        Path(spec.log).parent.mkdir(parents=True, exist_ok=True)
        with open(spec.log, "ab") as out:
            stamp = time.strftime("%Y-%m-%d %H:%M:%S")
            out.write(f"--- {stamp} {spec.name} {why}: {' '.join(spec.argv)}\n".encode())
            out.flush()
            try:
                p = envs.popen(list(spec.argv), env=env, keep=(TOKEN_ENV,), cwd=spec.cwd,
                               stdin=subprocess.DEVNULL, stdout=out,
                               stderr=subprocess.STDOUT, start_new_session=True)
            except OSError as e:
                self.starts += 1
                self.started_at = self._clock()
                self._failed_exit(None, 0.0, "spawn", None, str(e))
                return
        self._popen, self.state = p, "running"
        self.started_at, self.next_start, self._bad_checks = self._clock(), None, 0
        self._served = False
        self.starts += 1
        self._emit("node.proc.start", {"pid": p.pid, "start": self.starts, "why": why, **facts})

    def _failed_exit(self, pid, ran: float, why: str, rc, error: str | None = None) -> None:
        policy = self.spec.restart
        if ran >= policy.reset_after_s:
            self.failures = 0
        self.failures += 1
        self.last_exit = {"rc": rc, "ran_s": round(ran, 3), "why": why}
        if error:
            self.last_exit["error"] = error
        self._emit("node.proc.exit", {"pid": pid, **self.last_exit, "failures": self.failures},
                   "warn")
        slot, _, kind = self.name.rpartition(".")
        log.warning("%s exited: rc %s after %.1fs (%s%s); failure %d of %d, %s",
                    f"slot {slot} {kind}" if slot else self.name, rc, ran, why,
                    f": {error}" if error else "", self.failures, policy.max_failures,
                    "giving up" if self.failures >= policy.max_failures
                    else f"restarting in {policy.delay(self.failures):g}s")
        if self.failures >= policy.max_failures:
            self.state, self.next_start = "failed", None
            self.failed_reason = (f"{self.failures} failures in a row; last: {why} rc {rc}"
                                  + (f" ({error})" if error else ""))
            self._emit("node.proc.failed", {"failures": self.failures,
                                            "reason": self.failed_reason}, "error")
            return
        delay = policy.delay(self.failures)
        self.state, self.next_start = "backoff", self._clock() + delay
        self._emit("node.proc.restart", {"requested": False, "delay_s": delay,
                                         "failures": self.failures})


class ProcManager:
    """Named Procs and their monitor."""

    def __init__(self, *, tracer: Tracer | None = None, poll_s: float = 1.0,
                 clock: Callable[[], float] = time.monotonic, max_checkers: int = MAX_CHECKERS,
                 token: str | None = None):
        self.tracer = tracer or NullTracer("node")
        self.max_checkers = max(1, int(max_checkers))
        self.token = token or secrets.token_hex(12)
        self._pool: ThreadPoolExecutor | None = None
        self.poll_s = float(poll_s)
        self._clock = clock
        self._procs: dict[str, Proc] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def add(self, spec: ProcSpec) -> Proc:
        with self._lock:
            if spec.name in self._procs:
                raise ValueError(f"process {spec.name!r} already exists")
            p = self._procs[spec.name] = Proc(spec, tracer=self.tracer, clock=self._clock,
                                              token=self.token)
            return p

    def get(self, name: str) -> Proc:
        return self._procs[name]

    def __iter__(self) -> Iterator[Proc]:
        with self._lock:
            return iter(list(self._procs.values()))

    def _check(self, p: Proc) -> None:
        try:
            p.check()
        except Exception as e:  # noqa: BLE001
            self.tracer.event("node.proc.error", {"proc": p.name, "error": repr(e)}, lvl="error")

    def check_all(self) -> None:
        """One supervision pass over every process, concurrently."""
        procs = list(self)
        if len(procs) > 1:
            if self._pool is None:
                self._pool = ThreadPoolExecutor(self.max_checkers,
                                                thread_name_prefix="amp-procs-check")
            list(self._pool.map(self._check, procs))
        else:
            for p in procs:
                self._check(p)

    def start_monitor(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="amp-procs-monitor", daemon=True)
        self._thread.start()

    def stop_monitor(self) -> None:
        self._stop.set()
        t, self._thread = self._thread, None
        if t is not None and t is not threading.current_thread():
            t.join(timeout=60)
        pool, self._pool = self._pool, None
        if pool is not None:
            pool.shutdown(wait=False, cancel_futures=True)

    def _loop(self) -> None:
        while not self._stop.wait(self.poll_s):
            self.check_all()

    def close(self) -> list[int]:
        """Close every process for good, then kill anything still carrying the token (escaped
        grandchildren); returns those pids."""
        self.stop_monitor()
        procs = list(self)
        threads = [threading.Thread(target=p.close, daemon=True) for p in procs]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        stray = token_pids(self.token)
        if stray:
            self.tracer.event("node.proc.stray", {"pids": stray}, lvl="warn")
            kill_token(self.token)
        return stray
