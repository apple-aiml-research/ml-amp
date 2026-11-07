#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A child process on two pipes carrying length-prefixed JSON, killed as a whole process group."""

from __future__ import annotations

import contextlib
import json
import math
import os
import select
import signal
import struct
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

KILL_WAIT_S = 10.0
PS_ENV = {"PATH": "/usr/bin:/bin:/usr/sbin:/sbin", "LC_ALL": "C"}
MAX_FRAME = 8 << 20
_LOG_FLAGS = os.O_WRONLY | os.O_APPEND | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC
KERNEL_TAG_ENV = "AMP_KERNEL_TAG"
EPISODE_TAG_ENV = "AMP_EPISODE_KERNEL_TAG"


def readable(fd: int, timeout_s: float) -> bool:
    """Whether fd has data or hung up within timeout_s; poll, unlike select, takes fds past 1023."""
    p = select.poll()
    p.register(fd, select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR)
    return bool(p.poll(max(0, math.ceil(timeout_s * 1000))))


class ChildDied(Exception):
    """The child closed its end of a pipe."""


class FrameTooLarge(ChildDied):
    """The child announced a frame over MAX_FRAME; its process tree has been killed."""


def descendants(pid: int) -> list[int]:
    """Every live descendant of pid, by parent links, including ones that left its session."""
    try:
        out = subprocess.run(["ps", "-A", "-o", "pid=,ppid="], capture_output=True, text=True,
                             timeout=10, check=False, env=PS_ENV).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    kids: dict[int, list[int]] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
            kids.setdefault(int(parts[1]), []).append(int(parts[0]))
    found, todo = [], [pid]
    while todo:
        for c in kids.get(todo.pop(), ()):
            if c not in found and c != pid:
                found.append(c)
                todo.append(c)
    return found


def tagged(tag: str) -> list[int]:
    """Every live process whose environment holds KERNEL_TAG_ENV=tag (readable ones only)."""
    want = f"{KERNEL_TAG_ENV}={tag}"
    me = os.getpid()
    if os.path.isdir("/proc/self"):
        found = []
        for d in os.listdir("/proc"):
            if not d.isdigit() or int(d) == me:
                continue
            try:
                env = Path("/proc", d, "environ").read_bytes()
            except OSError:
                continue
            if want.encode() in env.split(b"\0"):
                found.append(int(d))
        return found
    try:
        out = subprocess.run(["ps", "-A", "-E", "-ww", "-o", "pid=,command="],
                             capture_output=True, text=True, timeout=10, check=False,
                             env=PS_ENV).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    found = []
    for line in out.splitlines():
        pid, _, rest = line.strip().partition(" ")
        if pid.isdigit() and int(pid) != me and want in rest.split():
            found.append(int(pid))
    return found


def kill_tagged(tag: str) -> list[int]:
    """SIGKILL every process carrying tag (stopped first); the pids it found."""
    procs = tagged(tag)
    for sig in (signal.SIGSTOP, signal.SIGKILL):
        procs = list(dict.fromkeys([*procs, *tagged(tag)]))
        for p in procs:
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.kill(p, sig)
    return procs


def kill_tree(pid: int, tag: str | None = None) -> None:
    """SIGKILL pid's process group, every descendant and every process carrying tag, stopped
    first so none can fork away."""

    def more() -> list[int]:
        return [*descendants(pid), *(tagged(tag) if tag else ())]

    procs = [pid, *more()]
    for sig in (signal.SIGSTOP, signal.SIGKILL):
        if sig == signal.SIGKILL:
            procs = list(dict.fromkeys([*procs, *more()]))
        with contextlib.suppress(ProcessLookupError, PermissionError):
            os.killpg(pid, sig)
        for p in procs:
            with contextlib.suppress(ProcessLookupError, PermissionError):
                os.kill(p, sig)


def encode(obj) -> bytes:
    data = json.dumps(obj).encode()
    return struct.pack(">I", len(data)) + data


class ChildProcess:
    """spawn() it with an argv builder that gets the child's (read fd, write fd)."""

    def __init__(self, proc: subprocess.Popen, wfd: int, rfd: int, tag: str | None = None):
        self.proc = proc
        self.tag = tag
        self._wfd: int | None = wfd
        self._rfd: int | None = rfd
        self._buf = bytearray()

    @classmethod
    def spawn(cls, argv: Callable[[int, int], list[str]], *, cwd: Path, env: dict,
              log_path: Path) -> ChildProcess:
        """Start argv(rfd, wfd) in its own session, stdout and stderr appended to log_path."""
        r1, w1 = os.pipe()
        r2, w2 = os.pipe()
        try:
            log = os.open(log_path, _LOG_FLAGS, 0o644)
            try:
                proc = subprocess.Popen(argv(r1, w2), cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                        stdout=log, stderr=subprocess.STDOUT, pass_fds=(r1, w2),
                                        start_new_session=True)
            finally:
                os.close(log)
        except BaseException:
            for fd in (w1, r2):
                os.close(fd)
            raise
        finally:
            os.close(r1)
            os.close(w2)
        return cls(proc, w1, r2, env.get(KERNEL_TAG_ENV) or None)

    @property
    def pid(self) -> int:
        return self.proc.pid

    def poll(self) -> int | None:
        return self.proc.poll()

    def wait(self, timeout: float) -> int | None:
        try:
            return self.proc.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def send(self, obj) -> None:
        """Write one message; ChildDied if the pipe is gone."""
        data = encode(obj)
        if self._wfd is None:
            raise ChildDied()
        try:
            while data:
                data = data[os.write(self._wfd, data):]
        except OSError as e:
            raise ChildDied() from e

    def recv_now(self) -> dict | None:
        """The next message if it is already in the pipe, whole; never waits."""
        while self._rfd is not None and readable(self._rfd, 0):
            chunk = os.read(self._rfd, 1 << 20)
            if not chunk:
                break
            self._buf += chunk
        return self.recv(0.0)

    def recv(self, deadline: float) -> dict | None:
        """The next message, None at the deadline; ChildDied when the child closed its pipe."""
        while True:
            if len(self._buf) >= 4:
                n = struct.unpack_from(">I", self._buf)[0]
                if n > MAX_FRAME:
                    self.stop()
                    self._buf = bytearray()
                    raise FrameTooLarge(f"the kernel sent a {n}-byte frame, over the "
                                        f"{MAX_FRAME}-byte limit")
                if len(self._buf) >= 4 + n:
                    msg = json.loads(bytes(self._buf[4:4 + n]))
                    del self._buf[:4 + n]
                    return msg
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            if self._rfd is None:
                raise ChildDied()
            if readable(self._rfd, min(left, 1.0)):
                chunk = os.read(self._rfd, 1 << 20)
                if not chunk:
                    raise ChildDied()
                self._buf += chunk

    def stop(self) -> None:
        """SIGKILL the child's process group and descendants; once reaped, only tagged processes."""
        if self.proc.returncode is None:
            kill_tree(self.proc.pid, self.tag)
        elif self.tag:
            kill_tagged(self.tag)

    def kill(self) -> None:
        """SIGKILL the child's whole process tree, reap it and close the pipes."""
        self.stop()
        try:
            self.proc.kill()
        except ProcessLookupError:
            pass
        self.wait(KILL_WAIT_S)
        for fd in (self._wfd, self._rfd):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._wfd = self._rfd = None
        self._buf = bytearray()
