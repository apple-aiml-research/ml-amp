#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""A persistent, sandboxed Python kernel per episode, holding a harness's SDK."""

from __future__ import annotations

import json
import os
import secrets
import signal
import threading
import time
import urllib.parse
import weakref
from pathlib import Path

from amp.contracts.eval import EPISODE_FILE
from amp.contracts.trace import KERNEL_STREAM, TRACE_ENV, NullTracer, Tracer
from amp.fs.atomic import write_bytes_atomic
from amp.trace import kernel_writer
from amp.trace.episode import EpisodeTrace, kernel_finalizer
from amp.trace.kernel_writer import STREAM_ENV

from .output import (
    CLIP_CHARS,
    KernelResult,
    clip,
    from_reply,
    mark_restarted,
    read_capture,
)
from .process import EPISODE_TAG_ENV, KERNEL_TAG_ENV, ChildDied, ChildProcess
from .sandbox import Sandbox, SandboxError
from .sandbox.core import CWD_BACKING, KERNEL_DIR, WRITABLE
from .sandbox.limits import resolve_limits

CHILD_SOURCE = Path(__file__).with_name("child.py").read_bytes()
CHILD_FILE = f"{KERNEL_DIR}/child.py"
DEADLINE_SIGNAL = signal.SIGUSR2
TRACE_MODULE = "amp_trace.py"
INTERRUPT_GRACE_S = 10.0
BLAS_THREADS = "4"
DISK_CHECK_S = 0.5
DISK_SLACK = 16 << 20
DISK_SETTLE_S = 2.0
KERNEL_LOG = "kernel.log"
START_SLACK_S = 60.0
TRACE_INLINE = 2000

__all__ = ["CLIP_CHARS", "Kernel", "KernelError", "KernelResult", "clip", "sandbox_backend"]


class KernelError(RuntimeError):
    """The kernel could not start (the startup traceback is in the message), or is not started."""


def sandbox_backend(sandbox: bool | str | None) -> str:
    """True and "auto" mean bwrap; False, None and "none" mean unsandboxed."""
    if sandbox is None or sandbox is False or sandbox == "none":
        return "none"
    if sandbox is True or sandbox == "auto":
        return "bwrap"
    if sandbox == "bwrap":
        return sandbox
    raise ValueError(f"unknown sandbox {sandbox!r}")


def _inline(text: str | None) -> str | None:
    return None if text is None else clip(text, TRACE_INLINE)


class Kernel:
    """One persistent child Python per episode, cwd the episode directory."""

    def __init__(self, episode_dir: Path, *, python: Path, sdk_dir: Path, gateway_port: int,
                 sandbox: bool | str = True, bootstrap: str | None = None,
                 limits: dict | None = None, tracer: Tracer | None = None,
                 deadline: bool = False):
        self.episode_dir = Path(os.path.realpath(episode_dir))
        self.python = Path(os.path.abspath(python))
        self.sdk_dir = Path(os.path.realpath(sdk_dir))
        self.gateway_port = int(gateway_port)
        self.backend = sandbox_backend(sandbox)
        self.sandbox = self.backend != "none"
        self.bootstrap = bootstrap
        self.deadline = deadline
        self.limits = resolve_limits(limits)
        self.disk_limit = self.limits.get("disk")
        self.kernel_dir = self.episode_dir / KERNEL_DIR
        self.startup_s: float | None = None
        self.startup_error: str | None = None
        self.restarts = 0
        self.sandbox_info: dict = {"backend": self.backend}
        self._tr: Tracer = (tracer or NullTracer()).child("kernel")
        self._sb: Sandbox | None = None
        self._child: ChildProcess | None = None
        self._ready_timeout_s = 900.0
        self._seq = 0
        self._lock = threading.Lock()
        self._disk_lock = threading.Lock()
        self._disk_note: str | None = None
        self._disk_ceiling: int | None = None
        self._watch_stop: threading.Event | None = None

    def start(self, ready_timeout_s: float = 900) -> None:
        """Spawn the child and wait until its bootstrap returned; KernelError if it did not."""
        self._ready_timeout_s = float(ready_timeout_s)
        self._check_episode()
        with self._tr.span("kernel.start", {"backend": self.backend, "python": str(self.python),
                                            "ready_timeout_s": self._ready_timeout_s}):
            if self._sb is None:
                self._sb = self._open_sandbox()
            self._set_relay_tracer(self._tr.child("kernel"))
            try:
                with self._lock:
                    msg = self._spawn()
            finally:
                self._set_relay_tracer(self._tr)
            self._tr.event("kernel.ready", {"startup_s": self.startup_s,
                                            "error": _inline(msg.get("error"))},
                           lvl="error" if msg.get("error") else "info")
        if msg.get("error"):
            self.close()
            raise KernelError("kernel startup failed:\n" + msg["error"])
        self._start_watcher()

    def _start_watcher(self) -> None:
        if self.disk_limit is None or self._watch_stop is not None:
            return
        stop, ref = threading.Event(), weakref.ref(self)
        self._watch_stop = stop

        def loop():
            while not stop.wait(DISK_CHECK_S):
                k = ref()
                if k is None:
                    return
                k._disk_check()
                del k

        threading.Thread(target=loop, name="kernel-disk-watch", daemon=True).start()

    def _open_sandbox(self) -> Sandbox:
        try:
            self._install_trace_module()
            return Sandbox(self.backend, self.episode_dir, python=self.python,
                           sdk_dir=self.sdk_dir, limits=self.limits, protect=(CHILD_FILE,),
                           tracer=self._tr, deadline=self.deadline).start()
        except (SandboxError, OSError, ValueError, KeyError) as e:
            raise KernelError(f"the {self.backend} sandbox could not be set up: "
                              f"{type(e).__name__}: {e}") from e

    def _install_trace_module(self) -> None:
        """Copy amp.trace.kernel_writer next to the SDK, where sandboxed code imports it."""
        data = Path(kernel_writer.__file__).read_bytes()
        target = self.sdk_dir / TRACE_MODULE
        if not target.is_file() or target.is_symlink() or target.read_bytes() != data:
            if target.is_symlink():
                target.unlink()
            write_bytes_atomic(target, data)

    def close(self) -> None:
        """Kill the child's whole process group and stop the sandbox's relay."""
        stop, self._watch_stop = self._watch_stop, None
        if stop is not None:
            stop.set()
        self._kill()
        sb, self._sb = self._sb, None
        if sb is not None:
            sb.close()
            self.sandbox_info = sb.describe()
            self._tr.event("kernel.close", {"restarts": self.restarts,
                                            "relay": self.sandbox_info.get("relay")})

    def register_finalizer(self, episode_trace: EpisodeTrace) -> None:
        """Have episode_trace ingest the sandbox's stream (KERNEL_STREAM) as untrusted."""
        if kernel_finalizer not in episode_trace.finalizers:
            episode_trace.register_finalizer(kernel_finalizer)

    @property
    def episode_file(self) -> str | None:
        """The episode file the child's SDK opens (the relay's view when sandboxed)."""
        return None if self._sb is None else self._sb.episode_file

    @property
    def kernel_uid(self) -> int | None:
        """The host uid sandboxed code runs as, when dropped to a separate one."""
        return None if self._sb is None else self._sb.kernel_uid

    @property
    def pid(self) -> int | None:
        return None if self._child is None else self._child.pid

    def describe_sandbox(self) -> dict:
        """What the sandbox isolated and what its relay saw; no secrets."""
        d = self._sb.describe() if self._sb is not None else dict(self.sandbox_info)
        d["disk_limit"] = self.disk_limit
        return d

    def interrupt(self, sig: int = signal.SIGINT) -> None:
        """Signal the child (SIGINT, or the deadline signal for a deadline call); idle, it is ignored."""
        c = self._child
        if c is not None and c.poll() is None:
            if self._sb is not None:
                self._sb.interrupt(c.proc, sig)
            else:
                os.kill(c.pid, sig)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:  # noqa: BLE001, S110
            pass

    def run(self, code: str, timeout_s: float = 600, *,
            deadline_error: str | None = None) -> KernelResult:
        """Run code; with deadline_error, timeout_s is a deadline: past it by this clock is timed_out."""
        with self._lock:
            if self._child is None:
                raise KernelError("the kernel is not started")
            restarts = self.restarts
            begin = self._exec_begin(code, timeout_s)
            if deadline_error is not None:
                begin["deadline_error"] = deadline_error
            with self._tr.span("kernel.exec", begin):
                self._set_relay_tracer(self._tr.child("kernel"))
                try:
                    r = self._run(code, float(timeout_s), deadline_error)
                finally:
                    self._set_relay_tracer(self._tr)
                self._tr.event("kernel.exec.result", self._exec_result(r, restarts),
                               lvl="warn" if r.error else "info",
                               blobs=self._output_blobs(r))
            return r

    def _run(self, code: str, timeout_s: float, deadline_error: str | None = None) -> KernelResult:
        t0 = time.monotonic()
        note, before = "", None
        if self._disk_note is None:
            self._disk_check()
        stopped, self._disk_note = self._disk_note, None
        if stopped is not None:
            return KernelResult(stdout="", stderr="", error=stopped, images=[],
                                duration_s=time.monotonic() - t0)
        if self._child.poll() is not None:
            before = f"the kernel had exited (code {self._child.poll()}); state lost"
            note = self._restart(before)
        self._seq += 1
        n = self._seq
        req = {"op": "run", "id": n, "code": code, "trace": self._tr.context()}
        if deadline_error is not None:
            req["alarm"] = deadline_error.removeprefix("TimeoutError: ")
        past = False
        try:
            self._child.send(req)
            msg = self._reply(n, t0 + timeout_s)
            if msg is not None:
                return mark_restarted(from_reply(msg, time.monotonic() - t0, note=note), before)
            past = deadline_error is not None
            if past:
                msg = self._reply(n, None)
                if msg is not None:
                    return mark_restarted(from_reply(msg, time.monotonic() - t0, note=note),
                                          before)
            self.interrupt(DEADLINE_SIGNAL if past else signal.SIGINT)
            msg = self._reply(n, time.monotonic() + INTERRUPT_GRACE_S)
            if msg is not None:
                r = from_reply(msg, time.monotonic() - t0, timed_out=True, note=note)
                head = deadline_error or (f"TimeoutError: the call ran over timeout_s="
                                          f"{timeout_s:g} and was interrupted")
                if not (r.error or "").startswith(head):
                    r.error = f"{head}\n{r.error or ''}"
                return mark_restarted(r, before)
            else:
                why = (f"{deadline_error}, and the call did not stop within "
                       f"{INTERRUPT_GRACE_S:g} s of it; state lost" if deadline_error else
                       f"the call ran over timeout_s={timeout_s:g} and did not stop within "
                       f"{INTERRUPT_GRACE_S:g} s of an interrupt; state lost")
                timed_out = True
        except ChildDied as e:
            code_ = self._child.wait(5)
            disk, self._disk_note = self._disk_note, None
            why = disk or (f"the kernel died during the call (exit code {code_}"
                           + (f": {e}" if str(e) else "") + "); state lost")
            timed_out = past
            if past:
                why = f"{deadline_error}; {why}"
        out, err = read_capture(self.episode_dir, n, self.kernel_uid)
        note += self._restart(why)
        return KernelResult(stdout=clip(out + note), stderr=clip(err),
                            error=f"KernelRestart: {why}", images=[],
                            duration_s=time.monotonic() - t0, timed_out=timed_out)

    def _reply(self, n: int, deadline: float | None) -> dict | None:
        """Call n's result, by deadline (None: only if already in the pipe); other ids are dropped."""
        while True:
            msg = self._child.recv_now() if deadline is None else self._child.recv(deadline)
            if msg is None or msg.get("id") == n:
                return msg
            self._tr.event("kernel.reply.dropped", {"expected": n, "got": repr(msg.get("id"))[:40]},
                           lvl="warn")

    def _exec_begin(self, code: str, timeout_s: float) -> dict:
        d = {"code": clip(code, TRACE_INLINE), "chars": len(code), "timeout_s": timeout_s,
             "seq": self._seq + 1}
        if len(code) > TRACE_INLINE:
            d["code_blob"] = self._tr.blob(code, role="code").ref
        return d

    def _exec_result(self, r: KernelResult, restarts_before: int) -> dict:
        return {"duration_s": r.duration_s, "timed_out": r.timed_out, "error": _inline(r.error),
                "stdout": _inline(r.stdout), "stderr": _inline(r.stderr),
                "stdout_chars": len(r.stdout), "stderr_chars": len(r.stderr),
                "images": [i.get("path") for i in r.images],
                "restarted": self.restarts > restarts_before}

    def _output_blobs(self, r: KernelResult) -> list:
        return [self._tr.blob(text, role=role)
                for role, text in (("stdout", r.stdout), ("stderr", r.stderr),
                                   ("error", r.error or ""))
                if len(text) > TRACE_INLINE]

    def _set_relay_tracer(self, tracer: Tracer) -> None:
        if self._sb is not None and self._sb.relay is not None:
            self._sb.relay.tracer = tracer

    def disk_usage(self, root: Path | None = None) -> int:
        """Bytes allocated under root, default the episode directory (symlinks not followed)."""
        total, todo = 0, [str(root or self.episode_dir)]
        while todo:
            try:
                it = os.scandir(todo.pop())
            except OSError:
                continue
            with it:
                for e in it:
                    try:
                        st = e.stat(follow_symlinks=False)
                    except OSError:
                        continue
                    total += getattr(st, "st_blocks", 0) * 512 or st.st_size
                    if e.is_dir(follow_symlinks=False):
                        todo.append(e.path)
        return total

    def writable_usage(self) -> int:
        """Bytes in the kernel-writable area: WRITABLE under the episode dir, plus its log."""
        total = 0
        for d in WRITABLE:
            p = self.episode_dir / d
            if p.is_dir() and not p.is_symlink():
                total += self.disk_usage(p)
        try:
            total += os.lstat(self.kernel_dir / KERNEL_LOG).st_size
        except OSError:
            pass
        return total

    def _disk_check(self) -> None:
        """Stop the kernel's process tree once the writable area is past its budget, and leave the
        reason for the next call."""
        if self.disk_limit is None:
            return
        with self._disk_lock:
            c = self._child
            if c is None or self._disk_note is not None:
                return
            size = self.writable_usage()
            if size <= self.disk_limit:
                self._disk_ceiling = None
                return
            ceiling = self._disk_ceiling
            if ceiling is not None and size + DISK_SLACK < ceiling:
                self._disk_ceiling = ceiling = size + DISK_SLACK
            if ceiling is not None and size <= ceiling:
                return
            self._disk_ceiling = size + DISK_SLACK
            self._disk_note = (
                f"DiskLimit: the kernel's files ({', '.join(f'{d}/' for d in WRITABLE)} and "
                f"{KERNEL_DIR}/{KERNEL_LOG}) reached {size / 2**20:.0f} MiB, past the "
                f"{self.disk_limit / 2**20:g} MiB budget, so the kernel and every process it "
                "started were stopped; state lost. Delete files there: until they are back "
                "under the budget the kernel may only shrink them")
            c.stop()
            self._disk_ceiling = self._settled_usage(size) + DISK_SLACK

    def _settled_usage(self, floor: int) -> int:
        """The usage once the stopped tree has stopped writing: writes in flight during the stop
        land after the measurement that triggered it, and must not count against the next call."""
        last, steady = max(floor, self.writable_usage()), 0
        deadline = time.monotonic() + DISK_SETTLE_S
        while steady < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
            now = self.writable_usage()
            steady = steady + 1 if now == last else 0
            last = max(last, now)
        return last

    def _check_episode(self) -> None:
        info = json.loads((self.episode_dir / EPISODE_FILE).read_text())
        u = urllib.parse.urlsplit(info["gateway"])
        if self.sandbox and (u.hostname != "127.0.0.1" or u.port != self.gateway_port):
            raise ValueError(f"{EPISODE_FILE}'s gateway {info['gateway']!r} is not "
                             f"http://127.0.0.1:{self.gateway_port}: the sandbox only lets the "
                             "kernel reach that address (and resolves no host names)")

    def _env(self) -> dict:
        ep = str(self.episode_dir)
        env = {
            "PATH": f"{self.python.parent}:/usr/bin:/bin",
            "HOME": ep,
            "LANG": os.environ.get("LANG") or "en_US.UTF-8",
            "TMPDIR": f"{ep}/tmp",  # nosec B108
            "MPLBACKEND": "Agg",
            "MPLCONFIGDIR": f"{ep}/tmp/matplotlib",  # nosec B108
            "PYTHONPATH": str(self.sdk_dir),
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONNOUSERSITE": "1",
            "OMP_NUM_THREADS": BLAS_THREADS,
            "OPENBLAS_NUM_THREADS": BLAS_THREADS,
            "MKL_NUM_THREADS": BLAS_THREADS,
            "NUMEXPR_NUM_THREADS": BLAS_THREADS,
            "OPENCV_FOR_THREADS_NUM": BLAS_THREADS,
        }
        ctx = self._tr.context()
        if ctx:
            env[TRACE_ENV] = ctx
        env[STREAM_ENV] = self._sb.kernel_stream if self._sb is not None else \
            str(self.episode_dir / KERNEL_STREAM)
        return env

    def _prepare_dirs(self) -> None:
        for d in (self.kernel_dir, self.episode_dir / "tmp", self.episode_dir / "images",
                  self.episode_dir / CWD_BACKING):
            if d.is_symlink():
                raise KernelError(f"{d} is a symlink")
            d.mkdir(parents=True, exist_ok=True)
        write_bytes_atomic(self.episode_dir / CHILD_FILE, CHILD_SOURCE)

    def _spawn(self) -> dict:
        t0 = time.monotonic()
        self._prepare_dirs()
        sb = self._sb
        child = str(self.episode_dir / CHILD_FILE)

        def argv(rfd: int, wfd: int) -> list[str]:
            cmd = [str(self.python), "-u", child, str(rfd), str(wfd)]
            if self.deadline:
                cmd.append("--deadline")
            return sb.wrap(cmd) if sb is not None else cmd

        tag = os.environ.get(EPISODE_TAG_ENV) or secrets.token_hex(16)
        env = {**self._env(), KERNEL_TAG_ENV: tag}
        env = sb.env(env) if sb is not None else env
        self._child = ChildProcess.spawn(argv, cwd=self.episode_dir, env=env,
                                         log_path=self.kernel_dir / KERNEL_LOG)
        wait = self._ready_timeout_s + START_SLACK_S
        try:
            self._child.send({"op": "init", "ready_timeout_s": self._ready_timeout_s,
                              "episode_file": self.episode_file
                              or str(self.episode_dir / EPISODE_FILE),
                              "bootstrap": self.bootstrap, "trace": self._tr.context()})
            msg = self._child.recv(time.monotonic() + wait)
        except ChildDied:
            msg = {"error": f"the kernel exited during startup (code {self._child.poll()}); "
                            f"see {self.kernel_dir / 'kernel.log'}"}
        if msg is None:
            msg = {"error": f"no ready message within {wait:g} s"}
        self.startup_s = time.monotonic() - t0
        self.startup_error = msg.get("error")
        return msg

    def _restart(self, why: str) -> str:
        self._kill()
        self.restarts += 1
        self._tr.event("kernel.restart", {"why": why, "restarts": self.restarts}, lvl="warn")
        msg = self._spawn()
        note = f"\n[kernel restarted: state lost] ({why})\n"
        if msg.get("error"):
            note += "[kernel restart: startup failed; `ep` is None]\n" + msg["error"]
        return note

    def _kill(self) -> None:
        c, self._child = self._child, None
        if c is not None:
            c.kill()
