#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The kernel's child REPL, run by path inside the sandbox: python -u <this file> <rfd> <wfd>.

It cannot import amp (the sandbox exposes no repo): stdlib, numpy and Pillow only. Messages are
length-prefixed JSON: init -> ready, then run -> result until exit or EOF.
"""

import ast
import builtins
import contextlib
import ctypes
import io
import json
import linecache
import os
import re
import signal
import struct
import sys
import time
import traceback
from pathlib import Path

# Bound now: model code can replace these, and the deadline must still be installed.
_set_handler, _get_handler = signal.signal, signal.getsignal
# Not SIGALRM: the model's own timers (signal.alarm, setitimer) stay its own. Kernel has the twin.
DEADLINE_SIGNAL = signal.SIGUSR2
SHOW_LIMIT = 6
# Bounds every reply under the host's frame cap (process.MAX_FRAME); the host keeps 20k chars.
PEEK = 100_000
CAPTION_CHARS = 500
CAPTURE_DIR = "tmp/kernel"
IMAGES_DIR = "images"
TRACE_ENV = "AMP_TRACE"
TRACE_MODULE = "amp_trace"
_HERE = os.path.abspath(__file__)


class Pipe:
    def __init__(self, rfd: int, wfd: int):
        self.rfd, self.wfd = rfd, wfd

    def _readn(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = os.read(self.rfd, n - len(buf))
            if not chunk:
                return b""
            buf += chunk
        return buf

    def recv(self):
        hdr = self._readn(4)
        if not hdr:
            return None
        return json.loads(self._readn(struct.unpack(">I", hdr)[0]))

    def send(self, obj) -> None:
        data = json.dumps(obj).encode()
        data = struct.pack(">I", len(data)) + data
        while data:
            data = data[os.write(self.wfd, data):]


def peek(path: Path) -> dict:
    """The file's text: whole up to 2 * PEEK bytes, else head and tail (the host clips)."""
    size = path.stat().st_size
    with open(path, "rb") as f:
        if size <= 2 * PEEK:
            return {"text": f.read().decode("utf-8", "replace"), "size": size}
        head = f.read(PEEK)
        f.seek(size - PEEK)
        tail = f.read()
    return {"text": head.decode("utf-8", "replace"), "tail": tail.decode("utf-8", "replace"),
            "size": size}


def clip(text, limit: int = 2 * PEEK):
    """Head and tail of text within about limit chars (None stays None)."""
    if text is None or len(text) <= limit:
        return text
    half = limit // 2
    return f"{text[:half]}\n... [{len(text) - 2 * half} chars cut] ...\n{text[-half:]}"


def slug(s: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", str(s).lower()).strip("_")
    return s[:40].strip("_") or "image"


def format_exc(e: BaseException) -> str:
    """The traceback from the model's own code down: this file's frames are dropped."""
    tb = e.__traceback__
    while tb is not None and os.path.abspath(tb.tb_frame.f_code.co_filename) == _HERE:
        tb = tb.tb_next
    if (isinstance(e, SyntaxError) and str(e.filename).startswith("<cell ") and tb is not None
            and not tb.tb_frame.f_code.co_filename.startswith("<cell ")):
        tb = None
    return "".join(traceback.format_exception(type(e), e, tb))


def fflush() -> None:
    """Flush Python's and C's stdio, so fd-level and C-level output lands in the capture."""
    with contextlib.suppress(Exception):
        sys.stdout.flush()
        sys.stderr.flush()
    with contextlib.suppress(Exception):
        ctypes.CDLL(None).fflush(None)


def set_trace_context(ctx: str) -> None:
    """Put this process's events, and its subprocesses', under ctx's span. The stream itself
    comes from $AMP_KERNEL_STREAM, which amp_trace reads."""
    if not ctx:
        return
    os.environ[TRACE_ENV] = ctx
    try:
        mod = __import__(TRACE_MODULE)
    except ImportError:
        return
    mod.configure(span=ctx)


class Repl:
    def __init__(self, pipe: Pipe, episode: Path):
        self.pipe = pipe
        self.episode = episode
        self.images = episode / IMAGES_DIR
        self.capture = episode / CAPTURE_DIR
        self.ns: dict = {}
        self.shown: list = []
        self.next_image = 1
        self.busy = False
        self.alarm = None
        self.alarmed = False
        self.pending = False
        self.finished = True
        self.hold = False  # set in a deadline-mode kernel: ours stays installed between calls
        self.model = signal.SIG_DFL  # the model's handler for the signal while ours is installed

    def on_sigint(self, signum, frame):
        if self.busy:
            raise KeyboardInterrupt

    def on_sigalrm(self, signum, frame):
        """The host's deadline: TimeoutError in the running code, held until a call starts it."""
        if self.alarm is None or self.alarmed:
            return
        if self.busy:
            self.alarmed = True
            raise TimeoutError(self.alarm)
        if not self.finished:
            self.pending = True

    def hold_alarm(self) -> None:
        """Install our deadline handler, keeping the model's: a stray signal never meets the default."""
        if _get_handler(DEADLINE_SIGNAL) != self.on_sigalrm:
            self.model = _set_handler(DEADLINE_SIGNAL, self.on_sigalrm)

    def arm(self, alarm) -> None:
        """Ours stays installed between calls and in a deadline call; the model's in the others."""
        self.alarm, self.alarmed, self.pending, self.finished = alarm, False, False, False
        if alarm is not None:
            self.hold_alarm()
        elif _get_handler(DEADLINE_SIGNAL) == self.on_sigalrm:
            _set_handler(DEADLINE_SIGNAL, self.model)

    def to_pil(self, x):
        import numpy as np
        from PIL import Image

        if isinstance(x, Image.Image):
            img = x
        elif isinstance(x, (str, Path)):
            p = Path(x)
            p = (p if p.is_absolute() else self.episode / p).resolve()
            if self.episode not in p.parents:
                raise ValueError(f"show(): {x} is not inside the episode directory")
            img = Image.open(p)
            img.load()
        elif hasattr(x, "savefig"):
            buf = io.BytesIO()
            x.savefig(buf, format="png", dpi=100, bbox_inches="tight")
            img = Image.open(io.BytesIO(buf.getvalue()))
            img.load()
        else:
            a = np.asarray(x)
            if a.ndim == 3 and a.shape[2] == 1:
                a = a[:, :, 0]
            if a.ndim not in (2, 3) or (a.ndim == 3 and a.shape[2] not in (3, 4)) or 0 in a.shape:
                raise ValueError(f"show(): need an HxW or HxWx3/4 array, got shape {a.shape}")
            if a.dtype == bool:
                a = a.astype(np.uint8) * 255
            elif a.dtype != np.uint8:
                f = np.nan_to_num(a.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
                if np.issubdtype(a.dtype, np.floating):
                    lo, hi = float(f.min()), float(f.max())
                    if lo >= 0.0 and hi <= 1.0:
                        f = f * 255.0
                    elif hi > lo:
                        f = (f - lo) / (hi - lo) * 255.0
                a = np.clip(f, 0, 255).astype(np.uint8)
            img = Image.fromarray(np.ascontiguousarray(a))
        if img.mode not in ("RGB", "RGBA", "L"):
            img = img.convert("RGBA" if "A" in img.getbands() else "RGB")
        return img

    def show(self, x, caption=None):
        """Save x (PIL image, HxW/HxWx3 array, figure or episode path) and attach it to the reply."""
        img = self.to_pil(x)
        stem = Path(x).stem if isinstance(x, (str, Path)) else None
        n = self.next_image
        self.next_image += 1
        buf = io.BytesIO()
        img.save(buf, "PNG")
        ext = "png"
        if buf.tell() > 1_000_000:
            buf = io.BytesIO()
            img.convert("RGB").save(buf, "JPEG", quality=90)
            ext = "jpg"
        self.images.mkdir(exist_ok=True)
        rel = f"{IMAGES_DIR}/{n:04d}_{slug(caption or stem or 'image')}.{ext}"
        (self.episode / rel).write_bytes(buf.getvalue())
        rec = {"path": rel, "caption": clip(None if caption is None else str(caption),
                                            CAPTION_CHARS),
               "w": img.width, "h": img.height}
        if len(self.shown) < SHOW_LIMIT:
            self.shown.append(rec)
        else:
            print(f"[show] limit of {SHOW_LIMIT} images per call reached: saved {rel} but did "
                  "not attach it")

    def first_image_number(self) -> int:
        nums = [int(m.group(1)) for p in (self.images.iterdir() if self.images.is_dir() else ())
                if (m := re.match(r"(\d{4,})_", p.name))]
        return max(nums, default=0) + 1

    def exec_cell(self, code: str, n: int) -> None:
        """Run code as cell n; a trailing expression's repr is printed, as in a REPL."""
        name = f"<cell {n}>"
        linecache.cache[name] = (len(code), None, code.splitlines(True), name)
        tree = ast.parse(code, name, "exec")
        last = None
        if tree.body and isinstance(tree.body[-1], ast.Expr):
            last = ast.Expression(tree.body.pop().value)
        exec(compile(tree, name, "exec"), self.ns)  # noqa: S102 - running the model's code is the job  # nosec B102 - running the model's code is the job
        if last is not None:
            value = eval(compile(last, name, "eval"), self.ns)  # nosec B307 - running the model's last expression is the job
            if value is not None:
                print(repr(value))

    def run(self, req: dict) -> dict:
        n = req["id"]
        self.arm(req.get("alarm"))
        set_trace_context(req.get("trace") or "")
        self.shown.clear()
        out_p, err_p = self.capture / f"out_{n}.txt", self.capture / f"err_{n}.txt"
        fflush()
        saved = os.dup(1), os.dup(2)
        out_f, err_f = open(out_p, "wb"), open(err_p, "wb")  # noqa: SIM115 - dup2 targets
        os.dup2(out_f.fileno(), 1)
        os.dup2(err_f.fileno(), 2)
        error = None
        t0 = time.monotonic()
        try:
            self.busy = True
            try:
                if self.pending:
                    self.alarmed = True
                    raise TimeoutError(self.alarm)
                self.exec_cell(req["code"], n)
            finally:
                self.busy, self.finished = False, True
                if self.hold:
                    self.hold_alarm()
        except SystemExit as e:
            error = f"SystemExit({e.code!r}) ignored: the kernel keeps running\n"
        except BaseException as e:  # noqa: BLE001 - every failure is the model's to read
            error = format_exc(e)
        duration = time.monotonic() - t0
        sys.stdout, sys.stderr = sys.__stdout__, sys.__stderr__
        fflush()
        os.dup2(saved[0], 1)
        os.dup2(saved[1], 2)
        for fd in saved:
            os.close(fd)
        out_f.close()
        err_f.close()
        reply = {"op": "result", "id": n, "stdout": peek(out_p), "stderr": peek(err_p),
                 "error": clip(error), "images": list(self.shown), "exec_s": duration}
        for p in (out_p, err_p):
            p.unlink(missing_ok=True)
        return reply

    def startup(self, init: dict) -> dict:
        """Build the namespace (the contract's BOOTSTRAP_NAMES) and run the bootstrap."""
        t0 = time.monotonic()
        self.capture.mkdir(parents=True, exist_ok=True)
        self.next_image = self.first_image_number()
        ready_s = float(init.get("ready_timeout_s", 900))
        episode_file = init.get("episode_file") or "episode.json"
        bootstrap = init.get("bootstrap")
        self.ns.clear()
        self.ns.update({"__name__": "__main__", "__builtins__": builtins, "show": self.show,
                        "ep": None, "READY_TIMEOUT_S": ready_s, "EPISODE_FILE": episode_file})
        error = None
        try:
            set_trace_context(init.get("trace") or "")
            import numpy as np
            import PIL
            from PIL import Image

            self.ns.update({"np": np, "PIL": PIL, "Image": Image})
            if bootstrap is not None:
                linecache.cache["<bootstrap>"] = (len(bootstrap), None,
                                                  bootstrap.splitlines(True), "<bootstrap>")
                exec(compile(bootstrap, "<bootstrap>", "exec"), self.ns)  # noqa: S102  # nosec B102 - the kernel's own bootstrap source
                if self.ns.get("ep") is None:
                    raise RuntimeError("the robot harness bootstrap did not bind `ep`")
        except BaseException as e:  # noqa: BLE001 - reported as the startup error
            error = format_exc(e)
            print(error, file=sys.stderr, flush=True)
        return {"op": "ready", "error": clip(error), "startup_s": time.monotonic() - t0,
                "pid": os.getpid()}

    def serve(self) -> None:
        init = self.pipe.recv()
        if init is None:
            return
        self.pipe.send(self.startup(init))
        while True:
            req = self.pipe.recv()
            if req is None or req.get("op") == "exit":
                return
            if req.get("op") == "run":
                self.pipe.send(self.run(req))


def main(argv) -> None:
    # Run by path, Python puts this file's dir (the host-owned kernel/) first on sys.path.
    if sys.path and os.path.abspath(sys.path[0] or ".") == os.path.dirname(_HERE):
        del sys.path[0]
    repl = Repl(Pipe(int(argv[1]), int(argv[2])), Path.cwd().resolve())
    signal.signal(signal.SIGINT, repl.on_sigint)
    if "--deadline" in argv[3:]:
        # Only a deadline-mode kernel: any other keeps the model's signal dispositions as is.
        repl.hold = True
        repl.hold_alarm()  # before any request: a deadline signal can outrun the call's arm()
    repl.serve()


if __name__ == "__main__":
    main(sys.argv)
