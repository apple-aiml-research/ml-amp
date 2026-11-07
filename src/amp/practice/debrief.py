#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The debrief: after practice episodes, the same agent, with one episode's full context, gets the
official results and may edit its workspace (docs/practice.md)."""
from __future__ import annotations

import copy
import errno
import hashlib
import json
import os
import re
import shutil
import stat
import sys
import time
from pathlib import Path

from amp.agents import tools as T
from amp.agents.context import ContextWindow
from amp.agents.llm import LLMError, Observer, converter
from amp.agents.recording import RequestLog, blobbed
from amp.fs.safe_files import rel_parts
from amp.practice import replay
from amp.practice import workspace as W
from amp.practice.replay import unblobbed

NUDGES = 3
DONE = "done"
NOT_RUN = "(not run: the episode's conversation ended at this call)"
NOT_HERE = "Not available in the debrief: the episode is over."
SCRATCH = ("tmp", "images", "kernel", "kernel_cwd")

PYTHON_TOOL = {
    "name": "python",
    "description": ("Run Python code in the debrief's own persistent Python (no robot: `ep` is not "
                    "connected). The working directory holds workspace/ (yours, writable), "
                    "history/ (your past practice episodes, read-only), tmp/ and images/ "
                    "(scratch). No network and no subprocesses. Print what you need; show(img) "
                    "displays a PIL image or numpy array."),
    "input_schema": {"type": "object", "properties": {
        "code": {"type": "string", "description": "Python source to run."},
        "timeout_s": {"type": "number", "description": "Seconds before the call is interrupted "
                      "(default 120, at most 600)."}}, "required": ["code"]}}
VIEW_TOOL = {
    "name": "view_image",
    "description": ("Look at an image file under the debrief's working directory, e.g. a frame in "
                    "history/<episode>/frames/ or an image in history/<episode>/images/."),
    "input_schema": {"type": "object", "properties": {
        "path": {"type": "string", "description": "Path relative to the working directory."}},
        "required": ["path"]}}
DECISIONS = ("keep", "change", "stop")
FORMATS = {
    "code": ("Your workspace may hold only code: Python files (.py); comments and docstrings in "
             "the code are fine, but no notes or other files."),
    "notes": "Your workspace may hold only notes: no Python files (.py) and no code meant to be run."}
DONE_TOOL_V3 = {
    "name": DONE,
    "description": ("End the debrief, save ./workspace as it stands and record your decision. "
                    "decision: keep (the workspace stays as it is and practice continues with it), "
                    "change (you edited the workspace and practice continues with the new "
                    "version) or stop (practice of this task ends and the workspace as it stands "
                    "now is frozen and evaluated). summary: one or two sentences on what you "
                    "changed and why (kept in the debrief's log; later episodes see only the "
                    "workspace)."),
    "input_schema": {"type": "object", "properties": {
        "summary": {"type": "string"},
        "decision": {"type": "string", "enum": list(DECISIONS)}},
        "required": ["summary", "decision"]}}

GUARD = r'''
def _debrief_guard(root, writable, readable, private):
    import os
    import pwd
    import sys

    real = os.path.realpath
    root = real(root)
    wr = [real(os.path.join(root, d)) for d in writable]
    rd = [root] + [real(p) for p in readable if p]
    home = real(pwd.getpwuid(os.getuid()).pw_dir)
    hidden = [home] + [real(p) for p in private if p]

    def inside(p, roots):
        return any(p == r or p.startswith(r + os.sep) for r in roots)

    def target(path):
        if isinstance(path, int):
            return None
        p = os.fsdecode(path)
        return real(p if os.path.isabs(p) else os.path.join(os.getcwd(), p))

    write_flags = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC
    mutate = {"os.remove", "os.rename", "os.rmdir", "os.mkdir", "os.chmod", "os.symlink",
              "os.link", "shutil.rmtree", "shutil.move", "shutil.copyfile", "os.truncate",
              "os.utime", "os.chown", "os.chflags"}
    banned = ("subprocess.Popen", "os.system", "os.exec", "os.posix_spawn", "os.spawn",
              "os.fork", "os.forkpty", "pty.spawn", "socket.connect", "socket.bind",
              "socket.getaddrinfo", "os.kill", "os.killpg", "ctypes.dlopen")

    def hook(event, args):
        if event == "open":
            p = target(args[0])
            if p is None:
                return
            mode, flags = args[1] or "", args[2] or 0
            writing = any(c in mode for c in "wax+") or bool(flags & write_flags)
            if writing and not inside(p, wr):
                raise PermissionError(f"debrief: {args[0]!s} is not writable here: only "
                                      "workspace/, tmp/ and images/ are")
            if not writing and inside(p, hidden) and not inside(p, rd):
                raise PermissionError(f"debrief: {args[0]!s} is outside the debrief")
        elif event in mutate:
            for a in args[:2]:
                p = target(a) if isinstance(a, (str, bytes, os.PathLike)) else None
                if p is not None and not inside(p, wr):
                    raise PermissionError(f"debrief: {event} on {a!s} is not allowed: only "
                                          "workspace/, tmp/ and images/ can change")
        elif event.startswith(banned):
            raise PermissionError(f"debrief: {event} is not allowed (no processes, no network)")

    sys.addaudithook(hook)


class _NoRobot:
    def __getattr__(self, name):
        raise RuntimeError("there is no robot in the debrief: the episode is over")


_debrief_guard(ROOT, WRITABLE, READABLE, PRIVATE)
del _debrief_guard
ep = _NoRobot()
'''


def bootstrap(work: Path, private=(), readable=()) -> str:
    """GUARD bound to this debrief's directory, the kernel Python's install and the dirs it must
    not read (the study's raw episodes, wherever they are); readable: dirs it may read anyway
    (the history view its history/ links to)."""
    import site
    import sysconfig

    readable = sorted({sys.prefix, sys.base_prefix, sys.exec_prefix,
                       *sysconfig.get_paths().values(), *site.getsitepackages(),
                       *(str(Path(r).resolve()) for r in readable)})
    head = (f"ROOT = {str(work.resolve())!r}\nWRITABLE = {('workspace',) + SCRATCH!r}\n"
            f"READABLE = {readable!r}\nPRIVATE = {[str(Path(p).resolve()) for p in private]!r}\n")
    return head + GUARD


class Sandbox:
    """amp.kernel.Kernel on the debrief directory, unsandboxed by the OS (its bwrap view would
    hide workspace/ and history/) and guarded by GUARD; no gateway is reached."""

    def __init__(self, work: Path, python: Path | None = None, kernel_cls=None, tracer=None,
                 private=(), readable=()):
        from amp.kernel import Kernel

        self.work = Path(work)
        (self.work / "sdk").mkdir(exist_ok=True)
        (self.work / "episode.json").write_text(json.dumps({"gateway": "http://127.0.0.1:9"}))
        self.kernel = (kernel_cls or Kernel)(
            self.work, python=Path(python or sys.executable), sdk_dir=self.work / "sdk",
            gateway_port=9, sandbox="none", bootstrap=bootstrap(self.work, private, readable),
            tracer=tracer)

    def start(self):
        self.kernel.start()

    def run(self, code: str, timeout_s: float):
        return self.kernel.run(code, timeout_s=timeout_s)

    def close(self):
        self.kernel.close()


def rebuild(trace) -> tuple[list, list, list, dict]:
    """(system, messages, tools, info) of the episode's last request, every body sha-verified,
    plus its last reply and that reply's tool results, as the next request would hold them."""
    view = trace if isinstance(trace, replay.TraceView) else replay.TraceView(trace)
    reqs = replay.requests(view)
    if not reqs:
        raise ValueError("the trace holds no model request")
    last = reqs[-1]
    ctx = copy.deepcopy(last["body"].context)
    system, messages, tools = ctx["system"], ctx["messages"], ctx["tools"]
    turn = last["turn"]
    reply = [e for e in view.kinds("agent.llm.response") if e.data.get("turn") == turn]
    info = {"turn": turn, "requests": len(reqs), "body_sha256": last["body_sha256"],
            "reply": bool(reply)}
    if not reply:
        return system, messages, tools, info
    content = copy.deepcopy(reply[-1].data.get("content") or [])
    messages.append({"role": "assistant", "content": content})
    results = {e.data.get("id"): e.data for e in view.kinds("agent.tool.result")
               if e.data.get("turn") == turn}
    blocks = []
    for tu in (b for b in content if b.get("type") == "tool_use"):
        r = results.get(tu.get("id"))
        if r is None:
            blocks.append({"type": "tool_result", "tool_use_id": tu.get("id"),
                           "content": [{"type": "text", "text": NOT_RUN}], "is_error": False})
            continue
        body = unblobbed(r.get("content") or [], lambda b: view.blob(b["ref"]))
        blocks.append({"type": "tool_result", "tool_use_id": tu.get("id"), "content": body,
                       "is_error": bool(r.get("is_error"))
                       and not any(x.get("type") == "image" for x in body)})
    info["tool_results"] = len(blocks)
    if blocks:
        messages.append({"role": "user", "content": blocks})
    return system, messages, tools, info


def results_message(results: list[dict], conversation: str, workspace_sha: str | None,
                    facts: str | None = None, max_bytes: int | None = None,
                    version_label: str | None = None, undelivered=(),
                    workspace_format: str | None = None) -> str:
    """The one user message of the debrief (notebook v3): the official results, then facts (the
    per-version table, the budget used, the decision), then where things are."""
    n = len(results)
    ver = f"workspace version {workspace_sha[:12]}" if workspace_sha else "an empty workspace"
    if version_label:
        ver = f"workspace version {version_label}"
    off = [r["episode"] for r in results if r["episode"] in set(undelivered)]
    if n == 1:
        lines = [f"Official benchmark result of this episode: {results[0]['result']}"
                 + (f" (its agent did not get {ver})" if off else "")]
    else:
        who = "its agent" if len(off) == 1 else "their agents"
        but = f"; {', '.join(off)} did not start from it: {who} did not get that workspace" if off else ""
        lines = [(f"Official benchmark results of the {n} practice episodes that ran in parallel "
                  f"from {ver} (this conversation is one of them{but}):")]
        for r in results:
            where = " (this conversation)" if r["episode"] == conversation else \
                f" (transcript, frames and report in history/{r['episode']}/)"
            scene = f", layout {r['layout']}" if r.get("layout") not in (None, "") else ""
            miss = " (not run from this version)" if r["episode"] in off else ""
            lines.append(f"- {r['episode']}{scene}{where}{miss}: {r['result']}")
    scenes = ("Every episode, practice or evaluation, is a new layout of this task: objects and "
              "their positions change.")
    if facts:
        lines += ["", facts]
    fits = f"About {max_bytes // 1024} KB fits" if max_bytes else "About 60 KB fits"
    versions = (" history/versions/ holds every workspace version so far, read-only: one folder "
                "per version, v<n>_<sha12>/ with its files (v0_no_notes/ is the empty workspace, "
                "no notes), and history/versions/index.md, which lists each version's parent and "
                "the rounds and episodes that ran from it.")
    done = ("done(summary, decision) ends the debrief, saves the workspace and records your "
            "decision: keep, change or stop. ")
    lines += ["", (
        "This is a debrief, outside any episode: nothing you do now moves the robot or changes "
        "these results. Your workspace for this task is ./workspace and it is yours: notes, "
        "code, data, in any form and organisation."
        + (f" {FORMATS[workspace_format]}" if workspace_format else "")
        + " Every later practice episode of this task, "
        "and the final evaluation, starts with the workspace exactly as you leave it, as files "
        f"in ./workspace, with one line in its first message pointing to it. {fits}; "
        "a file you delete is gone from later episodes."), "", (
        "./history holds every practice episode of this task so far, read-only: "
        "history/outcomes.csv (one row per episode: the workspace version it started from and "
        "its official result) and, per episode, history/<episode>/transcript.md (every turn's "
        "text, code, output and errors), frames/ (camera frames by step), images/ (images you "
        "were shown), report.json (your report) and outcome.json." + versions), "", (
        "Tools now: python runs code in a fresh Python with no robot, in a directory holding "
        "workspace/ (writable), history/ (read-only), tmp/ and images/ (scratch); view_image "
        "shows an image file under it; " + done + scenes)]
    return "\n".join(lines)


def task_names(names) -> frozenset[str]:
    """The names done() refuses: each given task or config name and, for a class name, its form
    without "Task" (PickDrillTask, PickDrill)."""
    out = set()
    for n in names:
        n = str(n or "").strip()
        if n:
            out.add(n)
            if n.endswith("Task") and len(n) > 4:
                out.add(n[:-4])
    return frozenset(out)


def task_name_problems(files: dict[str, bytes], names) -> list[str]:
    """Every file path or text that names a task: the whole name, not inside a longer word of
    letters and digits."""
    names = task_names(names)
    if not names:
        return []
    pat = re.compile(r"(?<![A-Za-z0-9])(" + "|".join(
        re.escape(n) for n in sorted(names, key=len, reverse=True)) + r")(?![A-Za-z0-9])")
    probs = []
    for rel, data in sorted(files.items()):
        text = data.decode("utf-8", "replace")
        found = sorted({m.group(1) for m in pat.finditer(rel)} | {m.group(1) for m in pat.finditer(text)})
        if found:
            probs.append(f"{rel}: task name {', '.join(found)}")
    return probs


def format_problems(files: dict[str, bytes], workspace_format: str | None) -> list[str]:
    """Every file workspace_format does not allow: code holds only .py files, notes none."""
    if workspace_format == "code":
        return [f"{rel}: not a Python file (.py): the workspace may hold only code"
                for rel in sorted(files) if not rel.endswith(".py")]
    if workspace_format == "notes":
        return [f"{rel}: a Python file (.py): the workspace may hold only notes"
                for rel in sorted(files) if rel.endswith(".py")]
    return []


def gate(work: Path, notebook: int = 1,
         names=(), max_bytes: int | None = None,
         workspace_format: str | None = None, extra=None
         ) -> tuple[dict[str, bytes] | None, list[str]]:
    """(the workspace's files, problems): links, the size limit (of the payload as submitted,
    notebook mark and all; max_bytes, default W.MAX_BYTES), task names, the study's
    workspace_format, and extra(files)'s problems when given."""
    try:
        files = W.read_dir(work / W.DIR)
    except W.WorkspaceError as e:
        return None, [str(e)]
    probs = []
    limit = W.MAX_BYTES if max_bytes is None else max_bytes
    n = W.size(W.with_notebook(W.encode(files), min(notebook, 2)))
    if n > limit:
        probs.append(f"the workspace is {n:,} bytes as an episode carries it, over "
                     f"{limit:,}: make it smaller")
    probs += task_name_problems(files, names)
    probs += format_problems(files, workspace_format)
    if extra is not None:
        probs += extra(files)  # nosec B610
    return files, probs


def gate_text(probs: list[str]) -> str:
    why = ""
    if any(": task name " in p for p in probs):
        why += (" The workspace may not name a benchmark task (a task class or config name such "
                "as those in history/ folder names): later episodes, the evaluation among them, "
                "are not told their task. Describe it in your own words and rename such files.")
    return ("The workspace was not saved:\n- " + "\n- ".join(probs) + "\n" + why.strip()
            + "\nFix it and call done again.").replace("\n\n", "\n")


def tools_for(original: list[dict], done: dict | None = None) -> list[dict]:
    """The debrief's tools, plus the episode's other tools (refused) so its history stays valid;
    done: the caller's done tool in place of the notebook's."""
    mine = [PYTHON_TOOL, VIEW_TOOL, done or DONE_TOOL_V3]
    names = {t["name"] for t in mine}
    rest = [{**t, "description": NOT_HERE} for t in original if t.get("name") not in names]
    return mine + rest


class _Recorder(Observer):
    """agent.llm.request events in the loop's format, so the debrief replays like an episode."""

    def __init__(self, deb):
        self.deb = deb

    def request(self, body, payload, attempt):
        self.deb.record_request(body, payload, attempt)

    def retry(self, err):
        self.deb.cost += err.get("cost_usd") or 0.0
        self.deb.tracer.event("agent.llm.retry", dict(err), lvl="warn")


class Debrief:
    """One debrief run in WORK: WORK/workspace (the starting version, edited in place),
    WORK/history (read-only copy) and WORK/trace (its own agent trace)."""

    def __init__(self, work: Path, llm, *, system, messages, tools, tracer,
                 sandbox: Sandbox, max_turns: int = 60, max_cost_usd: float = 20.0,
                 image_policy: str = T.DEFAULT_IMAGE_POLICY, notebook: int = 1, names=(),
                 max_bytes: int | None = None, workspace_format: str | None = None,
                 extra_gate=None):
        self.work, self.llm, self.tracer = Path(work), llm, tracer
        self.extra_gate = extra_gate
        self.workspace_format = workspace_format
        self.system, self.messages, self.tools = system, messages, tools
        self.sandbox, self.names = sandbox, tuple(names)
        self.notebook, self.max_bytes, self.decision = notebook, max_bytes, None
        self.max_turns, self.max_cost_usd, self.image_policy = max_turns, max_cost_usd, image_policy
        self.turn, self.cost, self.summary = 0, 0.0, None
        self.provider = getattr(llm, "provider", "scripted")
        self.requests = RequestLog()
        self.ctx = ContextWindow(getattr(llm, "context_tokens", None),
                                 getattr(llm, "estimate_image_tokens", None))
        self.files: dict[str, bytes] | None = None
        self.problems: list[str] = []
        self.end_reason = None
        if hasattr(llm, "observer"):
            llm.observer = _Recorder(self)

    def record_request(self, body: dict, payload: bytes, attempt: int) -> None:
        _, keys = converter(self.provider)
        data = {"turn": self.turn + 1, "attempt": attempt, "provider": self.provider,
                "keys": list(body), "params": {k: v for k, v in body.items() if k not in keys}}
        if self.requests.changed("system", self.system):
            ref = self.tracer.blob(json.dumps(self.system).encode(), mime="application/json",
                                   role="system_prompt", verbatim=True)
            data["system"] = {"ref": ref.ref, "size": ref.size}
        if self.requests.changed("tools", self.tools):
            ref = self.tracer.blob(json.dumps(self.tools).encode(), mime="application/json",
                                   role="tools", verbatim=True)
            data["tools"] = {"ref": ref.ref, "size": ref.size}
        base, new = self.requests.delta(self.messages)
        data.update(base=base, messages=blobbed(new, self.tracer), n_messages=len(self.messages),
                    body_sha256=hashlib.sha256(payload).hexdigest(), body_bytes=len(payload))
        self.tracer.event("agent.llm.request", data, verbatim=True)

    def tool_python(self, inp: dict):
        timeout_s = max(1.0, min(float(inp.get("timeout_s") or 120), 600.0))
        try:
            res = self.sandbox.run(str(inp.get("code") or ""), timeout_s)
        except Exception as e:  # noqa: BLE001
            msg = f"the debrief's Python failed: {type(e).__name__}: {e}"
            return [{"type": "text", "text": msg}], True, {"error": msg}
        blocks, is_error = T.python_result(res, timeout_s, self.work, None, None,
                                           policy=self.image_policy)
        return blocks, is_error, {"stdout": res.stdout, "stderr": res.stderr, "error": res.error}

    def tool_view_image(self, inp: dict):
        rel = str(inp.get("path") or "")
        try:
            rel = "/".join(rel_parts(self.work / rel, self.work))
            T.read_image(self.work, rel)
        except (OSError, ValueError) as e:
            msg = f"no image file {rel!r} under the debrief's directory"
            if isinstance(e, OSError) and e.errno == errno.ELOOP:
                msg += f" ({e.strerror})"
            return [{"type": "text", "text": msg}], True, {"error": msg}
        blocks = T.image_blocks(self.work, rel, None, None, self.image_policy)
        return blocks, len(blocks) == 1, {"error": None}

    def tool_done(self, inp: dict):
        decision = inp.get("decision")
        if self.notebook >= 3 and decision not in DECISIONS:
            msg = (f"decision {decision!r} is not one of keep, change, stop: call done again "
                   "with one of them")
            return [{"type": "text", "text": msg}], True, {"error": msg}
        self.files, self.problems = gate(self.work, self.notebook, self.names,
                                         self.max_bytes, self.workspace_format, self.extra_gate)
        if self.problems:
            msg = gate_text(self.problems)
            self.tracer.event("agent.debrief.gate", {"problems": self.problems}, lvl="warn")
            return [{"type": "text", "text": msg}], True, {"error": msg}
        self.summary = str(inp.get("summary") or "")
        if self.notebook >= 3:
            self.decision = decision
        return [{"type": "text", "text": "Workspace saved."}], False, {"error": None}

    def dispatch(self, tu: dict):
        fn = {"python": self.tool_python, "view_image": self.tool_view_image,
              DONE: self.tool_done}.get(tu.get("name"))
        if fn is None:
            return [{"type": "text", "text": NOT_HERE}], True, {"error": NOT_HERE}
        return fn(tu.get("input") or {})

    def run(self) -> str:
        nudges = 0
        while True:
            if self.turn >= self.max_turns:
                self.end_reason = "turn_limit"
                break
            if self.cost >= self.max_cost_usd:
                self.end_reason = "cost_limit"
                break
            self.ctx.fit(self.system, self.tools, self.messages)
            try:
                turn = self.llm.create(system=self.system, messages=self.messages,
                                       tools=self.tools)
            except LLMError as e:
                self.tracer.event("agent.llm.error", {"turn": self.turn + 1, "error": str(e)[:2000]},
                                  lvl="error")
                self.end_reason = "api_error"
                break
            self.turn += 1
            self.cost += turn.cost_usd or 0.0
            content = copy.deepcopy(turn.content)
            self.tracer.event("agent.llm.response", {
                "turn": self.turn, "stop_reason": turn.stop_reason, "content": content,
                "usage": turn.usage, "cost_usd": turn.cost_usd}, verbatim=True)
            self.messages.append({"role": "assistant", "content": content})
            results = []
            for tu in (b for b in content if b.get("type") == "tool_use"):
                self.tracer.event("agent.tool.call", {"turn": self.turn, "id": tu.get("id"),
                                                      "name": tu.get("name"),
                                                      "input": copy.deepcopy(tu.get("input"))},
                                  verbatim=True)
                blocks, is_error, rec = self.dispatch(tu)
                if is_error and any(b.get("type") == "image" for b in blocks):
                    is_error = False
                self.tracer.event("agent.tool.result", {
                    "turn": self.turn, "id": tu.get("id"), "name": tu.get("name"),
                    "is_error": is_error, **{k: rec.get(k) for k in ("stdout", "stderr", "error")},
                    "content": blobbed(blocks, self.tracer)}, verbatim=True)
                results.append({"type": "tool_result", "tool_use_id": tu.get("id"),
                                "content": blocks, "is_error": is_error})
            if self.summary is not None:
                self.end_reason = "done"
                break
            if results:
                nudges = 0
                self.messages.append({"role": "user", "content": results})
                continue
            nudges += 1
            if nudges > NUDGES:
                self.end_reason = "no_done"
                break
            self.messages.append({"role": "user", "content": [
                {"type": "text", "text": "Call done(summary, decision) when you have finished."
                 if self.notebook >= 3 else "Call done(summary) when you have finished."}]})
        if self.summary is None:
            self.files, self.problems = gate(self.work, self.notebook, self.names,
                                             self.max_bytes, self.workspace_format,
                                             self.extra_gate)
        return self.end_reason


def snapshot(root: Path) -> str:
    """One sha256 over every file under root, to prove history/ was not changed."""
    h = hashlib.sha256()
    for p in sorted(Path(root).rglob("*")):
        if p.is_file():
            h.update(f"{p.relative_to(root)}\0".encode() + hashlib.sha256(p.read_bytes()).digest())
    return h.hexdigest()


def read_only(root: Path) -> None:
    for p in [root, *Path(root).rglob("*")]:
        mode = p.stat().st_mode
        p.chmod(mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))


def writable(root: Path) -> None:
    for p in [root, *Path(root).rglob("*")]:
        p.chmod(p.stat().st_mode | stat.S_IWUSR)


def stat_sig(root: Path) -> dict[str, list]:
    """Every entry under root (not following links) with its mode, size, mtime, ctime and inode:
    a change to any file shows here (ctime cannot be set back) without reading it."""
    out = {}
    root = Path(root)
    for d, dirs, files in os.walk(root):
        for n in dirs + files:
            p = os.path.join(d, n)
            s = os.lstat(p)
            out[os.path.relpath(p, root)] = [s.st_mode, s.st_size, s.st_mtime_ns, s.st_ctime_ns,
                                             s.st_ino]
    return out


def _src_sig(p: Path) -> list:
    if p.is_file():
        s = p.stat()
        return [s.st_size, s.st_mtime_ns]
    return sorted([os.path.relpath(q, p), q.stat().st_size, q.stat().st_mtime_ns]
                  for q in p.rglob("*") if q.is_file())


def sync_view(history: Path, view: Path) -> dict[str, list]:
    """view/: a read-only copy of history/ kept between debriefs, each entry copied again only when
    its source changed; rebuilt whole when its recorded signature does not match (a debrief or
    anything else changed it)."""
    history, view = Path(history), Path(view)
    man_p = view.with_name(view.name + ".json")
    try:
        man = json.loads(man_p.read_text())
    except (OSError, ValueError):
        man = {}
    if view.exists() and (not man or man.get("sig") != stat_sig(view)):
        writable(view)
        shutil.rmtree(view)
        man = {}
    entries = man.get("entries", {}) if view.exists() else {}
    view.mkdir(parents=True, exist_ok=True)
    view.chmod(view.stat().st_mode | stat.S_IWUSR)
    src = {p.name: p for p in history.iterdir()} if history.is_dir() else {}
    for name in sorted(set(entries) - set(src)):
        q = view / name
        if q.exists():
            writable(q)
            shutil.rmtree(q) if q.is_dir() else q.unlink()
        del entries[name]
    for name, p in sorted(src.items()):
        sig = _src_sig(p)
        if entries.get(name) == sig and (view / name).exists():
            continue
        q = view / name
        if q.exists():
            writable(q)
            shutil.rmtree(q) if q.is_dir() else q.unlink()
        shutil.copytree(p, q) if p.is_dir() else shutil.copy2(p, q)
        read_only(q)
        entries[name] = sig
    read_only(view)
    sig = stat_sig(view)
    man_p.write_text(json.dumps({"entries": entries, "sig": sig}))
    return sig


def run(work: Path, *, trace: Path, results: list[dict], conversation: str,
        workspace_files: dict[str, bytes], workspace_sha: str | None, history: Path, profile,
        max_turns: int = 60, max_cost_usd: float = 20.0, notebook: int = 1,
        runtime=None, names=(),
        environ=None, kernel_cls=None, kernel_python: Path | None = None,
        private=None, facts: str | None = None, max_bytes: int | None = None,
        version_label: str | None = None, undelivered=(),
        workspace_format: str | None = None, message: str | None = None,
        done_tool: dict | None = None, history_mirror: Path | None = None, extra_gate=None) -> dict:
    """One debrief in a fresh WORK dir; its record: the saved files (None if the gate refused them),
    the summary, the end reason, turns, cost and the history check."""
    from amp.agents.factory import build_llm
    from amp.trace.writer import FileTracer

    work = Path(work)
    if (work / "history").is_symlink():
        (work / "history").unlink()
    if work.exists():
        writable(work)
        shutil.rmtree(work)
    work.mkdir(parents=True)
    W.write_dir(workspace_files, work / W.DIR)
    (work / W.DIR).mkdir(exist_ok=True)
    if history_mirror is None:
        shutil.copytree(history, work / "history")
        read_only(work / "history")
        before = snapshot(work / "history")

        def check():
            return snapshot(work / "history") == before
    else:
        sig = sync_view(history, history_mirror)
        (work / "history").symlink_to(Path(history_mirror).resolve(), target_is_directory=True)

        def check():
            return stat_sig(history_mirror) == sig
    system, messages, original_tools, info = rebuild(trace)
    text = message if message is not None else results_message(
        results, conversation, workspace_sha, facts, max_bytes, version_label, undelivered,
        workspace_format)
    block = {"type": "text", "text": text}
    if messages and messages[-1]["role"] == "user":
        messages[-1]["content"] = list(messages[-1]["content"]) + [block]
    else:
        messages.append({"role": "user", "content": [block]})
    tracer = FileTracer(work / "trace", "agent", episode_id=f"debrief-{conversation}")
    tracer.event("agent.debrief.begin", {
        "conversation": conversation, "rebuilt": info, "workspace_sha": workspace_sha,
        "results": results, "model_id": profile.model.model_id,
        "reasoning": profile.reasoning, "message": text})
    llm = build_llm(profile, runtime, environ=environ)
    if private is None:
        private = ([Path(trace).resolve().parent, Path(history).resolve()]
                   if notebook >= 3 or message is not None else [])
    sandbox = Sandbox(work, kernel_python, kernel_cls, tracer.child("kernel"), private,
                      () if history_mirror is None else (history_mirror,))
    t0 = time.monotonic()
    deb = Debrief(work, llm, system=system, messages=messages,
                  tools=tools_for(original_tools, done_tool), extra_gate=extra_gate,
                  tracer=tracer, sandbox=sandbox, max_turns=max_turns,
                  notebook=notebook,
                  max_cost_usd=max_cost_usd, names=names, max_bytes=max_bytes,
                  workspace_format=workspace_format if notebook >= 3 else None,
                  image_policy=profile.extra.get("image_policy") or T.DEFAULT_IMAGE_POLICY)
    try:
        sandbox.start()
        deb.run()
    finally:
        sandbox.close()
    history_ok = check()
    saved = deb.files is not None and not deb.problems and history_ok
    rec = {"end_reason": deb.end_reason, "summary": deb.summary, "turns": deb.turn,
           "cost_usd": round(deb.cost, 4), "wall_s": round(time.monotonic() - t0, 1),
           "problems": deb.problems, "history_unchanged": history_ok, "saved": saved,
           "sha": W.sha_of(deb.files) if saved else None, "rebuilt": info}
    if notebook >= 3:
        rec["decision"] = deb.decision
    tracer.event("agent.debrief.end", rec)
    tracer.close()
    if history_mirror is not None:
        (work / "history").unlink()
        if not history_ok:
            Path(history_mirror).with_name(Path(history_mirror).name + ".json").unlink(
                missing_ok=True)
    else:
        writable(work / "history")
        if notebook >= 3:
            shutil.rmtree(work / "history")
    rec["files"] = deb.files if saved else None
    return rec
