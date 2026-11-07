#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Inside Kit: the per-slot episode driver's socket, the job a begun episode takes, its save."""


from __future__ import annotations

import json
import os
import socket
import time
from collections.abc import Callable
from dataclasses import dataclass

from amp.link.client import LinkClient, LinkError
from amp.robots.robolab.records import RESULTS_FILE, find_record, load_records

BEGIN_TIMEOUT_S = 900.0
PING_TIMEOUT_S = 10.0
JOB_WAIT_S = 120.0
MAX_LINE = 1 << 20


class BridgeError(RuntimeError):
    """The driver refused or failed, or the robot server handed out the wrong job: an infra fault."""


def request(path: str, message: dict, timeout_s: float) -> dict:
    """One request on a fresh connection to the driver's Unix socket; its JSON reply."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        try:
            sock.connect(path)
            sock.sendall(json.dumps(message).encode() + b"\n")
            buf = b""
            while b"\n" not in buf:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                buf += chunk
                if len(buf) > MAX_LINE:
                    raise BridgeError(f"driver {path}: reply longer than {MAX_LINE} bytes")
        except OSError as exc:
            raise BridgeError(f"driver {path}: {message.get('op')} failed: {exc}") from exc
    line = buf.split(b"\n", 1)[0]
    try:
        reply = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise BridgeError(f"driver {path}: bad reply {line[:200]!r}") from exc
    if not isinstance(reply, dict):
        raise BridgeError(f"driver {path}: reply is not an object: {reply!r}")
    if reply.get("ok") is not True:
        raise BridgeError(f"driver {path} refused {message.get('op')}: "
                          f"{reply.get('error') or reply}")
    return reply


def ping(path: str, timeout_s: float = PING_TIMEOUT_S) -> None:
    request(path, {"op": "ping"}, timeout_s)


def take_job(link: LinkClient, task: str, k: int, *, wait_s: float = JOB_WAIT_S,
             clock: Callable[[], float] = time.monotonic) -> dict:
    """The job /sim/next hands out for (task, k); BridgeError for any other or none."""
    deadline = clock() + wait_s
    while True:
        try:
            job = link.next_job(loaded=task)
        except LinkError as exc:
            raise BridgeError(f"/sim/next for {task} episode {k} failed: {exc}") from exc
        if link.hello_required:
            raise BridgeError("the robot server has no hello on record: it restarted")
        if job is not None:
            break
        if clock() >= deadline:
            raise BridgeError(f"no job for {task} episode {k} after {wait_s:g} s")
    if job.get("quit"):
        raise BridgeError(f"the robot server sent quit instead of {task} episode {k}")
    if job.get("task") != task or job.get("episode") != k:
        raise BridgeError(f"the robot server handed out {job.get('task')} episode "
                          f"{job.get('episode')} (job {job.get('job_id')}), not {task} episode {k}")
    return job


@dataclass
class PendingSave:
    job_id: int | None
    task: str
    episode: int
    task_dir: str

    @property
    def results_file(self) -> str:
        return os.path.join(os.path.dirname(self.task_dir), RESULTS_FILE)


class Session:
    """The one link a run.py process owns, shared by every task's client, and its pending save."""

    def __init__(self, link: LinkClient, driver: str, *, instructions: dict | None = None,
                 begin_timeout_s: float = BEGIN_TIMEOUT_S, job_wait_s: float = JOB_WAIT_S,
                 log: Callable[[str], None] = print):
        self.link = link
        self.driver = driver
        self.instructions = dict(instructions or {})
        self.begin_timeout_s = begin_timeout_s
        self.job_wait_s = job_wait_s
        self.log = log
        self.pending: PendingSave | None = None

    def begin(self, task: str, k: int, instruction: str) -> dict:
        """Save episode k-1, have the driver claim (task, k), and take exactly that job."""
        self.flush_saved()
        reply = request(self.driver, {"op": "begin", "task": task, "episode": int(k),
                                      "instruction": instruction}, self.begin_timeout_s)
        job = take_job(self.link, task, int(k), wait_s=self.job_wait_s)
        self.log(f"{task} episode {k}: claim {reply.get('job_id')}, job {job.get('job_id')}")
        return job

    def ended(self, job_id: int | None, task: str, k: int, task_dir: str) -> None:
        """Episode k ended; its save waits until RoboLab has written its row."""
        self.pending = PendingSave(job_id, task, int(k), task_dir)

    def flush_saved(self) -> dict | None:
        """Send /sim/saved for the ended episode with RoboLab's row; the row, or None if none."""
        p, self.pending = self.pending, None
        if p is None:
            return None
        row = find_record(load_records(p.results_file), p.task, p.episode)
        if row is None:
            raise BridgeError(f"{p.results_file} has no row for {p.task} episode {p.episode}")
        try:
            self.link.report_saved(job_id=p.job_id, task=p.task, episode=p.episode, record=row)
        except LinkError as exc:
            raise BridgeError(f"/sim/saved for {p.task} episode {p.episode} failed: {exc}") from exc
        self.log(f"{p.task} episode {p.episode} saved: success={row.get('success')}")
        return row
