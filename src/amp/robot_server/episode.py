#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The episode state machine: directed select, the job the backend runs, release, publish."""

from __future__ import annotations

import time

from amp.contracts.server import EPISODE_STATES
from amp.robot_server.errors import SessionConflict
from amp.robot_server.results import RECORD_FORMAT, next_free_episode
from amp.robot_server.task_notes import problem, with_recipe

MODES = ("no_sim", "awaiting_plan", "executing", "episode_over")
PUBLISHING = "the episode is being published"
RELEASED = "the person's session on this worker ended"
LOST = "the sim backend was lost holding this job (a new sim client said hello)"
EVICTED = "the sim client holding this job was evicted by a takeover hello"
UNSETTLED = "the sim backend asked for its next job without saving or failing this one"
FORCED = "official_timeout: discarded for a re-run"
NO_GRADE = 'the sim backend does not grade ("grade" is not in its hello features)'
GRADE_WAIT_S = 8.0
NO_GRADE_FEATURE, NO_EPISODE, PUBLISHING_REASON = "no_grade_feature", "no_episode", "publishing"
ABANDONED, NOT_GRADED, NO_SIM = "abandoned", "not_graded", "no_sim"
SELECT_MODES = ("new", "redo", "resume", "quit")
RUNS_KEPT = 64


def utc_stamp(t: float | None = None) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() if t is None else t))


class JobQueue:
    """One pending job (a new one replaces it) and the job the backend is running."""

    def __init__(self):
        self.next_id = 1
        self.pending: dict | None = None
        self.active: dict | None = None
        self.runs: dict[int, str] = {}

    def submit(self, task=None, episode=None, *, quit=False, run="", instruction_type="default",
               official_attempt=0):
        job = {"job_id": self.next_id}
        self.next_id += 1
        if quit:
            job["quit"] = True
        else:
            job.update(task=task, episode=int(episode), replace=False, redo_of=None, run=run)
            if instruction_type != "default":
                job["instruction_type"] = instruction_type
            if official_attempt:
                job["official_attempt"] = official_attempt
            self.runs[job["job_id"]] = run
            while len(self.runs) > RUNS_KEPT:
                self.runs.pop(next(iter(self.runs)))
        replaced, self.pending = self.pending, job
        return job, replaced

    def take(self) -> dict | None:
        """Hand the pending job to the backend; a quit job never becomes active."""
        job, self.pending = self.pending, None
        if job is not None and not job.get("quit"):
            self.active = job
        return job

    def cancel_pending(self) -> dict | None:
        job, self.pending = self.pending, None
        return job

    def finish(self, job_id=None) -> dict | None:
        """Retire the active job (only if it is job_id, when given)."""
        job = self.active
        if job is None or (job_id is not None and job["job_id"] != job_id):
            return None
        self.active = None
        return job


def wire_job(job: dict | None) -> dict | None:
    """A job as /sim/next sends it."""
    if job is None:
        return None
    if job.get("quit"):
        return {"job_id": job["job_id"], "quit": True}
    out = {k: job[k] for k in ("job_id", "task", "episode", "replace")}
    if job.get("run"):
        out["run"] = job["run"]
    if job.get("instruction_type"):
        out["instruction_type"] = job["instruction_type"]
    if job.get("official_attempt"):
        out["official_attempt"] = job["official_attempt"]
    return out


class DraftStatus:
    """OLD's plan-draft bookkeeping, kept only as api/state's assist.plan.draft (pinned by parity)."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.saved_at = None
        self.rev = self.saved_rev = 0
        self.owned = self.frozen = False

    def changed(self) -> None:
        self.rev += 1

    def save(self, *, empty: bool, force: bool = False, final: bool = False) -> bool:
        if self.frozen and not final:
            return False
        if not (force or final) and self.rev == self.saved_rev:
            return False
        if empty and not self.owned and not force:
            return False
        self.owned = True
        self.saved_rev, self.saved_at = self.rev, utc_stamp()
        return True

    def json(self) -> dict:
        return {"saved_at": self.saved_at, "dirty": self.rev != self.saved_rev}


class Episode:
    """Under the server lock: which episode runs, its lifecycle mode and the job queue."""

    def __init__(self, srv):
        self._srv = srv
        self.mode = "no_sim"
        self.episode = None
        self.task = ""
        self.instruction = ""
        self.max_steps = None
        self.control_dt = None
        self.step = 0
        self.sim_time = 0.0
        self.result = None
        self.last_result = None
        self.jobs = JobQueue()
        self.job_ended = False
        self.loaded = None
        self.ep_job: dict | None = None
        self.abort: str | None = None
        self.last_end: dict | None = None
        self.publishing: dict | None = None
        self.grade_req: dict | None = None
        self.acked: set[int] = set()
        self.token = 0
        self.obstacles: str | None = None
        self.draft = DraftStatus()
        self._notes_logged: set[str] = set()

    def set_mode(self, mode: str) -> None:
        assert mode in MODES, mode
        self.mode = mode

    def state(self) -> str:
        """The contract's EPISODE_STATES name of the current mode."""
        if self.mode == "no_sim":
            busy = self.jobs.active is not None or self.jobs.pending is not None
            return EPISODE_STATES[1] if busy else EPISODE_STATES[0]
        return {"awaiting_plan": "ready", "executing": "running",
                "episode_over": "ended"}[self.mode]

    def running(self) -> bool:
        return self.jobs.active is not None and not self.job_ended

    def running_task(self) -> str | None:
        job = self.jobs.active
        return job["task"] if job is not None else None

    def begin(self, episode, task, instruction, max_steps, control_dt) -> None:
        self.episode, self.task, self.instruction = episode, task, instruction
        self.max_steps = max_steps
        self.control_dt = float(control_dt) if control_dt else None
        self.step, self.sim_time, self.result = 0, 0.0, None
        self.publishing = None
        self.grade_req = None
        self.acked = set()
        self.token += 1
        self.draft.reset()
        self.set_mode("no_sim")

    def update_from(self, obs) -> None:
        self.task = obs.task or self.task
        self.instruction = obs.instruction or self.instruction
        self.max_steps = obs.max_steps if obs.max_steps is not None else self.max_steps
        self.control_dt = float(obs.control_dt)
        self.step = int(obs.step)
        self.sim_time = float(obs.sim_time)

    def steps_left(self) -> int | None:
        return None if self.max_steps is None else max(0, int(self.max_steps) - int(self.step))

    def agent_instruction(self) -> str:
        """The instruction served to the agent: with task_notes, the task's recipe appended."""
        s = self._srv.settings
        if not s.task_notes:
            return self.instruction
        out = with_recipe(self.instruction, self.task, s.harness)
        key, why = "", problem(s.harness)
        if not why and out == self.instruction and self.task:
            key = self.task
            why = f"no overlay of {s.harness!r} has a recipe for task {self.task!r}"
        if why and key not in self._notes_logged:
            self._notes_logged.add(key)
            self._srv.log("error", f"task_notes is on, but {why}: serving the bare instruction")
        return out

    def bind_job(self, job_id, task, have_catalog) -> str | None:
        """Under the lock, at /sim/episode: bind the episode to the active job; a warning."""
        self.ep_job = None
        if not have_catalog:
            return None
        if task:
            self.loaded = task
        job = self.jobs.active
        if job_id is not None and (job is None or job["job_id"] != job_id):
            active = None if job is None else job["job_id"]
            return (f"/sim/episode names job {job_id!r} but the active job is {active}; the "
                    "episode is not bound to a job and no record is written")
        if job is None or self.job_ended or (job_id is None and job["task"] != task):
            return None
        self.ep_job = dict(job)
        if job["task"] != task or job["episode"] != self.episode:
            return (f"job {job['job_id']} is {job['task']} episode {job['episode']} but the sim "
                    f"began {task} episode {self.episode}")
        return None

    def unbind(self) -> None:
        self.ep_job = None

    def ended(self, result, reason):
        """Under the lock, at /sim/end: (the bound job to record, or None; a warning or None)."""
        self.publishing = None
        self.grade_req = None
        self.result = result
        self.last_result = {"episode": self.episode, "task": self.task, "result": result}
        self.set_mode("episode_over")
        job, self.ep_job = self.ep_job, None
        aborted = reason == "aborted"
        was_abort, self.abort = self.abort, None
        active = self.jobs.active
        if active is not None and (job is None or active["job_id"] == job["job_id"]):
            if aborted:
                self.set_last_end(self.jobs.finish(), "aborted")
                self.job_ended = False
                self.loaded = None
            else:
                self.job_ended = True
        warning = None
        if aborted and was_abort is None:
            warning = "the sim aborted the episode on its own"
        if not aborted and was_abort is not None:
            warning = (f"the episode ended ({reason}) before the abandon reached the sim; "
                       "it is kept")
        return (None if aborted else job), warning

    def teleop_record(self, job, rec, policy_label, reason) -> dict:
        """The teleop_<k>.json document of a finished, bound episode."""
        out = {"format": RECORD_FORMAT, "task": job["task"], "episode": job["episode"],
               "job_id": job["job_id"], "policy_label": policy_label,
               "instruction": self.instruction, "saved_at": utc_stamp(),
               "redo_of": job.get("redo_of"), "initial_state": rec["initial_state"],
               "chunks": rec["chunks"], "result": self.result, "end": reason}
        if job.get("run"):
            out["run_id"] = job["run"]
        return out

    def next_requested(self, loaded, failed) -> str | None:
        """Under the lock, at /sim/next: the backend is done with its active job; an error."""
        self.loaded = loaded
        message = self._job_failed(failed) if failed is not None else None
        job = self._retire(UNSETTLED, self.abort)
        if job is not None and message is None:
            message = (f"job {job['job_id']} ({job['task']} episode {job['episode']}) ended "
                       f"{self.last_end['outcome']}: {UNSETTLED}")
        self.publishing = None
        self.abort = None
        return message

    def take_job(self) -> dict | None:
        if self.jobs.pending is None:
            return None
        self.job_ended = False
        return self.jobs.take()

    def _job_failed(self, failed) -> str:
        """The backend reports a crashed job: retire it, never re-serve it."""
        job_id = failed.get("job_id")
        active, pending = self.jobs.active, self.jobs.pending
        if active is not None and active["job_id"] == job_id:
            self.jobs.finish()
            self.job_ended = False
        elif pending is not None and pending["job_id"] == job_id:
            self.jobs.pending = None
        error = str(failed.get("error") or "unknown error")
        self.set_last_end(failed, "failed", error, void=failed.get("void") is True)
        return (f"job {job_id} ({failed.get('task')} episode {failed.get('episode')}) failed in "
                f"the sim: {error}")

    def job_saved(self, job_id, task) -> None:
        """Under the lock, at /sim/saved: the backend saved the record; the job is over."""
        done = self.jobs.finish(job_id)
        if done is not None:
            self.job_ended = False
            self.set_last_end(done, "saved")
        if task == self.task:
            self.draft.frozen = True

    def link_connected(self, why: str | None = None) -> dict | None:
        """A new backend client; why: the last one left a job unsettled. Its closed job, if any."""
        self.grade_req = None
        self.acked = set()
        closed = self._close_lost(why) if why is not None else None
        if self.mode != "episode_over":
            self.set_mode("no_sim")
        return closed

    def _close_lost(self, why: str) -> dict | None:
        """The previous backend is gone holding a job (taken, running or saving): end it now."""
        was_abort = self.abort
        self.publishing, self.abort, self.ep_job = None, None, None
        return self._retire(why, was_abort)

    def _retire(self, why: str, was_abort) -> dict | None:
        """End the active job the backend will never settle; the job, if there was one."""
        job = self.jobs.active
        if job is None:
            return None
        self.jobs.finish()
        self.job_ended = False
        if was_abort is not None:
            self.set_last_end(job, "aborted")
        else:
            self.set_last_end(job, "failed", why, void=True)
        return job

    def plan_changed(self) -> None:
        self.draft.changed()

    def set_last_end(self, job, outcome, error=None, void=False) -> None:
        self.last_end = {"job_id": job.get("job_id"), "task": job.get("task"),
                         "episode": job.get("episode"), "outcome": outcome, "at": utc_stamp()}
        if error is not None:
            self.last_end["error"] = error
        if void:
            self.last_end["void"] = True

    def taken(self, task, running, run="") -> set[int]:
        """The indices of task a new episode of run may not take."""
        taken = self._srv.results.taken(task, run)
        active = self.jobs.active
        if (active is not None and active["task"] == task and active.get("run", "") == run
                and not running):
            taken.add(active["episode"])
        return taken

    def run_of(self, job_id) -> str:
        """The run_id job_id was selected for ("" for an unknown or unscoped job)."""
        return self.jobs.runs.get(job_id, "") if isinstance(job_id, int) else ""

    def need_catalog(self):
        cat = self._srv.results.catalog
        if cat is None:
            raise SessionConflict("no benchmark catalog (the sim backend has not sent /sim/catalog)")
        return cat

    def save_draft(self, *, force=False, final=False) -> bool:
        if not self.task:
            return False
        return self.draft.save(empty=self._srv.draft_empty(), force=force, final=final)

    def select(self, task, mode, abandon, episode, *, obstacles=None, run="",
               instruction_type="default", official_attempt=0) -> tuple[dict, str]:
        """Queue a job for the backend's next /sim/next: (job, message)."""
        srv = self._srv
        with srv.cond:
            if self.publishing is not None and not abandon:
                raise SessionConflict(PUBLISHING)
            running = self.running() and self.abort is None
            if running and not abandon:
                message = (f"an episode of {self.jobs.active['task']} is running; abandon it "
                           "to switch")
                raise SessionConflict(message, {"ok": False, "needs_abandon": True,
                                                "message": message})
            if episode is not None and episode in self.taken(task, running, run):
                where = f" in run {run}" if run else ""
                message = f"{task} episode {episode} is already saved or running{where}"
                raise SessionConflict(message, {"ok": False, "episode_taken": True,
                                                "message": message})
            if running:
                srv.stop_motion()
                if not self.draft.frozen:
                    self.save_draft(final=True)
                self.draft.frozen = True
            if mode == "quit":
                job, replaced = self.jobs.submit(quit=True)
                message = "the sim will quit after the current job"
            else:
                k = episode if episode is not None else next_free_episode(
                    self.taken(task, running, run))
                job, replaced = self.jobs.submit(task, k, run=run, instruction_type=instruction_type,
                                                 official_attempt=official_attempt)
                message = f"{task} episode {k} queued" + (
                    "" if instruction_type == "default" else f" ({instruction_type} instruction)")
            if replaced is not None:
                message += f" (replaces pending job {replaced['job_id']})"
            if running:
                self.abort = f"the human chose {task or 'to quit'} instead"
                message += "; the running episode is being aborted"
            self.obstacles = obstacles
            srv.cond.notify_all()
        return job, message

    def release(self, *, force: bool = False) -> dict:
        """End the person's session: a moved episode is published (unless force), an unmoved or
        forced one is aborted unscored, and the pending job is cancelled."""
        srv = self._srv
        with srv.cond:
            if self.publishing is not None and not force:
                raise SessionConflict(PUBLISHING)
            active = self.jobs.active
            running = self.running() and self.abort is None
            moved = running and not force and srv.moved()
        if moved:
            return {**self.publish(), "released": None, "published": True}
        with srv.cond:
            saved = False
            if running:
                srv.stop_motion()
                is_bound = (self.ep_job or {}).get("job_id") == active["job_id"]
                if is_bound and not self.draft.frozen:
                    saved = self.save_draft(final=True)
                self.draft.frozen = True
                self.abort = FORCED if force else RELEASED
            pending = self.jobs.pending
            cancelled = None
            if pending is not None and not pending.get("quit"):
                cancelled = self.jobs.cancel_pending()
            if running or cancelled is not None:
                srv.cond.notify_all()
        if not running and cancelled is None:
            return {"ok": True, "released": None}
        what = []
        if running:
            what.append(f"aborting {active['task']} episode {active['episode']}"
                        + (" (draft saved)" if saved else "") + (f" ({FORCED})" if force else ""))
        if cancelled is not None:
            what.append(f"cancelled pending job {cancelled['job_id']} ({cancelled['task']})")
        srv.log("info", f"released: {'; '.join(what)}")
        srv.trace_episode("release", aborted=running, cancelled=cancelled is not None,
                          **({"forced": True} if force else {}))
        out = {"ok": True, "released": {
            "task": active["task"] if running else None,
            "episode": active["episode"] if running else None, "draft_saved": saved,
            "cancelled_pending": None if cancelled is None else cancelled["task"]}}
        if force and running:
            out["discarded"] = True
        return out

    def publish(self, verdict: dict | None = None) -> dict:
        """End the running episode now; the backend scores it as it is (or by verdict's cell)."""
        srv = self._srv
        with srv.cond:
            pub = self.publishing
            if pub is not None:
                return {"ok": True, "message": f"{pub['task']} episode {pub['episode']} is "
                        "already being published"}
            if self.mode not in ("awaiting_plan", "executing"):
                raise SessionConflict(f"no episode is running (mode {self.mode!r})")
            if self.abort is not None:
                raise SessionConflict("the episode is being abandoned")
            if verdict is not None and not srv.link.grades():
                raise SessionConflict(NO_GRADE)
            requested = None
            if verdict is not None and verdict["cell"] not in self.acked:
                requested, verdict = dict(verdict), {"cell": None}
            if requested == verdict:
                requested = None
            stopped = srv.stop_motion(rebase=True)
            task, episode, step = self.task, self.episode, self.step
            self.publishing = {"reason": "published", "task": task, "episode": episode,
                               "step": step}
            if verdict is not None:
                self.publishing["verdict"] = dict(verdict)
            if requested is not None:
                self.publishing["requested"] = requested
            srv.cond.notify_all()
            self.save_draft(force=True)
        srv.log_stopped(stopped)
        message = f"publishing {task} episode {episode} at step {step}"
        srv.log("info", message)
        shown = {} if verdict is None else {"verdict": dict(verdict)}
        if requested is not None:
            shown["requested"] = requested
        srv.trace_episode("publish", task=task, episode=episode, step=step, **shown)
        return {"ok": True, "message": message, **shown}

    def grade(self, cell: int) -> dict:
        """Have the backend grade the current state as code cell `cell`'s, at its next step reply."""
        srv = self._srv

        def refuse(reason: str, message: str) -> SessionConflict:
            return SessionConflict(message, {"ok": False, "cell": cell, "message": message,
                                             "reason": reason})

        with srv.cond:
            if self.mode == "no_sim":
                raise refuse(NO_SIM, "the sim is not attached to an episode (mode 'no_sim')")
            if self.mode not in ("awaiting_plan", "executing"):
                raise refuse(NO_EPISODE, f"no episode is running (mode {self.mode!r})")
            if self.publishing is not None:
                raise refuse(PUBLISHING_REASON, PUBLISHING)
            if self.abort is not None:
                raise refuse(ABANDONED, "the episode is being abandoned")
            if not srv.link.grades():
                raise refuse(NO_GRADE_FEATURE, NO_GRADE)
            req = self.grade_req = {"cell": cell, "sent": False, "done": False}
            self.acked.discard(cell)
            srv.cond.notify_all()
            end = time.monotonic() + GRADE_WAIT_S
            while not req["done"] and "error" not in req and self.grade_req is req:
                left = end - time.monotonic()
                if left <= 0:
                    break
                srv.cond.wait(left)
            if self.grade_req is req:
                self.grade_req = None
            if req["done"]:
                self.acked.add(cell)
            step = self.step
        if not req["done"]:
            why = req.get("error") or "the sim backend did not answer the grade"
            message = f"cell {cell} was not graded: {why}"
            raise refuse(NOT_GRADED, message)
        srv.trace_episode("graded", cell=cell, step=step)
        return {"ok": True, "cell": cell}

    def graded(self, ack: dict) -> None:
        """Under the lock, at /sim/poll: the backend's graded ack of the pending grade."""
        req = self.grade_req
        if req is None or not req["sent"] or ack.get("cell") != req["cell"]:
            return
        if "error" in ack:
            req["error"] = str(ack["error"])
        else:
            req["done"] = True
        self._srv.cond.notify_all()

    def job_json(self, job) -> dict | None:
        if job is None:
            return None
        if job.get("quit"):
            return {"job_id": job["job_id"], "quit": True}
        out = {k: job.get(k) for k in ("job_id", "task", "episode", "replace", "redo_of")}
        if job.get("run"):
            out["run"] = job["run"]
        if job.get("instruction_type"):
            out["instruction_type"] = job["instruction_type"]
        return out

    def benchmark_json(self) -> dict | None:
        results = self._srv.results
        cat = results.catalog
        if cat is None:
            return None
        done, ok = results.summary()
        return {"policy_label": cat.policy_label, "output_dir": cat.output_dir,
                "episodes_per_task": cat.episodes_per_task, "n_tasks": len(cat.tasks),
                "done": done, "succeeded": ok, "loaded": self.loaded,
                "job": self.job_json(self.jobs.active), "pending": self.job_json(self.jobs.pending),
                "replay": None, "aborting": self.abort,
                "last_end": None if self.last_end is None else dict(self.last_end)}

    def on_json(self) -> tuple:
        """(task, episode) of the job this server is on, active before pending."""
        for j in (self.jobs.active, self.jobs.pending):
            if j is not None and not j.get("quit"):
                return j["task"], j["episode"]
        return None, None
