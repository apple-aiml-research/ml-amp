#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The trial clock for one official-protocol episode: blocks, the deadline and its two paths."""

from __future__ import annotations

import dataclasses
import hashlib
import hmac
from dataclasses import dataclass, field

STOP = "stop"
GRADE = "grade"
PATH2 = "path2"
IN_BLOCK, OUTSIDE_BLOCK = "in_block", "outside_block"
CAUSES = ("clock", "model_timeout")
INTERRUPT_SKEW_S = 0.05
VOID_REASON = ("official: the decisive block could not be graded (infrastructure): unscored, "
               "not retried")


STATE_REASONS = frozenset({"no_episode", "publishing", "abandoned"})
INFRA_REASONS = frozenset({"not_graded", "no_grade_feature", "no_sim", "bad_request"})


def grade_failure_is_infra(status: int | None, reply) -> bool:
    """Whether a failed api/episode/grade says the grading itself broke, by its 409 reason only."""
    if status != 409 or not isinstance(reply, dict):
        return True
    reason = reply.get("reason")
    return reason in INFRA_REASONS or reason not in STATE_REASONS


def key_digest(key: str) -> str:
    return hashlib.sha256(key.encode()).hexdigest()


@dataclass
class Official:
    """One attempt's clock and block bookkeeping; a block is a cell plus the motion it started."""

    trial_timeout_s: int
    attempt: int
    max_attempts: int
    key_sha256: str
    deadline: float | None = None
    mono_to_wall: float = 0.0
    started_at: float | None = None
    phase: str = "outside"
    cell: int = 0
    grading: list[int] = field(default_factory=list)
    completed: int | None = None
    late: list[int] = field(default_factory=list)
    handled: bool = False
    pending: bool = False
    fired: str | None = None
    cause: str | None = None
    in_block_at: float | None = None
    fired_at: float | None = None
    timeout_owed: bool = False
    grade_errors: int = 0
    suspect: list[int] = field(default_factory=list)
    suspect_late: list[int] = field(default_factory=list)
    untrusted: list[int] = field(default_factory=list)
    grade_unreliable: bool = False
    rescue: int | None = None
    regraded: int | None = None
    void: bool = False
    void_reason: str | None = None
    verdict_ignored: bool = False

    @classmethod
    def from_dict(cls, d: dict) -> Official:
        names = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in names})

    def key_ok(self, got: str) -> bool:
        return hmac.compare_digest(key_digest(got or ""), self.key_sha256)

    @property
    def armed(self) -> bool:
        return self.deadline is not None and self.fired is None

    @property
    def force(self) -> bool:
        """Path 2 that discards a moved episode: before the last attempt, or a void one."""
        return self.fired == OUTSIDE_BLOCK and (self.attempt < self.max_attempts or self.void)

    def wall(self, now: float) -> float:
        """A monotonic time as the wall time that is recorded."""
        return now + self.mono_to_wall

    def due(self, now: float) -> bool:
        """The deadline has passed and nothing has decided it yet."""
        return (self.deadline is not None and not self.handled and self.fired is None
                and now >= self.deadline)

    def refuses_motion(self) -> bool:
        """New motion commands run only inside a begun cell, and not past its deadline (A2)."""
        return self.phase != "cell" or self.pending

    def interrupt_in(self, now: float) -> float | None:
        """Seconds the kernel may still run a cell; None only once path 1 has disarmed the clock."""
        if not self.armed:
            return None
        return round(max(0.0, self.deadline - now), 3)

    def start(self, now: float, wall: float) -> None:
        if self.deadline is None:
            self.mono_to_wall = wall - now
            self.started_at, self.deadline = wall, now + self.trial_timeout_s

    def at_deadline(self, now: float, motion: bool) -> list:
        """Decide the deadline from the state at T (A1, A4, H2); a no-op unless it is due."""
        if not self.due(now):
            return []
        self.handled = True
        if self.phase == "cell":
            self.pending = True
            if motion:
                self._path1(now)
            return [(STOP,)]
        if motion:
            self._path1(now)
            self.timeout_owed = True
            return [(STOP,)]
        return [(STOP,), (PATH2,)]

    def settle(self, now: float, motion: bool) -> list:
        """A block ends when the later of its cell's end and its motion's settling is reached."""
        if self.phase == "settling" and not motion and not self.due(now):
            return self._end_block()
        return []

    def begin_close(self, now: float, motion: bool) -> list:
        """The first half of a begin: end the previous block, whose grade must finish first."""
        acts = []
        if self.phase == "cell":
            acts += self.end(now, self.cell, False, motion)
        if self.phase == "settling":
            acts += self._end_block()
        return acts

    def begin_open(self) -> dict:
        """The second half: admit the cell (the reply lacks interrupt_in_s, the gateway's)."""
        self.cell += 1
        self.phase = "cell"
        reply = {"ok": True, "cell": self.cell}
        if self.timeout_owed:
            self.timeout_owed = False
            reply["official_timeout"] = IN_BLOCK
        return reply

    def end(self, now: float, n: int, interrupted: bool, motion: bool) -> list:
        """Cell n's end report, however late; a repeat or a stale one changes nothing."""
        if n != self.cell or self.phase != "cell":
            return []
        acts = []
        if (interrupted and not self.pending and not self.handled and self.armed
                and self.deadline - now <= INTERRUPT_SKEW_S):
            self.handled = True
            self._path1(now)
            acts.append((STOP,))
        if self.pending:
            self.pending = False
            acts.append((STOP,))
            if self.fired is None:
                if interrupted:
                    self._path1(now)
                elif motion:
                    self._path1(now)
                    self.timeout_owed = True
                else:
                    self.phase = "outside"
                    return [*acts, (PATH2,)]
            elif not interrupted:
                self.timeout_owed = True
        if motion:
            self.phase = "settling"
        else:
            acts += self._end_block()
        return acts

    def finish_in_cell(self, now: float) -> None:
        """The agent finished from inside the cell the deadline caught: an in-cell interrupt."""
        if self.pending:
            self.pending = False
            if self.fired is None:
                self._path1(now)

    def grade_failed(self, now: float, n: int, infra: bool) -> None:
        """Block n's grade failed: suspect wherever it could still decide a verdict."""
        self.grade_errors += 1
        if not infra or self.fired == OUTSIDE_BLOCK or n in self.suspect:
            return
        if self.fired == IN_BLOCK or self.deadline is None or now < self.deadline:
            self.suspect.append(n)
        elif n not in self.suspect_late:
            self.suspect_late.append(n)

    def graded(self, now: float, n: int) -> None:
        """Block n's grade answered at now: it counts if that was before the deadline."""
        if n in self.untrusted:
            return
        if self.fired == IN_BLOCK or self.deadline is None or now < self.deadline:
            self.completed = max(n, self.completed or 0)
        elif self.fired is None:
            self.late.append(n)

    def distrust(self, now: float, n: int) -> None:
        """Block n's grade is still unknown after the barrier's cap: suspect, its answer ignored."""
        self.untrusted.append(n)
        self.grade_failed(now, n, True)
        self.grade_done(n)

    def grade_done(self, n: int) -> None:
        if n in self.grading:
            self.grading.remove(n)

    def unreliable(self) -> bool:
        """A block that ended before T failed to grade and no later one did: no sound verdict."""
        return any(n > (self.completed or 0) for n in self.suspect)

    def fire_outside(self, now: float, cause: str, idle: bool = True) -> None:
        """Path 2; on the last attempt an ungradable decisive block is re-graded once, else void."""
        self.fired, self.cause, self.fired_at = OUTSIDE_BLOCK, cause, self.wall(now)
        self.handled, self.pending = True, False
        if self.attempt >= self.max_attempts and self.unreliable():
            self.grade_unreliable = True
            ended_unscored = self.phase == "settling"
            if idle and self.cell > (self.completed or 0) and (
                    ended_unscored or (self.phase == "outside" and self.cell in self.suspect)):
                self.rescue = self.cell
            else:
                self.void, self.void_reason = True, VOID_REASON

    def regrade_done(self, ok: bool) -> None:
        """The re-grade of the rescued block answered: score from it, or the unit is void."""
        n, self.rescue = self.rescue, None
        if ok and n is not None:
            self.completed, self.regraded, self.grade_unreliable = n, n, False
        else:
            self.void, self.void_reason = True, VOID_REASON

    def verdict(self) -> dict:
        return {"cell": self.completed}

    def row(self) -> dict:
        """The ledger row's official block."""
        return {"attempt": self.attempt, "max_attempts": self.max_attempts,
                "trial_timeout_s": self.trial_timeout_s, "timeout": self.fired,
                "cause": self.cause, "cells": self.cell, "completed_cell": self.completed,
                "started_at": self.started_at, "in_block_at": self.in_block_at,
                "fired_at": self.fired_at, "grade_errors": self.grade_errors,
                **({"grade_failed": sorted(self.suspect)} if self.suspect else {}),
                **({"grade_unreliable": True} if self.grade_unreliable else {}),
                **({"regraded": self.regraded} if self.regraded is not None else {}),
                **({"void": True, "void_reason": self.void_reason} if self.void else {}),
                **({"verdict_ignored": True} if self.verdict_ignored else {})}

    def _path1(self, now: float) -> None:
        self.fired, self.cause, self.in_block_at = IN_BLOCK, "clock", self.wall(now)
        self.handled = True
        if self.late:
            self.completed = max(self.completed or 0, *self.late)
            self.late = []
        self.suspect += [n for n in self.suspect_late if n not in self.suspect]
        self.suspect_late = []

    def _end_block(self) -> list:
        self.phase = "outside"
        self.grading.append(self.cell)
        return [(GRADE, self.cell)]


def parse_claim(off) -> dict:
    """The claim body's "official": {trial_timeout_s, attempt, max_attempts}, checked."""
    if not isinstance(off, dict):
        raise TypeError("official must be an object")
    out = {}
    for k in ("trial_timeout_s", "attempt", "max_attempts"):
        v = off.get(k)
        if isinstance(v, bool) or not isinstance(v, int) or v < 1:
            raise ValueError(f"official.{k} must be an integer >= 1, got {v!r}")
        out[k] = v
    if out["attempt"] > out["max_attempts"]:
        raise ValueError(f"official.attempt {out['attempt']} > max_attempts {out['max_attempts']}")
    return out
