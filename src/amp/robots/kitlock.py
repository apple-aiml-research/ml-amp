#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""One Kit process start per GPU at a time: a flock its starter holds until the Kit is up."""

from __future__ import annotations

import contextlib
import fcntl
import os
import threading
import time
from collections.abc import Callable

KIT_START_LOCK_ENV = "AMP_KIT_START_LOCK"
MAX_WAIT_S = 300.0
POLL_S = 0.2


def _take(fd: int, timeout_s: float, on_error: Callable[[OSError], None] | None) -> bool:
    deadline = time.monotonic() + max(timeout_s, 0.0)
    while True:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except BlockingIOError:
            left = deadline - time.monotonic()
            if left <= 0:
                return False
            time.sleep(min(POLL_S, left))
        except OSError as exc:
            if on_error:
                on_error(exc)
            return False


class StartLock:
    """The lock at path, held from acquire() until release() or close(), from any thread."""

    def __init__(self, path: str | os.PathLike,
                 on_error: Callable[[OSError], None] | None = None):
        self.path, self.on_error = path, on_error
        self.held = False
        self._fd: int | None = None
        self._mutex = threading.Lock()

    def acquire(self, timeout_s: float = MAX_WAIT_S) -> bool:
        """Wait up to timeout_s; False on timeout or if unusable (see on_error)."""
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            self._fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        except OSError as exc:
            if self.on_error:
                self.on_error(exc)
            return False
        self.held = _take(self._fd, timeout_s, self.on_error)
        return self.held

    def release(self) -> bool:
        """Let the lock go now; whether this call is the one that did. Idempotent."""
        with self._mutex:
            if not self.held or self._fd is None:
                return False
            self.held = False
            with contextlib.suppress(OSError):
                fcntl.flock(self._fd, fcntl.LOCK_UN)
            return True

    def close(self) -> None:
        self.release()
        with self._mutex:
            if self._fd is not None:
                os.close(self._fd)
                self._fd = None
