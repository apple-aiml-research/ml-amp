#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CaP-Gym official mode, agent side: cell signals and model timeouts to the gateway's trial clock."""

from __future__ import annotations

import logging
import os
from collections.abc import MutableMapping

from amp.agents.credentials import register_secret
from amp.agents.episode import EpisodeClient, GatewayError
from amp.contracts.eval import CAPX_TRIAL_TIMEOUT_S, OFFICIAL_KEY_ENV

log = logging.getLogger("amp.agents.official")

DEADLINE_ERROR = f"TimeoutError: Trial exceeded {CAPX_TRIAL_TIMEOUT_S} seconds"
INTERRUPTED_NOTE = f"[interrupted: the trial reached its {CAPX_TRIAL_TIMEOUT_S} s time limit]"
EARLIER_MOTION = f"error (motion an earlier cell started):\n{DEADLINE_ERROR}"
IN_BLOCK = "in_block"


def take_key(environ: MutableMapping[str, str] | None = None) -> str | None:
    """Pop the key from the environment, so no child process inherits it; None: not official."""
    key = (os.environ if environ is None else environ).pop(OFFICIAL_KEY_ENV, None)
    if key:
        register_secret(key)
    return key or None


def deadline_text(error: str | None) -> str:
    """A grace-period interrupt past the deadline reads as the deadline's, not the cell's own."""
    error = error or ""
    if error.startswith("TimeoutError: the call ran over"):
        return DEADLINE_ERROR + "\n" + error.partition("\n")[2]
    return error if DEADLINE_ERROR in error else f"{DEADLINE_ERROR}\n{error}"


class AttemptOver(Exception):
    """The gateway refused a cell (409): CaP-X's trial limit ended this attempt (path 2)."""


class Official:
    """One official episode's key and its gateway calls; the key never reaches the kernel."""

    def __init__(self, client: EpisodeClient, key: str):
        self.client, self.key = client, key
        self.surfaced = False

    def begin(self) -> dict:
        """The gateway's begin reply; AttemptOver on a 409, GatewayError when it is unreachable."""
        try:
            reply = self.client.official_begin(self.key)
        except GatewayError as e:
            over = (e.body or {}).get("official_timeout")
            if e.status == 409 and over:
                raise AttemptOver(str(over)) from None
            raise
        return reply

    def surface(self, reply: dict) -> str | None:
        """EARLIER_MOTION once, the first begin after path 1 fired with no cell running (A6)."""
        if self.surfaced or reply.get("official_timeout") != IN_BLOCK:
            return None
        self.surfaced = True
        return EARLIER_MOTION

    def end(self, cell: int, interrupted: bool) -> str | None:
        """None once the gateway has the end report, else why not."""
        try:
            self.client.official_end(self.key, cell, interrupted)
        except (GatewayError, OSError) as e:
            log.warning("official end of cell %s failed: %s", cell, e)
            return f"{type(e).__name__}: {e}"[:500]
        return None

    def model_timeout(self) -> bool:
        try:
            self.client.official_model_timeout(self.key)
        except (GatewayError, OSError) as e:
            log.warning("official model_timeout failed: %s", e)
            return False
        return True
