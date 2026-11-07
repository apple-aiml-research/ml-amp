#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The practice agent process: `python -m amp.practice.agent <episode_dir>`."""

from __future__ import annotations

import sys
from pathlib import Path

from amp.agents import run as agent_run
from amp.agents.loop import Agent
from amp.contracts.eval import AgentRunConfig
from amp.contracts.trace import KERNEL_STREAM
from amp.practice import workspace


def kernel_cwd(ep_dir: Path, sandbox) -> Path:
    """The kernel's ./ on the host: under bwrap the episode's backing dir, else the episode dir."""
    from amp.kernel import sandbox_backend

    if sandbox_backend(sandbox) == "bwrap":
        return ep_dir / KERNEL_STREAM.split("/")[0]
    return ep_dir


class PracticeAgent(Agent):
    """The image agent's loop plus the workspace hook."""

    def __init__(self, *args, workspace: dict | None = None, **kw):
        super().__init__(*args, **kw)
        self.workspace = workspace

    def ev(self, kind: str, data: dict, **kw) -> None:
        if kind == "agent.run.begin" and self.first_user:
            data = {**data, "first_user": "\n\n".join(b["text"] for b in self.first_user)}
        super().ev(kind, data, **kw)

    def first_user_content(self, obs: dict, instruction: str | None) -> list[dict]:
        if self.workspace is not None:
            self.meta["workspace"] = {k: self.workspace[k] for k in ("sha", "files", "bytes")}
            self.ev("agent.workspace", dict(self.workspace))
        first = super().first_user_content(obs, instruction)
        if self.workspace is not None:
            first.append(workspace.first_user_block(self.workspace))
        return first


def practice_agent(ep_dir: Path, *, meta: dict, **kw) -> PracticeAgent:
    """The frozen run()'s Agent call, plus the workspace unpacked from the same profile."""
    cfg = AgentRunConfig.read(ep_dir)
    ws = workspace.of(cfg.agent)
    ws_meta = None
    if ws is not None:
        ws_meta = workspace.unpack(ws, ep_dir, kernel_cwd(ep_dir, cfg.protocol.sandbox))
        meta["profile"]["extra"][workspace.WORKSPACE_KEY] = workspace.describe(ws)
    return PracticeAgent(ep_dir, meta=meta, workspace=ws_meta, **kw)


def install() -> None:
    """Make amp.agents.run build the practice agent in place of the image agent's loop."""
    agent_run.Agent = practice_agent


def run(ep_dir: Path, **kw) -> int:
    """amp.agents.run.run with the practice agent."""
    install()
    return agent_run.run(ep_dir, **kw)


def main(argv=None, **kw) -> int:
    install()
    return agent_run.main(argv, **kw)


if __name__ == "__main__":
    sys.exit(main())
