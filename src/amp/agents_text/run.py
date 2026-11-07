#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The text-only agent process: `python -m amp.agents_text.run <episode_dir>`.

Reads <episode_dir>/agent_config.json (AgentRunConfig), composes the system prompt from the
profile's template and the harness's reference docs, starts the kernel, and runs the loop; for
vision "vdm_strict" it also builds the vision model, and the kernel's relay refuses the camera
images.

Exit 0 whenever the loop ended (any outcome; the trace says which), 1 when the loop crashed, 2 when
the process failed before or around it (a missing or unreadable episode dir included), and 143
(128 + SIGTERM) when terminated: the loop still writes agent.run.end with end_reason "killed".
agent.log passes through the trace's redactor, which knows the gateway token.
"""

from __future__ import annotations

import argparse
import dataclasses
import logging
import shutil
import signal
import sys
import urllib.parse
from pathlib import Path

from amp.agents.credentials import register_secret
from amp.agents.episode import EpisodeClient
from amp.agents.factory import build_llm, describe_llm
from amp.agents.models import (
    STRICT_REFUSED,
    STRICT_SERVER_ENV,
    strict_harness_problem,
    vdm_profile,
)
from amp.agents.official import Official, take_key
from amp.agents.prompt import (  # noqa: F401
    PROMPT_ID,
    PROMPTS,
    compose_system,
    prompt_text,
    system_text,
)
from amp.agents.runtime import detect_runtime
from amp.agents_text import surface
from amp.agents_text.loop import Agent
from amp.agents_text.surface import self_judged
from amp.agents_text.tools import tools_for, tools_sha256
from amp.contracts import CONTRACT_VERSION
from amp.contracts.eval import AGENT_CONFIG_FILE, EPISODE_FILE, AgentRunConfig
from amp.trace import from_env
from amp.trace.redact import DEFAULT

log = logging.getLogger("amp.agents.run")

LOG_FILE = "agent.log"
SDK_DIR = "sdk"


def gateway_port(info: dict) -> int:
    return urllib.parse.urlsplit(info["gateway"]).port or 80


def install_sdk(spec, source: Path, ep_dir: Path) -> Path:
    """<episode>/sdk/<package>, copied from the harness unless already there."""
    sdk = ep_dir / SDK_DIR
    dest = sdk / Path(spec.sdk_package).name
    if not dest.exists():
        sdk.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, dest, ignore=shutil.ignore_patterns(*spec.sdk_exclude))
    return sdk


def register_episode_secrets(ep_dir: Path) -> None:
    """The gateway token and base from episode.json, so neither reaches the trace or agent.log."""
    import json

    try:
        info = json.loads((ep_dir / EPISODE_FILE).read_text())
    except (OSError, ValueError):
        return
    for key in ("token", "base"):
        if isinstance(info.get(key), str):
            register_secret(info[key])


class RedactingFilter(logging.Filter):
    """Rewrites each record's message and traceback through the trace redactor."""

    def __init__(self, redactor=None):
        super().__init__()
        self.redactor = redactor or DEFAULT

    def filter(self, record: logging.LogRecord) -> bool:
        record.msg, record.args = self.redactor.text(record.getMessage()), None
        if record.exc_info:
            record.exc_text = self.redactor.text(
                logging.Formatter().formatException(record.exc_info))
            record.exc_info = None
        if record.stack_info:
            record.stack_info = self.redactor.text(record.stack_info)
        return True


def _registry():
    from amp.harnesses import registry

    return registry


def _kernel_cls():
    from amp.agents_text.kernel.kernel import Kernel

    return Kernel


def check_strict(spec, registry, sandbox) -> None:
    """ValueError unless harness `spec` can run vision vdm_strict."""
    import json

    reg = getattr(registry, "DEFAULT", registry)
    policy = json.loads(reg.path(spec, spec.route_policy).read_text())
    problem = strict_harness_problem(spec, policy)
    if problem:
        raise ValueError(problem)


def run(ep_dir: Path, *, tracer=None, registry=None, kernel_cls=None, episode_client=None,
        environ=None) -> int:
    """One episode's agent."""
    import json

    key = take_key(environ)
    cfg = AgentRunConfig.read(ep_dir)
    registry = registry or _registry()
    kernel_cls = kernel_cls or _kernel_cls()
    tracer = tracer or from_env("agent")
    spec = registry.load(cfg.harness)
    prof, model = cfg.agent, cfg.agent.model
    strict = prof.vision == "vdm_strict"
    if strict:
        check_strict(spec, registry, cfg.protocol.sandbox)
    info = json.loads((ep_dir / EPISODE_FILE).read_text())
    runtime = detect_runtime(environ)
    text = system_text(prof.prompt, registry.reference_text(spec))
    section = surface.vision_section(prof.prompt, prof.vision)
    if section:
        text = compose_system(text, section)
    judged_by_self = self_judged(prof.prompt)
    vdm_prof = vdm_profile(prof) if prof.vision != "image" else None
    tools = tools_for(spec.preloaded, judged_by_self, vision=prof.vision)
    llm = build_llm(prof, runtime, environ=environ)
    vdm = build_llm(vdm_prof, runtime, environ=environ) if vdm_prof else None
    sdk_dir = install_sdk(spec, registry.sdk_source(spec), ep_dir)
    port = gateway_port(info)

    refuse = ({"refuse": STRICT_REFUSED, "refuse_reason": surface.VDM_NOTES["refused"]}
              if strict else {})

    def kernel_factory():
        return kernel_cls(ep_dir, python=Path(cfg.kernel_python), sdk_dir=sdk_dir,
                          gateway_port=port, sandbox=cfg.protocol.sandbox,
                          bootstrap=spec.bootstrap or None, tracer=tracer.child("kernel"),
                          **({"deadline": True} if key else {}), **refuse)

    reasoning = prof.reasoning or model.default_reasoning
    meta = {"profile": prof.to_dict(), "agent": prof.label, "model_key": model.key,
            "model_id": model.model_id, "model_provider": model.provider,
            "reasoning": reasoning, "reasoning_kind": model.reasoning_kind,
            "prompt_id": prof.prompt, "harness": spec.name, "harness_version": spec.version,
            "contract": CONTRACT_VERSION, "sandbox": cfg.protocol.sandbox,
            "runtime": dataclasses.asdict(runtime), **describe_llm(llm)}
    if vdm_prof:
        meta["vision"] = prof.vision
        meta["vision_section"] = bool(section)
        vm = vdm_prof.model
        meta["vdm"] = {"model_key": vm.key, "model_id": vm.model_id,
                       "model_provider": vm.provider, "reasoning": vdm_prof.reasoning,
                       **describe_llm(vdm)}
    if strict:
        meta["strict"] = {"refused": list(STRICT_REFUSED),
                          "server_env": {k: spec.server.env.get(k) for k in STRICT_SERVER_ENV}}
    log.info("agent %s harness %s@%s tools %s %s", prof.label, spec.name, spec.version,
             tools_sha256(tools)[:12], describe_llm(llm))
    client = (episode_client or EpisodeClient.from_file)(ep_dir / EPISODE_FILE)
    agent = Agent(ep_dir, llm=llm, kernel_factory=kernel_factory, episode=client,
                  system_text=text, tracer=tracer, tools=tools, preloaded=spec.preloaded,
                  zero_shot=cfg.protocol.zero_shot, max_turns=prof.max_turns,
                  wall_s=prof.wall_s, max_cost_usd=prof.max_cost_usd, meta=meta,
                  judged_by_self=judged_by_self,
                  official=Official(client, key) if key else None, vdm_llm=vdm)
    try:
        agent.run()
    finally:
        close = getattr(tracer, "close", None)
        if close:
            close()
    log.info("loop ended: %s", agent.end_reason)
    return 1 if agent.end_reason == "crash" else 0


def setup_logging(ep_dir: Path) -> list[logging.Handler]:
    """INFO and up to <episode>/agent.log, WARNING and up to stderr."""
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(name)s %(levelname)s %(message)s")
    fh = logging.FileHandler(ep_dir / LOG_FILE, encoding="utf-8")
    err = logging.StreamHandler(sys.stderr)
    err.setLevel(logging.WARNING)
    for h in (fh, err):
        h.setFormatter(fmt)
        h.addFilter(RedactingFilter())
        root.addHandler(h)
    return [fh, err]


def main(argv=None, **kw) -> int:
    ap = argparse.ArgumentParser(prog="python -m amp.agents.run", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episode_dir", type=Path, help=f"holds {EPISODE_FILE} and "
                    f"{AGENT_CONFIG_FILE}")
    a = ap.parse_args(argv)
    ep_dir = a.episode_dir.resolve()
    handlers: list[logging.Handler] = []

    def on_term(signum, frame):
        raise SystemExit(128 + signum)

    signal.signal(signal.SIGTERM, on_term)
    try:
        register_episode_secrets(ep_dir)
        handlers = setup_logging(ep_dir)
        return run(ep_dir, **kw)
    except Exception as e:
        if handlers:
            log.exception("agent process crashed")
        else:
            print(f"agent process failed before logging: {DEFAULT.text(str(e))}",
                  file=sys.stderr)
        return 2
    finally:
        for h in handlers:
            logging.getLogger().removeHandler(h)
            h.close()


if __name__ == "__main__":
    sys.exit(main())
