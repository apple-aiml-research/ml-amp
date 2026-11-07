#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""python -m amp.gateway --config <GatewayConfig JSON>: the run's agent gateway, every harness."""

from __future__ import annotations

import argparse
import json
import logging
import signal
from pathlib import Path

from amp.contracts.gateway import GatewayConfig
from amp.gateway.app import make_server
from amp.gateway.claims import Gateway
from amp.gateway.policy import PolicyStore

OWN_HARNESSES = Path(__file__).resolve().parents[1] / "harnesses"


def build(cfg: GatewayConfig, poll_s: float = 1.0, idle_timeout_s: float = 1800.0,
          episode_timeout_s: float = 21600.0, concurrent_task_sets=(),
          slot_task_sets: str | None = None,
          load_timeouts: dict[str, float] | None = None) -> Gateway:
    if not cfg.slots:
        raise ValueError("the config names no slot")
    if cfg.harness_dir and not Path(cfg.harness_dir).is_dir():
        raise ValueError(f"harness_dir {cfg.harness_dir} is not a directory")
    policies = PolicyStore(cfg.harness_dir, default_dir=OWN_HARNESSES)
    return Gateway(list(cfg.slots.items()), policies,
                   Path(cfg.key_file).read_text().strip(), cfg.state_dir,
                   idle_timeout_s=idle_timeout_s, episode_timeout_s=episode_timeout_s,
                   load_timeout_s=cfg.claim_timeout_s, request_timeout_s=cfg.request_timeout_s,
                   poll_s=poll_s, concurrent_task_sets=concurrent_task_sets,
                   slot_task_sets=(lambda: json.loads(Path(slot_task_sets).read_text()))
                   if slot_task_sets else None, slot_task_sets_path=slot_task_sets,
                   load_timeouts=load_timeouts)


def parse_load_timeouts(raw: str) -> dict[str, float]:
    """--load-timeouts SET=S[,SET=S...]; ValueError on a malformed or non-positive entry."""
    out = {}
    for item in (t.strip() for t in raw.split(",")):
        if not item:
            continue
        name, sep, secs = item.partition("=")
        if not sep or not name.strip() or not float(secs) > 0:
            raise ValueError(f"--load-timeouts: {item!r} is not SET=seconds > 0")
        out[name.strip()] = float(secs)
    return out


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="python -m amp.gateway", description=__doc__)
    p.add_argument("--config", required=True,
                   help="a GatewayConfig JSON file (amp.contracts.gateway)")
    p.add_argument("--idle-timeout-s", type=float, default=1800.0)
    p.add_argument("--episode-timeout-s", type=float, default=21600.0)
    p.add_argument("--poll-s", type=float, default=1.0)
    p.add_argument("--concurrent-task-sets", default="", metavar="SET[,SET...]",
                   help="task sets whose claims of one run may hold several episodes of a task "
                        "live at once (default none); only on a slot --slot-task-sets maps to it")
    p.add_argument("--load-timeouts", default="", metavar="SET=S[,SET=S...]",
                   help="per task set, a longer claim timeout than the config's (default none)")
    p.add_argument("--slot-task-sets", metavar="PATH",
                   help="JSON {slot: task set its backend hosts}, reread per claim (the launcher "
                        "writes it)")
    args = p.parse_args(argv)
    try:
        cfg = GatewayConfig.from_json(Path(args.config).read_text())
        gateway = build(cfg, args.poll_s, args.idle_timeout_s, args.episode_timeout_s,
                        [t.strip() for t in args.concurrent_task_sets.split(",") if t.strip()],
                        args.slot_task_sets, parse_load_timeouts(args.load_timeouts))
        server = make_server(gateway, cfg.host, cfg.port)
    except (ValueError, TypeError, OSError, json.JSONDecodeError) as e:
        p.error(str(e))
    logging.basicConfig(level=logging.INFO, format="[amp gateway] %(message)s")
    host, port = server.server_address[:2]
    print(f"amp gateway: http://{host}:{port} for {len(gateway.slots)} slot(s)", flush=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        gateway.stopping.set()
        gateway.stop()
        server.server_close()


if __name__ == "__main__":
    main()
