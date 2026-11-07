#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""python -m amp.robot_server --config <ServerConfig JSON>: one slot's headless robot API server."""

from __future__ import annotations

import argparse
import signal
from pathlib import Path

from amp.contracts.server import ServerConfig
from amp.robot_server.app import build, make_server


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="python -m amp.robot_server", description=__doc__)
    p.add_argument("--config", required=True, help="a ServerConfig JSON file")
    p.add_argument("--port", type=int, help="override the config's port")
    args = p.parse_args(argv)
    cfg = ServerConfig.from_json(Path(args.config).read_text())
    srv = build(cfg)
    http = make_server(srv, port=args.port)
    host, port = http.server_address[:2]
    print(f"amp.robot_server: http://{host}:{port} (info mode {cfg.info_mode})", flush=True)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    try:
        http.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        http.server_close()


if __name__ == "__main__":
    main()
