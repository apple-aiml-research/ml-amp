#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""AMP's RoboLab policy runner, after RoboLab's policies/pi0_family/run.py: one task per process."""

from __future__ import annotations

import argparse
import os
import sys
import traceback

DEFAULT_POLICY_LABEL = "amp"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate AMP agents on RoboLab.")
    parser.add_argument("--policy", default=DEFAULT_POLICY_LABEL,
                        help="Label for run_evaluation: the output folder and the rows' policy.")
    parser.add_argument("--remote-host", "--remote_host", default="127.0.0.1",
                        help="The slot's robot server host.")
    parser.add_argument("--remote-port", "--remote_port", type=int, default=8765,
                        help="The slot's robot server port.")
    parser.add_argument("--amp-config", "--amp_config", default="{}",
                        help="The client's settings, a ClientConfig as JSON.")
    parser.add_argument("--amp-driver", "--amp_driver", required=True,
                        help="Unix socket of the slot's episode driver.")
    return parser


def _log(message: str) -> None:
    print(f"[amp.robots.robolab] {message}", flush=True)


def main(argv: list[str] | None = None) -> int:
    try:
        import cv2  # noqa: F401
    except ImportError:
        pass
    from isaaclab.app import AppLauncher

    from amp.robots.kitlock import KIT_START_LOCK_ENV, StartLock
    from amp.robots.robolab.client import ClientConfig

    parser = build_parser()
    from robolab.eval.runner import add_common_eval_args, run_evaluation

    add_common_eval_args(parser)
    AppLauncher.add_app_launcher_args(parser)
    args, _ = parser.parse_known_args(argv)
    args.enable_cameras = True
    if args.num_envs != 1:
        parser.error(f"--num-envs must be 1 (one agent drives one robot), got {args.num_envs}")
    if args.output_folder_name is None:
        parser.error("--output-folder-name is required: the launcher names each slot's folder")
    config = ClientConfig.from_json(args.amp_config)

    lock_path = os.environ.get(KIT_START_LOCK_ENV)
    lock = StartLock(lock_path, lambda exc: _log(f"Kit start lock unusable: {exc}")) \
        if lock_path else None
    if lock is not None and not lock.acquire():
        _log(f"Kit start lock {lock_path} not taken; starting anyway")
    simulation_app = AppLauncher(args).app
    try:
        _evaluate(args, config, run_evaluation, lock)
    except Exception as exc:  # noqa: BLE001
        print(f"[amp.robots.robolab] terminated with error: {exc}", flush=True)
        traceback.print_exc()
        simulation_app.close()
        return 1
    finally:
        if lock is not None:
            lock.close()
    simulation_app.close()
    return 0


def _evaluate(args, config, run_evaluation, lock) -> None:
    import robolab.constants
    from robolab.core.environments.factory import get_envs, get_global_env_factory

    from amp.link.client import LinkClient
    from amp.robots.robolab import bridge
    from amp.robots.robolab.catalog import catalog_tasks
    from amp.robots.robolab.client import make_client as amp_client
    from amp.robots.robolab.registration import register_amp_envs

    robolab.constants.ENABLE_SUBTASK_PROGRESS_CHECKING = args.enable_subtask
    register_amp_envs(task_dirs=args.task_dirs, task=args.task)

    output_dir = os.path.join(robolab.constants.PACKAGE_DIR, "output", args.output_folder_name)
    names = get_envs(task=args.task) if args.task else get_envs()
    tasks = catalog_tasks(names, lambda n: get_global_env_factory().get_env_info(n),
                          getattr(robolab.constants, "TASK_DIR", None), args.instruction_type)

    link = LinkClient(f"http://{args.remote_host}:{int(args.remote_port)}",
                      timeout_s=config.timeout_s, client="robolab")
    hello = link.hello(takeover=config.takeover)
    link.send_catalog(policy_label=args.policy, output_dir=output_dir,
                      episodes_per_task=int(args.num_runs), tasks=tasks)
    _log(f"robot server {link.base_url}: {hello}; catalog {[t['name'] for t in tasks]}, "
         f"{args.num_runs} run(s) each, results in {output_dir}")
    session = bridge.Session(link, args.amp_driver, log=_log,
                             instructions={t["name"]: t["instruction"] for t in tasks})
    bridge.ping(args.amp_driver)

    def make_client(_args: argparse.Namespace):
        if lock is not None:
            lock.release()
        return amp_client(session, config, log=_log)

    run_evaluation(args, policy=args.policy, client_factory=make_client)
    session.flush_saved()


if __name__ == "__main__":
    sys.exit(main())
