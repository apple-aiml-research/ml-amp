#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The composition root: one Server per slot, wiring the components around one lock."""

from __future__ import annotations

import sys
import threading
from pathlib import Path

from amp.contracts.server import ServerConfig
from amp.contracts.trace import Tracer
from amp.geometry.embodiment import Embodiment
from amp.robot_server import config as config_mod
from amp.robot_server import embodiment as rt
from amp.robot_server.episode import Episode
from amp.robot_server.errors import SessionConflict
from amp.robot_server.executor import Executor
from amp.robot_server.link_endpoint import LinkEndpoint
from amp.robot_server.observations import Observations
from amp.robot_server.perception_service import PerceptionService
from amp.robot_server.planning_service import PlanningService
from amp.robot_server.results import Results
from amp.robot_server.tracing import TraceRouter
from amp.robot_server.world import World


class Server:
    """All of a slot's state, as components that share one condition variable (`cond`)."""

    def __init__(self, cfg: ServerConfig, *, planner=None, tracer: Tracer | None = None,
                 echo_log: bool = True):
        self.config = cfg
        self.settings = config_mod.from_config(cfg)
        self.cond = threading.Condition(threading.RLock())
        self.stopping = threading.Event()
        self.traces = TraceRouter(Path(cfg.state_dir), root=tracer)
        self._echo = echo_log
        self.results = Results(config_mod.POLICY_LABEL)
        self.episode = Episode(self)
        self.embodiment = Embodiment.droid()
        self.embodiment_key = (None, "")
        self.obs = Observations(embodiment=self.embodiment, log=self.log)
        self.world = World()
        self.executor = Executor(self, self.settings.tracking)
        self.planning = PlanningService(self, planner=planner)
        self.perception = PerceptionService(self, interval_s=config_mod.PERCEPTION_INTERVAL_S)
        self.link = LinkEndpoint(self)

    def camera_role_refusal(self, emb: Embodiment | None) -> None:
        """ValueError when emb has no camera this harness keeps (None: nothing new to check)."""
        if emb is not None and self.settings.keep_camera_roles:
            self.cameras_with_kept_roles(emb)

    def cameras_with_kept_roles(self, emb: Embodiment) -> set[str]:
        """The names of emb's cameras whose role keep_camera_roles keeps; ValueError if none."""
        roles = self.settings.keep_camera_roles
        names = {c.name for r in roles for c in emb.cameras_with_role(r)}
        if not names:
            raise ValueError(f"keep_camera_roles {list(roles)}: embodiment {emb.name} has no camera "
                             "with that role")
        return names

    def set_embodiment(self, emb: Embodiment, key: tuple) -> None:
        """Under the lock: the declared robot's runtime replaces the current one (if it differs)."""
        if key == self.embodiment_key:
            return
        self.embodiment_key = key
        self.embodiment = emb
        self.obs.set_embodiment(emb)
        self.executor.set_embodiment(emb)
        self.perception.reset()
        self.planning.set_embodiment()
        self.log("info", f"embodiment {emb.name}: {emb.dof} joints, action dim {emb.action_dim}, "
                 f"end effectors {', '.join(emb.end_effectors)}")

    def declare(self, declared, emb: Embodiment | None) -> None:
        """Under the lock: what the backend declared, with its runtime once it is complete."""
        if emb is not None:
            self.set_embodiment(emb, rt.key(declared))

    def log(self, level: str, message: str) -> None:
        self.tracer_now().event("server.log", {"message": str(message)}, lvl=level)
        if self._echo and level in ("warn", "error"):
            print(f"[amp.robot_server {level}] {message}", file=sys.stderr, flush=True)

    def tracer_now(self) -> Tracer:
        return self.traces.current()

    def trace(self, kind: str, data: dict) -> None:
        self.tracer_now().event(kind, data)

    def trace_episode(self, event: str, **data) -> None:
        self.tracer_now().event("server.episode", {"event": event,
                                                   "state": self.episode.state(), **data})

    def close_episode_trace(self) -> None:
        self.traces.close_current()

    def close(self) -> None:
        self.stopping.set()
        with self.cond:
            self.perception.closed = True
            self.cond.notify_all()
        self.traces.close_all()

    def obstacles(self) -> str:
        return self.episode.obstacles or self.settings.obstacles

    def observations_live(self):
        """Under the lock: the latest observation; 409 before the first or once it is over."""
        obs = self.obs.live()
        if self.episode.mode == "episode_over":
            raise SessionConflict("the episode is over")
        return obs

    def table_z(self) -> float:
        sc = self.world.scene
        return sc.table_z if sc is not None else self.obs.table_z

    def moved(self) -> bool:
        """Under the lock: a chunk started this episode, so release must publish it."""
        return self.executor.moved()

    def motion(self) -> dict:
        """Under the lock: whether motion is in flight (a chunk, a steps chain, a base chunk)."""
        ch = self.executor.chunk
        chunk = ch is not None and ch.kind != "publish"
        steps = self.planning.executing()
        return {"in_flight": chunk or steps, "chunk": chunk, "steps": steps,
                "base": chunk and ch.base_track is not None}

    def draft_empty(self) -> bool:
        return not (self.planning.spec_count() or self.executor.recorded())

    def stop_motion(self, *, rebase: bool = False):
        """Under the lock: stop the multi-step execution and the running chunk."""
        ex = self.planning.cancel_execution()
        entry = self.executor.stop_chunk(rebase=rebase)
        if entry is None and ex is not None and rebase:
            self.planning.release_turn()
        return entry, ex

    def log_stopped(self, stopped) -> None:
        self.executor.log_stopped(*stopped)

    def link_connected(self, why: str | None = None) -> dict | None:
        """Under the lock: a backend said hello; what ran for the previous one stops (closed job)."""
        self.planning.cancel_execution()
        self.executor.link_reset()
        closed = self.episode.link_connected(why)
        self.cond.notify_all()
        return closed

    def end_episode(self, result, reason):
        """Under the lock, at /sim/end: ((teleop path, record) to write or None, a warning)."""
        self.planning.cancel_execution()
        self.executor.link_reset()
        job, warning = self.episode.ended(result, reason)
        rec = self.executor.rec
        if job is None or rec is None:
            return None, warning
        teleop = self.episode.teleop_record(job, rec, self.results.catalog.policy_label, reason)
        return (self.results.teleop_file(teleop, job.get("run", "")), teleop), warning

    def begin_episode(self, episode, task, instruction, max_steps, control_dt,
                      keep_scene=True) -> None:
        """Under the lock: every component starts the episode."""
        self.executor.drop("stopped")
        self.world.reset(episode, keep_scene=keep_scene)
        self.perception.reset()
        self.planning.reset()
        self.episode.begin(episode, task, instruction, max_steps, control_dt)
        self.obs.reset(self.episode.token)
        self.executor.reset(self.episode.control_dt)
        self.trace_episode("begin", episode=episode, task=task)

    def ingest(self, obs) -> None:
        """Under the lock: a /sim/step observation reaches every component that tracks it."""
        if self.settings.keep_camera_roles:
            names = self.cameras_with_kept_roles(self.embodiment)
            obs.cameras = {k: v for k, v in obs.cameras.items() if k in names}
        if self.settings.drop_depth:
            for fr in obs.cameras.values():
                fr.depth = fr.depth_model = None
        ep = self.episode
        if ep.mode == "episode_over" or obs.episode != ep.episode:
            ep.unbind()
            self.begin_episode(obs.episode, obs.task, obs.instruction, obs.max_steps,
                               obs.control_dt)
        self.obs.ingest(obs)
        self.executor.on_observation(obs)
        if self.obs.table_pending:
            z = self.obs.estimate_table(self.world.boxes(obs.episode))
            if z is not None:
                self.world.set_table_z(z)
        ep.update_from(obs)
        self.executor.track(obs)
        self.world.track_held(self.obs.observed_tcp())
