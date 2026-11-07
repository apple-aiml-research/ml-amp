#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The link endpoint: the /sim/* side a robot backend (an amp.link.LinkClient) drives."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field

from amp.contracts.embodiment import DROID, EmbodimentSpec, RobotDescription
from amp.contracts.robot import ACTION_DIM, LINK_PROTOCOL_VERSION
from amp.link.obs import unpack_observation
from amp.link.robot import (
    Mesh,
    chain_shas,
    description_sha256,
    hello_embodiment,
    parse_robot,
)
from amp.link.wire import GRADE, jsonable
from amp.robot_server import embodiment as rt
from amp.robot_server.config import OWNER_WINDOW_S, POLL_TIMEOUT_S, SAVE_WINDOW_S
from amp.robot_server.episode import EVICTED, LOST, wire_job
from amp.robot_server.errors import SessionConflict
from amp.robot_server.world import SCENE_IGNORED

NOT_OWNER = "not the owning sim client"
CONNECTED_WINDOW_S = 5.0


def short(cid) -> str:
    return "(no client_id)" if cid is None else str(cid)[:8]


@dataclass(frozen=True)
class DeclaredRobot:
    """What the owning backend declared: its embodiment (None = DROID) and its /sim/robot."""

    embodiment: EmbodimentSpec | None = None
    description: RobotDescription | None = None
    meshes: dict[str, Mesh] = field(default_factory=dict)
    description_sha256: str = ""

    @property
    def name(self) -> str:
        return DROID if self.embodiment is None else self.embodiment.name

    @property
    def action_dim(self) -> int:
        return ACTION_DIM if self.embodiment is None else self.embodiment.action_dim

    @property
    def needs_description(self) -> bool:
        """A chain arm's kinematics come from /sim/robot, so it must arrive before the catalog."""
        return self.embodiment is not None and bool(chain_shas(self.embodiment))

    @property
    def complete(self) -> bool:
        return not self.needs_description or self.description is not None


class LinkEndpoint:
    """Under the server lock: who owns the link, request-id idempotency, and the /sim/* handlers."""

    def __init__(self, srv):
        self._srv = srv
        self._last_rid = None
        self._last_reply = None
        self._last_sim = -float("inf")
        self.n_polling = 0
        self.owner: dict | None = None
        self.epoch = 0
        self.episode_open = False
        self.owner_requests = 0
        self._warned_legacy = False
        self._warned_foreign: set = set()
        self.clock = time.monotonic
        self.declared = DeclaredRobot()
        self.features: tuple[str, ...] = ()
        self.helloed = False

    def robot(self) -> DeclaredRobot:
        """Under the lock: the owning backend's embodiment and robot description."""
        return self.declared

    def connected(self) -> bool:
        return self.n_polling > 0 or self.clock() - self._last_sim < CONNECTED_WINDOW_S

    def liveness(self) -> dict:
        """Under the lock: api/state's link: connected, seconds since the last /sim/*, features."""
        seen = self._last_sim
        out = {"connected": self.connected(),
               "last_seen_s": None if seen == -float("inf") else round(self.clock() - seen, 3)}
        if self.features:
            out["features"] = list(self.features)
        return out

    def grades(self) -> bool:
        """Under the lock: the owning backend grades a state on a grade wait."""
        return GRADE in self.features

    def client_json(self) -> dict | None:
        o = self.owner
        if o is None:
            return None
        cid = o["client_id"]
        return {"client": o["client"], "client_id": None if cid is None else str(cid)[:8],
                "since": o["since"]}

    def _touch(self) -> None:
        self._last_sim = self.clock()

    def _note_legacy(self, cid) -> None:
        if cid is None and not self._warned_legacy:
            self._warned_legacy = True
            self._srv.log("warn", "a sim client sent no client_id (a legacy client); it cannot be "
                          "told apart from other legacy clients")

    def _set_owner(self, client, cid) -> None:
        same = self.owner is not None and self.owner["client_id"] == cid
        since = self.owner["since"] if same else time.time()
        self.owner = {"client": client, "client_id": cid, "since": since}
        self.epoch += 1
        self.episode_open = False
        self.owner_requests = 0
        self._warned_foreign = set()

    def _owner_label(self) -> str:
        return f"{self.owner['client']} {short(self.owner['client_id'])}"

    def _saving(self) -> bool:
        """The owner ended its job's episode and has not sent /sim/saved: it is saving the record."""
        ep = self._srv.episode
        return not self.episode_open and ep.jobs.active is not None and ep.job_ended

    def _window(self) -> float | None:
        """How long a silent owner still counts as live: mid-episode, or saving; else None."""
        if self.episode_open:
            return OWNER_WINDOW_S
        return SAVE_WINDOW_S if self._saving() else None

    def _owner_live(self) -> bool:
        """The owner is mid-episode or saving, and still talking (or long-polling right now)."""
        window = self._window()
        if window is None:
            return False
        return self.n_polling > 0 or self.clock() - self._last_sim < window

    def _owner_seen(self) -> str:
        """Who owns the link and how far it got: unchanged while the owner makes no request."""
        return f"{self.epoch}.{self.owner_requests}"

    def _retry_after(self) -> float:
        """Seconds until a silent owner stops counting as live (an orphan long poll included)."""
        window = self._window() or 0.0
        if self.n_polling > 0:
            return POLL_TIMEOUT_S + window
        return round(max(0.0, window - (self.clock() - self._last_sim)), 3)

    def _refuse_hello(self, client, cid, seen) -> SessionConflict:
        """Under the lock: the 409 for a hello while another client owns the link."""
        progressed = seen is not None and seen != self._owner_seen()
        ago = self.clock() - self._last_sim
        if progressed:
            message = (f"another sim client ({self._owner_label()}) owns this session and is "
                       f"live: it made requests while this client waited. Stop one of them, or "
                       'send /sim/hello with "takeover": true')
            body = {"ok": False, "error": message}
        else:
            job = self._srv.episode.jobs.active
            doing = (f"saving job {job['job_id']} ({job['task']} episode {job['episode']})"
                     if self._saving() else "mid-episode")
            message = (f"another sim client ({self._owner_label()}) owns this session; "
                       f"it is {doing} and was active {ago:.1f} s ago. Stop it, or "
                       'send /sim/hello with "takeover": true')
            body = {"ok": False, "error": message, "owner_seen": self._owner_seen(),
                    "retry_after_s": self._retry_after()}
        if seen is None or progressed:
            self._srv.log("warn", f"refused /sim/hello from {client} {short(cid)}: {message}")
        return SessionConflict(message, body)

    def _guard(self, body, rid, path) -> int:
        """Refuse (409, no side effects) a non-hello request from anyone but the owner."""
        cid = body.get("client_id")
        self._note_legacy(cid)
        if self.owner is None:
            if cid is not None:
                self._set_owner(body.get("client") or "unknown", cid)
            return self.epoch
        if cid == self.owner["client_id"]:
            return self.epoch
        if cid not in self._warned_foreign:
            self._warned_foreign.add(cid)
            self._srv.log("warn", f"refused {path} from sim client {short(cid)}: "
                          f"{self._owner_label()} owns this session")
        raise SessionConflict(NOT_OWNER, {"request_id": rid, "error": NOT_OWNER})

    def _error_reply(self, rid, path, exc) -> dict:
        message = f"{path} failed: {exc!r}"
        self._srv.log("error", message)
        return {"request_id": rid, "error": message}

    def _dedupe(self, rid):
        """The cached reply for a repeated request_id, else claim rid as the latest (None)."""
        if rid is None:
            return None
        last = self._last_rid
        if isinstance(rid, int) and isinstance(last, int) and rid < last:
            return {"request_id": rid, "error": f"request_id {rid} is older than {last}; "
                    "a restarted client must send /sim/hello first"}
        if rid == last:
            end = time.monotonic() + POLL_TIMEOUT_S + 10.0
            while self._last_reply is None and self._last_rid == rid:
                left = end - time.monotonic()
                if left <= 0:
                    break
                self._srv.cond.wait(min(left, 0.5))
            if self._last_rid == rid and self._last_reply is not None:
                return self._last_reply
            return {"request_id": rid, "wait": True}
        self._last_rid, self._last_reply = rid, None
        return None

    def _finish(self, rid, reply) -> None:
        if rid is not None and self._last_rid == rid:
            self._last_reply = reply
        self._touch()
        self._srv.cond.notify_all()

    def _enter(self, body, rid, path):
        """Under the lock: (owner epoch, cached reply or None) for a guarded request."""
        epoch = self._guard(body, rid, path)
        self.owner_requests += 1
        self._touch()
        return epoch, self._dedupe(rid)

    def hello(self, body) -> dict:
        srv = self._srv
        protocol = body.get("protocol", 1)
        if protocol != LINK_PROTOCOL_VERSION:
            raise ValueError(f"unsupported protocol {protocol!r}; this server speaks "
                             f"{LINK_PROTOCOL_VERSION}")
        features = body.get("features", [])
        if not isinstance(features, list) or not all(isinstance(f, str) for f in features):
            raise ValueError(f"features must be a list of names, got {features!r}")
        spec = hello_embodiment(body, srv.config.embodiments)
        declared = DeclaredRobot(spec)
        runtime = None if rt.key(declared) == srv.embodiment_key else rt.runtime(declared)
        srv.camera_role_refusal(runtime)
        cid, client = body.get("client_id"), body.get("client")
        took_over = None
        with srv.cond:
            self._note_legacy(cid)
            other = self.owner is not None and self.owner["client_id"] != cid
            seen = body.get("owner_seen")
            moved_on = seen is not None and seen != self._owner_seen()
            if other and (self._owner_live() or moved_on):
                if not body.get("takeover"):
                    raise self._refuse_hello(client, cid, seen)
                took_over = self._owner_label()
            gone = other or cid is None
            left_open = self.episode_open or srv.episode.jobs.active is not None
            why = (EVICTED if took_over else LOST) if gone and left_open else None
            self._set_owner(client, cid)
            self._touch()
            self._last_rid = body.get("request_id")
            closed = srv.link_connected(why)
            end = srv.episode.last_end
            self.declared = declared
            self.features = tuple(features)
            self.helloed = True
            srv.declare(declared, runtime)
            reply = {"ok": True, "protocol": LINK_PROTOCOL_VERSION}
            self._last_reply = reply
            srv.cond.notify_all()
        rt.warm_async(runtime, srv.log)
        if took_over is not None:
            srv.log("warn", f"sim client {client} {short(cid)} took over from {took_over}")
        if closed is not None:
            srv.log("error", f"job {closed['job_id']} ({closed['task']} episode "
                    f"{closed['episode']}) ended {end['outcome']}: {why}")
            srv.trace_episode("lost", job_id=closed["job_id"], task=closed["task"],
                              episode=closed["episode"], outcome=end["outcome"])
        srv.log("info", f"sim client connected: {client} {short(cid)}"
                + ("" if spec is None else f", embodiment {spec.name}"))
        return reply

    def robot_description(self, header, blobs) -> dict:
        """/sim/robot: the description a chain arm's kinematics come from, checked by its sha."""
        srv = self._srv
        rid = header.get("request_id")
        with srv.cond:
            _, cached = self._enter(header, rid, "/sim/robot")
            if cached is not None:
                return cached
            declared = self.declared
        try:
            description, meshes = parse_robot(header, blobs)
            sha = description_sha256(description, meshes)
            want = chain_shas(declared.embodiment) if declared.embodiment is not None else set()
            error = None if sha in want else (
                f"description sha256 {sha} is not the one embodiment {declared.name} names "
                f"({', '.join(sorted(want)) or 'none: its kinematics are not a chain'})")
            if error is None:
                declared = DeclaredRobot(declared.embodiment, description, meshes, sha)
                runtime = rt.runtime(declared)
                srv.camera_role_refusal(runtime)
        except ValueError as exc:
            error = str(exc)
        with srv.cond:
            if self._last_rid != rid:
                return {"request_id": rid, "ok": False, "error": "superseded by a newer request"}
            if error is not None:
                reply = {"request_id": rid, "ok": False, "error": f"/sim/robot refused: {error}"}
            else:
                self.declared = declared
                srv.declare(declared, runtime)
                reply = {"request_id": rid, "ok": True, "description_sha256": sha}
            self._finish(rid, reply)
        if error is not None:
            srv.log("warn", reply["error"])
        else:
            rt.warm_async(runtime, srv.log)
            srv.log("info", f"robot description {sha[:12]}: {len(description.links)} links, "
                    f"{len(description.joints)} joints, {len(meshes)} meshes")
        return reply

    def _description_missing(self, rid, path):
        """Under the lock: the refusal for a request a chain robot sends before /sim/robot."""
        if self.declared.complete:
            return None
        return {"request_id": rid, "ok": False,
                "error": f"{path} refused: embodiment {self.declared.name} has chain kinematics; "
                         "send /sim/robot first"}

    def episode(self, body) -> dict:
        srv = self._srv
        rid = body.get("request_id")
        with srv.cond:
            _, cached = self._enter(body, rid, "/sim/episode")
            if cached is not None:
                return cached
            missing = self._description_missing(rid, "/sim/episode")
            if missing is not None:
                self._finish(rid, missing)
                srv.log("warn", missing["error"])
                return missing
            try:
                self.episode_open = True
                srv.begin_episode(body.get("episode"), body.get("task") or "",
                                  body.get("instruction") or "", body.get("max_steps"),
                                  body.get("control_dt"), keep_scene=False)
                warning = srv.episode.bind_job(body.get("job_id"), body.get("task") or "",
                                               srv.results.catalog is not None)
                if warning:
                    srv.log("warn", warning)
                reply = {"ok": True}
            except Exception as exc:  # noqa: BLE001
                reply = self._error_reply(rid, "/sim/episode", exc)
            self._finish(rid, reply)
        if "error" not in reply:
            srv.log("info", f"episode {body.get('episode')} begins: "
                    f"{body.get('instruction') or ''}")
        return reply

    def step(self, header, blobs) -> dict:
        srv = self._srv
        rid = header.get("request_id")
        with srv.cond:
            epoch, cached = self._enter(header, rid, "/sim/step")
            if cached is not None:
                return cached
            srv.executor.step_in_flight()
            dof = srv.embodiment.dof
        try:
            obs = unpack_observation(header, blobs, dof)
        except Exception as exc:  # noqa: BLE001
            with srv.cond:
                reply = self._error_reply(rid, "/sim/step", exc)
                if self._last_rid == rid:
                    self._finish(rid, reply)
                return reply
        obs.gt = None
        obs.header = {k: v for k, v in obs.header.items() if k != "gt"}
        with srv.cond:
            if self._last_rid != rid or self.epoch != epoch:
                return {"request_id": rid, "wait": True}
            try:
                self.episode_open = True
                srv.ingest(obs)
                reply = self._next(rid, epoch)
            except Exception as exc:  # noqa: BLE001
                reply = self._error_reply(rid, "/sim/step", exc)
            self._finish(rid, reply)
            return reply

    def poll(self, body) -> dict:
        srv = self._srv
        rid = body.get("request_id")
        with srv.cond:
            epoch, cached = self._enter(body, rid, "/sim/poll")
            if isinstance(body.get("graded"), dict):
                srv.episode.graded(body["graded"])
            if cached is not None:
                return cached
            try:
                reply = self._next(rid, epoch)
            except Exception as exc:  # noqa: BLE001
                reply = self._error_reply(rid, "/sim/poll", exc)
            self._finish(rid, reply)
            return reply

    def _next(self, rid, epoch) -> dict:
        srv = self._srv
        self.n_polling += 1
        try:
            return srv.executor.next_action(
                rid, lambda: (rid is None or self._last_rid == rid) and self.epoch == epoch,
                time.monotonic() + POLL_TIMEOUT_S)
        finally:
            self.n_polling -= 1

    def end(self, body) -> dict:
        srv = self._srv
        rid = body.get("request_id")
        with srv.cond:
            _, cached = self._enter(body, rid, "/sim/end")
            if cached is not None:
                return cached
            self.episode_open = False
            result = jsonable(body.get("result"))
            out, warning = srv.end_episode(result, body.get("reason"))
            reply = {"ok": True}
            self._finish(rid, reply)
        self._ended(body, out, warning)
        return reply

    def _ended(self, body, out, warning) -> None:
        srv = self._srv
        if warning:
            srv.log("warn", warning)
        srv.log("info", f"episode {body.get('episode')} over ({body.get('reason')}): "
                f"{json.dumps(srv.episode.result)}")
        srv.trace_episode("end", reason=body.get("reason"), result=srv.episode.result)
        if out is not None and out[0] is not None:
            try:
                srv.results.write_teleop(*out)
                srv.log("info", f"teleop record written: {out[0]} ({len(out[1]['chunks'])} "
                        "chunks)")
            except (OSError, ValueError) as e:
                srv.log("warn", f"teleop record {out[0]} not written: {e}")

    def scene(self, header, blobs) -> dict:
        srv = self._srv
        rid = header.get("request_id")
        with srv.cond:
            _, cached = self._enter(header, rid, "/sim/scene")
            if cached is not None:
                return cached
            reply = {"request_id": rid, "ok": True, "objects": 0, "ignored": SCENE_IGNORED}
            episode = header.get("episode")
            first = srv.world.ignore_scene((episode, srv.episode.token))
            self._finish(rid, reply)
        if first:
            srv.log("info", f"/sim/scene for episode {episode} ignored: standard mode builds "
                    "the scene from the depth cloud")
        return reply

    def catalog(self, body) -> dict:
        srv = self._srv
        rid = body.get("request_id")
        with srv.cond:
            _, cached = self._enter(body, rid, "/sim/catalog")
            if cached is not None:
                return cached
            missing = self._description_missing(rid, "/sim/catalog")
            if missing is not None:
                self._finish(rid, missing)
                srv.log("warn", missing["error"])
                return missing
            try:
                cat = srv.results.set_catalog(body)
            except ValueError as exc:
                reply = {"request_id": rid, "ok": False, "error": f"/sim/catalog refused: {exc}"}
                self._finish(rid, reply)
                srv.log("warn", reply["error"])
                return reply
            reply = self._hello_check({"request_id": rid, "ok": True})
            self._finish(rid, reply)
        where = cat.root if cat.root is not None else f"{cat.output_dir} (not on this filesystem)"
        srv.log("info", f"benchmark catalog: {len(cat.tasks)} tasks, {cat.episodes_per_task} "
                f"episode(s) each, results in {where}")
        return reply

    def _hello_check(self, reply: dict) -> dict:
        """Under the lock: reply, asking for a hello when none is on record."""
        if not self.helloed:
            reply["hello_required"] = True
        return reply

    def next(self, body) -> dict:
        """/sim/next: long-poll for the job the gateway selects."""
        srv = self._srv
        ep = srv.episode
        rid = body.get("request_id")
        loaded = body.get("loaded")
        with srv.cond:
            epoch, cached = self._enter(body, rid, "/sim/next")
            if cached is not None:
                return cached
            if loaded is not None and not isinstance(loaded, str):
                reply = {"request_id": rid, "error": "loaded must be a task name or null"}
            elif srv.results.catalog is None:
                reply = {"request_id": rid, "error": "no catalog; send /sim/catalog first"}
            else:
                failed = body.get("failed")
                message = ep.next_requested(loaded, failed if isinstance(failed, dict) else None)
                if message:
                    srv.log("error", message)
                reply = self._hello_check({"request_id": rid, "job": self._wait_job(rid, epoch)})
            self._finish(rid, reply)
        job = reply.get("job")
        if job is not None:
            srv.log("info", f"sim took job {job['job_id']}: {json.dumps(job)}")
        return reply

    def _wait_job(self, rid, epoch):
        srv = self._srv
        ep = srv.episode
        deadline = time.monotonic() + POLL_TIMEOUT_S
        self.n_polling += 1
        try:
            while True:
                if self._last_rid != rid or self.epoch != epoch:
                    return None
                job = ep.take_job()
                if job is not None:
                    return wire_job(job)
                left = deadline - time.monotonic()
                if left <= 0:
                    return None
                srv.cond.wait(left)
        finally:
            self.n_polling -= 1

    def saved(self, body) -> dict:
        """/sim/saved: the episode_results.jsonl record RoboLab appended."""
        srv = self._srv
        ep = srv.episode
        rid = body.get("request_id")
        with srv.cond:
            _, cached = self._enter(body, rid, "/sim/saved")
            if cached is not None:
                return cached
            record = body.get("record")
            task, episode = body.get("task"), body.get("episode")
            if (not isinstance(record, dict) or not isinstance(task, str)
                    or isinstance(episode, bool) or not isinstance(episode, int)):
                reply = {"request_id": rid, "ok": False,
                         "error": "/sim/saved needs task, an integer episode and a record"}
            else:
                job_id = body.get("job_id")
                srv.results.save_record(task, episode, record, ep.run_of(job_id))
                ep.job_saved(job_id if isinstance(job_id, int) else None, task)
                reply = {"request_id": rid, "ok": True}
            self._finish(rid, reply)
        if reply.get("ok"):
            srv.log("info", f"saved {task} episode {episode}: success={record.get('success')}")
            srv.trace_episode("saved", task=task, episode=episode,
                              success=record.get("success"), score=record.get("score"))
            srv.close_episode_trace()
        else:
            srv.log("warn", reply["error"])
        return reply
