#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The latest observation: frames, depth, robot state, frame seq, tool calibration, table height."""

from __future__ import annotations

import copy
import io

import numpy as np

from amp.geometry import kinematics as kin
from amp.geometry.embodiment import Embodiment
from amp.link import pointcloud, wire
from amp.link.imaging import mime_for
from amp.perception.cloud import CLOUD_FRAMES, estimate_table_z, fuse_frames
from amp.perception.occupancy import OccupancyMemory
from amp.robot_server.config import _finite
from amp.robot_server.errors import NotFound, SessionConflict

TABLE_CLOUD_MAX_DEPTH_M = 3.0
GROUP_POSE_TOL = 1e-4
TABLE_Z0 = 0.0


def tolist(x):
    return None if x is None else np.asarray(x, dtype=np.float64).tolist()


class Observations:
    """Under the server lock: the newest observation and what is derived from it."""

    def __init__(self, *, embodiment: Embodiment, log):
        self._log = log
        self.base_memory = OccupancyMemory()
        self.set_embodiment(embodiment)
        self.table_z = TABLE_Z0
        self.obs = None
        self.frame_seq = 0
        self.cam_seq: dict[str, int] = {}
        self.calibrated_seq = None
        self.table_pending = False
        self._cloud_cache = None

    def set_embodiment(self, emb: Embodiment) -> None:
        """The robot's runtime, whose end effectors' tools this calibrates per frame."""
        self.embodiment = emb
        self.calibrated_seq = None
        self._cloud_cache = None
        self.base_memory.reset(self.base_memory.token)
        self._warned_calibrate = False

    @property
    def tool(self) -> kin.ToolModel:
        """The primary end effector's tool, owned (and calibrated in place) by the runtime."""
        return self.embodiment.tool()

    def arm_q(self, q, arm: str | None = None):
        """The joints of one arm (None: the primary end effector's) in a full proprio q."""
        emb = self.embodiment
        if len(emb.spec.arms) == 1:
            return q
        return emb.split(q)[emb.ee().arm if arm is None else arm]

    def reset(self, token=None) -> None:
        """A new episode (token): no observation, the default table height, no memory."""
        self.obs = None
        self.base_memory.reset(token)
        self.table_z = TABLE_Z0
        self.table_pending = True

    def ingest(self, obs) -> None:
        self.obs = obs
        self.frame_seq += 1
        for name in obs.cameras:
            self.cam_seq[name] = self.frame_seq
        self.embodiment.pose_groups(obs.q, GROUP_POSE_TOL)
        self.calibrate()

    def calibrate(self) -> None:
        obs = self.obs
        emb = self.embodiment
        try:
            first = emb.ee().name
            for e in emb.spec.end_effectors:
                pose = obs.ee.get(e.name)
                if pose is None and e.name == first:
                    pose = (obs.ee_pos, obs.ee_quat)
                if pose is not None:
                    emb.tool(e.name).calibrate(self.arm_q(obs.q, e.arm), *pose)
            self.calibrated_seq = self.frame_seq
        except Exception as e:  # noqa: BLE001
            if not self._warned_calibrate:
                self._warned_calibrate = True
                self._log("warn", f"tool calibration failed: {e}")

    def estimate_table(self, boxes) -> float | None:
        """limits.table_z from this episode's first depth cloud; the new z when it changed."""
        clouds = []
        for fr in self.obs.cameras.values():
            if fr.depth is not None and fr.depth_model is not None:
                xyz, _ = pointcloud.backproject(fr.depth, fr.depth_model.K,
                                                fr.depth_model.T_world_cam,
                                                max_depth=TABLE_CLOUD_MAX_DEPTH_M)
                clouds.append(xyz)
        if not clouds:
            return None
        self.table_pending = False
        est = estimate_table_z(np.concatenate(clouds), boxes, embodiment=self.embodiment)
        if est is None:
            self._log("info", f"too few table points in the first depth cloud; table_z stays "
                      f"{self.table_z * 1000:+.1f} mm")
            return None
        z, n = est
        changed = abs(z - self.table_z) > 1e-4
        self.table_z = z
        if changed:
            self._log("info", f"table surface at z = {z * 1000:+.1f} mm ({n} points)")
        return z

    def observed_tcp(self, ee: str | None = None) -> np.ndarray:
        """The TCP of an end effector (None: the primary) at its observed ee pose."""
        obs, emb = self.obs, self.embodiment
        name = emb.ee(ee).name
        pose = obs.ee.get(name)
        if pose is None:
            if name != emb.ee().name:
                raise KeyError(f"the observation has no pose for end effector {name!r}")
            pose = (obs.ee_pos, obs.ee_quat)
        return emb.tool(name).tcp_from_ee(kin.make_pose(*pose))

    def observed_tcp7(self, ee: str | None = None) -> np.ndarray:
        return np.asarray(kin.pose7(self.observed_tcp(ee)), dtype=np.float64)

    def gripper_of(self, ee: str | None = None) -> float | None:
        """An end effector's measured gripper (None: the primary's obs.gripper); None for a tool
        or when the observation has no reading for it."""
        obs, emb = self.obs, self.embodiment
        name = emb.ee(ee).name
        if emb.gripper(name) is None:
            return None
        if name == emb.ee().name:
            return float(obs.grippers.get(name, obs.gripper))
        g = obs.grippers.get(name)
        return None if g is None else float(g)

    def live(self):
        """The observation; 409 before the first."""
        if self.obs is None:
            raise SessionConflict("no robot observation yet")
        return self.obs

    def flange_ee(self):
        """Each end effector's (calibrated) T_flange_ee: one array for a one-arm, one-hand robot."""
        emb = self.embodiment
        T = {n: np.array(emb.tool(n).T_flange_ee, dtype=np.float64) for n in emb.end_effectors}
        if len(emb.spec.arms) == 1 and len(T) == 1:
            return next(iter(T.values()))
        return T

    def tool_copy(self, ee: str | None = None) -> kin.ToolModel:
        """A copy of an end effector's tool (None: the primary) sharing its arm (planners check it)."""
        tool = self.embodiment.tool(ee)
        arm = getattr(tool, "arm", None)
        return copy.deepcopy(tool, {} if arm is None else {id(arm): arm})

    def tool_json(self) -> dict:
        tool = self.tool
        res = getattr(tool, "last_residual", None)
        return {"tcp_offset": float(tool.tcp_offset), "finger_axis": tool.finger_axis,
                "calibrated": bool(tool.calibrated),
                "residual_mm": None if res is None else float(res[0]) * 1000.0,
                "residual_deg": None if res is None else float(np.degrees(float(res[1])))}

    def camera_json(self) -> dict:
        """{cam: intrinsics, extrinsics, depth size and seq}: the cameras of api/state."""
        out = {}
        for name, fr in (self.obs.cameras.items() if self.obs is not None else ()):
            m, dm = fr.model, fr.depth_model
            has = dm is not None
            out[name] = {"width": int(m.width), "height": int(m.height), "K": tolist(m.K),
                         "T_world_cam": tolist(m.T_world_cam),
                         "depth_width": int(dm.width) if has else None,
                         "depth_height": int(dm.height) if has else None,
                         "K_depth": tolist(dm.K) if has else None,
                         "seq": self.cam_seq.get(name)}
        return out

    def frame(self, cam):
        """(image bytes, mime, seq) of cam's latest frame, or None."""
        if self.obs is None or cam not in self.obs.cameras:
            return None
        fr = self.obs.cameras[cam]
        return fr.image, mime_for(fr.encoding), self.cam_seq.get(cam)

    def depth(self, cam):
        """(depth array, seq) of cam; NotFound for an unknown camera or one without depth."""
        obs = self.obs
        fr = obs.cameras.get(cam)
        if fr is None:
            raise NotFound(f"unknown camera {cam!r}; the observation has {sorted(obs.cameras)}")
        if fr.depth is None:
            raise NotFound(f"camera {cam!r} has no depth")
        return fr.depth, self.cam_seq.get(cam)

    def cloud_request(self, cams, stride, max_depth, crop, frame="root"):
        """Under the lock: (cache key, frames, cached bytes or None) of an api/cloud request."""
        stride = int(stride)
        if stride < 1 or stride > 64:
            raise ValueError("stride must be in [1, 64]")
        if max_depth is not None:
            max_depth = _finite(max_depth, "max_depth", 0.01, 100.0)
        if frame not in CLOUD_FRAMES:
            raise ValueError(f"frame must be one of {list(CLOUD_FRAMES)}")
        obs, seq = self.obs, self.frame_seq
        if frame == "world" and obs is not None and obs.T_world_base is None:
            raise ValueError("frame=world needs a robot whose frames carry T_world_base")
        if obs is None:
            frames = {}
        else:
            names = list(obs.cameras) if not cams else list(cams)
            unknown = [c for c in names if c not in obs.cameras]
            if cams and unknown:
                raise ValueError(f"unknown cameras: {unknown}")
            frames = {c: obs.cameras[c] for c in names}
        key = (seq, tuple(frames), stride, max_depth, bool(crop), frame)
        cached = self._cloud_cache[1] if self._cloud_cache and self._cloud_cache[0] == key \
            else None
        return key, frames, cached

    @staticmethod
    def pack_cloud(key, frames, embodiment=None, T_world_base=None) -> bytes:
        """Outside the lock: TLF1 bytes of the back-projected depth of frames."""
        seq, _, stride, max_depth, crop, frame = key
        xyz, rgb, per_cam = fuse_frames(frames, stride=stride, max_depth=max_depth, crop=crop,
                                        embodiment=embodiment, T_world_base=T_world_base,
                                        frame=frame)
        head = {"seq": seq, "count": len(xyz), "per_cam": per_cam}
        if T_world_base is not None:
            head["frame"] = frame
        return wire.pack_frame(head,
                               {"xyz": np.ascontiguousarray(xyz),
                                "rgb": np.ascontiguousarray(rgb)})

    def cache_cloud(self, key, data) -> None:
        self._cloud_cache = (key, data)

    @staticmethod
    def pick(fr, cam, u, v) -> dict:
        """The 3D point under full-resolution pixel (u, v) of camera frame fr (or a reason)."""
        if fr is None:
            return {"ok": False, "message": f"no camera {cam!r}"}
        if fr.depth is None or fr.depth_model is None:
            return {"ok": False, "message": f"camera {cam!r} has no depth"}
        dm = fr.depth_model
        u_d, v_d = u * dm.width / fr.model.width, v * dm.height / fr.model.height
        point, depth, normal = pointcloud.pick_point(fr.depth, dm.K, dm.T_world_cam, u_d, v_d,
                                                     window=5)
        if point is None:
            return {"ok": False, "message": "no valid depth near that pixel"}
        return {"ok": True, "point": tolist(point), "depth": float(depth),
                "normal": tolist(normal), "cam": cam}


def npy_bytes(arr) -> bytes:
    buf = io.BytesIO()
    np.save(buf, np.ascontiguousarray(arr), allow_pickle=False)
    return buf.getvalue()
