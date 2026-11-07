#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The robot's collision spheres: each rigidly attached to a link frame, posed by FK."""

from __future__ import annotations

import functools
import json
from pathlib import Path

import numpy as np

from amp.geometry import kinematics as K

SPHERES_PATH = Path(__file__).resolve().parents[1] / "assets" / "robot_spheres.json"
SPHERES_FORMAT = "teleop-spheres-1"
FRAME_INDEX = {**{f"link{i}": i for i in range(9)}, "flange": 8, "ee": 9}
EE_FRAME = 9


def fk_frames_batch(qs, T_flange_ee) -> np.ndarray:
    """(N,10,4,4): robot root, panda_link1..7, flange, ee for every row of qs."""
    chain = K.fk_chain_batch(qs)
    out = np.empty((len(chain), 10, 4, 4))
    out[:, 0] = np.eye(4)
    out[:, 1:9] = chain
    out[:, 9] = chain[:, 7] @ np.asarray(T_flange_ee, dtype=np.float64)
    return out


class RobotSpheres:
    """The robot's collision spheres; links maps a name to (frame name, (n, 4) spheres)."""

    @classmethod
    def builtin(cls) -> RobotSpheres:
        """The coarse fixed model from link AABBs, for tests that must not depend on the asset."""
        from amp.geometry.collision.coarse import builtin_spheres

        return builtin_spheres()

    def __init__(self, links: dict, source: str = ""):
        self.source = source
        self.link_names: list[str] = []
        self.link_frames: list[str] = []
        frame, local, radius, link = [], [], [], []
        for name, (frame_name, spheres) in links.items():
            if frame_name not in FRAME_INDEX:
                raise ValueError(f"link {name!r}: unknown frame {frame_name!r}")
            s = np.asarray(spheres, dtype=np.float64).reshape(-1, 4)
            if not np.all(np.isfinite(s)) or np.any(s[:, 3] <= 0):
                raise ValueError(f"link {name!r}: spheres need finite centres and radii > 0")
            idx = len(self.link_names)
            self.link_names.append(str(name))
            self.link_frames.append(frame_name)
            frame += [FRAME_INDEX[frame_name]] * len(s)
            local.append(s[:, :3])
            radius.append(s[:, 3])
            link += [idx] * len(s)
        if not frame:
            raise ValueError("a sphere model needs at least one sphere")
        self.frame = np.array(frame, dtype=np.int64)
        self.local = np.concatenate(local)
        self.radius = np.concatenate(radius)
        self.link = np.array(link, dtype=np.int64)

    @classmethod
    def from_dict(cls, data: dict, source: str = "") -> RobotSpheres:
        """Parse teleop-spheres-1 JSON, the format of assets/robot_spheres.json."""
        if not isinstance(data, dict) or data.get("format") != SPHERES_FORMAT:
            raise ValueError(f"not a {SPHERES_FORMAT} sphere model")
        raw = data.get("links")
        if not isinstance(raw, dict) or not raw:
            raise ValueError("sphere model needs a non-empty 'links' object")
        links = {}
        for name, spec in raw.items():
            if not isinstance(spec, dict) or "frame" not in spec or "spheres" not in spec:
                raise ValueError(f"link {name!r} needs 'frame' and 'spheres'")
            links[name] = (spec["frame"], spec["spheres"])
        return cls(links, source=source or str(data.get("source", "")))

    @classmethod
    def load(cls, path=None) -> RobotSpheres:
        """The fitted model at path, or the packaged one (built once) when path is None."""
        if path is None:
            return default_spheres()
        path = Path(path)
        return cls.from_dict(json.loads(path.read_text()), str(path))

    def __len__(self) -> int:
        return len(self.radius)

    def world_batch(self, qs, T_flange_ee) -> np.ndarray:
        """Sphere centres (N,S,3) in the robot-root frame."""
        F = fk_frames_batch(qs, T_flange_ee)[:, self.frame]
        return np.einsum("nsij,sj->nsi", F[..., :3, :3], self.local) + F[..., :3, 3]

    def world(self, q, T_flange_ee) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """(centres (S,3), radii (S,), link index (S,)) at one configuration."""
        return self.world_batch(q, T_flange_ee)[0], self.radius.copy(), self.link.copy()


@functools.cache
def default_spheres() -> RobotSpheres:
    """The packaged fitted sphere model (assets/robot_spheres.json), built once."""
    return RobotSpheres.from_dict(json.loads(SPHERES_PATH.read_text()), str(SPHERES_PATH))
