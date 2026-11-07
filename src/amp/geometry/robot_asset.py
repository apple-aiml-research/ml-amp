#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The packaged DROID robot asset: link meshes per frame and the Robotiq finger linkage."""

from __future__ import annotations

import functools
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from amp.link import wire

ROBOT_ASSET = Path(__file__).resolve().parent / "assets" / "droid_robot.tlf"
ROBOT_FORMAT = "teleop-robot-1"


@dataclass(frozen=True)
class RobotLink:
    """One link: its frame ("link<i>" or "ee"), mesh, and for a finger its joint to its parent."""

    name: str
    frame: str
    verts: np.ndarray
    faces: np.ndarray
    parent: str | None = None
    pivot: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] | None = None
    gearing: float = 0.0


@dataclass(frozen=True)
class RobotAsset:
    """The robot's links in asset order and the finger angle at which the gripper is closed."""

    links: dict[str, RobotLink]
    closed_angle: float
    source_sha256: str = ""

    @classmethod
    def load(cls, path: str | Path = ROBOT_ASSET) -> RobotAsset:
        """Parse a teleop-robot-1 TLF1 file."""
        header, blobs = wire.unpack_frame(Path(path).read_bytes())
        if header.get("format") != ROBOT_FORMAT:
            raise ValueError(f"{path}: not a {ROBOT_FORMAT} asset")
        links = {}
        for lk in header["links"]:
            links[lk["name"]] = RobotLink(
                name=lk["name"], frame=lk["frame"],
                verts=np.asarray(blobs[lk["verts"]], dtype=np.float64),
                faces=np.asarray(blobs[lk["faces"]]),
                parent=lk.get("parent"), pivot=tuple(lk.get("pivot", (0.0, 0.0, 0.0))),
                axis=None if lk.get("axis") is None else tuple(lk["axis"]),
                gearing=float(lk.get("gearing", 0.0)))
        return cls(links, float(header["closed_angle"]), str(header.get("source_sha256", "")))

    def finger_links(self) -> list[str]:
        """The ee-frame links that move with the fingers (those with a parent), in asset order."""
        return [n for n, lk in self.links.items() if lk.frame == "ee" and lk.parent]


@functools.cache
def robot_asset() -> RobotAsset:
    """The packaged asset, parsed once."""
    return RobotAsset.load(ROBOT_ASSET)
