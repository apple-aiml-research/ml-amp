#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""EmbodimentView: an Embodiment seen from one end effector, its other arms held as obstacles."""

from __future__ import annotations

from collections.abc import Mapping
from typing import TYPE_CHECKING

import numpy as np

from amp.geometry.embodiment.spheres import COVER_CELL

if TYPE_CHECKING:
    from amp.geometry.collision.peers import PeerSet
    from amp.geometry.embodiment.model import Embodiment


class EmbodimentView:
    """Duck-types Embodiment: ee/arm/tool/gripper/spheres default to ee_name, the rest delegates."""

    def __init__(self, base: Embodiment, ee_name: str,
                 peer_q: Mapping[str, np.ndarray] | None = None,
                 peer_held: Mapping[str, str] | None = None):
        self.base = base
        self.ee_name = ee_name
        self.arm_name = base.ee(ee_name).arm
        self.peers: tuple[str, ...] = tuple(a.name for a in base.spec.arms
                                            if a.name != self.arm_name)
        self.peer_q: dict[str, np.ndarray] | None = None
        if peer_q is not None:
            if not isinstance(peer_q, Mapping):
                peer_q = base.split(peer_q)
            missing = [a for a in self.peers if a not in peer_q]
            if missing:
                raise ValueError(f"{base.name}: no q for peer arms {missing}")
            self.peer_q = {a: np.array(peer_q[a], dtype=np.float64).reshape(-1)
                           for a in self.peers}
        self.peer_held = {a: str(o) for a, o in (peer_held or {}).items()
                          if a in self.peers and o is not None}
        self._peer_spheres: PeerSet | None = None
        self._peer_stamp: tuple = ()

    def __getattr__(self, name: str):
        base = self.__dict__.get("base")
        if base is None:
            raise AttributeError(name)
        return getattr(base, name)

    def __eq__(self, other) -> bool:
        return (isinstance(other, EmbodimentView) and other.base is self.base
                and other.ee_name == self.ee_name)

    def __hash__(self) -> int:
        return hash((id(self.base), self.ee_name))

    def __repr__(self) -> str:
        held = "" if self.peer_q is None else f", peers held: {list(self.peer_q)}"
        holding = f", peers holding: {self.peer_held}" if self.peer_held else ""
        return f"EmbodimentView({self.base.name}, {self.ee_name}{held}{holding})"

    def view(self, ee: str | None = None) -> EmbodimentView:
        return self if ee is None else self.base.view(ee)

    def arm(self, name: str | None = None):
        return self.base.arm(self.arm_name if name is None else name)

    def ee(self, name: str | None = None):
        return self.base.ee(self.ee_name if name is None else name)

    def tool(self, ee: str | None = None):
        return self.base.tool(self.ee_name if ee is None else ee)

    def new_tool(self, ee: str | None = None):
        return self.base.new_tool(self.ee_name if ee is None else ee)

    def gripper(self, ee: str | None = None):
        return self.base.gripper(self.ee_name if ee is None else ee)

    def tcp_pose(self, q, ee: str | None = None) -> np.ndarray:
        return self.tool(ee).fk_tcp(q)

    def ee_pose(self, q, ee: str | None = None) -> np.ndarray:
        return self.tool(ee).fk_ee(q)

    def spheres(self, arm: str | None = None, cell: float = COVER_CELL):
        return self.base.spheres(self.arm_name if arm is None else arm, cell)

    def with_peers(self, q_by_arm, held: Mapping[str, str] | None = None) -> EmbodimentView:
        """This ee's view, the peers at q_by_arm (a dict or full q) holding held: {arm: obj}."""
        return EmbodimentView(self.base, self.ee_name, q_by_arm, held)

    @property
    def peer_spheres(self) -> PeerSet | None:
        """The peers' spheres at peer_q (CollisionWorld's default peers), None until with_peers;
        a whole_body robot's body always."""
        arms = self.peer_q is not None and bool(self.peers)
        if not arms and not self.base.whole_body:
            return None
        stamp = self.base.groups_key() if self.base.whole_body else ()
        if self._peer_spheres is None or self._peer_stamp != stamp:
            self._peer_spheres = self.base.peer_set(self.peer_q if arms else None,
                                                    exclude_arm=self.arm_name,
                                                    held=self.peer_held)
            self._peer_stamp = stamp
        return self._peer_spheres
