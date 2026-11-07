#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""What the R1Pro touches, from OmniGibson's contact reports: every link of the robot."""

from __future__ import annotations

from collections.abc import Callable, Iterable

FLOOR_CATEGORIES = ("floors", "lawn", "driveway", "carpet")

STRUCTURE_CATEGORIES = frozenset({"floors", "walls", "ceilings", "lawn", "driveway", "fence", "roof",
                                  "background"})



def _under(path: str, root: str) -> bool:
    return path == root or path.startswith(root.rstrip("/") + "/")


class BodyContacts:
    """The "<link> touched <object>" pairs the robot makes now, minus non-obstacles."""

    def __init__(self, robot, objects: Callable[[], Iterable]):
        self.robot = robot
        self._objects = objects
        self.links = dict(robot.links)
        if not self.links:
            raise ValueError(f"{robot.name} has no links to read contacts from")
        self.owners = sorted(((o.prim_path, o.name, getattr(o, "category", None))
                              for o in objects() if o is not robot), key=lambda t: -len(t[0]))
        self.baseline: set[tuple[str, str]] = set()

    def _touching(self) -> list[tuple[str, str, str, str | None]]:
        """(link, the other body's prim path, its owner's name, its owner's category), own bodies
        out."""
        out = []
        for name, link in self.links.items():
            for c in link.contact_list():
                for other in (c.body0, c.body1):
                    if _under(other, self.robot.prim_path):
                        continue
                    _, owner, cat = next((o for o in self.owners if _under(other, o[0])),
                                         (None, other, None))
                    out.append((name, other, owner, cat))
        return out

    def rest(self) -> None:
        """Note the pairs touching now, the robot standing still before any move."""
        self.baseline = {(link, other) for link, other, _, _ in self._touching()}

    def read(self) -> tuple[str, ...]:
        """"<link> touched <object>" for each pair, once each, in a fixed order; () for none. A rest
        pair is left out only while it persists: once it parts, touching again is reported."""
        touching = self._touching()
        self.baseline &= {(link, other) for link, other, _, _ in touching}
        held = {o.name for o in (getattr(self.robot, "_ag_obj_in_hand", None) or {}).values()
                if o is not None}
        return tuple(sorted({f"{link} touched {owner}" for link, other, owner, cat in touching
                             if cat not in FLOOR_CATEGORIES and (link, other) not in self.baseline
                             and owner not in held}))
