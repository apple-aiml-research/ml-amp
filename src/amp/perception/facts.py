#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Symbolic facts of a perceived scene for the trace: "holding obj3", "on obj4 obj5"."""

from __future__ import annotations

from collections.abc import Iterable

from amp.perception.objects import PerceivedObject, footprint, footprint_fracs

Fact = tuple[str, ...]
TABLE = "table"
REST_OVERLAP = 0.8
REST_TOL_M = 0.015


def _feet(o: PerceivedObject) -> dict:
    f = footprint(o)
    f["zbot"], f["ztop"] = float(o.aabb[0][2]), float(o.aabb[1][2])
    return f


def rests_on(o: dict, s: dict, min_overlap: float = REST_OVERLAP,
             tol: float = REST_TOL_M) -> tuple[bool, float] | None:
    """(on top, s's top z) when footprint o is on top of or inside footprint s, else None."""
    over_o, over_s = footprint_fracs(o, s)
    if over_o >= min_overlap:
        if abs(o["zmin"] - s["ztop"]) <= tol:
            return True, s["ztop"]
        if s["zbot"] - tol <= o["zmin"] < s["ztop"] - tol:
            return False, s["ztop"]
    if over_s >= min_overlap and abs(o["zmin"] - s["ztop"]) <= tol:
        return True, s["ztop"]
    return None


def scene_facts(objects: Iterable[PerceivedObject], held: str | None = None, fixtures=(),
                table: str = TABLE) -> tuple[dict[str, str], set[Fact]]:
    """({name: type}, facts): an item rests on what it is on top of, else inside, else the table."""
    objs = list(objects)
    kinds = {o.id: "fixture" if o.kind == "fixture" or o.id in fixtures else "item" for o in objs}
    if held is not None:
        kinds[held] = "item"
    types = {**kinds, table: "fixture"}
    facts: set[Fact] = {("handempty",)} if held is None else {("holding", held)}
    facts |= {("clear", f) for f, t in types.items() if t == "fixture"}
    feet = {o.id: _feet(o) for o in objs if o.id != held}
    covered = set()
    for oid, f in feet.items():
        if kinds[oid] != "item":
            continue
        under = [(r, sid) for sid, s in feet.items() if sid != oid and (r := rests_on(f, s))]
        support = max(under)[1] if under else table
        facts.add(("on", oid, support))
        if types[support] == "item":
            covered.add(support)
    facts |= {("clear", i) for i, t in kinds.items() if t == "item" and i not in covered}
    return types, facts


def fact_strings(facts: Iterable[Fact]) -> list[str]:
    """The facts as sorted strings, without the "clear" bookkeeping."""
    return sorted(" ".join(f) for f in facts if f[0] != "clear")
