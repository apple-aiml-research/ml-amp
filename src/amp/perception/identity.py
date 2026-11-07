#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Object ids across passes: signatures, Hungarian matching, and the lost-object memory."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy.optimize import linear_sum_assignment

from amp.perception.color import color_feat
from amp.perception.params import PerceptionParams

if TYPE_CHECKING:
    from amp.perception.objects import Perception


def signature(centroid, points, color, kind="object") -> dict:
    pts = np.asarray(points, np.float64).reshape(-1, 3)
    ext = np.ptp(pts, axis=0) if len(pts) else np.zeros(3)
    c = np.asarray(centroid, np.float64)
    lo = pts[:, :2].min(axis=0) if len(pts) else c[:2]
    hi = pts[:, :2].max(axis=0) if len(pts) else c[:2]
    return {"centroid": c.tolist(), "extent": ext.tolist(), "color": [int(v) for v in color],
            "lo": lo.tolist(), "hi": hi.tolist(), "kind": str(kind)}


def occluded_ok(s: dict, c: dict, p: PerceptionParams) -> bool:
    """s is what the arm leaves visible of fixture or bin c: no larger, not too small, inside."""
    ec = np.asarray(c["extent"], np.float64)
    if (c.get("kind") != "fixture" and float(max(ec[:2])) < p.occluded_extent) or "lo" not in c:
        return False
    es = np.sort(np.asarray(s["extent"][:2], np.float64))
    eo = np.sort(ec[:2])
    if np.any(es > 1.1 * eo + 0.02) or np.any(es < p.occluded_frac * eo):
        return False
    xy = np.asarray(s["centroid"][:2])
    return bool(np.all(xy >= np.asarray(c["lo"]) - 0.01)
                and np.all(xy <= np.asarray(c["hi"]) + 0.01))


def id_num(i: str) -> int:
    return int(i[3:]) if i.startswith("obj") and i[3:].isdigit() else 0


def _match(sigs: list[dict], cands: list[tuple[str, dict, bool]], p: PerceptionParams,
           todo: list[int]) -> dict[int, str]:
    """Hungarian match of segments (indices todo) to candidate signatures."""
    if not todo or not cands:
        return {}
    big = 1e6
    cost = np.full((len(todo), len(cands)), big)
    for a, si in enumerate(todo):
        s = sigs[si]
        fs = color_feat(np.array([s["color"]]), p.intensity_weight)[0]
        es = np.sort(s["extent"][:2])
        for b, (_, c, anywhere) in enumerate(cands):
            dc = float(np.linalg.norm(np.subtract(s["centroid"], c["centroid"])))
            fc = color_feat(np.array([c["color"]]), p.intensity_weight)[0]
            dcol = float(np.linalg.norm(fs - fc))
            ec = np.sort(c["extent"][:2])
            de = float(np.abs(es - ec).sum() + abs(s["extent"][2] - c["extent"][2]))
            tol_e = max(0.03, 0.35 * float(max(max(es), max(ec))))
            if dcol > p.match_color:
                continue
            if de > (3 if anywhere else 2) * tol_e or (not anywhere and dc > p.match_dist):
                if not anywhere and occluded_ok(s, c, p):
                    cost[a, b] = 3.0 + dcol / p.match_color + min(dc / p.match_dist, 3.0)
                continue
            cost[a, b] = (0.0 if anywhere else dc / p.match_dist) + dcol / p.match_color \
                + de / tol_e
    rows, cols = linear_sum_assignment(cost)
    return {todo[r]: cands[c][0] for r, c in zip(rows, cols) if cost[r, c] < big}


def assign_ids(sigs: list[dict], prev: Perception | None, held_id: str | None,
                now: float, p: PerceptionParams):
    lost = {} if prev is None else {k: dict(v) for k, v in prev.lost.items()
                                    if now - v.get("t", now) <= p.lost_ttl_s or v.get("held")}
    in_place = []
    nxt = 1 if prev is None else int(prev.next_id)
    if prev is not None:
        for oid, o in prev.objects.items():
            nxt = max(nxt, id_num(oid) + 1)
            sig = signature(o.centroid, o.points, o.color, o.kind)
            if oid == held_id:
                lost[oid] = {**sig, "held": True, "t": now}
            else:
                in_place.append((oid, sig, False))
    for k in lost:
        nxt = max(nxt, id_num(k) + 1)
    if held_id is not None and held_id in lost:
        lost[held_id]["held"] = True
    ids = _match(sigs, in_place, p, list(range(len(sigs))))
    rest = [i for i in range(len(sigs)) if i not in ids]
    cands = [(k, v, bool(v.get("held")) and k != held_id) for k, v in lost.items()
             if k != held_id]
    ids.update(_match(sigs, cands, p, rest))
    for k in set(ids.values()):
        lost.pop(k, None)
    out = []
    for i in range(len(sigs)):
        if i in ids:
            out.append(ids[i])
        else:
            out.append(f"obj{nxt}")
            nxt += 1
    if prev is not None:
        for oid, o in prev.objects.items():
            if oid not in out and oid not in lost:
                lost[oid] = {**signature(o.centroid, o.points, o.color, o.kind),
                             "held": False, "t": now}
    return out, nxt, lost
