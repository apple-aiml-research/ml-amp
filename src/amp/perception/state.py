#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""PerceptionState: an episode's perception, its agent-defined objects and its id history."""

from __future__ import annotations

import dataclasses
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

from amp.geometry.collision import RobotSpheres
from amp.geometry.embodiment import Embodiment, LinkSpheres
from amp.perception.identity import id_num
from amp.perception.objects import (
    PerceivedObject,
    Perception,
    footprint,
    footprint_fracs,
    footprint_overlap,
    in_obb,
    obb,
    similar_size,
)
from amp.perception.params import PerceptionParams, params_for
from amp.perception.pipeline import object_from_points

AGENT_SNAP_M = 0.015
AGENT_MIN_POINTS = 30
AGENT_COVER_M = 0.006
REASSOC_MAX_M = 0.08
REASSOC_COLOR_MAX = 80.0
RESOLVED_KEYS = ("object", "target", "held", "id", "ignore", "replace")
RESOLVED_LIST_KEYS = ("ignore", "replace")
CleanCloud = Callable[[], tuple[np.ndarray, np.ndarray]]


@dataclass
class AgentObject:
    """An object the agent defined: kept and re-meshed across perception passes."""

    points: np.ndarray
    rgb: np.ndarray | None
    label: str | None
    kind: str | None
    mesh_c: np.ndarray


@dataclass
class Released:
    """Where an object was let go: its posed mesh's centre, verts, faces and colour."""


    c: np.ndarray
    verts: np.ndarray | None = None
    faces: np.ndarray | None = None
    color: tuple | None = None


@dataclass
class PerceptionState:
    """Everything perception remembers within an episode; reset() between episodes."""

    perception: Perception | None = None
    agent: dict[str, AgentObject] = field(default_factory=dict)
    forgotten: list[dict] = field(default_factory=list)
    suppressed: dict[str, PerceivedObject] = field(default_factory=dict)
    successors: dict[str, dict] = field(default_factory=dict)
    released: dict[str, Released] = field(default_factory=dict)
    params: PerceptionParams | None = None
    spheres: RobotSpheres | LinkSpheres | None = None
    embodiment: Embodiment | None = None

    def __post_init__(self):
        if self.params is None:
            self.params = params_for(self.embodiment)

    def reset(self) -> None:
        """Forget the episode: perception, agent objects, releases and id history."""
        self.perception = None
        self.agent, self.forgotten, self.suppressed = {}, [], {}
        self.successors, self.released = {}, {}

    def prev(self) -> Perception | None:
        """The prev for perceive(): agent objects out, suppressed segments in (to keep ids)."""
        p = self.perception
        if p is None or not (self.agent or self.suppressed):
            return p
        objs = {k: o for k, o in p.objects.items() if k not in self.agent}
        objs.update({k: o for k, o in self.suppressed.items() if k not in objs})
        return dataclasses.replace(p, objects=objs)

    def next_id(self) -> int:
        """The next free obj<N>, past every id seen, lost or suppressed."""
        p = self.perception
        n = int(p.next_id) if p is not None else 1
        for k in [*(p.objects if p else ()), *(p.lost if p else ()), *self.suppressed]:
            n = max(n, id_num(k) + 1)
        return n

    def note_release(self, oid: str, c, verts=None, faces=None, color=None) -> None:
        """The executor let oid go with its mesh centred at c: expect it there next pass."""
        self.released[oid] = Released(np.asarray(c, np.float64), verts, faces, color)

    def install(self, p: Perception, *, held: str | None = None, clean: CleanCloud | None = None,
                wanted: Iterable[str] = ()) -> Perception:
        """Make p current: merge agent objects, re-associate wanted ids, record successors."""
        self.merge_agent(p, held=held, clean=clean)
        self.reassociate(p, held=held, wanted=wanted)
        self.track_successors(p, held=held)
        self.perception = p
        return p

    def merge_agent(self, p: Perception, *, held: str | None = None,
                    clean: CleanCloud | None = None) -> None:
        """Put agent objects into p; suppress segments they cover or where one was forgotten."""
        if not (self.agent or self.forgotten):
            return
        cleaned = None
        for aid, a in self.agent.items():
            if aid == held:
                p.objects.pop(aid, None)
                continue
            pts, rgb = a.points, a.rgb
            e = self.released.pop(aid, None)
            if e is not None:
                pts = pts + (np.asarray(e.c) - a.mesh_c).astype(np.float32)
                if clean is not None:
                    if cleaned is None:
                        cleaned = clean()
                    m = in_obb(cleaned[0], obb(pts), AGENT_SNAP_M)
                    if m.sum() >= AGENT_MIN_POINTS:
                        pts, rgb = cleaned[0][m], cleaned[1][m]
                        p.notes.append(f"agent object {aid} found where it was released")
            obj = object_from_points(aid, pts, rgb, p.table_z, label=a.label, kind=a.kind,
                                     params=self.params)
            if obj is None:
                continue
            a.points, a.rgb, a.mesh_c = obj.points, obj.rgb, obj.verts.mean(axis=0)
            p.objects[aid] = obj
        trees = [(aid, cKDTree(np.asarray(self.agent[aid].points, dtype=np.float64)))
                 for aid in self.agent if aid in p.objects]
        self.suppressed = {}
        for oid, o in list(p.objects.items()):
            if o.source != "perception" or not len(o.points):
                continue
            why = self._suppressed_by(oid, np.asarray(o.points, np.float64).reshape(-1, 3), trees)
            if why is not None:
                self.suppressed[oid] = p.objects.pop(oid)
                p.notes.append(why)
        nums = [id_num(k) for k in p.objects if id_num(k)]
        if nums:
            p.next_id = max(int(p.next_id), max(nums) + 1)

    def _suppressed_by(self, oid: str, pts: np.ndarray, trees) -> str | None:
        for aid, tree in trees:
            d = tree.query(pts, distance_upper_bound=AGENT_COVER_M)[0]
            if float(np.isfinite(d).mean()) > 0.5:
                return f"{oid} suppressed: agent object {aid} covers it"
        ext = np.sort(np.ptp(pts[:, :2], axis=0))
        for f in self.forgotten:
            if (float(in_obb(pts, f["obb"], AGENT_SNAP_M).mean()) > 0.5
                    and np.all(ext <= 1.5 * f["ext"] + 0.02)
                    and np.all(ext >= f["ext"] / 1.5 - 0.02)):
                return f"{oid} suppressed: {f['id']} was forgotten there"
        return None

    def track_successors(self, p: Perception, *, held: str | None = None) -> None:
        """Before p is installed, record what each id p loses became."""
        prev = self.perception
        if prev is None:
            return
        forgotten = {f["id"] for f in self.forgotten}
        for oid in p.objects:
            self.successors.pop(oid, None)
        prev_ids = set(prev.objects)
        new = {nid: footprint(o) for nid, o in p.objects.items()}
        for oid, o in prev.objects.items():
            if oid in p.objects or oid == held or oid in forgotten or oid in self.successors:
                continue
            ref = footprint(o)
            cands = []
            for nid, sig in new.items():
                f_old, f_new = footprint_fracs(ref, sig)
                if nid in prev_ids and footprint_fracs(ref, footprint(prev.objects[nid]))[0] \
                        >= 0.5:
                    continue
                if f_old >= 0.5 or (f_new >= 0.5 and nid not in prev_ids):
                    cands.append((f_old + f_new, nid))
            if not cands:
                continue
            cands.sort(reverse=True)
            nid = cands[0][1]
            fresh = sorted(c[1] for c in cands if c[1] not in prev_ids)
            if p.objects[nid].source == "agent":
                why = f"replaced by agent object {nid}"
            elif nid in prev_ids:
                why = f"merged by perception into {nid}"
            elif len(fresh) > 1:
                why = f"split by perception into {', '.join(fresh)}; {nid} covers most of it"
            else:
                why = f"renumbered by perception: {nid} is where it was"
            self.successors[oid] = {"to": nid, "why": why}

    def reassociate(self, p: Perception, *, held: str | None = None,
                    wanted: Iterable[str] = ()) -> None:
        """Give each wanted (or released) id that vanished to the fresh object most like it."""
        prev = self.perception
        wanted = set(self.released) | {w for w in wanted if isinstance(w, str)}
        prev_ids = set(prev.objects) if prev is not None else set()
        fresh = [i for i in p.objects if i not in prev_ids and i not in wanted]
        refs = {}
        for rid in sorted(wanted):
            if rid in p.objects or rid == held:
                if rid in p.objects:
                    self.released.pop(rid, None)
                continue
            e = self.released.get(rid)
            if e is not None and e.verts is not None:
                v = np.asarray(e.verts, dtype=np.float64)
                refs[rid] = {"c": np.asarray(e.c), "lo": v.min(axis=0)[:2],
                             "hi": v.max(axis=0)[:2], "ext": np.sort(np.ptp(v, axis=0)[:2]),
                             "color": np.asarray(e.color or (128, 128, 128), float)}
            elif prev is not None and rid in prev.objects:
                refs[rid] = footprint(prev.objects[rid])
        pairs = []
        for rid, ref in refs.items():
            tol = max(REASSOC_MAX_M, 0.5 * float(ref["ext"][-1]))
            for i in fresh:
                s = footprint(p.objects[i])
                dcol = float(np.linalg.norm(s["color"] - ref["color"]))
                if dcol > REASSOC_COLOR_MAX or not similar_size(ref, s):
                    continue
                d = float(np.linalg.norm(s["c"] - ref["c"]))
                ov = footprint_overlap(ref, s)
                if d > tol and ov < 0.5:
                    continue
                pairs.append((d / tol + (1.0 - ov) + dcol / REASSOC_COLOR_MAX, rid, i, d, ov))
        for _, rid, i, d, ov in sorted(pairs):
            if rid not in refs or i not in fresh:
                continue
            p.objects[rid] = dataclasses.replace(p.objects.pop(i), id=rid)
            fresh.remove(i)
            del refs[rid]
            self.released.pop(rid, None)
            p.notes.append(f"{i} re-associated with {rid} ({d * 100:.1f} cm from where it was, "
                           f"{ov * 100:.0f}% footprint overlap)")
        for rid in list(refs):
            into = self._merged_into(p, rid)
            if into is not None:
                p.notes.append(f"{rid} merged into {into} after its place; kept at its release "
                               "pose")

    def _merged_into(self, p: Perception, rid: str) -> str | None:
        """A released rid seen as part of what it was put on stays, as released."""
        e = self.released.get(rid)
        if e is None or e.verts is None or not p.objects:
            return None
        c = np.asarray(e.c, dtype=np.float64)
        into = None
        for oid, o in p.objects.items():
            lo, hi = np.asarray(o.aabb, dtype=np.float64).reshape(2, 3)
            if np.all(c[:2] >= lo[:2]) and np.all(c[:2] <= hi[:2]) and c[2] <= hi[2] + 0.02:
                into = oid
                break
        if into is None:
            return None
        template = p.objects[into]
        v = np.asarray(e.verts, dtype=np.float64)
        p.objects[rid] = dataclasses.replace(
            template, id=rid, label=f"{template.label} (merged {rid})", points=v, verts=v,
            faces=np.asarray(e.faces), centroid=c, color=tuple(e.color or template.color),
            aabb=np.stack([v.min(axis=0), v.max(axis=0)]), kind="object")
        return into

    def resolve(self, oid, names: Iterable[str] | None = None):
        """(oid, None) if current, else (successor, {"from", "to", "why"}), following chains."""
        if names is None:
            names = self.perception.objects if self.perception is not None else ()
        names = set(names)
        if not isinstance(oid, str) or oid in names or oid not in self.successors:
            return oid, None
        cur, whys, seen = oid, [], {oid}
        while cur not in names:
            e = self.successors.get(cur)
            if e is None or e["to"] in seen:
                return oid, None
            whys.append(e["why"] if cur == oid else f"then {cur} {e['why']}")
            cur = e["to"]
            seen.add(cur)
        return cur, {"from": oid, "to": cur, "why": "; ".join(whys)}

    def resolve_request_ids(self, body, names: Iterable[str] | None = None):
        """(body with every stale id in RESOLVED_KEYS replaced by its successor, [resolutions])."""
        if not isinstance(body, dict) or not self.successors:
            return body, []
        names = set(names) if names is not None else None
        out, resolved = dict(body), []
        for key in RESOLVED_KEYS:
            v = body.get(key)
            if isinstance(v, str):
                out[key], r = self.resolve(v, names)
                if r is not None:
                    resolved.append(r)
            elif key in RESOLVED_LIST_KEYS and isinstance(v, list):
                items = []
                for x in v:
                    y, r = self.resolve(x, names)
                    items.append(y)
                    if r is not None:
                        resolved.append(r)
                out[key] = items
        return out, resolved
