#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""CollisionWorld: robot spheres and a held object against scene meshes and the table."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from amp.geometry import kinematics as K
from amp.geometry.collision.field import FieldCache, MeshField, mesh_field
from amp.geometry.collision.peers import PeerSet
from amp.geometry.collision.spheres import EE_FRAME, default_spheres, fk_frames_batch

HELD_PREFIX = "held:"
CLEARANCE_CAP = 0.10
_PARALLEL_QUERY = 4000


def _query(tree, pts, bound, workers: int) -> np.ndarray:
    workers = workers if len(pts) > _PARALLEL_QUERY else 1
    return tree.query(pts, distance_upper_bound=bound, workers=workers)[0]


def _row_min(out, rows, vals) -> None:
    """out[r] = min(out[r], vals where rows == r), for ascending rows."""
    starts = np.flatnonzero(np.r_[True, rows[1:] != rows[:-1]])
    r = rows[starts]
    out[r] = np.minimum(out[r], np.minimum.reduceat(vals, starts))


@dataclass
class Contact:
    ok: bool
    clearance: float
    link: str | None
    obstacle: str | None


@dataclass
class _Obstacle:
    name: str
    field: MeshField
    R: np.ndarray
    t: np.ndarray


class CollisionWorld:
    """Robot spheres (and an optionally held object) against scene objects and the table."""

    def __init__(
        self, scene, *, exclude=(), attached=None, margin=0.01, spheres=None, poses=None,
        kd_workers: int = 1, fields: FieldCache | None = None, embodiment=None,
        peers: PeerSet | None = None,
    ):
        if spheres is None:
            spheres = default_spheres() if embodiment is None else embodiment.spheres()
        self.spheres = spheres
        self.arm = getattr(spheres, "arm", None)
        self.n = 7 if self.arm is None else self.arm.n
        self.kd_workers = max(1, int(kd_workers))
        self.margin = float(margin)
        self.table_z = float(getattr(scene, "table_z", 0.0))
        self.checks = 0
        poses = poses or {}
        exclude = set(exclude)
        self.attached_name = None if attached is None else str(attached[0])
        self._obs: list[_Obstacle] = []
        for name, obj in scene.objects.items():
            if name in exclude or name == self.attached_name:
                continue
            field = mesh_field(obj.verts, obj.faces, fields)
            if field is None:
                continue
            T = np.asarray(poses.get(name, obj.pose), dtype=np.float64)
            self._obs.append(_Obstacle(name, field, T[:3, :3].copy(), T[:3, 3].copy()))
        if peers is None:
            peers = getattr(embodiment, "peer_spheres", None)
        self.peers = peers if peers is not None and len(peers) else None
        self.obstacle_names = [o.name for o in self._obs] + (
            [] if self.peers is None else self.peers.obstacle_names) + ["table"]
        self._table = len(self.obstacle_names) - 1
        holders = set() if self.peers is None else self.peers.holding(self.attached_name)
        self.co_held = [k for k, n in enumerate(self.obstacle_names) if n in holders]

        sp = self.spheres
        act = moving_spheres(sp)
        self._frame = sp.frame[act]
        self._local = sp.local[act]
        self.link_names = list(sp.link_names)
        if self.arm is None:
            gripper = np.array([fr == "ee" for fr in sp.link_frames])
        else:
            grip = gripper_links(sp, embodiment)
            gripper = np.array([n in grip for n in sp.link_names])
        self.gripper_links = {n for n, g in zip(sp.link_names, gripper) if g}
        radius, link, kind = [sp.radius[act]], [sp.link[act]], [np.zeros(act.sum(), np.int8)]
        self._grip = gripper[sp.link[act]]
        self._att_pts = self._att_hull = None
        if attached is not None:
            obj = scene.objects[self.attached_name]
            field = mesh_field(obj.verts, obj.faces, fields)
            if field is None:
                raise ValueError(f"attached object {self.attached_name!r} has no geometry")
            pts, r_att, hull = field.attach_points()
            T = np.asarray(attached[1], dtype=np.float64)
            self._att_pts = pts @ T[:3, :3].T + T[:3, 3]
            self._att_hull = hull @ T[:3, :3].T + T[:3, 3]
            self.link_names.append(HELD_PREFIX + self.attached_name)
            h = len(self.link_names) - 1
            radius += [np.full(len(pts), r_att), np.zeros(len(hull))]
            link += [np.full(len(pts), h), np.full(len(hull), h)]
            kind += [np.ones(len(pts), np.int8), np.full(len(hull), 2, np.int8)]
            self._grip = np.concatenate([self._grip, np.zeros(len(pts) + len(hull), bool)])
        self._r = np.concatenate(radius)
        self._link = np.concatenate(link)
        self._kind = np.concatenate(kind)
        self._r_max = float(self._r.max())
        mount = {} if self.peers is None else self.peers.mount
        self._mount = np.array([self.link_names[lk] if k == 0 and self.link_names[lk] in mount
                                else "" for lk, k in zip(self._link, self._kind)], dtype=object)
        self._ignore_cache: dict = {}
        self._group_bounds()

    def _group_bounds(self) -> None:
        """Per rigid point group (link, held surface, hull): a reference point, covering radius."""
        local = [self._local]
        if self._att_pts is not None:
            local += [self._att_pts, self._att_hull]
        local = np.vstack(local)
        key = self._link * 3 + self._kind
        _, first, gid = np.unique(key, return_index=True, return_inverse=True)
        gid = gid.ravel()
        self._gid, self._gref = gid, first
        reach = np.linalg.norm(local - local[first[gid]], axis=1) + self._r
        self._grad = np.zeros(len(first))
        np.maximum.at(self._grad, gid, reach)
        self._grad += 1e-9

    def motion_bounds(self, tool) -> np.ndarray:
        """(n,) metres a checked point can move per radian (or metre) of each joint."""
        if self.arm is not None:
            return self._chain_bounds(tool)
        seg = np.hypot(K.DH[:, 0], K.DH[:, 1])
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        cum = np.append(cum, cum[8] + np.linalg.norm(tool.T_flange_ee[:3, 3]))
        reach = cum[self._frame] + np.linalg.norm(self._local, axis=1)
        frames = self._frame
        if self._att_pts is not None:
            tcp = cum[9] + np.linalg.norm(tool.T_ee_tcp[:3, 3])
            att = np.vstack([self._att_pts, self._att_hull])
            reach = np.concatenate([reach, tcp + np.linalg.norm(att, axis=1)])
            frames = np.concatenate([frames, np.full(len(att), EE_FRAME)])
        return np.array([
            max(0.0, float((reach[frames >= j] - cum[j]).max(initial=0.0))) for j in range(1, 8)
        ])

    def _chain_bounds(self, tool) -> np.ndarray:
        arm = self.arm
        ext, kinds = _path_extents(arm)
        L = len(arm.link_names)
        cum = np.concatenate([[0.0], np.cumsum(ext)])
        cum_f = np.concatenate([[0.0], cum, [cum[-1] + np.linalg.norm(tool.T_flange_ee[:3, 3])]])
        reach = cum_f[self._frame] + np.linalg.norm(self._local, axis=1)
        frames = self._frame
        if self._att_pts is not None:
            tcp = cum_f[L + 1] + np.linalg.norm(tool.T_ee_tcp[:3, 3])
            att = np.vstack([self._att_pts, self._att_hull])
            reach = np.concatenate([reach, tcp + np.linalg.norm(att, axis=1)])
            frames = np.concatenate([frames, np.full(len(att), L + 1)])
        out = []
        for i, kind in enumerate(kinds):
            if kind == "fixed":
                continue
            down = frames >= i + 2
            if kind == "prismatic":
                out.append(1.0 if down.any() else 0.0)
            else:
                out.append(max(0.0, float((reach[down] - cum_f[i + 2]).max(initial=0.0))))
        return np.array(out)

    def _link_mask(self, sel) -> np.ndarray:
        robot = self._kind == 0
        if sel in (None, "*"):
            return robot
        if sel == "gripper":
            return robot & self._grip
        if sel == "arm":
            return robot & ~self._grip
        if sel == "held" or (self.attached_name is not None and sel in (
            self.attached_name, HELD_PREFIX + self.attached_name
        )):
            return self._kind != 0
        if isinstance(sel, str) and sel.startswith(HELD_PREFIX):
            return np.zeros(len(self._kind), bool)
        if sel in self.link_names:
            return robot & (self._link == self.link_names.index(sel))
        raise ValueError(f"unknown link {sel!r}")

    def _ignore(self, table_links_ignored, ignore) -> np.ndarray:
        key = (tuple(table_links_ignored), tuple(tuple(p) for p in ignore))
        m = self._ignore_cache.get(key)
        if m is None:
            m = np.zeros((len(self.obstacle_names), len(self._kind)), dtype=bool)
            for sel in key[0]:
                m[-1] |= self._link_mask(sel)
            for sel, obstacle in key[1]:
                rows = (
                    slice(None) if obstacle == "*"
                    else [i for i, n in enumerate(self.obstacle_names) if n == obstacle]
                )
                m[rows] |= self._link_mask(sel)
            self._ignore_cache[key] = m
        return m

    def frames(self, qs, tool) -> tuple[np.ndarray, int]:
        """(N,F,4,4) sphere frames at every configuration, and the index of the ee frame."""
        if self.arm is None:
            return fk_frames_batch(qs, tool.T_flange_ee), EE_FRAME
        links = self.arm.fk_links_batch(qs)
        L = links.shape[1]
        F = np.empty((len(links), L + 2, 4, 4))
        F[:, 0] = np.eye(4)
        F[:, 1:L + 1] = links
        F[:, L + 1] = links[:, -1] @ np.asarray(tool.T_flange_ee, dtype=np.float64)
        return F, L + 1

    def points(self, qs, tool) -> np.ndarray:
        """(N,M,3): robot sphere centres, then held-object points, at every configuration."""
        F, ee = self.frames(qs, tool)
        Fs = F[:, self._frame]
        P = (Fs[..., :3, :3] @ self._local[..., None])[..., 0] + Fs[..., :3, 3]
        if self._att_pts is None:
            return P
        T_tcp = F[:, ee] @ tool.T_ee_tcp
        att = np.vstack([self._att_pts, self._att_hull])
        Pa = att @ T_tcp[:, :3, :3].transpose(0, 2, 1) + T_tcp[:, None, :3, 3]
        return np.concatenate([P, Pa], axis=1)

    def _uses(self, table_links_ignored, ignore) -> list[np.ndarray]:
        key = (tuple(table_links_ignored), tuple(tuple(p) for p in ignore), "use")
        u = self._ignore_cache.get(key)
        if u is None:
            ign = self._ignore(table_links_ignored, ignore)
            u = [np.flatnonzero(~ign[k] & (self._kind != 2)) for k in range(self._table)]
            u.append(np.flatnonzero(~ign[-1] & (self._kind != 1)))
            self._ignore_cache[key] = u
        return u

    def evaluate(
        self, qs, tool, *, table_links_ignored=(), ignore=()
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per configuration: (clearance (N,), link index (N,), obstacle index (N,), -1 = none)."""
        qs = np.asarray(qs, dtype=np.float64).reshape(-1, self.n)
        n = len(qs)
        self.checks += n
        P = self.points(qs, tool)
        m = P.shape[1]
        uses = self._uses(table_links_ignored, ignore)
        qcap = CLEARANCE_CAP + self._r_max + self.margin
        ub = np.full(n, CLEARANCE_CAP)
        pairs = []
        Pref = P[:, self._gref]
        for k, o in enumerate(self._obs):
            use = uses[k]
            if not len(use):
                continue
            lo_b, hi_b = o.field.lo, o.field.hi
            Lr = (Pref - o.t) @ o.R
            gbox = np.linalg.norm(np.maximum(np.maximum(lo_b - Lr, Lr - hi_b), 0), axis=2)
            live = gbox - self._grad - self.margin < CLEARANCE_CAP
            if not live.any():
                continue
            rows, cu = np.nonzero(live[:, self._gid[use]])
            L = (P[rows, use[cu]] - o.t) @ o.R
            r = self._r[use[cu]]
            box = np.linalg.norm(np.maximum(np.maximum(lo_b - L, L - hi_b), 0), axis=1)
            cand = np.flatnonzero(box - r - self.margin < CLEARANCE_CAP)
            if not len(cand):
                continue
            rows, cu, Lc = rows[cand], cu[cand], L[cand]
            ctree, eps = o.field.coarse()
            dc = _query(ctree, Lc, qcap + eps, self.kd_workers)
            ins = o.field.inside(Lc)
            rc = r[cand] + self.margin
            dlo, dhi = np.minimum(dc - eps, qcap), np.minimum(dc, qcap)
            lo = np.where(ins, -dhi, dlo) - rc
            _row_min(ub, rows, np.where(ins, -dlo, dhi) - rc)
            pairs.append((k, rows, use[cu], Lc, ins, lo, dhi, rc))
        peer_hits = [] if self.peers is None else self._peer_hits(P, Pref, uses, ub)
        tuse = uses[-1]
        tclr = None
        if len(tuse):
            tclr = P[:, tuse, 2] - self.table_z - self._r[tuse] - self.margin
            np.minimum(ub, tclr.min(axis=1), out=ub)
        best = np.full(n * m, CLEARANCE_CAP)
        best_obs = np.full(n * m, -1, dtype=np.int64)
        for k, rows, cols, Lc, ins, lo, dhi, rc in pairs:
            need = np.flatnonzero(lo <= ub[rows] + 1e-12)
            if not len(need):
                continue
            rows, cols, ins = rows[need], cols[need], ins[need]
            reach = np.where(ins, dhi[need], np.minimum(dhi[need], ub[rows] + rc[need]))
            bound = min(qcap, max(0.0, float(reach.max()))) * (1 + 1e-9) + 1e-12
            d = np.minimum(_query(self._obs[k].field.tree, Lc[need], bound, self.kd_workers),
                           qcap)
            clr = np.where(ins, -d, d) - self._r[cols] - self.margin
            flat = rows * m + cols
            better = clr < best[flat]
            best[flat[better]] = clr[better]
            best_obs[flat[better]] = k
        for k, rows, cols, clr in peer_hits:
            flat = rows * m + cols
            better = clr < best[flat]
            best[flat[better]] = clr[better]
            best_obs[flat[better]] = k
        best, best_obs = best.reshape(n, m), best_obs.reshape(n, m)
        if tclr is not None:
            sub, sub_obs = best[:, tuse], best_obs[:, tuse]
            better = tclr < sub
            sub[better] = tclr[better]
            sub_obs[better] = self._table
            best[:, tuse], best_obs[:, tuse] = sub, sub_obs
        j = np.argmin(best, axis=1)
        rows = np.arange(n)
        return best[rows, j], self._link[j], best_obs[rows, j]

    def _peer_hits(self, P, Pref, uses, ub) -> list:
        """Per peer arm: (obstacle index, rows, point columns, exact clearance) near enough."""
        out = []
        n, G = Pref.shape[:2]
        held = self._kind != 0
        for g in range(len(self.peers.arms)):
            k = len(self._obs) + g
            use = uses[k]
            parts = [(use, True)]
            if k in self.co_held:
                parts = [(use[~held[use]], True), (use[held[use]], False)]
            parts = [(p[self._mount[p] == lk], hand, lk or None) for p, hand in parts
                     for lk in np.unique(self._mount[p])]
            for part, hand, link in parts:
                if not len(part):
                    continue
                slack = self.peers.r_max(g, hand, link) + self.margin
                bound = CLEARANCE_CAP + float(self._grad.max()) + slack
                d = self.peers.nearest(Pref.reshape(-1, 3), g, bound, hand, link).reshape(n, G)
                live = d - self._grad - slack < CLEARANCE_CAP
                rows, cu = np.nonzero(live[:, self._gid[part]])
                if not len(rows):
                    continue
                cols = part[cu]
                clr = self.peers.clearance(P[rows, cols], self._r[cols] + self.margin, g,
                                           CLEARANCE_CAP, hand, link)
                _row_min(ub, rows, clr)
                out.append((k, rows, cols, clr))
        return out

    def set_peers(self, peers) -> None:
        """Swap in the same peers at another pose (a joint group moved): same obstacles, mounts."""
        peers = peers if peers is not None and len(peers) else None
        same = (peers is None) == (self.peers is None) and (peers is None or (
            peers.obstacle_names == self.peers.obstacle_names
            and peers.mount.keys() == self.peers.mount.keys()
            and peers.held == self.peers.held))
        if not same:
            raise ValueError("set_peers takes the same peer obstacles, mounts and held objects")
        self.peers = peers

    def carry_obstacle(self, name: str, T_delta) -> None:
        """Move mesh obstacle name by the world-frame rigid motion T_delta (a hand carries it)."""
        D = np.asarray(T_delta, dtype=np.float64)
        for o in self._obs:
            if o.name == name:
                o.R, o.t = D[:3, :3] @ o.R, D[:3, :3] @ o.t + D[:3, 3]

    def sphere_clearance(self, C, R, ignore=()) -> tuple[np.ndarray, np.ndarray]:
        """Per world sphere (C, R): (clearance to the scene's meshes, obstacle index, -1: none)."""
        C = np.asarray(C, dtype=np.float64).reshape(-1, 3)
        R = np.asarray(R, dtype=np.float64).reshape(-1)
        best, which = np.full(len(R), CLEARANCE_CAP), np.full(len(R), -1, dtype=np.int64)
        for k, o in enumerate(self._obs):
            if o.name in ignore or not len(R):
                continue
            L = (C - o.t) @ o.R
            box = np.linalg.norm(np.maximum(np.maximum(o.field.lo - L, L - o.field.hi), 0), axis=1)
            near = np.flatnonzero(box - R - self.margin < CLEARANCE_CAP)
            if not len(near):
                continue
            d = np.minimum(_query(o.field.tree, L[near], CLEARANCE_CAP + float(R.max()),
                                  self.kd_workers), CLEARANCE_CAP)
            clr = np.where(o.field.inside(L[near]), -d, d) - R[near] - self.margin
            better = clr < best[near]
            best[near[better]], which[near[better]] = clr[better], k
        return best, which

    def contact(self, clearance, link, obstacle) -> Contact:
        """A Contact from one row of evaluate()."""
        near = int(obstacle) >= 0
        return Contact(
            ok=bool(clearance >= 0),
            clearance=float(clearance),
            link=self.link_names[int(link)] if near else None,
            obstacle=self.obstacle_names[int(obstacle)] if near else None,
        )

    def check(self, q, tool, *, table_links_ignored=(), ignore=()) -> Contact:
        """The closest contact at one configuration."""
        c, lk, ob = self.evaluate(q, tool, table_links_ignored=table_links_ignored, ignore=ignore)
        return self.contact(c[0], lk[0], ob[0])


def _path_extents(arm) -> tuple[np.ndarray, list[str]]:
    """Per joint on a chain arm's base -> flange path: its maximum origin offset and its type."""
    from amp.geometry.embodiment.description import path_joints

    cache = arm.__dict__.get("_collision_extents")
    if cache is None:
        ext, kinds = [], []
        for j in path_joints(arm.description, arm.spec.base_link, arm.spec.flange_link):
            e = float(np.linalg.norm(np.array(j.origin).reshape(4, 4)[:3, 3]))
            if j.type == "prismatic":
                e += max(abs(j.limits[0]), abs(j.limits[1]))
            ext.append(e)
            kinds.append(j.type)
        cache = arm.__dict__["_collision_extents"] = (np.array(ext), kinds)
    return cache


def moving_spheres(spheres) -> np.ndarray:
    """Mask of the spheres an arm's joints move (a chain arm's static base and mount are not)."""
    arm = getattr(spheres, "arm", None)
    return spheres.frame > (0 if arm is None else _static_frames(arm))


def _static_frames(arm) -> int:
    """The count of leading sphere frames of a chain arm that never move."""
    _, kinds = _path_extents(arm)
    first = next((i for i, k in enumerate(kinds) if k != "fixed"), len(kinds))
    return first + 1


def gripper_link_names(embodiment, arm) -> set[str]:
    """The described links of an arm's gripper: everything mounted beyond its flange."""
    desc = embodiment.description
    children: dict[str, list[str]] = {}
    for j in desc.joints:
        children.setdefault(j.parent, []).append(j.child)
    by_joint = {j.name: j.child for j in desc.joints}
    roots = [arm.link_names[-1]]
    for e in embodiment.spec.end_effectors:
        if e.arm == arm.name:
            roots.append(e.ee_link)
            if e.gripper is not None:
                roots += [by_joint[f] for f in e.gripper.finger_joints if f in by_joint]
    out: set[str] = set()
    seen, path = set(), set(arm.link_names)
    while roots:
        link = roots.pop()
        if link in seen:
            continue
        seen.add(link)
        if link not in path:
            out.add(link)
        roots += [c for c in children.get(link, []) if c not in path]
    return out


def gripper_links(spheres, embodiment=None) -> set[str]:
    """The sphere links that are a chain arm's gripper (gripper_link_names); without them (or an
    embodiment) the links riding on the flange."""
    arm = spheres.arm
    out: set[str] = set()
    if getattr(embodiment, "description", None) is not None:
        out = gripper_link_names(embodiment, arm) & set(spheres.link_names)
    if not out:
        flange = arm.link_names[-1]
        out = {n for n, f in zip(spheres.link_names, spheres.link_frames) if f == flange}
    return out
