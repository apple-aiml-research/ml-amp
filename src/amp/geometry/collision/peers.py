#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Other arms as static sphere obstacles: PeerSet, and arm-vs-arm clearance along two paths."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import cKDTree

PEER_PREFIX = "arm:"
BODY = "robot_body"
_K = 8


@dataclass(frozen=True, eq=False)
class PeerSet:
    """Other arms' spheres, static in the root frame, named "<arm>:<link>"; held: what each has."""

    centres: np.ndarray
    radii: np.ndarray
    names: tuple[str, ...]
    held: Mapping[str, str] = field(default_factory=dict)
    hand: tuple[bool, ...] | None = None
    mount: Mapping[str, np.ndarray] | None = None
    arms: tuple[str, ...] = field(init=False)
    _groups: tuple = field(init=False, repr=False)
    _bare: tuple = field(init=False, repr=False)
    _sub: dict = field(init=False, repr=False)

    def __post_init__(self):
        c = np.asarray(self.centres, dtype=np.float64).reshape(-1, 3)
        r = np.asarray(self.radii, dtype=np.float64).reshape(-1)
        names = tuple(self.names)
        if not len(c) == len(r) == len(names):
            raise ValueError(f"peers: {len(c)} centres, {len(r)} radii, {len(names)} names")
        hand = np.zeros(len(r), bool) if self.hand is None else np.asarray(self.hand, bool)
        if hand.shape != r.shape:
            raise ValueError(f"peers: {len(hand)} hand flags for {len(r)} spheres")
        arm = np.array([n.split(":", 1)[0] for n in names])
        arms = tuple(dict.fromkeys(arm.tolist()))
        groups = tuple(_group(np.flatnonzero(arm == a), c, r) for a in arms)
        bare = tuple(_group(np.flatnonzero((arm == a) & ~hand), c, r) for a in arms)
        mount = {str(k): np.asarray(m, bool).reshape(-1) for k, m in (self.mount or {}).items()}
        if any(m.shape != r.shape for m in mount.values()):
            raise ValueError(f"peers: a mount mask is not {len(r)} long")
        for k, v in (("centres", c), ("radii", r), ("names", names), ("hand", tuple(hand)),
                     ("mount", mount), ("arms", arms), ("_groups", groups), ("_bare", bare),
                     ("_sub", {})):
            object.__setattr__(self, k, v)

    def __len__(self) -> int:
        return len(self.radii)

    @property
    def obstacle_names(self) -> list[str]:
        """One obstacle per peer arm: "arm:<peer>", and BODY for the robot's own body."""
        return [a if a == BODY else PEER_PREFIX + a for a in self.arms]

    def holding(self, name) -> set[str]:
        """The obstacle names of the peer arms holding object name."""
        return {PEER_PREFIX + a for a in self.arms if name is not None and self.held.get(a) == name}

    def _of(self, g: int, hand: bool, link: str | None = None) -> tuple:
        base = self._groups[g] if hand else self._bare[g]
        if link is None or link not in self.mount:
            return base
        key = (g, hand, link)
        if key not in self._sub:
            idx = base[0]
            self._sub[key] = _group(idx[~self.mount[link][idx]], self.centres, self.radii)
        return self._sub[key]

    def r_max(self, g: int, hand: bool = True, link: str | None = None) -> float:
        return self._of(g, hand, link)[5]

    def nearest(self, pts, g: int, bound: float, hand: bool = True,
                link: str | None = None) -> np.ndarray:
        """(N,) distance from each point to the nearest centre of peer arm g (inf past bound)."""
        tree = self._of(g, hand, link)[3]
        if tree is None:
            return np.full(len(np.asarray(pts).reshape(-1, 3)), np.inf)
        return tree.query(pts, distance_upper_bound=bound)[0]

    def clearance(self, pts, r, g: int, cap: float, hand: bool = True,
                  link: str | None = None) -> np.ndarray:
        """(N,) capped clearance to arm g's spheres (hand ones if hand), less link's mount."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        r = np.broadcast_to(np.asarray(r, dtype=np.float64), (len(pts),))
        _, C, R, tree, _, r_max = self._of(g, hand, link)
        if not len(pts):
            return np.empty(0)
        if tree is None:
            return np.full(len(pts), float(cap))
        k = min(_K, len(R))
        bound = cap + float(r.max()) + r_max
        d, idx = tree.query(pts, k=k, distance_upper_bound=bound)
        d, idx = d.reshape(len(pts), k), idx.reshape(len(pts), k)
        Rp = np.append(R, 0.0)[idx]
        val = np.where(np.isfinite(d), d - Rp, np.inf).min(axis=1)
        dk = d[:, -1]
        need = np.flatnonzero(np.isfinite(dk) & (val > dk - r_max) & (dk - r_max - r < cap))
        if len(need):
            val[need] = (np.linalg.norm(pts[need, None] - C[None], axis=2) - R).min(axis=1)
        return np.minimum(val - r, cap)

    def body_parts(self) -> list[str]:
        """The robot-body links whose spheres this set holds."""
        return list(dict.fromkeys(
            n.split(":", 1)[1] for n in self.names if n.startswith(BODY + ":")))

    def only_body_part(self, part: str) -> PeerSet:
        """This set with the body's spheres cut to those of link `part` (other arms kept)."""
        keep = np.array([not n.startswith(BODY + ":") or n == f"{BODY}:{part}" for n in self.names])
        return PeerSet(self.centres[keep], self.radii[keep],
                       tuple(n for n, k in zip(self.names, keep) if k), self.held,
                       tuple(np.asarray(self.hand)[keep]),
                       mount={k: m[keep] for k, m in self.mount.items()})

    def link_at(self, pt, g: int | None = None, hand: bool = True,
                link: str | None = None) -> str:
        """The peer sphere name nearest a point by surface distance (of arm g, else any)."""
        p = np.asarray(pt, dtype=np.float64).reshape(3)
        idx = np.arange(len(self.radii)) if g is None else self._of(g, hand, link)[0]
        return self.names[int(idx[np.argmin(
            np.linalg.norm(self.centres[idx] - p, axis=1) - self.radii[idx])])]


def _group(idx, c, r) -> tuple:
    """(indices, centres, radii, KD-tree or None, min radius, max radius) of spheres idx."""
    if not len(idx):
        return idx, c[idx], r[idx], None, 0.0, 0.0
    return idx, c[idx], r[idx], cKDTree(c[idx]), float(r[idx].min()), float(r[idx].max())


def pair_path_clearance(emb, qa: np.ndarray, qb: np.ndarray, arm_a: str, arm_b: str,
                        margin: float) -> tuple[np.ndarray, int | None]:
    """(n,) clearance of arm_a at qa[i] vs arm_b at qb[i]; first negative index or None."""
    from amp.geometry.collision.checker import CLEARANCE_CAP, moving_spheres

    A, B = emb.arm(arm_a), emb.arm(arm_b)
    qa = np.asarray(qa, dtype=np.float64).reshape(-1, A.n)
    qb = np.asarray(qb, dtype=np.float64).reshape(-1, B.n)
    if len(qa) != len(qb):
        raise ValueError(f"paths of {len(qa)} and {len(qb)} steps")
    sa, sb = emb.spheres(arm_a), emb.spheres(arm_b)
    ma, mb = moving_spheres(sa), own_spheres(sb, B, emb)
    Pa, Pb = sa.world_batch(qa)[:, ma], sb.world_batch(qb)[:, mb]
    ra = sa.radius[ma] + float(margin)
    names = tuple(f"{arm_b}:{sb.link_names[i]}" for i in sb.link[mb])
    if not len(names) or not ma.any():
        return np.full(len(qa), CLEARANCE_CAP), None
    out = np.array([PeerSet(Pb[i], sb.radius[mb], names).clearance(Pa[i], ra, 0, CLEARANCE_CAP)
                    .min(initial=CLEARANCE_CAP) for i in range(len(qa))])
    bad = np.flatnonzero(out < 0)
    return out, (int(bad[0]) if len(bad) else None)


def own_spheres(spheres, arm, emb=None) -> np.ndarray:
    """Mask of an arm's own spheres: its base link and what it carries, but no arm of emb on it."""
    if getattr(spheres, "arm", None) is None:
        raise ValueError(f"arm {arm.name}: a dh_craig arm has no pose in a shared root")
    mine = spheres.frame >= 1
    other = mounted_links(emb, arm) if emb is not None else set()
    if other:
        mine &= ~np.array([n in other for n in spheres.link_names], dtype=bool)[spheres.link]
    return mine


def mounted_links(emb, arm) -> set[str]:
    """Every other arm's described links: its base and all below it, short of arm's own base."""
    desc = getattr(emb, "description", None)
    if desc is None:
        return set()
    children: dict[str, list[str]] = {}
    for j in desc.joints:
        children.setdefault(j.parent, []).append(j.child)
    own = arm.link_names[0]
    out: set[str] = set()
    todo = [emb.arms[a.name].link_names[0] for a in emb.spec.arms if a.name != arm.name]
    while todo:
        lk = todo.pop()
        if lk != own and lk not in out:
            out.add(lk)
            todo += children.get(lk, [])
    return out
