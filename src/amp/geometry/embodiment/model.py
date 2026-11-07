#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The embodiment runtime: everything the robot server needs, built from a declared embodiment."""

from __future__ import annotations

import copy
from collections.abc import Mapping

import numpy as np
from scipy.spatial import cKDTree

from amp.contracts.embodiment import (
    CameraSpec,
    EmbodimentSpec,
    EndEffectorSpec,
    JointGroupSpec,
    RobotDescription,
)
from amp.geometry.collision.checker import CLEARANCE_CAP, gripper_links, moving_spheres
from amp.geometry.collision.peers import BODY, PeerSet, own_spheres
from amp.geometry.collision.spheres import RobotSpheres, default_spheres
from amp.geometry.embodiment.arm import Arm, ChainArm, DHArm, make_arm
from amp.geometry.embodiment.description import (
    description_sha256,
    joint_motion,
    parent_joints,
    path_joints,
)
from amp.geometry.embodiment.profiles import droid
from amp.geometry.embodiment.spheres import COVER_CELL, LinkSpheres, body_cover, link_cover
from amp.geometry.embodiment.tool import Gripper, make_tool
from amp.geometry.embodiment.view import EmbodimentView
from amp.geometry.kinematics.tool import ToolModel

SELF_MARGIN, SELF_SAMPLES, SELF_ALWAYS = 0.02, 128, 0.95


def _pair_clearance(Ca, Ra, Cb, Rb, cap: float) -> float:
    """min |ca - cb| - ra - rb over two sphere sets, capped at cap."""
    if not len(Ra) or not len(Rb):
        return cap
    hits = cKDTree(Cb).query_ball_point(Ca, Ra + float(Rb.max()) + cap)
    best = cap
    for i, js in enumerate(hits):
        if js:
            js = np.asarray(js)
            best = min(best, float((np.linalg.norm(Cb[js] - Ca[i], axis=1) - Ra[i] - Rb[js]).min()))
    return best


class Embodiment:
    """Arms, tools, grippers, collision spheres and cameras of an EmbodimentSpec (+ description)."""

    def __init__(self, spec: EmbodimentSpec, description: RobotDescription | None = None,
                 blobs: Mapping | None = None):
        self.spec = spec
        self.name = spec.name
        self.description = description
        self.blobs = blobs
        self.description_sha256 = None
        if description is not None and blobs is not None:
            self.description_sha256 = description_sha256(description, blobs)
            for a in spec.arms:
                k = a.kinematics
                if k.kind == "chain" and k.description_sha256 != self.description_sha256:
                    raise ValueError(f"arm {a.name}: kinematics names description "
                                     f"{k.description_sha256}, got {self.description_sha256}")
        self.arms: dict[str, Arm] = {a.name: make_arm(a, description) for a in spec.arms}
        self.joint_groups: dict[str, JointGroupSpec] = {g.name: g for g in spec.joint_groups}
        self.carrier: dict[str, str] = {a: g.name for g in spec.joint_groups for a in g.carries}
        for g in spec.joint_groups:
            _check_group(g, description, spec)
        self.group_q: dict[str, np.ndarray] = {g.name: np.zeros(len(g.joints))
                                               for g in spec.joint_groups}
        self.end_effectors: dict[str, EndEffectorSpec] = {e.name: e for e in spec.end_effectors}
        self.grippers = {e.name: Gripper(e.gripper) for e in spec.end_effectors
                         if e.gripper is not None}
        self._tools = {e.name: self.new_tool(e.name) for e in spec.end_effectors}
        self._spheres: dict = {}
        self._views: dict[str, EmbodimentView] = {}

    @classmethod
    def droid(cls) -> Embodiment:
        """The DROID runtime (profiles.droid())."""
        return cls(droid())

    @property
    def dof(self) -> int:
        return self.spec.dof

    @property
    def action_dim(self) -> int:
        return self.spec.action_dim

    def arm(self, name: str | None = None) -> Arm:
        """An arm by name; None = the first declared."""
        return self.arms[self.spec.arms[0].name if name is None else name]

    def ee(self, name: str | None = None) -> EndEffectorSpec:
        """An end effector by name; None = the first declared."""
        return self.end_effectors[self.spec.end_effectors[0].name if name is None else name]

    def arm_of_ee(self, ee: str) -> str:
        """The name of the arm an end effector rides on."""
        return self.ee(ee).arm

    def view(self, ee: str | None = None) -> EmbodimentView:
        """This runtime seen from one end effector (or the only one of a named arm); one per ee."""
        if ee is not None and ee not in self.end_effectors and ee in self.arms:
            on = [e.name for e in self.spec.end_effectors if e.arm == ee]
            if len(on) != 1:
                raise ValueError(f"arm {ee} has {len(on)} end effectors: name one of {on}")
            ee = on[0]
        name = self.ee(ee).name
        if name not in self._views:
            self._views[name] = EmbodimentView(self, name)
        return self._views[name]

    def peer_set(self, q_by_arm, *, exclude_arm: str, held=None) -> PeerSet:
        """Other arms' spheres (and a whole_body robot's body) as exclude_arm's PeerSet."""
        if q_by_arm is not None and not isinstance(q_by_arm, Mapping):
            q_by_arm = self.split(q_by_arm)
        if exclude_arm not in self.arms:
            raise ValueError(f"{self.name}: unknown arm {exclude_arm!r}")
        C, R, names, hand = [np.empty((0, 3))], [np.empty(0)], [], []
        for a in self.spec.arms:
            if a.name == exclude_arm or q_by_arm is None:
                continue
            if a.name not in q_by_arm:
                raise ValueError(f"{self.name}: no q for peer arm {a.name}")
            arm = self.arms[a.name]
            sp = self.spheres(a.name)
            mine = own_spheres(sp, arm, self)
            q = np.asarray(q_by_arm[a.name], dtype=np.float64).reshape(arm.n)
            C.append(sp.world_batch(q)[0][mine])
            R.append(sp.radius[mine])
            names += [f"{a.name}:{sp.link_names[i]}" for i in sp.link[mine]]
            grip = gripper_links(sp, self)
            hand += [sp.link_names[i] in grip for i in sp.link[mine]]
        mount = None
        if self.whole_body:
            bc, br, bn, bm = self.body_peer(exclude_arm)
            before = len(names)
            C.append(bc)
            R.append(br)
            names += [f"{BODY}:{n}" for n in bn]
            hand += [False] * len(bn)
            mount = {lk: np.r_[np.zeros(before, bool), m] for lk, m in bm.items()}
        held = {a: str(o) for a, o in (held or {}).items() if a != exclude_arm and o is not None}
        return PeerSet(np.concatenate(C), np.concatenate(R), tuple(names), held, tuple(hand),
                       mount=mount)

    def groups_key(self) -> tuple:
        """Where the joint groups are held, as a hashable key."""
        return tuple(round(float(x), 6) for g in self.spec.joint_groups
                     for x in self.group_q[g.name])

    def body_peer(self, arm: str) -> tuple[np.ndarray, np.ndarray, list[str], dict]:
        """(centres, radii, links, {arm link: mask of its mount spheres}) of the body."""
        key = ("body_peer", arm, self.groups_key())
        if key not in self._spheres:
            for k in [k for k in list(self._spheres) if k[:2] == ("body_peer", arm)]:
                del self._spheres[k]
            C, R, names = self.body_world()
            self._spheres[key] = (C, R, names, self._home_mount(arm))
        return self._spheres[key]

    def _home_mount(self, arm: str) -> dict[str, np.ndarray]:
        """_mounted at the groups' home q, so a torso move never turns a new contact into a mount."""
        key = ("mount", arm)
        if key not in self._spheres:
            home = self.with_groups({g.name: g.q_home for g in self.spec.joint_groups})
            self._spheres[key] = home._mounted(arm, *home.body_world()[:2])
        return self._spheres[key]

    def moved_by(self, group: str) -> set[str]:
        """The body links joint group `group` moves (its joints lie on their path from the root)."""
        up, joints = parent_joints(self.description), set(self.joint_groups[group].joints)
        out = set()
        for link in self.body_spheres():
            at = link
            while at in up:
                if up[at].name in joints:
                    out.add(link)
                    break
                at = up[at].parent
        return out

    def body_pairs(self) -> list[tuple[str, str]]:
        """Body link pairs a joint-group joint between them moves relative to each other."""
        key = ("body_pairs",)
        if key not in self._spheres:
            self._spheres[key] = self._body_pairs()
        return self._spheres[key]

    def _body_pairs(self) -> list[tuple[str, str]]:
        """Non-adjacent pairs with a group joint between them, apart at home and in most poses."""
        up = parent_joints(self.description)
        actuated = {j for g in self.spec.joint_groups for j in g.joints}
        links = list(self.body_spheres())
        cand = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]
                if _joints_between(up, a, b) & actuated and not (a in up and up[a].parent == b)
                and not (b in up and up[b].parent == a)]
        if not cand:
            return []
        rng = np.random.default_rng(0)
        groups = self.spec.joint_groups
        qs = [{g.name: np.asarray(g.q_home, float) for g in groups}] + [
            {g.name: rng.uniform(g.q_min, g.q_max) for g in groups} for _ in range(SELF_SAMPLES)]
        touched = np.zeros((len(cand), len(qs)), bool)
        for k, gq in enumerate(qs):
            C, R, names = self.body_world(gq)
            names = np.asarray(names)
            for i, (a, b) in enumerate(cand):
                touched[i, k] = _pair_clearance(C[names == a], R[names == a], C[names == b],
                                                R[names == b], SELF_MARGIN) < SELF_MARGIN
        keep = ~touched[:, 0] & (touched[:, 1:].mean(axis=1) < SELF_ALWAYS)
        return [p for p, k in zip(cand, keep) if k]

    def body_self_clearances(self, group_q=None) -> dict[tuple[str, str], float]:
        """{(link, link): clearance} of every body_pairs pair at group_q (default: held)."""
        C, R, names = self.body_world(group_q)
        names = np.asarray(names)
        return {(a, b): _pair_clearance(C[names == a], R[names == a], C[names == b],
                                        R[names == b], CLEARANCE_CAP)
                for a, b in self.body_pairs()}

    def _mounted(self, arm: str, C: np.ndarray, R: np.ndarray) -> dict[str, np.ndarray]:
        """Per moving link of arm, the body spheres it is mounted on."""
        sp, a = self.spheres(arm), self.arms[arm]
        mv = moving_spheres(sp)
        if not mv.any() or not len(R):
            return {}
        lo, hi = np.asarray(a.spec.q_min, float), np.asarray(a.spec.q_max, float)
        rng = np.random.default_rng(0)
        qs = np.vstack([np.asarray(a.spec.q_home, float), rng.uniform(lo, hi, (SELF_SAMPLES, a.n))])
        W = sp.world_batch(qs)[:, mv]
        n, m = W.shape[:2]
        links, link_of = np.unique(sp.link[mv], return_inverse=True)
        link_of = link_of.ravel()
        r = np.tile(sp.radius[mv], n)
        flat = W.reshape(-1, 3)
        hits = cKDTree(C).query_ball_point(flat, r + float(R.max()) + SELF_MARGIN)
        touched = np.zeros((len(links), len(R), n), bool)
        for i, js in enumerate(hits):
            if not js:
                continue
            js = np.asarray(js)
            d = np.linalg.norm(C[js] - flat[i], axis=1)
            touched[link_of[i % m], js[d < r[i] + R[js] + SELF_MARGIN], i // m] = True
        mounted = touched[:, :, 0] | (touched[:, :, 1:].mean(axis=2) >= SELF_ALWAYS)
        return {sp.link_names[lk]: mounted[k] for k, lk in enumerate(links) if mounted[k].any()}

    def tool(self, ee: str | None = None) -> ToolModel:
        """The end effector's tool, owned by this runtime: calibrating it calibrates the runtime."""
        return self._tools[self.ee(ee).name]

    def new_tool(self, ee: str | None = None) -> ToolModel:
        """A fresh tool at the nominal T_flange_ee."""
        e = self.ee(ee)
        return make_tool(self.arms[e.arm], e)

    def gripper(self, ee: str | None = None) -> Gripper | None:
        """The end effector's gripper model; None for a tool without fingers."""
        return self.grippers.get(self.ee(ee).name)

    def tcp_pose(self, q, ee: str | None = None) -> np.ndarray:
        """The TCP pose at the arm's q, by the end effector's (possibly calibrated) tool."""
        return self.tool(ee).fk_tcp(q)

    def ee_pose(self, q, ee: str | None = None) -> np.ndarray:
        return self.tool(ee).fk_ee(q)

    def set_group_q(self, name: str, q) -> None:
        """Hold joint group `name` at q: every arm it carries is then posed (FK, IK, tools,
        spheres) from its base at that q, which stays fixed until the next call."""
        g = self.joint_groups[name]
        q = np.asarray(q, dtype=np.float64).reshape(len(g.joints)).copy()
        values = dict(zip(g.joints, q))
        for a in g.carries:
            arm = self.arms[a]
            arm.T_root_base = _path_pose(self.description, arm.spec.base_link, values)
        self.group_q[name] = q

    def pose_groups(self, q, tol: float = 0.0) -> bool:
        """set_group_q for each joint group of a full q that moved more than tol; whether any did."""
        if not self.joint_groups:
            return False
        parts = self.split(q)
        moved = False
        for name in self.joint_groups:
            if np.abs(parts[name] - self.group_q[name]).max(initial=0.0) > tol:
                self.set_group_q(name, parts[name])
                moved = True
        return moved

    def with_groups(self, group_q: Mapping[str, np.ndarray]) -> Embodiment:
        """A private copy with joint groups held at group_q; this runtime is left as it is."""
        new = copy.copy(self)
        new.arms = {n: copy.copy(a) for n, a in self.arms.items()}
        memo = {id(a): new.arms[n] for n, a in self.arms.items()}
        new._tools = {e: copy.deepcopy(t, dict(memo)) for e, t in self._tools.items()}
        new.group_q = dict(self.group_q)
        new._spheres = {k: v for k, v in self._spheres.items()
                        if k[0] in ("body", "mount", "body_pairs")}
        new._views = {}
        for name, q in group_q.items():
            new.set_group_q(name, q)
        return new

    def split(self, q) -> dict[str, np.ndarray]:
        """A full proprio vector (arms, then joint groups, in declaration order) by arm and group."""
        q = np.asarray(q, dtype=np.float64).reshape(-1)
        if len(q) != self.dof:
            raise ValueError(f"{self.name}: q has {len(q)} values, the arms {self.dof}")
        out, k = {}, 0
        for a in (*self.spec.arms, *self.spec.joint_groups):
            out[a.name] = q[k:k + len(a.joints)]
            k += len(a.joints)
        return out

    def link_poses(self, q) -> dict[str, np.ndarray]:
        """Every link frame at the full q: the described tree (non-arm joints at 0, within their
        limits) for a chain; the arm's links and its end effectors' ee_link for dh_craig."""
        qa = self.split(q)
        out: dict[str, np.ndarray] = {}
        for a in self.spec.arms:
            if a.name in self.carrier:
                continue
            arm = self.arms[a.name]
            out.update(zip(arm.link_names, arm.fk_links(qa[a.name])))
            for e in self.spec.end_effectors:
                if e.arm == a.name and isinstance(arm, DHArm):
                    out[e.ee_link] = self.tool(e.name).fk_ee(qa[a.name])
        if self.description is not None:
            values = {j: float(x) for a in (*self.spec.arms, *self.spec.joint_groups)
                      for j, x in zip(a.joints, qa[a.name])}
            out.update(_tree_poses(self.description, values, out))
        return out

    def spheres(self, arm: str | None = None,
                cell: float = COVER_CELL) -> RobotSpheres | LinkSpheres:
        """The arm's collision spheres keyed by link name: the fitted asset model on dh_craig, a
        conservative cover of the description's meshes on a chain."""
        a = self.arm(arm)
        if isinstance(a, DHArm):
            return default_spheres()
        key = (a.name, cell)
        if key not in self._spheres:
            if self.description is None or self.blobs is None:
                raise ValueError(f"arm {a.name}: chain spheres need the description's mesh blobs")
            joints = [j for s in (*self.spec.arms, *self.spec.joint_groups) for j in s.joints]
            assert isinstance(a, ChainArm)
            self._spheres[key] = LinkSpheres(
                a, link_cover(self.description, self.blobs, a, joints, cell),
                source=f"cover:{self.description_sha256}")
        return self._spheres[key]

    @property
    def whole_body(self) -> bool:
        """Whether a moving body outside the arms is described: self-filtered and collided."""
        return bool((self.spec.base is not None or self.spec.joint_groups)
                    and self.description is not None and self.blobs is not None
                    and all(isinstance(a, ChainArm) for a in self.arms.values()))

    def warm(self) -> None:
        """Build a whole_body robot's arm and body covers and body peers ahead of first use."""
        if not self.whole_body:
            return
        for a in self.spec.arms:
            self.spheres(a.name)
        self.body_spheres()
        for a in self.spec.arms:
            self.body_peer(a.name)
        self.body_pairs()

    def arm_links(self) -> set[str]:
        """Every arm's described links: its base link and all mounted below it."""
        children: dict[str, list[str]] = {}
        for j in self.description.joints:
            children.setdefault(j.parent, []).append(j.child)
        out: set[str] = set()
        todo = [self.arms[a.name].link_names[0] for a in self.spec.arms]
        while todo:
            lk = todo.pop()
            if lk not in out:
                out.add(lk)
                todo += children.get(lk, [])
        return out

    def body_spheres(self, cell: float = COVER_CELL) -> dict[str, tuple[str, np.ndarray]]:
        """The cover of each link outside the arms: {link: (frame link, (n, 4) spheres)}."""
        key = ("body", cell)
        if key not in self._spheres:
            if not self.whole_body:
                raise ValueError(f"{self.name}: no described body outside its arms")
            joints = [j for s in (*self.spec.arms, *self.spec.joint_groups) for j in s.joints]
            self._spheres[key] = body_cover(self.description, self.blobs, joints,
                                            self.arm_links(), cell)
        return self._spheres[key]

    def whole_spheres(self, q) -> tuple[np.ndarray, np.ndarray, list[str]]:
        """(centres, radii, link per sphere) of the whole robot at the full q, root frame."""
        poses = self.link_poses(np.asarray(q, dtype=np.float64).reshape(-1)[:self.dof])
        C, R, names, covered = [], [], [], set()
        for a in self.spec.arms:
            sp = self.spheres(a.name)
            frames = np.stack([np.eye(4), *(poses[n] for n in sp.arm.link_names)])[sp.frame]
            C.append(np.einsum("sij,sj->si", frames[:, :3, :3], sp.local) + frames[:, :3, 3])
            R.append(sp.radius)
            names += [sp.link_names[i] for i in sp.link]
            covered |= set(sp.link_names)
        for link, (frame, s) in self.body_spheres().items():
            if link in covered:
                continue
            T = poses[frame]
            C.append(s[:, :3] @ T[:3, :3].T + T[:3, 3])
            R.append(s[:, 3])
            names += [link] * len(s)
        return np.concatenate(C), np.concatenate(R), names

    def body_world(self, group_q: Mapping[str, np.ndarray] | None = None):
        """(centres, radii, link per sphere) of the body at group_q (default: held)."""
        held = {**self.group_q, **(group_q or {})}
        q = np.concatenate([np.zeros(len(a.joints)) for a in self.spec.arms]
                           + [np.asarray(held[g.name], dtype=np.float64).reshape(-1)
                              for g in self.spec.joint_groups])
        poses = self.link_poses(q)
        C, R, names = [np.empty((0, 3))], [np.empty(0)], []
        for link, (frame, s) in self.body_spheres().items():
            T = poses[frame]
            C.append(s[:, :3] @ T[:3, :3].T + T[:3, 3])
            R.append(s[:, 3])
            names += [link] * len(s)
        return np.concatenate(C), np.concatenate(R), names

    @property
    def cameras(self) -> tuple[CameraSpec, ...]:
        return self.spec.cameras

    def camera(self, name: str) -> CameraSpec:
        return self.spec.camera(name)

    def cameras_with_role(self, role: str) -> tuple[CameraSpec, ...]:
        return tuple(c for c in self.spec.cameras if c.role == role)


def _check_group(g: JointGroupSpec, description: RobotDescription | None,
                 spec: EmbodimentSpec) -> None:
    """A joint group is the description's movable path base_link -> tip_link, and every arm it
    carries hangs from tip_link through fixed joints only."""
    if description is None:
        raise ValueError(f"joint group {g.name}: needs the robot description")
    movable = [j.name for j in path_joints(description, g.base_link, g.tip_link)
               if j.type != "fixed"]
    if tuple(movable) != g.joints:
        raise ValueError(f"joint group {g.name}: the movable joints {g.base_link} -> {g.tip_link} "
                         f"are {movable}, the spec declares {list(g.joints)}")
    for name in g.carries:
        arm = next(a for a in spec.arms if a.name == name)
        between = [j.name for j in path_joints(description, g.tip_link, arm.base_link)
                   if j.type != "fixed"]
        if between:
            raise ValueError(f"arm {name}: undeclared joints {between} between joint group "
                             f"{g.name} and its base")


def _joints_between(up, a: str, b: str) -> set[str]:
    """The joints on the tree path between links a and b."""
    def climb(link):
        out, js = {link: ()}, []
        while link in up:
            js.append(up[link].name)
            link = up[link].parent
            out[link] = tuple(js)
        return out

    ua, ub = climb(a), climb(b)
    common = next(lk for lk in ua if lk in ub)
    return set(ua[common]) | set(ub[common])


def _path_pose(description: RobotDescription, link: str, values: Mapping[str, float]) -> np.ndarray:
    """The root -> link pose, the path's joints at values (0 when not given)."""
    T = np.eye(4)
    for j in path_joints(description, description.root, link):
        T = T @ joint_motion(j, float(values.get(j.name, 0.0)))
    return T


def _tree_poses(description: RobotDescription, values: Mapping[str, float],
                known: Mapping[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Every described link's pose from the root, joints at values (else 0 clipped to limits)."""
    up = parent_joints(description)
    out: dict[str, np.ndarray] = {description.root: np.eye(4)}

    def pose(link: str) -> np.ndarray:
        if link not in out:
            if link in known:
                out[link] = known[link]
            else:
                j = up[link]
                lo, hi = j.limits
                x = values.get(j.name, min(max(0.0, lo), hi) if lo <= hi else 0.0)
                out[link] = pose(j.parent) @ joint_motion(j, x)
        return out[link]

    for lk in description.links:
        pose(lk.name)
    return out
