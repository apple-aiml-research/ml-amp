#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The declared robot on the link: the hello's embodiment, and the /sim/robot description frame."""

from __future__ import annotations

import hashlib
import json

import numpy as np

from amp.contracts.embodiment import DROID, EmbodimentSpec, RobotDescription, supports
from amp.contracts.robot import ACTION_DIM

Mesh = tuple[np.ndarray, np.ndarray]
DESCRIPTION_KEYS = ("root", "links", "joints", "format")


class RobotError(ValueError):
    """A /sim/robot frame that cannot be accepted."""


def mesh_names(description: RobotDescription) -> list[str]:
    """Every mesh the description names, once each, in description order."""
    return list(dict.fromkeys(m for lk in description.links for m in lk.meshes))


def _canonical(verts, faces) -> Mesh:
    return (np.ascontiguousarray(verts, dtype="<f4"), np.ascontiguousarray(faces, dtype="<u4"))


def description_sha256(description: RobotDescription, meshes: dict[str, Mesh]) -> str:
    """Canonical JSON of the description, then every link's meshes' verts and faces, in order."""
    h = hashlib.sha256(json.dumps(description.to_dict(), sort_keys=True,
                                  separators=(",", ":")).encode())
    for name in (m for lk in description.links for m in lk.meshes):
        verts, faces = _canonical(*meshes[name])
        h.update(verts.tobytes())
        h.update(faces.tobytes())
    return h.hexdigest()


def chain_shas(spec: EmbodimentSpec) -> set[str]:
    """The description shas the spec's chain arms name; empty when no arm needs /sim/robot."""
    return {a.kinematics.description_sha256 for a in spec.arms if a.kinematics.kind == "chain"}


def robot_payload(description: RobotDescription, meshes: dict[str, Mesh]) -> tuple[dict, dict]:
    """(header, blobs) of a /sim/robot frame."""
    blobs: dict[str, np.ndarray] = {}
    for name in mesh_names(description):
        if name not in meshes:
            raise RobotError(f"mesh {name!r} is named by the description but not given")
        blobs[f"verts/{name}"], blobs[f"faces/{name}"] = _canonical(*meshes[name])
    return description.to_dict(), blobs


def parse_robot(header: dict, blobs: dict) -> tuple[RobotDescription, dict[str, Mesh]]:
    """The description and its meshes from a /sim/robot frame; raises RobotError."""
    try:
        description = RobotDescription.from_dict({k: header[k] for k in DESCRIPTION_KEYS
                                                  if k in header})
    except (KeyError, TypeError, ValueError) as exc:
        raise RobotError(f"bad description: {exc}") from exc
    meshes: dict[str, Mesh] = {}
    for name in mesh_names(description):
        verts, faces = blobs.get(f"verts/{name}"), blobs.get(f"faces/{name}")
        if not isinstance(verts, np.ndarray) or not isinstance(faces, np.ndarray):
            raise RobotError(f"mesh {name!r}: verts/{name} and faces/{name} must be raw blobs")
        if verts.ndim != 2 or verts.shape[1] != 3 or not len(verts) or verts.dtype.kind != "f" \
                or not np.isfinite(verts).all():
            raise RobotError(f"mesh {name!r}: verts must be finite non-empty (N, 3) floats")
        if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces) \
                or faces.dtype.kind not in "iu" or faces.min() < 0 or faces.max() >= len(verts):
            raise RobotError(f"mesh {name!r}: faces must be non-empty (M, 3) indices into verts")
        meshes[name] = _canonical(verts, faces)
    return description, meshes


def hello_embodiment(body: dict, accepted: tuple[str, ...]) -> EmbodimentSpec | None:
    """The hello's embodiment (None = DROID), checked against action_dim and what is accepted."""
    raw = body.get("embodiment")
    spec = None
    if raw is not None:
        try:
            spec = EmbodimentSpec.from_dict(raw)
        except (KeyError, TypeError, ValueError, AttributeError) as exc:
            raise ValueError(f"bad embodiment: {exc}") from exc
    want = ACTION_DIM if spec is None else spec.action_dim
    action_dim = body.get("action_dim", want)
    if action_dim != want:
        what = "" if spec is None else f" (embodiment {spec.name})"
        raise ValueError(f"action_dim must be {want}{what}, got {action_dim!r}")
    name = DROID if spec is None else spec.name
    if not supports(accepted, name):
        raise ValueError(f"embodiment {name!r} is not one this server accepts "
                         f"({', '.join(accepted)})")
    return spec
