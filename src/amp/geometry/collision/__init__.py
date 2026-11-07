#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Robot collision spheres against scene meshes and the table half-space."""

from amp.geometry.collision.checker import (
    CLEARANCE_CAP,
    HELD_PREFIX,
    CollisionWorld,
    Contact,
)
from amp.geometry.collision.field import (
    ATTACHED_STEP,
    COARSE_CELL,
    COLUMN_STEP_RANGE,
    FIELD_CACHE_SIZE,
    FIELDS,
    MAX_ATTACHED_POINTS,
    MAX_SURFACE_SAMPLES,
    SURFACE_STEP,
    FieldCache,
    MeshField,
    mesh_field,
)
from amp.geometry.collision.peers import PEER_PREFIX, PeerSet, pair_path_clearance
from amp.geometry.collision.spheres import (
    EE_FRAME,
    FRAME_INDEX,
    SPHERES_FORMAT,
    SPHERES_PATH,
    RobotSpheres,
    default_spheres,
    fk_frames_batch,
)

__all__ = [
    "ATTACHED_STEP", "CLEARANCE_CAP", "COARSE_CELL", "COLUMN_STEP_RANGE", "EE_FRAME", "FIELDS",
    "FIELD_CACHE_SIZE", "FRAME_INDEX", "HELD_PREFIX", "MAX_ATTACHED_POINTS", "MAX_SURFACE_SAMPLES",
    "PEER_PREFIX", "SPHERES_FORMAT", "SPHERES_PATH", "SURFACE_STEP", "CollisionWorld", "Contact",
    "FieldCache", "MeshField", "PeerSet", "RobotSpheres", "default_spheres", "fk_frames_batch",
    "mesh_field", "pair_path_clearance",
]
