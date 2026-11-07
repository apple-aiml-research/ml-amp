#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""PerceptionParams: every tunable of segmentation, meshing and id tracking."""

from __future__ import annotations

import dataclasses
import functools
from dataclasses import dataclass

from amp.contracts.embodiment import WorkspaceSpec
from amp.geometry.embodiment import Embodiment


@dataclass(frozen=True)
class TableRegion:
    """Where the support plane is looked for: an xy box, a z window and the clamp on the result."""

    x: tuple[float, float] = (0.3, 0.8)
    y_abs: float = 0.35
    z_window: tuple[float, float] = (-0.05, 0.10)
    z_clamp: tuple[float, float] = (-0.02, 0.05)
    object_margin: float = 0.03
    min_points: int = 200
    search: bool = True
    fallback_z: float = 0.0


@dataclass(frozen=True)
class SupportProfile:
    """What a support kind changes: where its plane is searched for, and the heights above it."""

    z_window: tuple[float, float] | None
    z_clamp: tuple[float, float] | None
    floor_band: float = 0.15


SUPPORTS: dict[str, SupportProfile] = {
    "table": SupportProfile(z_window=(-0.05, 0.10), z_clamp=(-0.02, 0.05)),
    "floor": SupportProfile(z_window=None, z_clamp=None),
    "none": SupportProfile(z_window=None, z_clamp=None),
}


def table_region(ws: WorkspaceSpec) -> TableRegion:
    """The support-plane search of a workspace: its table_region, else derived from its box."""
    sp = SUPPORTS[ws.support]
    x0, x1, y0, y1, z0, z1 = ws.box
    if ws.table_region:
        rx0, rx1, ry, wz0, wz1, cz0, cz1 = ws.table_region
        return TableRegion(x=(rx0, rx1), y_abs=ry, z_window=(wz0, wz1), z_clamp=(cz0, cz1),
                           search=ws.support != "none", fallback_z=min(max(0.0, cz0), cz1))
    d = TableRegion()
    x = (max(d.x[0], x0), min(d.x[1], x1))
    if x[0] >= x[1]:
        x = (x0, x1)
    y_abs = min(d.y_abs, max(abs(y0), abs(y1)))
    if sp.z_window is not None:
        win = (max(sp.z_window[0], z0), min(sp.z_window[1], z1))
    else:
        win = (z0, min(z0 + sp.floor_band, z1))
    clamp = sp.z_clamp if sp.z_clamp is not None else win
    clamp = (max(clamp[0], win[0]), min(clamp[1], win[1]))
    fallback = z0 if ws.support == "none" else min(max(0.0, clamp[0]), clamp[1])
    return TableRegion(x=x, y_abs=y_abs, z_window=win, z_clamp=clamp,
                       search=ws.support != "none", fallback_z=fallback)


def workspace_bounds(ws: WorkspaceSpec) -> tuple[tuple[float, float], ...]:
    """The workspace box as the ((xmin, xmax), (ymin, ymax), (zmin, zmax)) crop."""
    b = ws.box
    return ((b[0], b[1]), (b[2], b[3]), (b[4], b[5]))


@functools.cache
def _droid() -> Embodiment:
    return Embodiment.droid()


def embodiment_or_droid(embodiment: Embodiment | None) -> Embodiment:
    """embodiment, or the DROID runtime when None."""
    return _droid() if embodiment is None else embodiment


SIM_TABLE = table_region(_droid().spec.workspace)


@dataclass(frozen=True)
class PerceptionParams:
    voxel: float = 0.004
    table_margin: float = 0.005
    robot_inflate: float = 0.02
    remnant_dist: float = 0.05
    remnant_lift: float = 0.02
    held_margin: float = 0.012
    cluster_tol: float = 0.011
    color_split: bool = True
    color_edge: float = 0.08
    color_merge: float = 0.18
    intensity_weight: float = 0.1
    min_region_voxels: int = 20
    concave_split: bool = True
    concave_deg: float = 25.0
    crease_pairs: int = 2
    patch_voxels: int = 80
    patch_frac: float = 0.15
    normal_k: int = 10
    normal_conf: float = 0.25
    min_voxels: int = 10
    absorb_voxels: int = 60
    absorb_dist: float = 0.022
    overlap_frac: float = 0.6
    overlap_dist: float = 0.05
    fragment_ratio: float = 0.25
    same_base: float = 0.008
    shared_footprint: float = 0.4
    cell: float = 0.004
    tight_footprint: bool = True
    footprint_pad: float = 0.001
    max_cells: int = 6000
    close_iters: int = 1
    max_hole_cells: int = 150
    free_margin: float = 0.01
    min_thickness: float = 0.003
    support_gap: float = 0.004
    match_dist: float = 0.05
    match_color: float = 0.3
    lost_ttl_s: float = 120.0
    occluded_frac: float = 0.25
    occluded_extent: float = 0.15
    floor_at_support: bool = True
    table: TableRegion = SIM_TABLE


DEFAULT_PARAMS = PerceptionParams()


def params_for(embodiment: Embodiment | None = None,
               base: PerceptionParams = DEFAULT_PARAMS) -> PerceptionParams:
    """base with the embodiment's support region and support thresholds (None = DROID)."""
    ws = embodiment_or_droid(embodiment).spec.workspace
    return dataclasses.replace(base, table=table_region(ws))
