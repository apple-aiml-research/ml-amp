#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Sensor routes: camera frames, the fused point cloud, the scene meshes."""

from __future__ import annotations

from urllib.parse import unquote

from amp.geometry import world as scene_mod
from amp.link import wire
from amp.robot_server.api import Reply, binary
from amp.robot_server.errors import HTTPError


def frame(srv, req) -> Reply:
    cam = req.params["cam"]
    with srv.cond:
        got = srv.obs.frame(cam)
    if got is None:
        raise HTTPError(404, f"no frame from camera {cam!r} yet")
    data, mime, seq = got
    return binary(data, mime, **{"X-Frame-Seq": str(seq)})


def path_query(rest: str) -> dict:
    """The query dict of a query-free path: <k>=<v>/<k>=<v>..., values percent-encoded."""
    query = {}
    for seg in rest.split("/"):
        if not seg:
            continue
        key, eq, value = seg.partition("=")
        if not eq or not key:
            raise ValueError(f"expected <key>=<value> path segments, got {seg!r}")
        query.setdefault(unquote(key), []).append(unquote(value))
    return query


def cloud(srv, req) -> Reply:
    query = path_query(req.params["query"]) if "query" in req.params else req.query

    def arg(name, default):
        v = query.get(name)
        return v[-1] if v else default

    cams = [c for c in arg("cams", "").split(",") if c] or None
    crop = arg("crop", "1").lower() not in ("0", "false", "no", "")
    depth = arg("max_depth", None)
    stride, max_depth = int(arg("stride", "1")), None if depth is None else float(depth)
    with srv.cond:
        key, frames, cached = srv.obs.cloud_request(cams, stride, max_depth, crop,
                                                    arg("frame", "root"))
        emb, obs = srv.embodiment, srv.obs.obs
        T_world_base = None if obs is None else obs.T_world_base
    if cached is None:
        cached = srv.obs.pack_cloud(key, frames, emb, T_world_base)
        with srv.cond:
            srv.obs.cache_cloud(key, cached)
    return binary(cached, wire.CONTENT_TYPE)


def scene(srv, req) -> Reply:
    """GET api/scene: TLF1 bytes of the scene with display-decimated meshes."""
    with srv.cond:
        snap = srv.world.display_snapshot()
    if snap is None:
        raise HTTPError(404, "the current episode has no scene")
    sc, meta, meshes = snap
    if meshes is None:
        meshes = scene_mod.display_meshes(sc)
        with srv.cond:
            srv.world.cache_display(sc.version, meshes)
    return binary(scene_mod.display_frame(sc, meshes, meta), wire.CONTENT_TYPE)
