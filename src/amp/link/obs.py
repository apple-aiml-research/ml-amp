#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The /sim/step observation: build its header and blobs, and parse them back."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from amp.link.imaging import encode_image
from amp.link.pointcloud import CameraModel
from amp.link.wire import jsonable

_F16_MAX = 65504.0


@dataclass
class CameraFrame:
    """One camera of an observation: the encoded image, and the subsampled depth if sent."""

    model: CameraModel
    image: bytes
    encoding: str
    depth_model: CameraModel | None
    rgb_small: np.ndarray | None
    depth: np.ndarray | None


@dataclass
class Observation:
    """A parsed /sim/step frame; header is the raw frame header."""

    episode: int
    step: int
    sim_time: float
    control_dt: float
    max_steps: int | None
    instruction: str
    task: str
    q: np.ndarray
    gripper: float
    ee_pos: np.ndarray
    ee_quat: np.ndarray
    cameras: dict[str, CameraFrame]
    gt: dict | None
    header: dict
    ee: dict[str, tuple[np.ndarray, np.ndarray]] = field(default_factory=dict)
    grippers: dict[str, float] = field(default_factory=dict)
    T_world_base: np.ndarray | None = None
    body_contacts: tuple[str, ...] | None = None
    contacts_unavailable: bool = False
    env_error: str | None = None


MAX_BODY_CONTACTS = 64
MAX_ENV_ERROR = 2000
ENV_ERROR_LOG_EVERY = 64


def env_error_logged(n: int) -> bool:
    """Whether the n-th env_error in a row is logged: the 1st, 2nd, 4th, 8th, ... and every 64th."""
    return n & (n - 1) == 0 or n % ENV_ERROR_LOG_EVERY == 0
MAX_CONTACT_NAME = 200


def _depth_f16(depth: np.ndarray) -> np.ndarray:
    d = np.asarray(depth, dtype=np.float32)
    with np.errstate(invalid="ignore"):
        bad = ~np.isfinite(d) | (d > _F16_MAX) | (d < 0)
    return np.where(bad, 0.0, d).astype(np.float16)


def k_for_stride(K, stride: int) -> np.ndarray:
    """K of the depth[::stride, ::stride] subsample."""
    K2 = np.array(K, dtype=np.float64).reshape(3, 3)
    K2[:2] /= stride
    K2[2] = (0.0, 0.0, 1.0)
    return K2


def _pack_camera(model: CameraModel, rgb, depth, stride: int, jpeg_quality: int):
    """(header entry, blobs, encodings) for one camera, blob names without the camera suffix."""
    rgb = np.asarray(rgb)
    if rgb.ndim == 3 and rgb.shape[2] == 4:
        rgb = rgb[:, :, :3]
    h, w = rgb.shape[:2]
    if (w, h) != (model.width, model.height):
        model = model.scaled(w, h)
    data, enc = encode_image(rgb, "jpeg", jpeg_quality)
    blobs: dict[str, np.ndarray | bytes] = {"rgb": data}
    entry = model.to_json()
    if depth is not None:
        d = np.asarray(depth, dtype=np.float32)
        if d.ndim == 3:
            d = d[:, :, 0]
        ds = d[::stride, ::stride]
        dh, dw = ds.shape
        entry.update(depth_width=dw, depth_height=dh,
                     K_depth=k_for_stride(model.K, stride).tolist())
        blobs["depth"] = _depth_f16(ds)
        blobs["rgb_small"] = np.ascontiguousarray(rgb[::stride, ::stride][:dh, :dw], dtype=np.uint8)
    return entry, blobs, {"rgb": enc}


def _vec(v) -> np.ndarray:
    return np.asarray(v, dtype=np.float64).reshape(-1)


def _scalar(v) -> float:
    return float(np.asarray(v).reshape(-1)[0])


def _proprio(q, gripper, ee_pos, ee_quat, ee, grippers) -> dict:
    if ee is None:
        if gripper is None or ee_pos is None or ee_quat is None:
            raise ValueError("proprio needs gripper, ee_pos and ee_quat, or ee and grippers")
        return {"q": _vec(q), "gripper": _scalar(gripper), "ee_pos": _vec(ee_pos),
                "ee_quat": _vec(ee_quat)}
    if gripper is not None or ee_pos is not None or ee_quat is not None:
        raise ValueError("proprio takes ee and grippers, or gripper/ee_pos/ee_quat, not both")
    grippers = grippers or {}
    if not set(grippers) <= set(ee):
        raise ValueError(f"grippers names unknown end effectors {sorted(set(grippers) - set(ee))}")
    return {"q": _vec(q),
            "ee": {str(n): {"pos": _vec(p), "quat": _vec(r)} for n, (p, r) in ee.items()},
            "grippers": {str(n): _scalar(v) for n, v in grippers.items()}}


def pack_observation(
    *,
    episode: int,
    step: int,
    control_dt: float,
    max_steps: int | None,
    instruction: str,
    task: str,
    q,
    gripper: float | None = None,
    ee_pos=None,
    ee_quat=None,
    cameras: dict[str, tuple[CameraModel, np.ndarray, np.ndarray | None]],
    depth_stride: int = 4,
    jpeg_quality: int = 85,
    gt: dict | None = None,
    ee: dict | None = None,
    grippers: dict | None = None,
    T_world_base=None,
    body_contacts=None,
    contacts_unavailable: bool = False,
    env_error: str | None = None,
) -> tuple[dict, dict, dict]:
    """(header, blobs, encodings) for LinkClient.step; cameras maps name -> (model, rgb, depth)."""
    s = max(int(depth_stride), 1)
    cams: dict[str, dict] = {}
    blobs: dict[str, np.ndarray | bytes] = {}
    encodings: dict[str, str] = {}
    for name, (model, rgb, depth) in cameras.items():
        cams[name], cb, ce = _pack_camera(model, rgb, depth, s, jpeg_quality)
        blobs.update({f"{k}/{name}": v for k, v in cb.items()})
        encodings.update({f"{k}/{name}": v for k, v in ce.items()})
    header = {
        "episode": int(episode),
        "step": int(step),
        "sim_time": float(step) * float(control_dt),
        "control_dt": float(control_dt),
        "max_steps": None if max_steps is None else int(max_steps),
        "instruction": str(instruction),
        "task": str(task),
        "proprio": _proprio(q, gripper, ee_pos, ee_quat, ee, grippers),
        "cameras": cams,
        "gt": gt,
    }
    if T_world_base is not None:
        T = np.asarray(T_world_base, dtype=np.float64).reshape(-1)
        if T.shape != (16,) or not np.isfinite(T).all():
            raise ValueError("T_world_base is a finite 4x4 pose")
        header["proprio"]["T_world_base"] = T
    names = [str(n) for n in body_contacts] if body_contacts is not None else []
    if names:
        if len(names) > MAX_BODY_CONTACTS or \
                any(not n or len(n) > MAX_CONTACT_NAME for n in names):
            raise ValueError(f"body_contacts is 1 to {MAX_BODY_CONTACTS} names of up to "
                             f"{MAX_CONTACT_NAME} characters, or None for no contact")
        header["proprio"]["body_contacts"] = names
    if contacts_unavailable:
        header["proprio"]["contacts_unavailable"] = True
    if env_error:
        header["proprio"]["env_error"] = str(env_error)[:MAX_ENV_ERROR]
    return jsonable(header), blobs, encodings


def _unpack_camera(name: str, cam: dict, blobs: dict, meta: dict) -> CameraFrame:
    model = CameraModel.from_json(name, cam)
    key = f"rgb/{name}"
    image = blobs.get(key, b"")
    depth = blobs.get(f"depth/{name}")
    rgb_small = blobs.get(f"rgb_small/{name}")
    depth_model = None
    if depth is not None:
        depth = np.asarray(depth, dtype=np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        dh, dw = depth.shape
        K_depth = cam.get("K_depth")
        if K_depth is None:
            depth_model = model.scaled(dw, dh)
        else:
            depth_model = CameraModel(name, dw, dh, K_depth, model.T_world_cam.copy())
    return CameraFrame(
        model=model,
        image=image if isinstance(image, bytes) else bytes(image),
        encoding=(meta.get(key) or {}).get("encoding", "bytes"),
        depth_model=depth_model,
        rgb_small=None if rgb_small is None else np.asarray(rgb_small, dtype=np.uint8),
        depth=depth,
    )


def _unpack_ee(proprio: dict) -> tuple[dict, dict]:
    ee = {str(n): (np.asarray(v.get("pos", np.zeros(3)), dtype=np.float64),
                   np.asarray(v.get("quat", (1.0, 0.0, 0.0, 0.0)), dtype=np.float64))
          for n, v in (proprio.get("ee") or {}).items()}
    grippers = {str(n): float(v) for n, v in (proprio.get("grippers") or {}).items()}
    return ee, grippers


def unpack_observation(header: dict, blobs: dict, dof: int = 7) -> Observation:
    """Parse a /sim/step frame (already split by unpack_frame); dof sizes a missing q."""
    meta = header.get("blobs") or {}
    cameras = {name: _unpack_camera(name, cam, blobs, meta)
               for name, cam in (header.get("cameras") or {}).items()}
    proprio = header.get("proprio") or {}
    ee, grippers = _unpack_ee(proprio)
    if ee:
        first = next(iter(ee))
        proprio = {"ee_pos": ee[first][0], "ee_quat": ee[first][1],
                   "gripper": grippers.get(first, 0.0), **proprio}
    control_dt = float(header.get("control_dt") or 0.0)
    step = int(header.get("step", 0))
    T_world_base = proprio.get("T_world_base")
    contacts = proprio.get("body_contacts")
    return Observation(
        episode=int(header.get("episode", 0)),
        step=step,
        sim_time=float(header.get("sim_time", step * control_dt)),
        control_dt=control_dt,
        max_steps=header.get("max_steps"),
        instruction=str(header.get("instruction", "")),
        task=str(header.get("task", "")),
        q=np.asarray(proprio.get("q", np.zeros(dof)), dtype=np.float64),
        gripper=float(proprio.get("gripper", 0.0)),
        ee_pos=np.asarray(proprio.get("ee_pos", np.zeros(3)), dtype=np.float64),
        ee_quat=np.asarray(proprio.get("ee_quat", (1.0, 0.0, 0.0, 0.0)), dtype=np.float64),
        cameras=cameras,
        gt=header.get("gt"),
        header=header,
        ee=ee,
        grippers=grippers,
        T_world_base=None if T_world_base is None else np.asarray(
            T_world_base, dtype=np.float64).reshape(4, 4),
        body_contacts=tuple(str(n) for n in contacts)[:MAX_BODY_CONTACTS]
        if isinstance(contacts, (list, tuple)) and contacts else None,
        contacts_unavailable=proprio.get("contacts_unavailable") is True,
        env_error=str(proprio["env_error"])[:MAX_ENV_ERROR]
        if isinstance(proprio.get("env_error"), str) and proprio["env_error"] else None,
    )
