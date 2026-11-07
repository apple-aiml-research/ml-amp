#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""RGB image encode/decode: cv2, then PIL, then a stdlib PNG codec (no hard dependency)."""

from __future__ import annotations

import io
import struct
import zlib

import numpy as np

_PNG_SIG = b"\x89PNG\r\n\x1a\n"
_JPEG_SIG = b"\xff\xd8"
_MIME = {"jpeg": "image/jpeg", "jpg": "image/jpeg", "png": "image/png"}


def _as_rgb(rgb) -> np.ndarray:
    arr = np.asarray(rgb)
    if arr.ndim == 2:
        arr = np.repeat(arr[:, :, None], 3, axis=2)
    if arr.ndim != 3 or arr.shape[2] not in (3, 4):
        raise ValueError(f"expected an (H, W, 3) image, got shape {arr.shape}")
    arr = arr[:, :, :3]
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(arr)


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))


def _png_encode(rgb: np.ndarray, level: int = 6) -> bytes:
    h, w, _ = rgb.shape
    rows = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)], axis=1)
    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return b"".join([
        _PNG_SIG,
        _png_chunk(b"IHDR", ihdr),
        _png_chunk(b"IDAT", zlib.compress(rows.tobytes(), level)),
        _png_chunk(b"IEND", b""),
    ])


def _paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def _unfilter(ftype: int, line: np.ndarray, prev: np.ndarray, channels: int) -> np.ndarray:
    stride = len(line)
    if ftype == 0:
        return line.copy()
    if ftype == 1:
        cur = np.cumsum(line.reshape(-1, channels).astype(np.uint64), axis=0) % 256
        return cur.astype(np.uint8).reshape(stride)
    if ftype == 2:
        return line + prev
    if ftype not in (3, 4):
        raise ValueError(f"bad PNG filter type {ftype}")
    cur = np.zeros(stride, np.uint8)
    for i in range(stride):
        a = int(cur[i - channels]) if i >= channels else 0
        b = int(prev[i])
        if ftype == 3:
            pred = (a + b) // 2
        else:
            pred = _paeth(a, b, int(prev[i - channels]) if i >= channels else 0)
        cur[i] = (int(line[i]) + pred) & 0xFF
    return cur


def _png_decode(data: bytes) -> np.ndarray:
    if not data.startswith(_PNG_SIG):
        raise ValueError("not a PNG")
    pos, idat, ihdr = len(_PNG_SIG), [], None
    while pos + 8 <= len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        kind = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if kind == b"IHDR":
            ihdr = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat.append(body)
        elif kind == b"IEND":
            break
    if ihdr is None:
        raise ValueError("PNG without IHDR")
    w, h, depth, color, _, _, interlace = ihdr
    channels = {0: 1, 2: 3, 6: 4}.get(color)
    if depth != 8 or channels is None or interlace != 0:
        raise ValueError(f"unsupported PNG (depth {depth}, color {color}, interlace {interlace})")
    stride = w * channels
    raw = np.frombuffer(zlib.decompress(b"".join(idat)), np.uint8)
    if raw.size != h * (stride + 1):
        raise ValueError("PNG data size mismatch")
    raw = raw.reshape(h, stride + 1)
    out = np.zeros((h, stride), np.uint8)
    prev = np.zeros(stride, np.uint8)
    for y in range(h):
        out[y] = prev = _unfilter(int(raw[y, 0]), raw[y, 1:], prev, channels)
    return _as_rgb(out.reshape(h, w, channels))


def encode_image(rgb, fmt: str = "jpeg", quality: int = 85) -> tuple[bytes, str]:
    """Encode (H, W, 3|4) uint8 RGB; returns (data, the encoding actually used)."""
    fmt = {"jpg": "jpeg"}.get(fmt.lower(), fmt.lower())
    if fmt not in ("jpeg", "png"):
        raise ValueError(f"unsupported image format {fmt!r}")
    img = _as_rgb(rgb)
    try:
        import cv2

        if fmt == "jpeg":
            ext, params = ".jpg", [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        else:
            ext, params = ".png", []
        ok, buf = cv2.imencode(ext, np.ascontiguousarray(img[:, :, ::-1]), params)
        if ok:
            return buf.tobytes(), fmt
    except ImportError:
        pass
    try:
        from PIL import Image

        out = io.BytesIO()
        kwargs = {"quality": int(quality)} if fmt == "jpeg" else {}
        Image.fromarray(img, "RGB").save(out, format=fmt.upper(), **kwargs)
        return out.getvalue(), fmt
    except ImportError:
        pass
    return _png_encode(img), "png"


def decode_image(data) -> np.ndarray:
    """Decode JPEG/PNG bytes to (H, W, 3) uint8 RGB."""
    data = bytes(data)
    try:
        import cv2

        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            return np.ascontiguousarray(bgr[:, :, ::-1])
    except ImportError:
        pass
    try:
        from PIL import Image

        with Image.open(io.BytesIO(data)) as im:
            return np.asarray(im.convert("RGB")).copy()
    except ImportError:
        pass
    if data.startswith(_PNG_SIG):
        return _png_decode(data)
    kind = "JPEG" if data.startswith(_JPEG_SIG) else "image"
    raise ValueError(f"cannot decode {kind}: neither cv2 nor PIL is installed")


def mime_for(encoding: str) -> str:
    """MIME type for a blob encoding."""
    return _MIME.get(encoding, "application/octet-stream")
