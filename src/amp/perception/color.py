#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""Colour features for segmentation and coarse English colour names for labels."""

from __future__ import annotations

import colorsys

import numpy as np


def color_feat(rgb: np.ndarray, w_i: float) -> np.ndarray:
    """Shading-tolerant colour feature: chroma over intensity, plus a little intensity."""
    c = np.asarray(rgb, np.float64) / 255.0
    i = c.mean(axis=1, keepdims=True)
    return np.hstack([(c - i) / (i + 0.03), w_i * i])


def color_name(rgb) -> str:
    """Coarse English name of an RGB colour."""
    r, g, b = (float(v) / 255.0 for v in rgb)
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    if v < 0.18:
        return "black"
    if s < 0.18 or (s < 0.3 and v < 0.35):
        return "white" if v > 0.75 else "grey"
    deg = h * 360.0
    if deg < 15 or deg >= 340:
        name = "red"
    elif deg < 40:
        name = "orange"
    elif deg < 70:
        name = "yellow"
    elif deg < 165:
        name = "green"
    elif deg < 200:
        name = "teal"
    elif deg < 255:
        name = "blue"
    elif deg < 290:
        name = "purple"
    else:
        name = "pink"
    if name in ("red", "orange", "yellow") and v < 0.55 and s > 0.3:
        name = "brown"
    return name
