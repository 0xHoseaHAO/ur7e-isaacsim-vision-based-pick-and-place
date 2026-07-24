"""RGB-D back-projection helpers."""

from typing import Optional, Tuple

import numpy as np


def depth_at_pixel(
    depth_image: np.ndarray,
    u: int,
    v: int,
    depth_scale: float = 0.8,
    window_radius: int = 2,
) -> Optional[float]:
    """Return robust depth in metres near a pixel, or ``None`` if unavailable.

    ``16UC1`` depth images are normally in millimetres, so callers pass 0.001.
    ``32FC1`` images are conventionally already in metres and use 1.0.
    """
    if not (0 <= u < depth_image.shape[1] and 0 <= v < depth_image.shape[0]):
        return None
    x0, x1 = max(0, u - window_radius), min(depth_image.shape[1], u + window_radius + 1)
    y0, y1 = max(0, v - window_radius), min(depth_image.shape[0], v + window_radius + 1)
    values = depth_image[y0:y1, x0:x1].astype(np.float64).reshape(-1) * depth_scale
    values = values[np.isfinite(values) & (values > 0.0)]
    return float(np.median(values)) if values.size else None


def project_pixel_to_3d(
    u: int,
    v: int,
    depth_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """Project an image pixel to camera optical-frame coordinates in metres."""
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError('CameraInfo contains invalid focal lengths.')
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return x, y, depth_m
