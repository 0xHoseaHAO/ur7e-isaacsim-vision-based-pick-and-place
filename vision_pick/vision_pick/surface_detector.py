"""Depth-based flat-surface detection.

A generic "find the nearest sizeable flat surface(s) in view" detector --
not tied to colour or any specific object model (a table, a box sitting on
it, a shelf, ...), so it works the same way for any of them: whatever
near-planar region is both closest to the camera and big enough to be a
real surface rather than noise or a small object sitting on top of it.
"""

from typing import List, Optional, Tuple

import cv2
import numpy as np


# (centroid_x, centroid_y, representative_depth_m, (bbox_width_px, bbox_height_px))
SurfacePatch = Tuple[int, int, float, Tuple[int, int]]


def detect_surfaces(
    depth_image: np.ndarray,
    depth_scale: float = 2.0,
    tolerance: float = 0.02,
    min_area_fraction: float = 0.05,
    max_surfaces: int = 10,
) -> List[SurfacePatch]:
    """Find every distinct near-planar region in a depth image, nearest
    first: a box sitting on a table gets its own entry separate from the
    table itself, since they're at different depths, rather than one
    "biggest surface" swallowing or hiding the other.

    Repeatedly runs the same single-surface scan, each time excluding the
    pixels the previous pass already claimed, until nothing left clears
    ``min_area_fraction`` or ``max_surfaces`` is reached.
    """
    depths = depth_image.astype(np.float64) * depth_scale
    remaining = np.isfinite(depths) & (depths > 0.0)

    surfaces = []
    for _ in range(max_surfaces):
        found = _detect_one_surface(depths, remaining, tolerance, min_area_fraction)
        if found is None:
            break
        patch, mask = found
        surfaces.append(patch)
        remaining = remaining & ~mask
    return surfaces


def detect_surface(
    depth_image: np.ndarray,
    depth_scale: float = 0.8,
    tolerance: float = 0.02,
    min_area_fraction: float = 0.15,
) -> Optional[SurfacePatch]:
    """Find just the single nearest sizeable near-planar region. See
    :func:`detect_surfaces` for the general (possibly multi-surface) case."""
    surfaces = detect_surfaces(
        depth_image,
        depth_scale=depth_scale,
        tolerance=tolerance,
        min_area_fraction=min_area_fraction,
        max_surfaces=1,
    )
    return surfaces[0] if surfaces else None


def _detect_one_surface(depths, valid, tolerance, min_area_fraction):
    """Nearest sizeable near-planar region among pixels where ``valid`` is
    True. Returns ``(SurfacePatch, mask)`` for the matched pixels, or
    ``None``.

    Bins valid depth readings into 1cm buckets and scans them nearest
    (smallest depth) first, picking the first one whose tolerance-window
    pixel count clears ``min_area_fraction`` of the *whole frame* --
    nearest-first (rather than just "most pixels overall") matters for an
    eye-in-hand camera: the working surface is whatever sizeable flat
    thing is closest to the camera, while a much bigger but farther-away
    plane (the room's floor, visible past a table's edge) would otherwise
    win a pure pixel-count vote despite not being what the arm is actually
    working over. Small objects (a rod on the surface) get skipped
    automatically since they're not big enough to clear the area threshold.
    """
    if not np.any(valid):
        return None

    bin_size = 0.01
    tolerance_bins = max(1, int(round(tolerance / bin_size)))
    # Bin indices for every pixel (not just the valid ones) so they can be
    # reused directly as a mask below -- comparing against a bin *index*
    # keeps everything in integers. Re-deriving a float depth threshold
    # from the bin (e.g. `abs(depths - chosen_bin * bin_size) <= tolerance`)
    # sounds equivalent but isn't: bin_size=0.01 isn't exactly representable
    # in binary floating point, so a real depth sitting exactly at the
    # tolerance edge can end up spuriously excluded by rounding error.
    all_bins = np.round(depths / bin_size).astype(np.int64)
    offset = int(all_bins[valid].min())
    counts = np.bincount(all_bins[valid] - offset)
    cumulative = np.concatenate(([0], np.cumsum(counts)))

    min_pixels = min_area_fraction * depths.size
    chosen_bin = None
    for b in range(counts.size):  # ascending depth order == nearest first
        lo = max(0, b - tolerance_bins)
        hi = min(counts.size, b + tolerance_bins + 1)
        if cumulative[hi] - cumulative[lo] >= min_pixels:
            # Report/mask around the actual peak within this window, not
            # scan position b itself -- b is just wherever the window
            # first covered enough pixels, which may have few or none of
            # its own (the real cluster could sit anywhere inside it).
            chosen_bin = lo + int(np.argmax(counts[lo:hi]))
            break
    if chosen_bin is None:
        return None

    mode_depth = float((chosen_bin + offset) * bin_size)
    depth_window_mask = valid & (np.abs(all_bins - (chosen_bin + offset)) <= tolerance_bins)

    contours, _ = cv2.findContours(
        (depth_window_mask * 255).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)

    moments = cv2.moments(contour)
    if moments['m00'] == 0:
        return None
    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    _, _, bbox_w, bbox_h = cv2.boundingRect(contour)

    # Only this contour's own pixels get excluded from the next pass, not
    # every pixel at this depth: an object (e.g. a box) sitting mid-surface
    # can split one real surface into two disconnected same-depth patches,
    # and excluding the whole depth window here would silently drop the
    # smaller patch instead of letting a later pass report it as its own
    # surface.
    contour_mask = np.zeros(depth_window_mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 1, thickness=cv2.FILLED)
    return (cx, cy, mode_depth, (bbox_w, bbox_h)), contour_mask.astype(bool)
