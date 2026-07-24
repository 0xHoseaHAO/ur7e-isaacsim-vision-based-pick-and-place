"""Colour segmentation utilities used by the ROS 2 node."""

from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


HSVRange = Tuple[np.ndarray, np.ndarray]
# (centroid_x, centroid_y, mask, (bbox_width_px, bbox_height_px))
Detection = Tuple[int, int, np.ndarray, Tuple[int, int]]

COLOR_RANGES: Dict[str, List[HSVRange]] = {
    'red': [
        (np.array([0, 100, 100]), np.array([10, 255, 255])),
        (np.array([170, 100, 100]), np.array([180, 255, 255])),
    ],
    'orange': [(np.array([10, 100, 100]), np.array([25, 255, 255]))],
    'yellow': [(np.array([25, 100, 100]), np.array([35, 255, 255]))],
    'green': [(np.array([35, 50, 50]), np.array([85, 255, 255]))],
    'blue': [(np.array([85, 100, 100]), np.array([130, 255, 255]))],
    'purple': [(np.array([130, 50, 50]), np.array([160, 255, 255]))],
}


def detect_color(
    image: np.ndarray,
    target: str,
    min_area: float = 100.0,
) -> Optional[Detection]:
    """Return centroid, mask, and pixel bounding-box size for the target region.

    Args:
        image: BGR image as returned by ``cv_bridge`` with ``bgr8`` encoding.
        target: One of the keys in :data:`COLOR_RANGES`.
        min_area: Reject contours smaller than this number of pixels.

    The bounding-box size lets a caller with depth + intrinsics turn a pixel
    footprint into a real-world size estimate, instead of that size having
    to be hand-configured.
    """
    if target not in COLOR_RANGES:
        raise ValueError(
            f'Unsupported target_color {target!r}. '
            f'Choose one of: {", ".join(sorted(COLOR_RANGES))}'
        )

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[target]:
        mask |= cv2.inRange(hsv, lower, upper)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not valid_contours:
        return None

    contour = max(valid_contours, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if moments['m00'] == 0:
        return None

    cx = int(moments['m10'] / moments['m00'])
    cy = int(moments['m01'] / moments['m00'])
    _, _, bbox_w, bbox_h = cv2.boundingRect(contour)
    return cx, cy, mask, (bbox_w, bbox_h)
