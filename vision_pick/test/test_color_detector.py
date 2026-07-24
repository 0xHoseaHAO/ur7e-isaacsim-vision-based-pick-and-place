import cv2
import numpy as np
import pytest

from vision_pick.color_detector import COLOR_RANGES, detect_color
from vision_pick.depth_processor import depth_at_pixel, project_pixel_to_3d


def test_red_centroid_and_mask():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (50, 30), (110, 90), (0, 0, 255), -1)

    result = detect_color(image, 'red')

    assert result is not None
    u, v, mask, (bbox_w, bbox_h) = result
    assert (u, v) == (80, 60)
    assert mask[v, u] == 255
    assert (bbox_w, bbox_h) == (61, 61)  # cv2.rectangle's corners are inclusive


@pytest.mark.parametrize('color', ['red', 'orange', 'yellow', 'green', 'blue', 'purple'])
def test_six_colors_supported(color):
    assert color in COLOR_RANGES


def test_depth_and_back_projection():
    depth_mm = np.zeros((5, 5), dtype=np.uint16)
    depth_mm[1:4, 1:4] = 800

    z = depth_at_pixel(depth_mm, 2, 2, depth_scale=0.001, window_radius=1)

    assert z == 0.8
    assert project_pixel_to_3d(420, 240, z, 200.0, 200.0, 320.0, 240.0) == (
        0.4,
        0.0,
        0.8,
    )
