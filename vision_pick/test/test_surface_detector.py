import numpy as np

from vision_pick.surface_detector import detect_surface, detect_surfaces


def test_detect_surface_finds_a_large_near_plane():
    depth = np.full((100, 100), 10.0, dtype=np.float32)  # far background (e.g. the floor)
    depth[20:80, 20:80] = 1.0  # a big near plane filling well over min_area_fraction

    found = detect_surface(depth, min_area_fraction=0.15)

    assert found is not None
    cx, cy, z, (bbox_w, bbox_h) = found
    assert abs(z - 1.0) < 1e-6
    assert 20 <= cx <= 80 and 20 <= cy <= 80
    assert bbox_w > 50 and bbox_h > 50


def test_detect_surface_prefers_the_nearer_plane_even_if_smaller():
    # A small-but-near table should win over a big-but-far floor visible
    # past its edge -- nearest-first, not a pure pixel-count vote.
    depth = np.full((100, 100), 5.0, dtype=np.float32)  # the floor, most of the frame
    depth[10:60, 10:60] = 1.0  # the table: nearer, still clears min_area_fraction

    found = detect_surface(depth, min_area_fraction=0.15)

    assert found is not None
    assert abs(found[2] - 1.0) < 1e-6


def test_detect_surface_skips_regions_below_the_area_threshold():
    depth = np.full((100, 100), 5.0, dtype=np.float32)
    depth[0:5, 0:5] = 1.0  # a tiny near blob -- too small to be "a surface"

    found = detect_surface(depth, min_area_fraction=0.15)

    assert found is not None
    assert abs(found[2] - 5.0) < 1e-6  # falls through to the actual (far) surface


def test_detect_surfaces_finds_a_box_on_a_table_as_a_separate_entry():
    depth = np.full((100, 100), 5.0, dtype=np.float32)  # floor
    depth[10:90, 10:90] = 2.0  # table
    depth[40:60, 40:60] = 1.0  # a box sitting on the table, nearer still

    found = detect_surfaces(depth, depth_scale=1.0, min_area_fraction=0.03, max_surfaces=4)
    depths_found = [round(patch[2], 3) for patch in found]

    assert depths_found[0] == 1.0  # nearest first: the box
    assert 2.0 in depths_found  # the table is still its own entry
    assert 5.0 in depths_found  # and the floor beyond it


def test_detect_surfaces_respects_max_surfaces():
    depth = np.full((100, 100), 5.0, dtype=np.float32)
    depth[10:90, 10:90] = 2.0
    depth[40:60, 40:60] = 1.0

    found = detect_surfaces(depth, depth_scale=1.0, min_area_fraction=0.03, max_surfaces=1)

    assert len(found) == 1
    assert round(found[0][2], 3) == 1.0


def test_detect_surface_handles_a_tolerance_boundary_without_dropping_pixels():
    # Regression test: re-deriving a float depth threshold from a bin index
    # (instead of comparing bin indices directly) could spuriously exclude
    # real pixels sitting exactly at the tolerance edge due to binary
    # floating-point rounding (0.01 isn't exact). Build a plane whose depth
    # sits precisely on a bin boundary and make sure it's still found whole.
    depth = np.full((100, 100), 9.0, dtype=np.float32)
    depth[10:90, 10:90] = 0.48  # 48 * bin_size(0.01) -- exactly on a bin edge

    found = detect_surface(depth, tolerance=0.02, min_area_fraction=0.15)

    assert found is not None
    assert abs(found[2] - 0.48) < 1e-6
    _, _, _, (bbox_w, bbox_h) = found
    assert bbox_w >= 79 and bbox_h >= 79  # the whole 80x80 plane, not a clipped fragment


def test_detect_surfaces_returns_nothing_when_view_is_empty():
    depth = np.zeros((100, 100), dtype=np.float32)  # no valid depth anywhere

    assert detect_surfaces(depth) == []
    assert detect_surface(depth) is None
