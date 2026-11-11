"""Local independent sampling also applies to the release's sparse-band fallback."""
import hashlib
import math

import numpy as np

from test_global_localization_query import glq, make_scan, write_occupancy_map


def test_sparse_band_query_hash_and_heading_use_the_same_rotated_scan(tmp_path):
    yaml_path = write_occupancy_map(tmp_path)
    config = glq.GlobalLocalizationConfig(
        independent_sampling=True, sparse_band_min_points=100000,
        max_scan_points=128, sampling_seed=7,
    )
    engine = glq.GlobalLocalizationEngine(config, occupancy_yaml=yaml_path)
    points = make_scan(yaml_path, -1.0, 7.0, math.radians(30.0))
    searched = []

    def capture_search(_grid, scan_xy, *_args, **_kwargs):
        searched.append(scan_xy.copy())
        return []

    engine._search = capture_search
    first = engine.query(points)
    second = engine.query(points)
    expected = glq.bbs_engine.prepare_scan_xy(
        points, glq.SPARSE_BAND_Z_MIN_M, config.z_max_m,
        engine.occupancy_map.resolution_m,
        max(config.max_scan_points, glq.SPARSE_BAND_MAX_SCAN_POINTS),
        min_range_m=config.min_range_m,
        yaw_offset_rad=config.angular_resolution_rad * 0.5,
    )
    # The fixture is smaller than the fallback cap, so sampling does not reorder it.
    assert len(expected) < glq.SPARSE_BAND_MAX_SCAN_POINTS
    assert second.sparse_band_search
    assert second.sampling_mode == "independent_random"
    np.testing.assert_allclose(searched[1], expected)
    assert second.scan_hash == hashlib.sha256(expected.astype("<f8").tobytes()).hexdigest()
    assert first.scan_hash != second.scan_hash
