import numpy as np
import pytest

from odometry.primary_scan import LegacyGraphBootstrap, PrimaryScanProjector, origin_mux_index


def config():
    return dict(
        sensor_translation_xyz_m=[0.3, 0, 0.149],
        sensor_rotation_matrix=np.eye(3).tolist(),
        body_to_base_xyz_m=[-0.3, 0, -0.14],
        body_to_base_yaw_rad=0.0,
        source_sensor_count=2,
        min_height=-0.1,
        max_height=0.8,
        range_min=0.5,
        range_max=20.0,
    )


def test_real_primary_origin_nearest_hit_and_no_fabricated_missing_beams():
    p = PrimaryScanProjector(config())
    ranges = p.project([[2, 0, 0.009, 100], [3, 0, 0.009, 100], [0.7, 0, 0.109, 101]])
    assert ranges[360] == pytest.approx(2.0)  # virtual base would incorrectly be 2.3m
    assert np.isfinite(ranges).sum() == 1
    assert np.isinf(np.delete(ranges, 360)).all()


def test_real_base_height_filter_and_sensor_local_angles_are_preserved():
    p = PrimaryScanProjector(config())
    ranges = p.project([[0, 2, 0.009, 100], [0, -2, 1.0, 100], [0.1, 0, 0.009, 100]])
    assert ranges[540] == pytest.approx(2.0)
    assert np.isfinite(ranges).sum() == 1  # high and too-near returns not invented


@pytest.mark.parametrize("label", [100.5, 102.0, float("nan")])
def test_lost_or_averaged_source_identity_is_rejected(label):
    with pytest.raises(ValueError):
        PrimaryScanProjector(config()).project([[2, 0, 0.009, label]])


def test_only_verified_supported_upright_extrinsics_are_accepted():
    cfg = config()
    cfg["body_to_base_yaw_rad"] = 0.1
    with pytest.raises(ValueError):
        PrimaryScanProjector(cfg)


def test_rear_uses_only_actual_rear_points_and_its_distinct_height_origin():
    cfg = config()
    cfg.update(source_sensor_index=1, sensor_translation_xyz_m=[-0.28, 0, 0.249])
    ranges = PrimaryScanProjector(cfg).project([[1.42, 0, 0.109, 101], [0.7, 0, 0.009, 100]])
    assert ranges[360] == pytest.approx(2.0)
    assert np.isfinite(ranges).sum() == 1


def test_mux_covers_both_sources_despite_one_second_native_acceptance_interval():
    accepted = [origin_mux_index(n * 1_000_000_000, 2) for n in range(22)]
    assert set(accepted) == {0, 1}
    assert accepted.count(0) >= 9 and accepted.count(1) >= 9
    assert origin_mux_index(1_100_000_000, 2) == 1
    with pytest.raises(ValueError):
        origin_mux_index(-1, 2)


def test_physical_origins_wait_for_actual_native_ack_of_current_legacy_scan():
    bootstrap = LegacyGraphBootstrap()
    assert not bootstrap.accepted_pose(90) and not bootstrap.ready
    assert bootstrap.forwarded(100)
    assert not bootstrap.accepted_pose(99)
    assert bootstrap.accepted_pose(100) and bootstrap.accepted_stamp == 100
    assert not bootstrap.forwarded(101)
    cfg = config()
    cfg["sensor_rotation_matrix"][0][0] = 2
    with pytest.raises(ValueError):
        PrimaryScanProjector(cfg)
