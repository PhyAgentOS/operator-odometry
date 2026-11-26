import pytest

from odometry.simulation_clock import SimulationClock


def test_pause_is_allowed_but_clock_reset_is_rejected():
    clock = SimulationClock()
    assert clock.observe(1_000_000_000)
    assert not clock.observe(1_000_000_000)
    assert clock.observe(1_002_000_000)
    with pytest.raises(ValueError, match="regressed"):
        clock.observe(2_000_000)


@pytest.mark.parametrize("stamp", [None, True, 1.5, "1000", -1, 0, 2_147_483_648 * 10**9])
def test_clock_cannot_publish_invalid_ros_timestamp(stamp):
    with pytest.raises(ValueError):
        SimulationClock().observe(stamp)
