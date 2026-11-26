"""Validate an explicitly configured simulation clock before publishing to ROS."""

from numbers import Integral


class SimulationClock:
    """One simulator owns the clock for a ROS domain; reset requires a new run."""

    def __init__(self):
        self.last_ns = None

    def observe(self, value):
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise ValueError("simulation clock must contain integer nanoseconds")
        value = int(value)
        if not 0 < value <= 2_147_483_647 * 10**9 + 999_999_999:
            raise ValueError("simulation clock outside positive ROS Time range")
        if self.last_ns is not None and value < self.last_ns:
            raise ValueError("simulation clock regressed; restart the navigation session")
        changed = value != self.last_ns
        self.last_ns = value
        return changed
