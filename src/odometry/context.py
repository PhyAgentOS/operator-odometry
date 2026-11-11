"""Required sensor metadata, independent of Dora's transport timestamp."""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SampleContext:
    capture_timestamp_ns: int
    frame_id: str
    clock_domain: str
    time_mapping_id: str
    ros_time_offset_ns: int

    @classmethod
    def from_metadata(cls, metadata: Mapping[str, Any]) -> "SampleContext":
        values = {}
        for name in cls.__dataclass_fields__:
            value = metadata.get(name)
            if name in ("capture_timestamp_ns", "ros_time_offset_ns"):
                if isinstance(value, bool) or not isinstance(value, int):
                    raise ValueError(f"missing or invalid integer metadata: {name}")
            elif not isinstance(value, str) or not value:
                raise ValueError(f"missing or invalid metadata: {name}")
            values[name] = value
        context = cls(**values)
        if context.capture_timestamp_ns < 0:
            raise ValueError("negative source timestamp")
        if not 0 <= context.ros_ns <= 2_147_483_647 * 10**9 + 999_999_999:
            raise ValueError("mapped timestamp outside ROS2 Time range")
        return context

    @property
    def ros_ns(self) -> int:
        return self.capture_timestamp_ns + self.ros_time_offset_ns


class StreamClockGuard:
    """Reject time regressions and mixed point/IMU clock mappings.

    Reset by creating a new guard when restarting the backend; silently accepting
    a sensor reboot inside an active LIO session would corrupt integration.
    """

    def __init__(self) -> None:
        self._mapping: tuple | None = None
        self._last: dict[str, int] = {}

    def observe(self, stream: str, context: SampleContext) -> None:
        mapping = (context.clock_domain, context.time_mapping_id, context.ros_time_offset_ns)
        if self._mapping is not None and mapping != self._mapping:
            raise ValueError("point and IMU clock mappings differ or changed during a session")
        if stream in self._last and context.capture_timestamp_ns <= self._last[stream]:
            raise ValueError(f"{stream} timestamp repeated or regressed; restart LIO session")
        self._mapping = mapping
        self._last[stream] = context.capture_timestamp_ns
