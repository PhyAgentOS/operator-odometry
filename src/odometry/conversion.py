"""Forge -> FAST-LIO2 ROS message content, testable without ROS installed."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from forge_msgs import Imu, PointCloudBuffer

from odometry.context import SampleContext


@dataclass(frozen=True)
class LivoxFrame:
    timebase: int
    frame_id: str
    lidar_id: int
    points: np.ndarray


def decode_livox(cloud: PointCloudBuffer, metadata: Mapping) -> LivoxFrame:
    context = SampleContext.from_metadata(metadata)
    expected = {
        "x": "float32",
        "y": "float32",
        "z": "float32",
        "time_offset_ns": "uint32",
        "reflectivity": "uint8",
        "line": "uint8",
        "tag": "uint8",
    }
    fields = {f.name: f for f in cloud.fields}
    for name, datatype in expected.items():
        if name not in fields or fields[name].datatype != datatype or fields[name].count != 1:
            raise ValueError(f"FAST-LIO requires {name}: {datatype}[1]")
    lidar_id = metadata.get("lidar_id")
    if isinstance(lidar_id, bool) or not isinstance(lidar_id, int) or not 0 <= lidar_id <= 255:
        raise ValueError("missing or invalid lidar_id")
    endian = "<" if cloud.byte_order == "little_endian" else ">"
    numpy_types = {"float32": endian + "f4", "uint32": endian + "u4", "uint8": "u1"}
    names = list(expected)
    dtype = np.dtype(
        {
            "names": names,
            "formats": [numpy_types[expected[n]] for n in names],
            "offsets": [fields[n].offset for n in names],
            "itemsize": cloud.point_stride,
        }
    )
    # Honour row padding and byte order; do not reinterpret arbitrary bytes as XYZ-only.
    points = np.ndarray(
        (cloud.height, cloud.width),
        dtype=dtype,
        buffer=cloud.data,
        strides=(cloud.row_stride, cloud.point_stride),
    ).reshape(-1)
    if len(points) == 0:
        raise ValueError("FAST-LIO requires a nonempty cloud")
    if not all(np.isfinite(points[k]).all() for k in ("x", "y", "z")):
        raise ValueError("non-finite cloud coordinates")
    if context.ros_ns + int(points["time_offset_ns"].max()) > 2_147_483_647 * 10**9 + 999_999_999:
        raise ValueError("point end time outside ROS2 Time range")
    return LivoxFrame(context.ros_ns, context.frame_id, lidar_id, points)


def imu_values(imu: Imu, metadata: Mapping) -> dict:
    context = SampleContext.from_metadata(metadata)
    return {
        "stamp_ns": context.ros_ns,
        "frame_id": context.frame_id,
        "orientation": None if imu.orientation is None else imu.orientation.model_dump(),
        "orientation_covariance": (
            [-1.0] + [0.0] * 8
            if imu.orientation is None
            else imu.orientation_covariance or [0.0] * 9
        ),
        "angular_velocity": imu.angular_velocity.model_dump(),
        "linear_acceleration": imu.linear_acceleration.model_dump(),
        "angular_velocity_covariance": imu.angular_velocity_covariance or [0.0] * 9,
        "linear_acceleration_covariance": imu.linear_acceleration_covariance or [0.0] * 9,
    }
