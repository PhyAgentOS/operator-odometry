"""Run in a sourced ROS2/Livox environment, no hardware needed."""

import time

import numpy as np
import pytest

pytest.importorskip("livox_ros_driver2.msg")
from livox_ros_driver2.msg import CustomMsg
from rclpy.serialization import deserialize_message, serialize_message

from odometry.conversion import LivoxFrame
from odometry.ros_messages import cloud_to_ros, cloud_to_serialized_ros


@pytest.mark.parametrize("frame_id", ["x", "livox_frame", "雷达"])
@pytest.mark.parametrize("count", [1, 2, 31, 20000])
def test_bulk_points_equal_official_serialization(frame_id, count):
    points = np.zeros(
        count,
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("time_offset_ns", "<u4"),
            ("reflectivity", "u1"),
            ("line", "u1"),
            ("tag", "u1"),
        ],
    )
    points["x"] = np.arange(count) * 0.25
    points["y"] = -2.5
    points["z"] = 3.75
    points["time_offset_ns"] = np.arange(count) * 5000
    points["reflectivity"] = np.arange(count) % 256
    points["line"] = np.arange(count) % 4
    points["tag"] = 16
    frame = LivoxFrame(1_790_000_000_123456789, frame_id, 7, points)
    expected = serialize_message(cloud_to_ros(frame))
    start = time.perf_counter()
    actual = cloud_to_serialized_ros(frame)
    elapsed = time.perf_counter() - start
    assert len(actual) == len(expected)
    assert deserialize_message(actual, CustomMsg) == deserialize_message(expected, CustomMsg)
    restored = deserialize_message(actual, CustomMsg)
    assert restored.point_num == count and restored.header.frame_id == frame_id
    assert restored.points[-1].offset_time == (count - 1) * 5000
    if count == 20000:
        print("bulk_20k_ms", round(elapsed * 1000, 3))


def test_big_endian_padded_cloud_roundtrip():
    from forge_msgs import PointCloudBuffer
    from forge_msgs.point_cloud_buffer import PointField

    from odometry.conversion import decode_livox

    dtype = np.dtype(
        [
            ("x", ">f4"),
            ("y", ">f4"),
            ("z", ">f4"),
            ("time_offset_ns", ">u4"),
            ("reflectivity", "u1"),
            ("line", "u1"),
            ("tag", "u1"),
        ]
    )
    points = np.array(
        [(1.25, -2.5, 3.75, 12345, 255, 3, 16), (-1.5, 2.75, -0.25, 98765, 30, 1, 32)], dtype=dtype
    )
    fields = [
        PointField(
            name=n,
            offset=dtype.fields[n][1],
            datatype="float32"
            if n in ("x", "y", "z")
            else "uint32"
            if n == "time_offset_ns"
            else "uint8",
            count=1,
        )
        for n in dtype.names
    ]
    cloud = PointCloudBuffer(
        width=1,
        height=2,
        is_dense=True,
        byte_order="big_endian",
        point_stride=dtype.itemsize,
        row_stride=dtype.itemsize + 5,
        fields=fields,
        data=b"".join(p.tobytes() + b"\0" * 5 for p in points),
    )
    frame = decode_livox(
        cloud,
        {
            "capture_timestamp_ns": 1_790_000_000_123456789,
            "frame_id": "livox_frame",
            "clock_domain": "test",
            "time_mapping_id": "test",
            "ros_time_offset_ns": 0,
            "lidar_id": 7,
        },
    )
    restored = deserialize_message(cloud_to_serialized_ros(frame), CustomMsg)
    assert restored == cloud_to_ros(frame)
    assert restored.points[1].x == -1.5 and restored.points[1].offset_time == 98765
