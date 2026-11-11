"""Construct actual generated ROS2 messages only in a sourced ROS environment."""

from functools import lru_cache

from .conversion import LivoxFrame


def set_stamp(header, ns: int, frame_id: str) -> None:
    header.stamp.sec, header.stamp.nanosec = divmod(ns, 10**9)
    header.frame_id = frame_id


def cloud_to_ros(frame: LivoxFrame):
    from livox_ros_driver2.msg import CustomMsg, CustomPoint

    msg = CustomMsg()
    set_stamp(msg.header, frame.timebase, frame.frame_id)
    msg.timebase = frame.timebase
    msg.lidar_id = frame.lidar_id
    msg.point_num = len(frame.points)
    points = []
    for p in frame.points:
        point = CustomPoint()
        point.x, point.y, point.z = (float(p[k]) for k in ("x", "y", "z"))
        point.offset_time = int(p["time_offset_ns"])
        point.reflectivity, point.line, point.tag = (
            int(p[k]) for k in ("reflectivity", "line", "tag")
        )
        points.append(point)
    msg.points = points
    return msg


def imu_to_ros(values: dict):
    from sensor_msgs.msg import Imu

    msg = Imu()
    set_stamp(msg.header, values["stamp_ns"], values["frame_id"])
    orientation = values["orientation"]
    if orientation is not None:
        for axis in "xyzw":
            setattr(msg.orientation, axis, orientation["q" + axis])
    for field in ("angular_velocity", "linear_acceleration"):
        for axis, value in values[field].items():
            setattr(getattr(msg, field), axis, value)
    for field in (
        "orientation_covariance",
        "angular_velocity_covariance",
        "linear_acceleration_covariance",
    ):
        setattr(msg, field, values[field])
    return msg


@lru_cache(maxsize=1)
def _bulk_layout_supported():
    from livox_ros_driver2.msg import CustomMsg, CustomPoint

    return CustomPoint.get_fields_and_field_types() == {
        "offset_time": "uint32",
        "x": "float",
        "y": "float",
        "z": "float",
        "reflectivity": "uint8",
        "tag": "uint8",
        "line": "uint8",
    } and CustomMsg.get_fields_and_field_types() == {
        "header": "std_msgs/Header",
        "timebase": "uint64",
        "point_num": "uint32",
        "lidar_id": "uint8",
        "rsvd": "uint8[3]",
        "points": "sequence<livox_ros_driver2/CustomPoint>",
    }


def cloud_to_serialized_ros(frame: LivoxFrame) -> bytes:
    """Encode Livox points in bulk; avoid allocating 200k ROS Python objects/s.

    Use the installed ROS serializer for the variable-size Header and prefix.
    Livox's fixed CustomPoint wire layout is XCDR1: uint32, 3*float32,
    reflectivity/tag/line bytes. Elements align to four bytes, with no trailing
    pad after the final element. The ROS integration test verifies the decoded message against the
    official generated message (CDR alignment padding is unspecified).
    """
    import struct

    import numpy as np
    from livox_ros_driver2.msg import CustomMsg
    from rclpy.serialization import serialize_message

    if not _bulk_layout_supported():
        return serialize_message(cloud_to_ros(frame))
    msg = CustomMsg()
    set_stamp(msg.header, frame.timebase, frame.frame_id)
    msg.timebase = frame.timebase
    msg.lidar_id = frame.lidar_id
    msg.point_num = len(frame.points)
    prefix = serialize_message(msg)
    if prefix[:4] != b"\x00\x01\x00\x00" or prefix[-4:] != b"\x00" * 4:
        # Preserve correctness if a future ROS middleware changes CDR encoding.
        return serialize_message(cloud_to_ros(frame))
    # The native serializer leaves alignment padding unspecified; zero it
    # before publishing the generated prefix.
    header_end = 17 + len(frame.frame_id.encode("utf-8"))
    timebase_start = 4 + ((header_end - 4 + 7) // 8) * 8
    prefix = prefix[:header_end] + bytes(timebase_start - header_end) + prefix[timebase_start:]
    packed = np.zeros(
        len(frame.points),
        dtype=np.dtype(
            {
                "names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
                "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
                "offsets": [0, 4, 8, 12, 16, 17, 18],
                "itemsize": 20,
            }
        ),
    )
    for field in packed.dtype.names:
        packed[field] = frame.points["time_offset_ns" if field == "offset_time" else field]
    payload = packed.tobytes()
    if len(packed):
        payload = payload[:-1]
    return prefix[:-4] + struct.pack("<I", len(packed)) + payload
