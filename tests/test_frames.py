import math

import pytest
from forge_msgs import Pose

from odometry.frames import BaseExtrinsic


def extrinsic(**updates):
    return {
        "verified": True,
        "source_child_frame": "body",
        "target_child_frame": "base_link",
        "calibration_id": "test-only",
        "xyz_m": [1, 0, 0],
        "rpy_rad": [0, 0, math.pi / 2],
        **updates,
    }


def test_base_pose_composition_uses_inverse_in_correct_frame():
    # World/base = (2,3), yaw=90 degrees. Base/IMU=(1,0), yaw=90 degrees.
    # Therefore world/IMU=(2,4), yaw=180 degrees.
    converted = BaseExtrinsic(extrinsic()).convert(Pose.from_xy_yaw(2, 4, math.pi), "body")
    x, y, yaw = converted.to_xy_yaw()
    assert (x, y, yaw) == pytest.approx((2, 3, math.pi / 2))


def test_unverified_or_wrong_frame_never_produces_base_pose():
    with pytest.raises(ValueError, match="verified"):
        BaseExtrinsic(extrinsic(verified=False))
    with pytest.raises(ValueError, match="child frame"):
        BaseExtrinsic(extrinsic()).convert(Pose(x=0, y=0), "laser")


@pytest.mark.parametrize("yaw", [0, math.pi / 2, math.pi, -math.pi / 2])
def test_identity_extrinsic_preserves_pose_including_half_turn(yaw):
    converter = BaseExtrinsic(extrinsic(xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0]))
    actual = converter.convert(Pose.from_xy_yaw(3, 4, yaw), "body")
    assert actual.to_xy_yaw() == pytest.approx((3, 4, yaw))
