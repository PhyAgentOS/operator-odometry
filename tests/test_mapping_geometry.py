import math

import pytest
from forge_msgs import Pose

from odometry.geometry import PlanarBaseFrame


def frame():
    return PlanarBaseFrame(body_to_base_xyz=[0, 0.2511, 0], body_to_base_yaw=-math.pi / 2)


def test_forward_motion_is_positive_x_in_normalized_planar_odom():
    f = frame()
    assert f.convert(Pose.from_xy_yaw(0, 0, 0)) == pytest.approx((0, 0, 0))
    assert f.convert(Pose.from_xy_yaw(0, -1, 0)) == pytest.approx((1, 0, 0))


def test_rotation_about_base_center_does_not_make_base_orbit_lidar():
    f = frame()
    f.convert(Pose.from_xy_yaw(0, 0, 0))
    # World base remains (0,.2511). Rotate body at that fixed base origin.
    yaw = 0.4
    p = [math.sin(yaw) * 0.2511, 0.2511 - math.cos(yaw) * 0.2511]
    x, y, out_yaw = f.convert(Pose.from_xy_yaw(*p, yaw))
    assert (x, y, out_yaw) == pytest.approx((0, 0, yaw))
