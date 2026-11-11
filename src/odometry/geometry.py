"""Planar reference-frame geometry; never represents a precision calibration."""

import math

import numpy as np

from odometry.frames import matrix_from_pose


class PlanarBaseFrame:
    def __init__(self, *, body_to_base_xyz, body_to_base_yaw):
        if len(body_to_base_xyz) != 3 or not all(
            math.isfinite(v) for v in [*body_to_base_xyz, body_to_base_yaw]
        ):
            raise ValueError("finite body->base initial reference required")
        c, s = math.cos(body_to_base_yaw), math.sin(body_to_base_yaw)
        self.body_base = np.eye(4)
        self.body_base[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
        self.body_base[:3, 3] = body_to_base_xyz
        self.base_body = np.linalg.inv(self.body_base)
        self.odom_world = None

    def convert(self, pose):
        world_base = matrix_from_pose(pose) @ self.body_base
        if self.odom_world is None:
            # Odom starts at the first base position with the robot facing +X.
            yaw = math.atan2(world_base[1, 0], world_base[0, 0])
            c, s = math.cos(yaw), math.sin(yaw)
            initial = np.eye(4)
            initial[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
            initial[:3, 3] = world_base[:3, 3]
            self.odom_world = np.linalg.inv(initial)
        odom_base = self.odom_world @ world_base
        return (
            float(odom_base[0, 3]),
            float(odom_base[1, 3]),
            math.atan2(odom_base[1, 0], odom_base[0, 0]),
        )


def wrapped_angle(value):
    return math.atan2(math.sin(value), math.cos(value))
