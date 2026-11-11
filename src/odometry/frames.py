"""Convert world/IMU pose to world/base using an explicitly verified T_base_imu."""

import math

import numpy as np
from forge_msgs import Pose


def matrix_from_pose(pose: Pose):
    x, y, z, w = pose.qx, pose.qy, pose.qz, pose.qw
    result = np.eye(4)
    result[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    result[:3, 3] = [pose.x, pose.y, pose.z]
    return result


class BaseExtrinsic:
    def __init__(self, config):
        if config.get("verified") is not True:
            raise ValueError("base extrinsic must be verified before publishing base pose")
        self.source = config["source_child_frame"]
        self.target = config["target_child_frame"]
        self.calibration_id = config["calibration_id"]
        if not all(
            isinstance(v, str) and v for v in (self.source, self.target, self.calibration_id)
        ):
            raise ValueError("extrinsic frames and calibration_id are required")
        xyz, rpy = config["xyz_m"], config["rpy_rad"]
        if not (len(xyz) == len(rpy) == 3 and all(math.isfinite(v) for v in [*xyz, *rpy])):
            raise ValueError("extrinsic needs three finite xyz and roll/pitch/yaw values")
        roll, pitch, yaw = (v / 2 for v in rpy)
        cr, sr, cp, sp, cy, sy = (
            math.cos(roll),
            math.sin(roll),
            math.cos(pitch),
            math.sin(pitch),
            math.cos(yaw),
            math.sin(yaw),
        )
        base_imu = Pose(
            x=xyz[0],
            y=xyz[1],
            z=xyz[2],
            qx=sr * cp * cy - cr * sp * sy,
            qy=cr * sp * cy + sr * cp * sy,
            qz=cr * cp * sy - sr * sp * cy,
            qw=cr * cp * cy + sr * sp * sy,
        )
        self.imu_base = np.linalg.inv(matrix_from_pose(base_imu))

    def convert(self, world_imu: Pose, child_frame: str):
        if child_frame != self.source:
            raise ValueError("odometry child frame differs from calibrated IMU frame")
        transform = matrix_from_pose(world_imu) @ self.imu_base
        r = transform[:3, :3]
        # Stable quaternion extraction, including rotations close to 180 degrees.
        trace = float(np.trace(r))
        if trace > 0:
            s = math.sqrt(trace + 1) * 2
            q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4]
        else:
            i = int(np.argmax(np.diag(r)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = math.sqrt(1 + r[i, i] - r[j, j] - r[k, k]) * 2
            q = [0.0] * 4
            q[i], q[j], q[k], q[3] = (
                s / 4,
                (r[i, j] + r[j, i]) / s,
                (r[i, k] + r[k, i]) / s,
                (r[k, j] - r[j, k]) / s,
            )
        q = np.array(q) / np.linalg.norm(q)
        return Pose(
            x=float(transform[0, 3]),
            y=float(transform[1, 3]),
            z=float(transform[2, 3]),
            qx=q[0],
            qy=q[1],
            qz=q[2],
            qw=q[3],
        )
