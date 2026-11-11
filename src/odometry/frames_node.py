"""Planar TF support owned by the odometry component, with configured extrinsics."""

import json
import math
import time
from pathlib import Path

from forge_msgs import Pose

from .geometry import PlanarBaseFrame, wrapped_angle


def attach_planar_frames(ros, cfg: dict) -> None:
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    frame = PlanarBaseFrame(
        body_to_base_xyz=cfg["body_to_base_xyz_m"], body_to_base_yaw=cfg["body_to_base_yaw_rad"]
    )
    pub = ros.create_publisher(Odometry, cfg.get("output_topic", "/odom"), 20)
    dynamic = TransformBroadcaster(ros)
    static = StaticTransformBroadcaster(ros)
    t = TransformStamped()
    t.header.stamp = ros.get_clock().now().to_msg()
    t.header.frame_id = "base_link"
    t.child_frame_id = "body"
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(
        float, frame.base_body[:3, 3]
    )
    yaw = -cfg["body_to_base_yaw_rad"]
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    static.sendTransform(t)
    previous = None
    count = 0
    last = None

    def odometry(msg):
        nonlocal previous, count, last
        if msg.header.frame_id != "camera_init" or msg.child_frame_id != "body":
            raise ValueError("unexpected FAST-LIO frames")
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite LIO pose")
        x, y, yaw = frame.convert(Pose(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w))
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "odom"
        out.child_frame_id = "base_link"
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = (
            math.sin(yaw / 2),
            math.cos(yaw / 2),
        )
        # Full LIO covariance has not been propagated through the uncertain
        # extrinsic. Publish conservative planar values instead of false precision.
        for i, v in enumerate([0.05, 0.05, 1e3, 1e3, 1e3, 0.05]):
            out.pose.covariance[i * 7] = v
            out.twist.covariance[i * 7] = v * 2
        if previous:
            old_ns, old_x, old_y, old_yaw = previous
            dt = (ns - old_ns) / 1e9
            if dt <= 0:
                raise ValueError("LIO timestamp regressed")
            if dt < 0.5:
                dx, dy = (x - old_x) / dt, (y - old_y) / dt
                out.twist.twist.linear.x = math.cos(yaw) * dx + math.sin(yaw) * dy
                out.twist.twist.linear.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                out.twist.twist.angular.z = wrapped_angle(yaw - old_yaw) / dt
        previous = (ns, x, y, yaw)
        pub.publish(out)
        trans = TransformStamped()
        trans.header = out.header
        trans.child_frame_id = "base_link"
        trans.transform.translation.x, trans.transform.translation.y = x, y
        trans.transform.rotation = out.pose.pose.orientation
        dynamic.sendTransform(trans)
        count += 1
        last = time.monotonic()
        if count == 1:
            Path(cfg["alignment_file"]).write_text(
                json.dumps(
                    {
                        "T_odom_camera_init": frame.odom_world.tolist(),
                        "body_to_base_xyz_m": cfg["body_to_base_xyz_m"],
                        "body_to_base_yaw_rad": cfg["body_to_base_yaw_rad"],
                        "quality": "initial_reference_not_precision_calibration",
                        "z_reference": "planar_frame_at_body_height_not_ground",
                    },
                    indent=2,
                )
            )

    ros.create_subscription(Odometry, cfg["input_topic"], odometry, 100)
