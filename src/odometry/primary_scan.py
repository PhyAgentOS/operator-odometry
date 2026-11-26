"""Opt-in simulation mapping scan at its actual primary rigid sensor origin.

Uses real FAST-LIO deskewed body points. Synthetic reflectivity labels preserve
source identity only; unknown/missing angular bins never become free endpoints.
"""

import math

import numpy as np


class PrimaryScanProjector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.sensor_index = cfg.get("source_sensor_index", 0)
        if (
            isinstance(self.sensor_index, bool)
            or not isinstance(self.sensor_index, int)
            or not 0 <= self.sensor_index < cfg["source_sensor_count"]
        ):
            raise ValueError("invalid physical source sensor index")
        self.mount = np.asarray(cfg["sensor_translation_xyz_m"], dtype=float)
        self.body_to_base = np.asarray(cfg["body_to_base_xyz_m"], dtype=float)
        self.rotation = np.asarray(cfg["sensor_rotation_matrix"], dtype=float)
        if (
            self.mount.shape != (3,)
            or self.body_to_base.shape != (3,)
            or self.rotation.shape != (3, 3)
            or not np.isfinite(self.mount).all()
            or not np.isfinite(self.body_to_base).all()
            or not np.isfinite(self.rotation).all()
            or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-8)
            or np.linalg.det(self.rotation) < 0.999
            or not np.allclose(self.rotation[2], [0, 0, 1], atol=1e-8)
            or abs(cfg["body_to_base_yaw_rad"]) > 1e-8
        ):
            raise ValueError("primary scan requires verified finite upright fixed extrinsics")
        if np.linalg.norm(self.mount[:2]) > 0.75:
            raise ValueError("sensor origin outside physical circle")
        self.bins = 720
        self.increment = 2 * math.pi / self.bins

    def project(self, points):
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 4 or not np.isfinite(points).all():
            raise ValueError("deskewed cloud requires finite x/y/z/intensity")
        labels = points[:, 3]
        expected = np.arange(100, 100 + self.cfg["source_sensor_count"])
        if not np.isin(labels, expected).all():
            raise ValueError("real source-identity labels were lost/averaged")
        # body_T_base is the declared inverse transform. Both frames are
        # upright/aligned here, so this converts real IMU-body points to base.
        base = points[labels == 100 + self.sensor_index, :3] - self.body_to_base
        base = base[(base[:, 2] >= self.cfg["min_height"]) & (base[:, 2] <= self.cfg["max_height"])]
        sensor = (base - self.mount) @ self.rotation
        distances = np.linalg.norm(sensor[:, :2], axis=1)
        valid = (distances >= self.cfg["range_min"]) & (distances < self.cfg["range_max"])
        angles = np.arctan2(sensor[valid, 1], sensor[valid, 0])
        distances = distances[valid]
        indices = np.floor((angles + math.pi) / self.increment).astype(int) % self.bins
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, indices, distances.astype(np.float32))
        return ranges


def attach_primary_scan(ros, cfg):
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from tf2_ros import StaticTransformBroadcaster

    projector = PrimaryScanProjector(cfg)
    broadcaster = StaticTransformBroadcaster(ros)
    t = TransformStamped()
    t.header.stamp = ros.get_clock().now().to_msg()
    t.header.frame_id = "base_link"
    t.child_frame_id = cfg["frame_id"]
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(
        float, projector.mount
    )
    yaw = math.atan2(projector.rotation[1, 0], projector.rotation[0, 0])
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    broadcaster.sendTransform(t)
    pub = ros.create_publisher(
        LaserScan,
        cfg["output_topic"],
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
    )

    def receive(msg):
        if msg.header.frame_id != "body":
            raise ValueError("expected actual FAST-LIO deskewed IMU-body cloud")
        data = read_points_numpy(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=False
        ).reshape(-1, 4)
        ranges = projector.project(data)
        if np.isfinite(ranges).sum() < 40:
            raise ValueError("primary mapping scan lacks 40 real finite angular returns")
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = cfg["frame_id"]
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - projector.increment
        scan.angle_increment = projector.increment
        scan.scan_time = 0.1
        scan.time_increment = 0.0
        scan.range_min = cfg["range_min"]
        scan.range_max = cfg["range_max"]
        scan.ranges = ranges.tolist()
        pub.publish(scan)
        if cfg.get("snapshot_dir"):
            from .snapshot import write_snapshot

            write_snapshot(
                cfg["snapshot_dir"],
                "mapping_scan",
                dict(
                    frame_id=scan.header.frame_id,
                    angle_min=scan.angle_min,
                    angle_increment=scan.angle_increment,
                    range_min=scan.range_min,
                    range_max=scan.range_max,
                    ranges=[float(x) if math.isfinite(x) else None for x in ranges],
                    capture_timestamp_ns=msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                    sensor_translation_xyz_m=cfg["sensor_translation_xyz_m"],
                    source="real primary returns retained through FAST-LIO deskew; absent angles remain invalid",
                ),
            )

    sub = ros.create_subscription(
        PointCloud2,
        "/cloud_registered_body",
        receive,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    return projector, broadcaster, pub, sub


def origin_mux_index(capture_ns, count):
    if (
        isinstance(capture_ns, bool)
        or not isinstance(capture_ns, int)
        or capture_ns < 0
        or count < 1
    ):
        raise ValueError(
            "positive physical stream count and nonnegative capture timestamp required"
        )
    # 1.1s blocks avoid locking the native 1Hz acceptance interval to one source.
    return (capture_ns // 1_100_000_000) % count


class LegacyGraphBootstrap:
    """Switch only after native SLAM acknowledges one current legacy scan."""

    def __init__(self):
        self.stamps = []
        self.ready = False
        self.accepted_stamp = None

    def forwarded(self, stamp):
        if self.ready:
            return False
        self.stamps.append(stamp)
        self.stamps = self.stamps[-64:]
        return True

    def accepted_pose(self, stamp):
        if stamp in self.stamps:
            self.ready = True
            self.accepted_stamp = stamp
        return self.ready


def attach_origin_scan_mux(ros, cfg):
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py.point_cloud2 import read_points_numpy
    from tf2_ros import StaticTransformBroadcaster

    from .snapshot import write_snapshot

    projectors = [PrimaryScanProjector(item) for item in cfg["sensors"]]
    if len(projectors) < 2 or [p.sensor_index for p in projectors] != list(range(len(projectors))):
        raise ValueError("multi-origin scan requires each actual source exactly once")
    static = StaticTransformBroadcaster(ros)
    transforms = []
    for p in projectors:
        t = TransformStamped()
        t.header.stamp = ros.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = p.cfg["frame_id"]
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(
            float, p.mount
        )
        yaw = math.atan2(p.rotation[1, 0], p.rotation[0, 0])
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        transforms.append(t)
    static.sendTransform(transforms)
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = ros.create_publisher(LaserScan, cfg["output_topic"], qos)
    last_stamp = None
    bootstrap = LegacyGraphBootstrap()

    def legacy(msg):
        if msg.header.frame_id != "base_link":
            raise ValueError("serialized graph bootstrap requires the actual base_link legacy scan")
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if bootstrap.forwarded(stamp):
            pub.publish(msg)
            write_snapshot(
                cfg["snapshot_dir"],
                "mapping_bootstrap",
                dict(
                    phase="legacy_waiting",
                    forwarded_capture_ns=stamp,
                    source="current real base scan only; no archive or replay",
                ),
            )

    def accepted(msg):
        if msg.header.frame_id != "map":
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if not bootstrap.ready and bootstrap.accepted_pose(stamp):
            write_snapshot(
                cfg["snapshot_dir"],
                "mapping_bootstrap",
                dict(
                    phase="physical_ready",
                    accepted_legacy_capture_ns=stamp,
                    source="native SLAM accepted-pose timestamp matches a current forwarded real legacy scan",
                ),
            )

    legacy_sub = ros.create_subscription(
        LaserScan, cfg.get("bootstrap_legacy_topic", "/scan"), legacy, qos
    )
    pose_sub = ros.create_subscription(PoseWithCovarianceStamped, "/pose", accepted, qos)

    def receive(msg):
        nonlocal last_stamp
        if not bootstrap.ready:
            return
        if msg.header.frame_id != "body":
            raise ValueError("multi-origin scan requires real FAST-LIO deskewed body points")
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if last_stamp is not None and stamp <= last_stamp:
            raise ValueError("deskewed physical scan timestamp regressed/repeated")
        last_stamp = stamp
        index = origin_mux_index(stamp, len(projectors))
        p = projectors[index]
        data = read_points_numpy(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=False
        ).reshape(-1, 4)
        ranges = p.project(data)
        if np.isfinite(ranges).sum() < 40:
            raise ValueError("selected physical mapping scan lacks 40 actual angular returns")
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = p.cfg["frame_id"]
        scan.angle_min = -math.pi
        scan.angle_max = math.pi - p.increment
        scan.angle_increment = p.increment
        scan.scan_time = 0.1
        scan.time_increment = 0.0
        scan.range_min = p.cfg["range_min"]
        scan.range_max = p.cfg["range_max"]
        scan.ranges = ranges.tolist()
        pub.publish(scan)
        value = dict(
            frame_id=scan.header.frame_id,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            ranges=[float(x) if math.isfinite(x) else None for x in ranges],
            capture_timestamp_ns=stamp,
            sensor_translation_xyz_m=p.cfg["sensor_translation_xyz_m"],
            source_sensor_index=index,
            source="actual selected physical returns after FAST-LIO deskew; absent beams remain invalid",
        )
        write_snapshot(cfg["snapshot_dir"], "mapping_scan", value)
        write_snapshot(cfg["snapshot_dir"], "mapping_scan_" + str(index), value)

    sub = ros.create_subscription(PointCloud2, "/cloud_registered_body", receive, qos)
    return projectors, static, pub, sub, legacy_sub, pose_sub, bootstrap
