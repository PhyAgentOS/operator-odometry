"""Independent Forge lifecycle and pose outputs for our ROS2 FAST-LIO backend."""

import json
import math
import os
import time
import uuid
from collections import OrderedDict
from pathlib import Path

from dora import Node
from forge_msgs import Imu, PointCloudBuffer, PolicyCommand, PolicyCommandStatus, Pose

from odometry.frames import BaseExtrinsic
from odometry.process import OwnedProcess


def run(config: dict) -> None:
    extrinsic_config = config.get("base_extrinsic", {})
    extrinsic = (
        BaseExtrinsic(extrinsic_config) if extrinsic_config.get("verified") is True else None
    )
    root = Path(config["deployment_root"]).resolve()
    executable = Path(config["backend_executable"]).resolve()
    params = Path(config["backend_params"]).resolve()
    for p in (executable, params):
        if not p.is_relative_to(root) or not p.is_file():
            raise ValueError(f"backend paths must be existing files in our deployment: {p}")
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    domain = int(config["ros_domain_id"])
    rclpy.init(domain_id=domain)
    ros = rclpy.create_node("odometry")
    from livox_ros_driver2.msg import CustomMsg
    from sensor_msgs.msg import Imu as RosImu

    from .context import SampleContext, StreamClockGuard
    from .conversion import decode_livox, imu_values
    from .ros_messages import cloud_to_serialized_ros, imu_to_ros

    cloud_pub = ros.create_publisher(
        CustomMsg,
        config["cloud_topic"],
        QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
    )
    imu_pub = ros.create_publisher(
        RosImu, config["imu_topic"], QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    )
    clock_guard = StreamClockGuard()
    if config.get("planar_frames"):
        from .frames_node import attach_planar_frames

        attach_planar_frames(
            ros, {**config["planar_frames"], "input_topic": config["odometry_topic"]}
        )
    dora = Node()
    backend = OwnedProcess()
    cache = OrderedDict()
    state = "idle"
    pending_start = None
    last_pose_at = None
    last_published = 0.0
    last_stamp = None
    start_deadline = None
    pose_count = 0

    def emit(command, status, message="", outputs=None, *, remember=True):
        value = PolicyCommandStatus.from_outputs(
            policy_id="odometry",
            command=command.command,
            request_id=command.request_id,
            status=status,
            message=message,
            outputs=outputs or {},
        )
        dora.send_output("status", value.to_arrow())
        if remember and command.request_id:
            fingerprint = (command.command, json.dumps(command.inputs(), sort_keys=True))
            cache[command.request_id] = (fingerprint, value)
            cache.move_to_end(command.request_id)
            while len(cache) > 128:
                cache.popitem(last=False)

    def runtime(status, message):
        dora.send_output(
            "status",
            PolicyCommandStatus.from_outputs(
                policy_id="odometry",
                command="runtime",
                request_id="",
                status=status,
                message=message,
                outputs={"state": state, "pose_count": pose_count},
            ).to_arrow(),
        )

    def fail(message):
        nonlocal state, pending_start
        backend.stop()
        state = "error"
        if pending_start is not None:
            emit(pending_start, "error", message)
            pending_start = None
        runtime("error", message)

    def on_odometry(msg):
        nonlocal last_pose_at, last_published, last_stamp, pose_count, state, pending_start
        if state not in ("starting", "running"):
            return
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if last_stamp is not None and stamp <= last_stamp:
            raise ValueError("FAST-LIO odometry sample time regressed or repeated")
        last_stamp = stamp
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            raise ValueError("non-finite FAST-LIO pose")
        pose = Pose(x=p.x, y=p.y, z=p.z, qx=q.x, qy=q.y, qz=q.z, qw=q.w)
        if not msg.header.frame_id or not msg.child_frame_id:
            raise ValueError("odometry frame context missing")
        pose_count += 1
        last_pose_at = time.monotonic()
        if state == "starting":
            state = "running"
            emit(
                pending_start,
                "done",
                "FAST-LIO produced its first valid odometry sample",
                {
                    "ready": True,
                    "frame_id": msg.header.frame_id,
                    "child_frame_id": msg.child_frame_id,
                    "base_pose_ready": extrinsic is not None,
                },
            )
            pending_start = None
            runtime("running", "continuous odometry active")
        if last_pose_at - last_published >= 1 / float(config.get("pose_output_hz", 5)):
            metadata = {
                "capture_timestamp_ns": stamp,
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
                "clock_domain": "ros_time",
                "time_mapping_id": config["time_mapping_id"],
                "ros_time_offset_ns": 0,
                "estimate_source": "fast_lio",
                "pose_index": pose_count,
            }
            dora.send_output("odometry_pose", pose.to_arrow(), metadata)
            if extrinsic is not None:
                converted = extrinsic.convert(pose, msg.child_frame_id)
                dora.send_output(
                    "base_pose",
                    converted.to_arrow(),
                    {
                        **metadata,
                        "child_frame_id": extrinsic.target,
                        "extrinsic_id": extrinsic.calibration_id,
                    },
                )
            last_published = last_pose_at

    def new_subscription():
        return ros.create_subscription(
            Odometry,
            config["odometry_topic"],
            on_odometry,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
        )

    subscription = new_subscription()
    runtime("running", "node ready; backend idle until start command")
    try:
        for event in dora:
            if event["type"] == "STOP":
                break
            if event["type"] == "ERROR":
                raise RuntimeError(str(event))
            if event["type"] != "INPUT":
                continue
            if event["id"] == "finish":
                break
            if event["id"] in ("point_cloud", "imu"):
                metadata = event["metadata"]
                context = SampleContext.from_metadata(metadata)
                clock_guard.observe(event["id"], context)
                if event["id"] == "point_cloud":
                    cloud_pub.publish(
                        cloud_to_serialized_ros(
                            decode_livox(PointCloudBuffer.from_arrow(event["value"]), metadata)
                        )
                    )
                else:
                    imu_pub.publish(
                        imu_to_ros(imu_values(Imu.from_arrow(event["value"]), metadata))
                    )
            if event["id"] == "policy_command":
                command = PolicyCommand.from_arrow(event["value"])
                if command.policy_id != "odometry":
                    continue
                if not command.request_id:
                    emit(command, "rejected", "request_id required", remember=False)
                    continue
                fingerprint = (command.command, json.dumps(command.inputs(), sort_keys=True))
                if command.request_id in cache:
                    previous, response = cache[command.request_id]
                    if previous != fingerprint:
                        emit(
                            command,
                            "rejected",
                            "request_id reused with different content",
                            remember=False,
                        )
                    else:
                        dora.send_output("status", response.to_arrow())
                    continue
                if command.inputs():
                    emit(command, "rejected", "start/stop currently accept no input overrides")
                    continue
                if command.command == "start":
                    if state not in ("idle", "error"):
                        emit(command, "rejected", "backend already starting or running")
                        continue
                    emit(command, "accepted")
                    pending_start = command
                    state = "starting"
                    pose_count, last_stamp, last_pose_at = 0, None, None
                    start_deadline = time.monotonic() + float(
                        config.get("startup_timeout_seconds", 30)
                    )
                    # A new volatile subscription prevents queued output from a
                    # previous backend session from satisfying startup readiness.
                    ros.destroy_subscription(subscription)
                    subscription = new_subscription()
                    env = {**os.environ, "ROS_DOMAIN_ID": str(domain)}
                    argv = [
                        str(executable),
                        "--ros-args",
                        "--params-file",
                        str(params),
                        "-r",
                        "/Odometry:=" + config["odometry_topic"],
                    ]
                    # The planar support node owns public odom->base_link. Keep
                    # the backend's camera_init->body TF on a private topic.
                    if config.get("private_backend_tf", False):
                        argv += ["-r", "/tf:=" + config.get("private_tf_topic", "/lio/tf_raw")]
                    backend.start(
                        argv,
                        cwd=root,
                        env=env,
                        log_path=root / "runtime/logs" / f"fastlio-{uuid.uuid4().hex}.log",
                    )
                    emit(command, "running", "backend starting; awaiting valid odometry")
                elif command.command == "stop":
                    emit(command, "accepted")
                    backend.stop()
                    if pending_start is not None:
                        emit(pending_start, "error", "stopped before odometry became ready")
                        pending_start = None
                    state = "idle"
                    emit(command, "done", "owned backend stopped", {"stopped": True})
                    runtime("running", "backend idle")
                else:
                    emit(command, "rejected", "unsupported command")
            for _ in range(8):
                rclpy.spin_once(ros, timeout_sec=0)
            now = time.monotonic()
            if state in ("starting", "running") and backend.poll() is not None:
                fail("FAST-LIO process exited unexpectedly")
            elif state == "starting" and now > start_deadline:
                fail("no valid odometry before startup timeout")
            elif state == "running" and now - last_pose_at > float(
                config.get("odometry_timeout_seconds", 2)
            ):
                fail("odometry stream timed out")
    except Exception as exc:
        fail(str(exc))
        raise
    finally:
        backend.stop()
        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
