# 接口

输入：tick、finish、policy_command（PolicyCommand，policy_id=odometry）。命令：start / stop。输出：status（PolicyCommandStatus，保留 request_id）。另需 point_cloud、imu 输入，输出 odometry_pose / base_pose（Pose）。点云 metadata 不可丢弃。

ROS2 的 LaserScan、Odometry、OccupancyGrid、Path、TF 在后端交换，不新增 Forge 消息类型。具体配置键见示例 YAML；topic、ROS domain、程序路径由调用方指定。

## 最小调用

```python
from forge_msgs import PolicyCommand

request = PolicyCommand.from_inputs(
    policy_id="odometry", command="start", request_id="example-1", inputs={}
)
node.send_output("policy_command", request.to_arrow())
```

每次新任务使用唯一 request_id；重复相同请求返回已有状态，重复 ID 配不同参数被拒绝。status 常见值为 accepted/running/done/rejected/error；done 是此命令完成，不表示后端进程退出。

| command | inputs 示例 | 作用 |
|---|---|---|
| `start` | `{}` | 启动已配置 FAST-LIO2；首条有效位姿后 done |
| `stop` | `{}` | 停止本节点启动的后端 |

`examples/dataflow.yaml` 展示完整输入输出连接。dynamic 节点是调用方占位，不会凭空提供数据；实际应用需用自己的节点替换。`tick` 为调度输入，`finish` 任意负载触发退出。所有进程在同一机器人运行，统一 ROS_DOMAIN_ID。

点云/IMU metadata 必需：capture_timestamp_ns(int)、frame_id(str)、clock_domain(str)、time_mapping_id(str)、ros_time_offset_ns(int)。数据使用同一时钟映射，采样时间严格递增。Pose metadata 说明父/子坐标系和来源时间；只有 base_extrinsic.verified=true 才输出严格外参转换的 base_pose。可选 planar_frames 提供平面 odom/TF，精度由调用方外参决定。
