# 激光惯性里程计

将 Forge 点云与 IMU 转为 FAST-LIO 输入，输出连续位姿；按调用方配置生成平面底盘里程计与坐标变换。

## 运行要求

- Node 版本：`0.2.0`；兼容 **Forge 1**。
- Linux x86_64、Ubuntu 24.04、Python 3.12 ABI、ROS 2 Jazzy。
- Dora CLI / Python 绑定 `0.4.1`、Forge Msgs `1.3.0`、Forge Common `1.0.1`。
- ROS 2、算法后端、Livox 消息工作空间由部署方提供，不包含在 Node 包内。启动前加载对应 ROS setup，运行 `--check-runtime` 检查环境；该命令不连接硬件。

制品是 `directory_tar_gz`，解压后保留整个 `odometry/` 目录，入口为 `odometry/odometry`。冻结包包含 Dora Python 绑定，不能只升级外部 Dora CLI 来切换到 Forge 2。

## 能力边界与可选配置

- `backend_overrides` 在会话目录生成参数副本，原始参数文件不变。
- 可选 `scan_projection` 管理点云转 LaserScan 进程；未配置时不启动该进程。
- `snapshot_dir` 可输出带单调接收时间的里程计、扫描快照；不提供网页或远程访问服务。

这些能力通过配置启用，未配置的可选功能保持关闭。Node 仅拥有自己的后端子进程，不接管其他应用进程或写入机器人厂家工程。快照是运行观测，不是控制接口；读取方负责检查时效。

## 开发与构建

```bash
uv sync --group dev
uv run odometry --check-config --config config/policy.example.yaml
uv run pytest
uv run ruff check .
uv pip install --python .venv/bin/python -r requirements-build.txt
source /opt/ros/jazzy/setup.bash
# 按部署环境加载所需消息及算法工作空间。
BUILD_PYTHON=.venv/bin/python bash scripts/build_pyinstaller.sh
```

配置示例中的路径和话题需由使用者填入；实际执行应由 Dora dataflow 编排。构建时使用与测试相同的依赖环境。`--version`、配置检查和单元测试不替代 ROS 后端及真机验收。

接口说明见 [docs/interfaces.md](docs/interfaces.md)，配置在 `config/`，实现位于 `src/`，测试位于 `tests/`。构建工具、测试及源码不随冻结运行制品提供。
