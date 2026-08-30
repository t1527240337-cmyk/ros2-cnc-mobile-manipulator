# 可复现环境与云端执行

## 1. 环境基线

项目唯一验收基线是 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic/DART、
Nav2、MoveIt 2 和 BehaviorTree.CPP。仓库根目录的 `Dockerfile` 基于
`osrf/ros:jazzy-desktop-full-noble`，通过各 ROS 包的 `package.xml` 和
`rosdep` 安装依赖，再执行一次完整 `colcon build`。镜像中不保存 API Key、
测试结果或云平台账号。

官方依据：

- ROS 官方镜像说明：https://hub.docker.com/_/ros
- OSRF Jazzy desktop-full 镜像：https://hub.docker.com/r/osrf/ros
- ROS 2 rosdep 工作区安装方式：
  https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html

## 2. 构建与运行

```bash
cd Embodied_Robotic_Arm
docker build -t embodied-factory:jazzy .
docker compose run --rm factory ./scripts/run_checks.sh
```

默认基础镜像始终是官方 `osrf/ros:jazzy-desktop-full-noble`。若云供应商到
Docker Hub 的 DNS/链路异常，可仅在下载阶段指定镜像代理：

```bash
FACTORY_DOCKER_BASE_IMAGE=docker.m.daocloud.io/osrf/ros:jazzy-desktop-full-noble \
  ./scripts/cloud_validate_docker.sh
```

这只改变镜像下载来源，不改变 Ubuntu/ROS 版本或项目运行逻辑；验证脚本会记录镜像
identity。容器还显式安装 `rmw_cyclonedds_cpp`，与项目统一环境脚本选择的 RMW 一致。

有 NVIDIA Container Toolkit 时，`compose.yaml` 将一张 GPU 暴露给容器。
Gazebo 物理、Nav2 和 MoveIt 主要使用 CPU；GPU 负责无界面相机渲染、视频编码，
并为以后迁移 Isaac Sim 或 VLA 留出接口。没有 GPU 时仍可移除 compose 中的
GPU reservation，运行不含离屏录像的单元测试。

本地 GUI：

```bash
xhost +local:root
docker compose run --rm factory \
  ros2 launch factory_bringup physical_stack.launch.py \
  headless:=false use_navigation:=true use_moveit:=true
```

云端无 GUI：

```bash
tmux new-session -s factory
./scripts/cloud_build_regression.sh
./scripts/cloud_run_campaign.sh \
  /home/ubuntu/Embodied_Robotic_Arm \
  101-130 \
  /home/ubuntu/Embodied_Robotic_Arm/artifacts/physical_seed_campaign/final/production
```

脚本默认遇到单个种子失败后继续下一个种子，不使用失败即停止，因此不会因一个
样本浪费整段云机租期。

## 3. 数据同步原则

代码的唯一真源是本机 Git 仓库。云端只执行构建、仿真、录像与统计；同步时排除
`.git`、`build`、`install`、`log` 和已有 `artifacts`，避免云端缓存覆盖
本机历史。正式结果同步回本机 `artifacts/<cloud>_<date>/`，其中保留：

- 每个 seed 的场景参数、控制台日志、物理证据和结果 JSON；
- 汇总 JSON、代码版本、配置哈希与结果哈希；
- CPU/GPU/实时率监控；
- Agent trace、最终 ROS 状态、MP4 和 SHA-256。

## 4. 可迁移性边界

Docker 可以复用用户态的 Ubuntu、ROS、Gazebo、依赖和项目代码，不能把云平台的
NVIDIA 内核驱动装进镜像。新服务器只需满足：

1. x86-64 Ubuntu/Linux；
2. NVIDIA 驱动兼容容器请求的 CUDA 能力；
3. Docker 与 NVIDIA Container Toolkit；
4. DART 和 EGL/无界面渲染可用。

因此更换 UCloud、AutoDL 或其他服务器不会废弃当前环境；只需重新拉取或构建镜像，
再挂载结果目录。正式验收仍记录宿主驱动、GPU、CPU、镜像 digest 和 Git commit。
