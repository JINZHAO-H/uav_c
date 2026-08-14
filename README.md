# UAV ROS2 PX4 DDS 工作空间

这是一个面向 PX4 无人机的 ROS2 工作空间。工程使用 PX4 官方推荐的
DDS / uXRCE-DDS 通信方式，不使用 MAVROS；SLAM / 定位前端使用
Point-LIO。

当前工程仍在开发中，现阶段主要完成：

- 使用 `ros2.repos` 管理第三方 ROS2 依赖；
- 通过 launch 同时组织 Livox、Point-LIO、PX4 DDS Agent、定位桥接和任务节点；
- 将 Point-LIO 输出的里程计转换为 PX4 可用的 visual odometry；
- 提供 C++ PX4 OFFBOARD 控制桥接和 mission 任务编排节点。

## 目标技术栈

- ROS2 Humble
- PX4 1.15.0
- PX4 DDS / uXRCE-DDS 通信
- Livox MID360
- Point-LIO ROS2

第三方源码通过 `ros2.repos` 导入，不使用 git submodule。

## 仓库结构

```text
.
├── ros2.repos
├── scripts
│   ├── build.sh
│   └── setup_deps.sh
└── src
    ├── uav_bringup
    │   └── launch
    │       └── slam_location.launch.py
    └── uav_control_cpp
        ├── include
        │   └── uav_control_cpp
        │       └── flight_bridge.hpp
        └── src
            ├── flight_bridge.cpp
            ├── location_bridge.cpp
            └── mission_runner.cpp
```

主要自定义包：

- `uav_bringup`：ROS2 launch 文件。
- `uav_control_cpp`：运行时控制和桥接节点，使用 C++ 编写。

当前通过 `ros2.repos` 导入的第三方包：

- `px4_msgs`：`release/1.15`
- `px4_ros_com`：`release/1.15`
- `livox_ros_driver2`
- `point_lio_ros2`

## 环境准备

克隆仓库并进入工作空间：

```bash
git clone git@github.com:JINZHAO-H/uav_c.git
cd uav_c
```

安装系统依赖、导入第三方源码到 `src/`，并执行 `rosdep`：

```bash
./scripts/setup_deps.sh
```

这个脚本会执行：

```bash
vcs import src < ros2.repos
```

因此第三方仓库会按照 `ros2.repos` 中声明的地址和分支导入到 `src/`
目录下。

## 编译

编译整个工作空间：

```bash
./scripts/build.sh
```

只编译自定义控制包：

```bash
colcon build --symlink-install --packages-select uav_control_cpp --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

编译完成后加载环境：

```bash
source install/setup.bash
```

## 启动

主 launch 文件是：

```bash
ros2 launch uav_bringup slam_location.launch.py
```

这个 launch 文件可以启动：

- `MicroXRCEAgent`
- Livox ROS2 driver
- Point-LIO
- `location_bridge`
- `mission_runner`

`mission_runner` 内部会创建并使用 `FlightBridge`，主 launch 不再单独启动
独立的 `flight_bridge` 节点，避免出现两个飞控控制源。

如果只是做无雷达、无 DDS Agent 的基础启动测试，可以运行：

```bash
ros2 launch uav_bringup slam_location.launch.py start_dds_agent:=false start_livox:=false start_point_lio:=false
```

当前默认配置：

- Livox 启动文件：`msg_MID360_launch.py`
- Point-LIO 配置文件：`point_lio/config/mid360.yaml`
- Point-LIO 里程计话题：`/aft_mapped_to_init`
- PX4 visual odometry 输入话题：`/fmu/in/vehicle_visual_odometry`
- mission 配置文件：`/home/jzh/uav_c/config/mission.yaml`
- 默认路线：`custom_points`
- 默认启动 mission：`start_mission:=true`

如果实际 DDS 串口设备不同，可以在启动时覆盖：

```bash
ros2 launch uav_bringup slam_location.launch.py dds_device:=/dev/ttyUSB0
```

## 自定义节点

### `location_bridge`

`location_bridge` 负责把 Point-LIO 输出的里程计转换成 PX4 可用的
visual odometry。

输入：

- 话题：`/aft_mapped_to_init`
- 类型：`nav_msgs/msg/Odometry`

输出：

- 话题：`/fmu/in/vehicle_visual_odometry`
- 类型：`px4_msgs/msg/VehicleOdometry`

主要功能：

- 将 ROS ENU 坐标转换为 PX4 NED 坐标；
- 将 ROS FLU 机体系转换为 PX4 FRD 机体系；
- 支持初始位置归零；
- 当里程计超时后停止发布过期定位数据。

如果没有接入雷达，或者 Point-LIO 没有输出里程计，这个节点启动后可能只
打印启动信息。这是正常现象，因为当前没有 odometry 输入。

### `flight_bridge`

`FlightBridge` 封装 PX4 OFFBOARD 飞行控制相关的 DDS 话题读写。它主要被
`mission_runner` 在进程内部创建并调用；如果只想单独测试底层飞控桥，也可
以手动运行 `ros2 run uav_control_cpp flight_bridge`。

订阅：

- `/fmu/out/vehicle_status`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_command_ack`
- `/fmu/out/event`
- `/fmu/out/estimator_status_flags`
- `/fmu/out/vehicle_land_detected`

发布：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

这个类提供连接等待、本地位置有效性检查、setpoint 预热、解锁、请求
OFFBOARD、飞向目标点、悬停、降落、关闭和起飞前诊断等 C++ 控制逻辑。

### `mission_runner`

`mission_runner` 是 C++ mission / state machine 节点，负责读取
`config/mission.yaml` 并按配置执行任务。

当前主流程：

- 等待 PX4 DDS 连接；
- 等待 PX4 local position 正常；
- 打印 PX4 起飞前诊断；
- prime setpoints；
- 按旧实飞逻辑自动 ARM；
- 不主动切 OFFBOARD，等待飞手手动切 OFFBOARD；
- 按 YAML 航点执行 `custom_points`；
- 完成后请求 PX4 LAND。

起飞前诊断会读取 PX4 ACK、event、estimator flags 和 land detected 状态，
用于判断 ARM / OFFBOARD / EKF 健康问题。

## 安全说明

`FlightBridge` 单独运行时，主动飞控输出默认是关闭的：

```text
request_offboard_mode := false
enable_offboard_publish := false
enable_setpoint_publish := false
```

主 launch 当前默认启动 `mission_runner`：

```text
start_mission := true
```

这意味着接入真实硬件且 PX4 DDS、定位和 local position 都正常后，mission
会进入自动 ARM 流程。它不会主动切 OFFBOARD，因为：

```text
request_offboard_mode := false
```

飞手仍需手动切入 OFFBOARD，任务才会继续执行航点。

真正试飞前，应先完成 SITL 或无桨安全测试，确认 DDS 链路、PX4 状态反馈、
Point-LIO 里程计、visual odometry 注入和 PX4 local position 都正常后，
再运行完整 mission。

## 当前开发状态

已完成：

- ROS2 工作空间结构；
- `ros2.repos` 第三方依赖管理；
- Livox MID360 + Point-LIO launch 集成；
- Point-LIO odometry 到 PX4 visual odometry 的桥接；
- C++ PX4 OFFBOARD 控制桥接工具函数；
- C++ mission runner 主流程；
- PX4 起飞前诊断日志。

计划继续完善：

- 面向 SITL 的验证流程；
- 面向真实设备的试飞前检查清单。
