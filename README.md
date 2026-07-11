# Basic Navigation Framework

面向搭载 Livox 固态雷达的全向移动机器人的一套完整 3D 激光导航框架。基于 ROS 2 与 Navigation2 构建，覆盖从雷达驱动、激光惯性里程计、全局重定位、地形可通行性分析，到路径规划、全向控制与行为树巡逻决策的完整链路，并同时提供仿真与实车两套参数配置。

框架以**可复用的底层导航基座**为设计目标：核心节点通过参数与插件解耦，可在不同机器人平台与项目间迁移，无需改动源码。

## 目录

- [特性](#特性)
- [系统架构](#系统架构)
- [数据流](#数据流)
- [依赖环境](#依赖环境)
- [构建](#构建)
- [快速开始](#快速开始)
- [软件包一览](#软件包一览)
- [配置](#配置)
- [坐标系约定](#坐标系约定)
- [文档](#文档)
- [许可证与第三方声明](#许可证与第三方声明)
- [致谢](#致谢)
- [已知问题](#已知问题)

## 特性

- **紧耦合激光惯性里程计**：集成 Point-LIO，基于 IKFoM 流形卡尔曼滤波，输出高频里程计与配准点云。
- **先验地图重定位**：基于 small_gicp 的扫描-地图 GICP 配准，周期性广播 `map → odom` 变换，支持 RViz 初始位姿注入。
- **3D 地形可通行性分析**：双尺度地形分析（近场 + 大尺度连通性去天花板），将三维点云转化为可供 Navigation2 代价地图消费的可通行性栅格。
- **全向 PID 纯追踪控制器**：面向全向底盘的 Navigation2 控制器插件，纯追踪选点 + 双路 PID + 曲率限速 + 靠近减速。
- **行为树决策层**：一组自定义 Navigation2 行为树节点，支持定点导航、往返巡逻、路径选择与恢复行为。
- **仿真与实车统一**：同一套 launch 体系，通过参数文件在仿真回环与真实硬件之间切换。

## 系统架构

```
感知层        livox_ros_driver2 / ign_sim_pointcloud_tool   原始点云 + IMU
                              │
里程计层      point_lio                                     cloud_registered + aft_mapped_to_init (lidar_odom 系)
                              │
适配层        loam_interface                                registered_scan + lidar_odometry (odom 系)
                ┌─────────────┼───────────────────────────┐
扫描/定位层    sensor_scan_generation   terrain_analysis    small_gicp_relocalization
              (odom→base TF + 底盘里程计) (terrain_map)       (map→odom TF)
                              │          terrain_analysis_ext
                              │          (terrain_map_ext)
                              ▼
规划控制层    Navigation2 栈
              ├─ 代价地图层  nav2_plugins::IntensityVoxelLayer  ← terrain_map / terrain_map_ext
              ├─ 控制器插件  OmniPidPursuitController (FollowPath)
              ├─ 行为树节点  nav2_plugins::* (巡逻 / 选路 / 发布)
              └─ 恢复行为    BackUpFreeSpace
                              ▼
                         cmd_vel (经 velocity_smoother 平滑) → 底盘
```

Navigation2 与本框架的插件（控制器、代价地图层、行为树节点、恢复行为）之间是 pluginlib 运行时加载关系，加载配置位于 `nav2_params.*.yaml` 与行为树 XML 中，源码层面无编译期依赖。

## 数据流

1. **感知 → 里程计**：`livox_ros_driver2`（实车）或 `ign_sim_pointcloud_tool`（仿真）产出原始点云，`point_lio` 紧耦合 LIO 输出配准点云 `cloud_registered` 与里程计 `aft_mapped_to_init`（内部 `lidar_odom` 系）。
2. **坐标系适配**：`loam_interface` 依据 TF 将 point_lio 的输出从 `lidar_odom` 系变换到 `odom` 系，发布 `registered_scan` 与 `lidar_odometry`。
3. **扫描与 TF 生成**：`sensor_scan_generation` 时间同步里程计与点云，广播 `odom → base_footprint` TF，输出底盘系里程计；`terrain_analysis` 与 `terrain_analysis_ext` 累积点云、估计地面高程，产出 `terrain_map` / `terrain_map_ext`（每点强度值即相对地面的高度代价）。
4. **重定位**：`small_gicp_relocalization` 基于先验 PCD 地图做 GICP 配准，以 2 Hz 更新、20 Hz 广播 `map → odom` TF。
5. **规划 → 控制**：Navigation2 代价地图的 `IntensityVoxelLayer` 消费地形图标记障碍；规划器产出全局路径；`OmniPidPursuitController` 纯追踪输出全向速度，经 `velocity_smoother` 平滑后下发底盘。
6. **决策 / 巡逻**：`bt_navigator` 加载行为树，由 `nav2_plugins` 的巡逻、选路与发布节点驱动多点巡航与恢复。

**cmd_vel 重映射链**（关键不变量）：控制器与 `bt_navigator` 输出映射为 `cmd_vel_nav2_result`；`velocity_smoother` 消费 `cmd_vel_nav2_result`，其平滑输出 `cmd_vel_smoothed` 映射回最终 `cmd_vel`。改动控制链拓扑时须同步这三处重映射。

## 依赖环境

- ROS 2（在 Humble 上开发；其他发行版需自行验证）
- Navigation2（`navigation2`、`nav2_smac_planner`、`slam_toolbox` 等）
- PCL、Eigen3
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) 与 BehaviorTree.ROS2
- [small_gicp](https://github.com/koide3/small_gicp)
- Livox SDK2（随 `livox_ros_driver2` 提供）

部分第三方依赖已随仓库置于 `src/dependencies/` 下（BehaviorTree.ROS2、joint_state_publisher、sdformat_tools）。其余系统依赖建议通过 `rosdep` 安装：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
# 在工作空间根目录（本仓库即为工作空间，包含 src/）
colcon build --symlink-install
source install/setup.bash
```

首次构建 `point_lio` 与 `livox_ros_driver2` 耗时较长，属正常现象。

## 快速开始

### 仿真

启动完整仿真栈（回环仿真里程计 + 地图服务 + 导航 + RViz）：

```bash
ros2 launch nav_bringup simulation.launch.py
```

常用启动参数（`simulation.launch.py`）：

| 参数           | 默认值                        | 说明                              |
| -------------- | ----------------------------- | --------------------------------- |
| `use_sim_time` | `true`                        | 使用仿真时钟                      |
| `use_rviz`     | `true`                        | 启动 RViz                         |
| `autostart`    | `true`                        | 自动激活 Navigation2 生命周期节点 |
| `params_file`  | `nav2_params.simulation.yaml` | 参数文件路径                      |
| `map`          | 空                            | 地图 yaml 路径                    |
| `namespace`    | 空                            | 顶层命名空间                      |

### 实车

实车部署时，分模块启动并使用实车参数：

```bash
# 导航栈（地形分析 + loam_interface + sensor_scan + Navigation2）
ros2 launch nav_bringup navigation_launch.py params_file:=<path>/nav2_params.reality.yaml use_sim_time:=false

# 定位（重定位 + 地图服务）
ros2 launch nav_bringup localization_launch.py

# SLAM 建图
ros2 launch nav_bringup slam_launch.py

# 手柄遥控
ros2 launch nav_bringup joy_teleop_launch.py
```

## 软件包一览

### 自研软件包

| 软件包                        | 说明                                                         |
| ----------------------------- | ------------------------------------------------------------ |
| `nav_bringup`                 | 编排枢纽：launch 体系与仿真/实车参数文件，聚合全栈节点       |
| `loam_interface`              | 里程计坐标系适配：`lidar_odom` 系 → `odom` 系                |
| `sensor_scan_generation`      | 时间同步、`odom → base_footprint` TF 广播、底盘里程计生成    |
| `small_gicp_relocalization`   | 基于先验 PCD 地图的 GICP 全局重定位                          |
| `terrain_analysis`            | 近场地形可通行性分析（21×21 体素，1.0 m）                    |
| `terrain_analysis_ext`        | 大尺度地形分析 + 连通性去天花板（41×41 体素，2.0 m）         |
| `nav2_plugins`                | 自定义 Navigation2 插件：行为树节点、代价地图层、恢复行为    |
| `omni_pid_pursuit_controller` | 全向 PID 纯追踪控制器插件                                    |
| `ign_sim_pointcloud_tool`     | 仿真点云格式转换（补 ring / time 字段以适配 point_lio）      |
| `teleop_twist_joy`            | 手柄遥控（支持 enable 按钮、stamped twist、坐标变换）        |
| `pcd2pgm`                     | 离线工具：PCD 点云地图转 2D 占据栅格                         |
| `robot_interfaces`            | 自定义消息定义（Gimbal / GimbalCmd / Models / RobotStateInfo） |

### 集成的第三方软件包

| 软件包                        | 说明                                       |
| ----------------------------- | ------------------------------------------ |
| `point_lio`                   | Point-LIO 紧耦合激光惯性里程计（含 IKFoM） |
| `livox_ros_driver2`           | Livox MID360 / HAP 固态雷达驱动            |
| `pointcloud_to_laserscan`     | 点云与激光扫描互转                         |
| `nav2_loopback_sim`           | Navigation2 回环仿真（免物理引擎里程计）   |
| `rosbag2_composable_recorder` | 可组合的 rosbag2 录制节点                  |

### 行为树节点（`nav2_plugins`）

| 节点                                | 类型          | 用途                                 |
| ----------------------------------- | ------------- | ------------------------------------ |
| `SelectFixedPath`                   | Action        | 由固定索引生成单点路径               |
| `SelectPatrolPath`                  | Action        | 生成往返巡逻路径                     |
| `SelectPathGoalPose`                | Action        | 取路径末点作为目标                   |
| `SendNav2Goal`                      | Action        | 调用 NavigateToPose action           |
| `SendNavThroughPoses`               | Action        | 异步调用 NavigateThroughPoses action |
| `PublishNavGoal`                    | Action        | 向话题直接发布目标位姿               |
| `PublishTwist` / `PublishSpinSpeed` | Action        | 直接发布速度 / 自旋速度              |
| `HoldStopFlag`                      | Action        | 定时发布停留标志                     |
| `IsPathGoalReached`                 | Condition     | 判断是否到达路径终点                 |
| `RecoveryNode`                      | Control       | 主动作 + 恢复动作循环                |
| `RateController`                    | Decorator     | 按频率限流子节点                     |
| `BackUpFreeSpace`                   | Behavior      | 朝最空旷方向后退的恢复行为           |
| `IntensityVoxelLayer`               | Costmap Layer | 强度过滤的 3D 体素障碍层             |

## 配置

主参数文件位于 `src/navigation/nav_bringup/config/`：

- `nav2_params.simulation.yaml` — 仿真参数
- `nav2_params.reality.yaml` — 实车参数

涵盖 point_lio、坐标系名、代价地图层、控制器插件与参数、行为树等全栈配置。行为树 XML 位于 `src/navigation/nav_bringup/behavior_trees/`。

## 坐标系约定

TF 树的典型层级（仿真默认）：

```
map ──(small_gicp_relocalization)──▶ odom ──(sensor_scan_generation)──▶ base_footprint
                                                                            ├─ front_mid360   (lidar)
                                                                            └─ gimbal_yaw     (robot_base)
```

坐标系名均可通过参数覆盖，无需改动源码即可适配不同机器人平台。

## 文档

`docs/PROJECT_DOC.md` 提供分层的代码理解文档：架构总览、逐模块公共契约与不变量、风险图与符号索引，供快速理解代码全貌与评估改动影响面。

## 许可证与第三方声明

本仓库是多个软件包的集合，各软件包保留其原始许可证与版权声明。所有许可证原文位于各软件包目录下（`LICENSE` / `LICENSE.txt` / `NOTICE`），未作改动。集成第三方代码时，其原始版权与许可证均予以保留。

各软件包许可证如下（以各软件包 `package.xml` 声明与随附 LICENSE 文件为准）：

| 软件包                                                       | 许可证      | 来源 / 版权                                                  |
| ------------------------------------------------------------ | ----------- | ------------------------------------------------------------ |
| `point_lio`                                                  | BSD         | Point-LIO（HKU-MARS）；含 Ji Zhang, CMU (2013)、Southwest Research Institute (2016) 等衍生版权 |
| `point_lio/include/IKFoM`                                    | **GPL-2.0** | IKFoM 工具箱（见下方说明）                                   |
| `livox_ros_driver2`                                          | MIT         | Livox Technology                                             |
| `livox_ros_driver2/3rdparty/rapidjson`                       | MIT         | THL A29 Limited (Tencent) / Milo Yip                         |
| `pointcloud_to_laserscan`                                    | BSD         | Willow Garage (2010–2012)、Eurotec (2019)                    |
| `nav2_loopback_sim`                                          | Apache-2.0  | Open Navigation LLC                                          |
| `rosbag2_composable_recorder`                                | Apache-2.0  | 上游第三方                                                   |
| `nav2_plugins`                                               | Apache-2.0  | Lihan Chen (2025)；部分节点参考 ros-navigation/navigation2 与 PolarisXQ/SCURM_SentryNavigation，详见 `NOTICE` |
| `omni_pid_pursuit_controller`                                | Apache-2.0  | Lihan Chen                                                   |
| `small_gicp_relocalization`                                  | Apache-2.0  | 依赖 [small_gicp](https://github.com/koide3/small_gicp)（k. Koide） |
| `teleop_twist_joy`                                           | Apache-2.0  | 改自 ROS `teleop_twist_joy`                                  |
| `pcd2pgm`                                                    | Apache-2.0  | —                                                            |
| `terrain_analysis` / `terrain_analysis_ext`                  | BSD         | 移植自 CMU 自主探索开发环境（Ji Zhang 等）                   |
| `loam_interface` / `sensor_scan_generation` / `ign_sim_pointcloud_tool` / `nav_bringup` / `robot_interfaces` | Apache-2.0  | 本项目自研（`package.xml` 声明，无独立 LICENSE 文件）        |
| `BehaviorTree.ROS2`（`src/dependencies/`）                   | MIT         | BehaviorTree.CPP 团队                                        |
| `joint_state_publisher`（`src/dependencies/`）               | BSD         | ROS 社区                                                     |
| `sdformat_tools`（`src/dependencies/`）                      | Apache-2.0  | 上游第三方                                                   |

**关于 GPL-2.0 组件的重要提示**：`point_lio` 依赖的 IKFoM 工具箱（`src/navigation/point_lio/include/IKFoM/`）采用 GPL-2.0 许可证。GPL 属于 copyleft 许可证，与本仓库其余软件包普遍采用的 Apache-2.0 / BSD / MIT 存在传染性与兼容性差异。若你计划分发衍生作品或用于闭源产品，请先评估 GPL-2.0 的合规影响，或参考 point_lio 上游对该组件的处理方式。

> 说明：上表依据仓库内实际 LICENSE 文件与 `package.xml` 声明整理，仅供导航之用，不构成法律意见。以各软件包目录下的许可证原文为准。

## 致谢

本框架建立在以下开源工作之上：

- [Point-LIO](https://github.com/hku-mars/Point-LIO)（HKU-MARS）
- [Navigation2](https://github.com/ros-navigation/navigation2)
- [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
- [small_gicp](https://github.com/koide3/small_gicp)
- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- CMU [Autonomous Exploration Development Environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)（terrain_analysis 系列）

## 已知问题

代码级的待办事项与风险标注详见 `docs/PROJECT_DOC.md` 的「Layer 3: 风险图」。其中需要使用者注意的一项：

- **控制器插件命名**：`nav2_params.*.yaml` 中 `FollowPath.plugin` 配置为 `omni_pid_pursuit_controller::OmniPidPursuitController`，而插件实际注册名带 `pb_` 前缀（`pb_omni_pid_pursuit_controller::OmniPidPursuitController`）。若控制器加载失败，请将 YAML 中的插件名改为带 `pb_` 前缀的形式。

地形分析节点当前采用文件作用域全局状态，为单实例设计，不支持进程内多实例组合，重构计划见风险图。

