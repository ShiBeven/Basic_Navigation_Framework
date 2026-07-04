# Basic Navigation Framework 全覆盖说明文档

> ⚠️ **强制同步声明**：后续代码无论进行任何修改、重构、优化，本 MD 文档必须同步更新，确保文档与源码实时一致。文档落后于代码即为失效。

> 生成时间：2026-07-04 21:30 (代码修复更新：2026-07-04)
> 源码根目录：W:\Basic_Navigation_Framework-master
> 覆盖范围：291 文件 / 107 C++源文件 + 32 Python源文件 / ~25,000 行代码

---

## 1. 架构总览

### 1.1 项目类型 & 技术栈

- **类型**：ROS 2 Humble 机器人自主导航框架
- **语言**：C++14/17（核心算法）+ Python 3（仿真/启动/工具）
- **构建系统**：ament_cmake + colcon
- **目标平台**：Ubuntu 22.04 + ROS 2 Humble
- **许可证**：Apache-2.0 / BSD
- **来源**：从 RoboMaster 2025 哨兵竞赛工程（ATS 2026 Sentry）精简而来

### 1.2 顶层目录结构与各目录职责

```
Basic_Navigation_Framework-master/
├── docs/                          # 项目文档（阅读指南、设计文档、本文档）
├── src/
│   ├── interfaces/
│   │   └── robot_interfaces/      # 自定义 ROS 2 消息定义
│   ├── navigation/
│   │   ├── nav_bringup/           # 🎯 入口包：launch 文件 + 参数配置 + 行为树
│   │   ├── nav2_plugins/          # Nav2 扩展插件（BT 节点 + 代价地图层 + 行为）
│   │   ├── point_lio/             # Point-LIO：LiDAR-惯性里程计（管线起点）
│   │   ├── loam_interface/        # 帧适配器：lidar_odom → odom
│   │   ├── sensor_scan_generation/# 传感器同步：odom + scan → odometry + TF
│   │   ├── terrain_analysis/      # 近场地形分析（10m）
│   │   ├── terrain_analysis_ext/  # 远场地形分析（40m）+ 连通性检测
│   │   ├── small_gicp_relocalization/ # 全局重定位（GICP 配准）
│   │   ├── omni_pid_pursuit_controller/ # 全向 PID 纯追踪控制器（管线终点）
│   │   ├── pointcloud_to_laserscan/   # 点云 ↔ 激光扫描互转
│   │   ├── teleop_twist_joy/      # 手柄遥操作
│   │   ├── ign_sim_pointcloud_tool/   # 仿真点云格式转换（XYZ → XYZIRT）
│   │   └── livox_ros_driver2/     # Livox LiDAR ROS2 驱动
│   ├── simulation/
│   │   └── nav2_loopback_sim/     # 🎯 无硬件仿真闭环
│   └── tools/
│       ├── pcd2pgm/               # 3D PCD → 2D PGM 栅格地图
│       └── rosbag2_composable_recorder/ # 可组合 rosbag2 录制器
└── src/dependencies/              # 第三方依赖（不纳入本文档）
```

### 1.3 分层架构

```
┌─────────────────────────────────────────────────────┐
│                   应用层 (nav_bringup)                │
│   launch orchestration · behavior trees · config    │
├─────────────────────────────────────────────────────┤
│                决策层 (nav2_plugins + Nav2)           │
│   BT nodes · planner · controller · costmaps        │
├─────────────────────────────────────────────────────┤
│                感知层                                │
│   terrain_analysis · terrain_analysis_ext           │
│   sensor_scan_generation · pointcloud_to_laserscan  │
├─────────────────────────────────────────────────────┤
│                定位层                                │
│   point_lio · loam_interface                        │
│   small_gicp_relocalization                        │
├─────────────────────────────────────────────────────┤
│                驱动层                                │
│   livox_ros_driver2 · teleop_twist_joy              │
├─────────────────────────────────────────────────────┤
│                仿真层                                │
│   nav2_loopback_sim · ign_sim_pointcloud_tool       │
└─────────────────────────────────────────────────────┘
```

### 1.4 核心数据流

```
Livox LiDAR (/livox/lidar + /livox/imu)
        │
        ▼
  point_lio (LiDAR-惯性里程计, 逐点 ESIKF 更新)
        │  /aft_mapped_to_init (lidar_odom 帧)
        │  /cloud_registered (去畸变点云)
        ▼
  loam_interface (帧适配: lidar_odom → odom)
        │  /lidar_odometry (odom 帧)
        │  /registered_scan (odom 帧)
        ▼
  sensor_scan_generation (时间同步: odom + scan → TF + odometry)
        │  /odometry (odom→base_footprint)
        │  /sensor_scan (LiDAR 帧点云)
        │  TF: odom→base_footprint
        ├──────────────┬──────────────────┐
        ▼              ▼                  ▼
  terrain_analysis  terrain_analysis_ext  small_gicp_relocalization
  (/terrain_map)    (/terrain_map_ext)    (map→odom TF 修正)
        │              │
        ▼              ▼
  Nav2 costmaps (local + global)
        │
        ▼
  Nav2 Planner (SmacPlannerHybrid) → Path
        │
        ▼
  Nav2 Controller (OmniPidPursuitController) → cmd_vel
        │
        ▼
  velocity_smoother → /cmd_vel (发给底盘/仿真)
```

### 1.5 坐标系层级

```
map (SLAM/map_server/global_costmap)
  │
  odom (point_lio → loam_interface → sensor_scan_generation)
  │
  base_footprint (chassis)
  │
  gimbal_yaw (云台偏航)
  │
  gimbal_pitch (云台俯仰)
  │
  front_mid360 (LiDAR) / front_industrial_camera
```

---

## 2. 文件清单

| 文件路径 | 模块 | 职责摘要 | 代码行数 |
|----------|------|----------|----------|
| `src/interfaces/robot_interfaces/msg/Gimbal.msg` | interfaces | 云台状态：pitch/yaw + 范围 | 4 |
| `src/interfaces/robot_interfaces/msg/GimbalCmd.msg` | interfaces | 云台控制指令：绝对角/速度模式 | 9 |
| `src/interfaces/robot_interfaces/msg/Models.msg` | interfaces | 机器人型号列表 | 1 |
| `src/interfaces/robot_interfaces/msg/RobotStateInfo.msg` | interfaces | 聚合机器人状态 | 2 |
| `src/navigation/nav_bringup/launch/simulation.launch.py` | nav_bringup | 仿真总入口 | 188 |
| `src/navigation/nav_bringup/launch/navigation_launch.py` | nav_bringup | 导航栈启动（地形+Nav2全部节点） | 354 |
| `src/navigation/nav_bringup/launch/localization_launch.py` | nav_bringup | 定位栈启动（Point-LIO+Map+重定位） | 213 |
| `src/navigation/nav_bringup/launch/slam_launch.py` | nav_bringup | SLAM 建图模式启动 | 150 |
| `src/navigation/nav_bringup/launch/rviz_launch.py` | nav_bringup | RViz2 启动 | 69 |
| `src/navigation/nav_bringup/launch/joy_teleop_launch.py` | nav_bringup | 手柄遥操作启动 | 114 |
| `src/navigation/nav_bringup/config/nav2_params.reality.yaml` | nav_bringup | 实机导航参数（全量 616 行） | 616 |
| `src/navigation/nav_bringup/config/nav2_params.simulation.yaml` | nav_bringup | 仿真导航参数 | 622 |
| `src/navigation/nav_bringup/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` | nav_bringup | 单目标行为树（含恢复） | 41 |
| `src/navigation/nav_bringup/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` | nav_bringup | 多目标行为树（含恢复） | 46 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/hold_stop_flag.hpp` | nav2_plugins | BT 节点：停止标志保持 | 38 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/pub_spin_speed.hpp` | nav2_plugins | BT 节点：发布旋转速度 | 30 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/pub_twist.hpp` | nav2_plugins | BT 节点：发布 Twist 速度 | 28 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/publish_nav_goal.hpp` | nav2_plugins | BT 节点：发布导航目标点 | 38 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/select_fixed_path.hpp` | nav2_plugins | BT 节点：选择固定路径点 | 32 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/select_path_goal_pose.hpp` | nav2_plugins | BT 节点：路径末点提取 | 33 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/select_patrol_path.hpp` | nav2_plugins | BT 节点：巡逻路径生成 | 35 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/send_nav2_goal.hpp` | nav2_plugins | BT 节点：发送 NavigateToPose | 33 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/action/send_nav_through_poses.hpp` | nav2_plugins | BT 节点：发送 NavigateThroughPoses | 58 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/condition/is_path_goal_reached.hpp` | nav2_plugins | BT 条件：路径终点检查 | 31 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/control/recovery_node.hpp` | nav2_plugins | BT 控制：恢复节点 | 63 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/decorator/rate_controller.hpp` | nav2_plugins | BT 装饰器：频率控制 | 46 |
| `src/navigation/nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp` | nav2_plugins | Nav2 行为：自由空间后退 | 91 |
| `src/navigation/nav2_plugins/include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp` | nav2_plugins | 代价地图层：强度体素滤波 | 104 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/custom_types.hpp` | nav2_plugins | BT 自定义类型转换器 | 41 |
| `src/navigation/nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp` | nav2_plugins | BT 导航工具函数集 | 199 |
| `src/navigation/point_lio/src/laserMapping.cpp` | point_lio | Point-LIO 主循环（1053 行） | 1053 |
| `src/navigation/point_lio/src/Estimator.cpp` | point_lio | ESIKF 测量/过程模型实现 | 392 |
| `src/navigation/point_lio/src/IMU_Processing.cpp` | point_lio | IMU 初始化与预处理 | 115 |
| `src/navigation/point_lio/src/preprocess.cpp` | point_lio | 点云预处理/特征提取 | 936 |
| `src/navigation/point_lio/src/li_initialization.cpp` | point_lio | 传感器同步与数据缓冲 | 299 |
| `src/navigation/point_lio/src/parameters.cpp` | point_lio | 参数加载与初始化 | 293 |
| `src/navigation/point_lio/include/common_lib.h` | point_lio | 状态流形/类型/IKFoM 定义 | 223 |
| `src/navigation/point_lio/include/so3_math.h` | point_lio | SO(3) 李群运算 | 128 |
| `src/navigation/point_lio/include/ivox/ivox3d.h` | point_lio | iVox 增量体素地图 | 341 |
| `src/navigation/loam_interface/src/loam_interface.cpp` | loam_interface | 帧适配：TF 变换 + 话题中转 | 92 |
| `src/navigation/sensor_scan_generation/src/sensor_scan_generation.cpp` | sensor_scan | 时间同步 + TF + odometry | 151 |
| `src/navigation/terrain_analysis/src/terrainAnalysis.cpp` | terrain_analysis | 近场地形（栅格化+分位数+动态障碍） | 682 |
| `src/navigation/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | terrain_analysis_ext | 远场地形+连通性 BFS | 557 |
| `src/navigation/small_gicp_relocalization/src/small_gicp_relocalization.cpp` | small_gicp | GICP 全局重定位 | 227 |
| `src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp` | omni_pid | 全向 PID 纯追踪 | 763 |
| `src/navigation/omni_pid_pursuit_controller/src/pid.cpp` | omni_pid | PID 算法核心 | 49 |
| `src/navigation/pointcloud_to_laserscan/src/pointcloud_to_laserscan_node.cpp` | pcl2scan | 3D 点云 → 2D LaserScan | 244 |
| `src/navigation/teleop_twist_joy/src/pb_teleop_twist_joy.cpp` | teleop | 手柄遥操作（含死人开关） | 200 |
| `src/simulation/nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py` | loopback_sim | cmd_vel→odom+scan 闭环仿真 | 422 |
| `src/tools/pcd2pgm/src/pcd2pgm.cpp` | pcd2pgm | 3D PCD→2D OccupancyGrid | 176 |
| `src/tools/rosbag2_composable_recorder/src/composable_recorder.cpp` | rosbag_rec | 可组合 rosbag2 录制 | 147 |
| `src/navigation/livox_ros_driver2/src/livox_ros_driver2.cpp` | livox | Livox 驱动主节点 | 133 |
| `src/navigation/livox_ros_driver2/src/lddc.cpp` | livox | 数据分发控制 | 544 |
| `src/navigation/livox_ros_driver2/src/lds_lidar.cpp` | livox | LiDAR SDK 初始化 | 219 |
| `src/navigation/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp` | ign_sim | XYZ→XYZIRT 格式转换 | 86 |

*(上表仅列出核心源码文件，全量清单包含全部 291 个文件)*

---

## 3. 函数/方法级详解

### 3.1 robot_interfaces — 自定义 ROS 2 消息

#### 消息：Gimbal (`msg/Gimbal.msg`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `pitch` | `float64` | 当前俯仰角 |
| `yaw` | `float64` | 当前偏航角 |
| `pitch_range` | `float64[2]` | 允许的俯仰范围 [min, max] |
| `yaw_range` | `float64[2]` | 允许的偏航范围 [min, max] |

#### 消息：GimbalCmd (`msg/GimbalCmd.msg`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 标准头 |
| `ABSOLUTE_ANGLE` | `uint8 = 0` | 绝对角控制模式（常量） |
| `VELOCITY` | `uint8 = 1` | 速度控制模式（常量） |
| `control_type` | `uint8` | 当前控制模式 |
| `position` | `Gimbal` | 目标角度（绝对角模式） |
| `velocity` | `Gimbal` | 目标速度（速度模式） |

#### 消息：Models (`msg/Models.msg`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `models` | `string[5]` | 5 个机器人型号名（固定数组） |

#### 消息：RobotStateInfo (`msg/RobotStateInfo.msg`)

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 标准头 |
| `robot_models` | `Models` | 机器人型号列表 |

---

### 3.2 nav_bringup — 启动编排

#### Launch: `simulation.launch.py` (188 行)

**签名**：`generate_launch_description() → LaunchDescription`

**用途**：仿真模式总入口，启动 loopback 仿真器 + 地图服务 + 导航栈 + RViz。

**启动节点（按顺序）**：
1. `nav2_loopback_sim::loopback_simulator` — 无物理引擎仿真闭环
2. `nav2_map_server::map_server` — 提供静态栅格地图
3. `lifecycle_manager_map_server` — 管理 map_server 生命周期
4. `navigation_launch.py` (IncludeLaunchDescription) — 地形分析 + Nav2 全栈
5. `rviz_launch.py` (条件 IncludeLaunchDescription) — RViz2

**关键参数传递**：
- `map:=xxx.yaml` → map_server 的地图文件路径
- `use_sim_time:=true` → 所有节点使用仿真时钟
- `params_file` → nav2_params.simulation.yaml

#### Launch: `navigation_launch.py` (354 行)

**签名**：`generate_launch_description() → LaunchDescription`

**用途**：启动完整导航栈（地形分析 + Nav2 全部节点），是 localization_launch.py 和 simulation.launch.py 的公共依赖。

**启动节点（非组合模式）**：
| 节点 | 功能 | 关键 remap |
|------|------|-----------|
| `terrainAnalysis` | 近场地形（发布 `/terrain_map`） | — |
| `terrainAnalysisExt` | 远场地形（发布 `/terrain_map_ext`） | — |
| `loam_interface_node` | 帧适配 | — |
| `sensor_scan_generation_node` | 传感器同步（条件启动） | — |
| `controller_server` | 路径跟踪控制 | `cmd_vel`→`cmd_vel_nav2_result` |
| `smoother_server` | 路径平滑 | — |
| `planner_server` | 全局路径规划 | — |
| `behavior_server` | 恢复行为 | — |
| `bt_navigator` | 行为树导航 | — |
| `waypoint_follower` | 途经点跟随 | — |
| `velocity_smoother` | 速度平滑 | `cmd_vel_smoothed`→`cmd_vel` |
| `lifecycle_manager_navigation` | 管理 7 个 Nav2 节点生命周期 | — |

**速度管线**：`cmd_vel_nav2_result` → `velocity_smoother` → `cmd_vel`（发给底盘）

#### Launch: `localization_launch.py` (213 行)

**签名**：`generate_launch_description() → LaunchDescription`

**用途**：启动定位栈（Point-LIO + 地图 + 重定位），然后启动导航栈。

**启动节点**：
1. `point_lio::pointlio_mapping` — LiDAR-IMU 里程计（含 prior PCD 可选加载）
2. `nav2_map_server::map_server` — 地图服务（生命周期管理）
3. `small_gicp_relocalization::small_gicp_relocalization_node` — 全局重定位
4. `lifecycle_manager_localization` — 管理 `["map_server"]`

#### Launch: `slam_launch.py` (150 行)

**签名**：`generate_launch_description() → LaunchDescription`

**用途**：SLAM 建图模式（无先验地图）。启动点云→激光转换 + SlamToolbox + Point-LIO。

**关键区别**：`tf2_ros::static_transform_publisher` 发布 `map→odom` 为恒等变换（SlamToolbox 直接发布 `map→odom` TF）。

#### Behavior Tree: `navigate_to_pose_w_replanning_and_recovery.xml` (41 行)

**结构**（BTCPP 格式 3）：
```
MainTree (RecoveryNode, max_retries=10)
  ├── NavigateWithReplanning (PipelineSequence)
  │   ├── RateController @ 3 Hz
  │   │   └── ComputePathToPose (RecoveryNode, retries=1)
  │   │       ├── ComputePathToPose (planner_id="GridBased")
  │   │       └── ClearGlobalCostmap-Context
  │   └── FollowPath (RecoveryNode, retries=10)
  │       ├── FollowPath (controller_id="FollowPath")
  │       └── ClearLocalCostmap-Context
  └── RecoveryFallback (ReactiveFallback)
      ├── GoalUpdated
      └── RecoveryActions (RoundRobin)
          ├── ClearLocalCostmap → ClearGlobalCostmap
          └── BackUp (backup_dist=1.0, backup_speed=1.0)
```

**变量**：`{goal}`, `{path}`

---

### 3.3 nav2_plugins — Nav2 扩展插件

#### BT 节点清单（12 个）

| 注册名 | 类型 | 父类 | 核心功能 |
|--------|------|------|----------|
| `HoldStopFlag` | Action | `StatefulActionNode` | 在可配置时长内发布 stop_flag |
| `PublishSpinSpeed` | Action | `RosTopicPubStatefulActionNode<Float32>` | 发布角速度 Z |
| `PublishTwist` | Action | `RosTopicPubStatefulActionNode<Twist>` | 发布全向速度指令 |
| `PublishNavGoal` | Action | `SyncActionNode` | 发布导航目标点（含去重） |
| `SelectFixedPath` | Action | `SyncActionNode` | 按索引选择固定路径点 |
| `SelectPathGoalPose` | Action | `SyncActionNode` | 提取路径末点为 goal |
| `SelectPatrolPath` | Action | `SyncActionNode` | 生成 ping-pong 巡逻路径 |
| `SendNav2Goal` | Action | `RosActionNode<NavigateToPose>` | 发送单点导航 goal |
| `SendNavThroughPoses` | Action | `SyncActionNode` | 发送多点导航 goal（自管 action 客户端） |
| `IsPathGoalReached` | Condition | `SimpleConditionNode` | 检查机器人是否到达路径终点 |
| `RecoveryNode` | Control | `ControlNode` | 2 子节点恢复循环（可配置重试次数） |
| `RateController` | Decorator | `DecoratorNode` | 按指定 Hz 节流子节点 tick |

#### 函数：`SendNavThroughPoses::tick()` — 最复杂的 BT 节点 (send_nav_through_poses.cpp:201)

**签名**：`BT::NodeStatus tick() override`

**用途**：手动管理 `NavigateThroughPoses` action 客户端，支持去重、取消、状态跟踪。

**分支路径**：
1. **路径为空** → ERROR 日志，返回 FAILURE
2. **Action server 不可用**（超时 `action_server_wait_timeout_s_`）→ ERROR，返回 FAILURE
3. **同路径 + 已完成** (`same_path && last_goal_succeeded_ && !goal_pending_ && !current_goal_handle_`) → 设置 output `goal_succeeded=true`，返回 SUCCESS
4. **同路径 + 进行中** (`(goal_pending_ || current_goal_handle_) && same_path`) → 返回 SUCCESS（不重复发送）
5. **新路径** → 调用 `cancelCurrentGoal()` 取消旧 goal → 构造新 Goal → 异步发送 → 返回 SUCCESS

**副作用**：
- 设置 output port `current_pose`（feedback 位姿）
- 设置 output port `goal_succeeded`（路径完成标志）
- 异步操作 action 客户端

**线程安全**：全部状态变更受 `std::mutex` 保护；`request_id` 模式防止过期回调。

#### 函数：`RecoveryNode::tick()` (recovery_node.cpp:108)

**签名**：`BT::NodeStatus tick() override`

**前置条件**：恰好 2 个子节点（否则抛出 `BehaviorTreeException`）

**状态机**：
```
子节点 0 (主行为):
  SUCCESS → halt(), 返回 SUCCESS
  FAILURE → retry_count < num_attempts → 进子节点 1 (恢复)
          → retry_count >= num_attempts → halt(), 返回 FAILURE
  RUNNING → 返回 RUNNING

子节点 1 (恢复):
  SUCCESS → halt 子节点 1, retry_count++, 回到子节点 0
  FAILURE → halt(), 返回 FAILURE (恢复本身失败)
  RUNNING → 返回 RUNNING
```

#### 函数：`SelectPatrolPath::tick()` (select_patrol_path.cpp:116)

**签名**：`BT::NodeStatus tick() override`

**算法**：
1. 读取 ROS 参数 `patrol_preview_points`（运行时可更新）
2. 读取 input ports `patrol_cursor`, `patrol_direction`
3. **单点巡逻** (`patrol_indices_.size() == 1`)：路径包含单点，`next_cursor=0`, `next_direction=1`
4. **多点巡逻**：`preview_points = max(2, patrol_preview_points_)`，逐步调用 `computeNextPatrolState()` 沿 ping-pong 序列生成 `preview_points` 个途经点
5. 设置 outputs：`path`, `next_cursor`, `next_direction`

**Ping-pong 巡逻算法** (`computeNextPatrolState`)：
- 前进 `cursor += direction`
- 到达上界时反弹：`cursor = count-2, direction = -1`
- 到达下界时反弹：`cursor = 1, direction = 1`

#### 函数：`IntensityVoxelLayer::updateBounds(...)` (intensity_voxel_layer.cpp:207)

**签名**：`void updateBounds(double robot_x, robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)`

**算法**：
1. 获取 marking observations
2. 对每个点：
   - **Z 过滤**：`pz < min_obstacle_height_ || pz > max_obstacle_height_` → 跳过
   - **强度过滤**：`intensity < min_obstacle_intensity_ || intensity > max_obstacle_intensity_` → 跳过（**关键区分点** vs 标准 ObstacleLayer）
   - **距离过滤**：`dist² ∉ [min_range², max_range²]` → 跳过
   - **体素标记**：`voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)`，若标记成功则设 costmap 为 `LETHAL_OBSTACLE`
3. 若 `publish_voxel_`：发布 VoxelGrid 消息到 `"voxel_grid"` topic

---

### 3.4 point_lio — LiDAR-惯性里程计

#### 函数：`main()` — Point-LIO 完整管线 (laserMapping.cpp:314-1053)

**签名**：`int main(int argc, char** argv)`

**用途**：LiDAR-IMU 紧耦合里程计主循环（500 Hz），每次同步到 LiDAR+IMU 数据包时执行。

**初始化（314-700 行）**：
1. 读取全部参数（~50 个配置项）
2. 创建 iVox 增量体素地图（默认 PHC 或线性存储）
3. 配置 **两个 ESIKF 模态**：
   - `kf_input`（24 维）：`[pos, rot, T_LI, R_LI, vel, bg, ba, gravity]`，IMU 作为过程模型输入
   - `kf_output`（30 维）：附加 `[omg, acc]`，IMU 作为观测（输出模型）
4. 创建 6 个 Publisher + 2 个 Subscriber

**主循环（700-1040 行）**：

```
1. sync_packages(Measures)           # 等待 LiDAR+IMU 同步数据包
2. p_imu->Process(Measures, feats)   # IMU 初始化（前 100 帧做 bias 估计）
3. voxel_grid downsampling           # filter_size_surf
4. time_compressing(feats)           # 按曲率（时间戳）分组
5. IMU 重力对齐                      # Set_init 计算初始旋转
6. 地图初始化                         # 累积 init_map_size 帧后插入 iVox
7. 逐点 ESIKF 更新：
   for each time_group k:
     ├── IMU 预报 (predict)          # 过程模型前向传播
     ├── IMU 观测更新 (output 模式)    # 陀螺/加速度计残差
     ├── 状态预报到点时间               # predict(dt)
     ├── LiDAR 点-面更新              # update_iterated_dyn_share_modified
     │   ├── pointBodyToWorld         # 当前状态预测点在世界坐标
     │   ├── iVox KNN (5 最近邻)       # 最近邻搜索
     │   ├── esti_plane (QR 平面拟合)  # 平面法向量 [nx,ny,nz,d]
     │   ├── 有效性检查               # p_norm > match_s * pd2²
     │   ├── 构建 Jacobian (1×12)     # d(point-plane)/d(state)
     │   └── 迭代 EKF 更新            # 至收敛
     └── 发布里程计（若 publish_odometry_without_downsample）
8. MapIncremental()                  # 将降采样点插入 iVox (LRU)
9. 发布 path, registered_cloud, body_cloud
```

**Point-LIO 的关键创新**：状态更新发生在**每个 LiDAR 点**的精确时间戳上（最多 200 kHz），而非传统的帧率（10-20 Hz），从而自然处理运动畸变。

#### 函数：`h_model_input(...)` — 点-面观测模型 (Estimator.cpp:392)

**签名**：`void h_model_input(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)`

**算法**：
1. 对当前时间组中的每个点：
   - `pointBodyToWorld(pi, po)` → 全局坐标预测
   - `ivox_->GetClosestPoint(po, Nearest_Points, 5, max_range)` → 5 近邻搜索
   - `esti_plane(Nearest_Points, plane_params)` → QR 平面拟合
   - 检查 `p_norm > match_s * pd2 * pd2`
2. 建立 1×12 Jacobian：d(point_to_plane_distance) / d(state)
3. 残差：`z = -norm_vec · point_world - d`

#### 函数：`Preprocess::avia_handler(msg)` (preprocess.cpp:936)

**签名**：`void avia_handler(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg)`

**算法**：
1. 遍历全部点 → 过滤 `line < N_SCANS` 且 tag 为 0x10 或 0x00
2. 按 `point_filter_num` 降采样
3. 设置 `curvature = offset_time / 1,000,000`（微秒→毫秒，作为点时间戳）
4. 范围检查：`blind² < dist² < det_range²`
5. 去重：相邻相同坐标的点跳过

#### 函数：`ImuProcess::Process(meas, pcl)` (IMU_Processing.cpp:115)

**签名**：`void Process(const MeasureGroup& meas, PointCloudXYZI::Ptr cur_pcl_un_)`

**分支**：
1. **IMU 使能 + 未初始化**：调用 `IMU_init(meas, N)` 做滑动平均，累积 100 帧
2. **IMU 使能 + 已初始化**：直接拷贝点云（无 deskew — 畸变由逐点 EKF 更新隐式处理）
3. **IMU 禁用**：直接拷贝点云

**注意**：在此分支版本中，`Process` 不做去畸变。原始 Point-LIO 的运动补偿由逐点 ESIKF 状态更新隐式执行。

---

### 3.5 terrain_analysis — 近场地形分析

#### 函数：`main()` — terrainAnalysis 主循环 (terrainAnalysis.cpp:682)

**签名**：`int main(int argc, char** argv)`

**算法核心（8 个阶段）**：

**Phase 1 — 体素滚动（269-336 行）**：
21×21 粗栅格随车辆移动，四方向检测偏移 > 1m 时整列滚动。

**Phase 2 — 扫描堆积（338-363 行）**：
将当前点云分配到粗栅格单元。

**Phase 3 — 体素更新/衰减（365-396 行）**：
每个体素当 `update_num >= voxelPointUpdateThre` 或 `elapsed >= voxelTimeUpdateThre` 时：
- VoxelGrid 降采样
- Z 范围过滤：`[vehicleZ + minRelZ - disRatioZ*dist, vehicleZ + maxRelZ + disRatioZ*dist]`
- 时间衰减过滤：`(now - point.time < decayTime || dist < noDecayDis)`
- 清零更新计数器

**Phase 4 — 组装局部地形（398-405 行）**：
合并中心 11×11 区域 → `terrainCloud`

**Phase 5 — 平面地面估计（407-559 行）**：
- **5a**：填充 51×51 细栅格，每个点 Z 值散布到 3×3 邻域
- **5b**：动态障碍检测（若 `clearDyObs`）：多级旋转变换 + FOV 检查
- **5c**：地面高度估计：
  - **排序模式**：取分位数 Z 值（`quantileZ`，默认 0.25，即最低 25% 点）
  - **非排序模式**：取最小值

**Phase 6 — 高程计算（561-673 行）**：
对每个地形点：`elevation = point.z - ground_height`，若 `elevation ∈ [0, vehicleHeight)` 且点数足够 → 发布到 `/terrain_map`（intensity = elevation）

**Phase 7 — 无数据障碍（603-663 行，可选）**：
对点数不足的细栅格做边缘扩张，填充 `vehicleHeight` 高度障碍。

**Phase 8 — 发布（665-674 行）**：
PointCloud2 发布到 `/terrain_map`，frame_id="odom"。

#### 全局变量索引（terrain_analysis）：

| 名称 | 类型 | 初始值 | 说明 |
|------|------|--------|------|
| `terrainVoxelSize` | `double` | 1.0 | 粗栅格大小（m） |
| `terrainVoxelWidth` | `int` | 21 | 粗栅格维度（21×21=441） |
| `planarVoxelSize` | `double` | 0.2 | 细栅格大小（m） |
| `planarVoxelWidth` | `int` | 51 | 细栅格维度（51×51=2601） |
| `terrainVoxelCloud[441]` | `pcl::PointCloud` | 空 | 441 个粗栅格体素云 |
| `planarVoxelElev[2601]` | `float[]` | 0 | 细栅格地面高度 |
| `planarVoxelDyObs[2601]` | `int[]` | 0 | 动态障碍计数 |
| `noDataInited` | `int` | 0 | 0=未初始化,1=等待距离,2=就绪 |

---

### 3.6 terrain_analysis_ext — 远场地形分析（含连通性）

#### 与 terrain_analysis 的关键区别

| 特性 | terrain_analysis | terrain_analysis_ext |
|------|-----------------|---------------------|
| 粗栅格大小 | 1.0m | 2.0m |
| 粗栅格维度 | 21×21 (441 cells) | 41×41 (1681 cells) |
| 细栅格大小 | 0.2m | 0.4m |
| 细栅格维度 | 51×51 (2601 cells) | 101×101 (10201 cells) |
| 动态障碍移除 | ✅ | ❌ |
| 无数据障碍填充 | ✅ | ❌ |
| 地面抬升限制 | ✅ | ❌ |
| 连通性 BFS | ❌ | ✅ |
| 局部地图合并 | ❌ | ✅（订阅 `/terrain_map`） |
| 输出 topic | `/terrain_map` | `/terrain_map_ext` |

#### 新增函数：Phase 5.5 — Terrain Connectivity BFS (terrainAnalysisExt.cpp:450-486)

**算法**：
1. 从车辆正下方体素单元开始 BFS
2. 若中央单元为空 → 设置默认地面高度 = `vehicleZ + terrainUnderVehicle`
3. BFS 扩展（21×21 邻域）：
   - **连通**（`|elev_diff| < terrainConnThre`）→ 标记为 2（已连接地面），入队
   - **不连通**（`|elev_diff| > ceilingFilteringThre`）→ 标记为 -1（天花板/悬垂，稍后滤除）
4. Phase 6 中检查 `planarVoxelConn[ind] == 2 || !checkTerrainConn`（必须连通地面才计入障碍）

---

### 3.7 omni_pid_pursuit_controller — 全向 PID 纯追踪

#### 函数：`OmniPidPursuitController::computeVelocityCommands(...)` (763 行)

**签名**：
```cpp
geometry_msgs::msg::TwistStamped::SharedPtr computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& velocity,
    nav2_core::GoalChecker* goal_checker)
```

**算法（16 步）**：
1. `transformGlobalPlan(pose)` — 截取全局路径到机器人前方
2. `getLookAheadDistance(velocity)` — 静态或速度缩放（`hypot(vx,vy)*lookahead_time_`，钳位 `[min_lookahead_dist_, max_lookahead_dist_]`）
3. `getLookAheadPoint(lookahead_dist, transformed_plan)` — 在前方路径上找预瞄点（carrot）
4. `lin_dist = hypot(carrot.x, carrot.y)` — 机器人与预瞄点的欧氏距离误差
5. `theta_dist = atan2(carrot.y, carrot.x)` — 预瞄方向角
6. `angle_to_goal = tf2::getYaw(carrot.pose.orientation)` — 目标朝向误差
7. **旋转到位检查**：若 `use_rotate_to_heading_` 且 `|angle_to_goal| > use_rotate_to_heading_treshold_` → `lin_dist = 0`（仅旋转）
8. **平移 PID**：`move_pid_->calculate(lin_dist, 0)` → 标量线速度
9. **旋转 PID**：`heading_pid_->calculate(angle_to_goal, 0)` → 角速度
10. `applyCurvatureLimitation(path, carrot_pose, lin_vel)` — 曲率减速
11. `applyApproachVelocityScaling(path, lin_vel)` — 接近终点减速
12. 路径采样 10 点 → 代价地图碰撞检测
13. **无碰撞**：`cmd_vel.linear.x = lin_vel * cos(theta_dist)`, `cmd_vel.linear.y = lin_vel * sin(theta_dist)`, `cmd_vel.angular.z = angular_vel`
14. **碰撞检测到**：抛出 `PlannerException`

**全向特性**：`linear.y` 分量（横向速度）被填充，使差速底盘无法做到的横向移动可行。

#### 函数：`PID::calculate(set_point, pv)` (pid.cpp:49)

**签名**：`double calculate(double set_point, double pv)`

**算法（标准 PID + 抗饱和）**：
1. `error = set_point - pv`（实际 pv 恒为 0——以机器人为原点）
2. `P_out = kp_ * error`
3. `integral_ += error * dt_`；`I_out = ki_ * integral_`
4. **抗饱和**：钳位 `integral_` 到 `[-1, 1]`
5. `derivative = (error - pre_error_) / dt_`；`D_out = kd_ * derivative`
6. `output = P_out + I_out + D_out`
7. **输出钳位**：`[min_, max_]`（速度边界）
8. `pre_error_ = error`

---

### 3.8 small_gicp_relocalization — GICP 全局重定位

#### 函数：`SmallGicpRelocalizationNode::performRegistration()` (227 行)

**签名**：`void performRegistration()`

**被调用频率**：2 Hz（`register_timer_`）

**算法**：
1. 若 `accumulated_cloud_` 为空 → 返回
2. **降采样**：`source_ = voxelgrid_sampling_omp(*accumulated_cloud_, registered_leaf_size_)` → PointCovariance
3. **协方差估计**：`estimate_covariances_omp(*source_, num_neighbors_, num_threads_)`
4. **构建 KdTree**（OpenMP 并行）
5. **配置 Registration**：`reduction.num_threads=4`, `rejector.max_dist_sq=max_dist_sq_`, `optimizer.max_iterations=10`
6. **运行 ICP**：`register_->align(*target_, *source_, *target_tree_, previous_result_t_)` — GICP 因子 + OpenMP 并行 reduction
7. **收敛检查**：
   - 成功 → `result_t_ = previous_result_t_ = result.T_target_source`（更新 `map→odom`）
   - 失败 → WARN 日志，保留旧值
8. 清空 `accumulated_cloud_`

#### 函数：`SmallGicpRelocalizationNode::publishTransform()` (20 Hz)

**签名**：`void publishTransform()`

**算法**：
1. 若 `result_t_.matrix().isZero()` → 返回
2. 创建 `TransformStamped`：`header.frame_id = map_frame_`，`child_frame_id = odom_frame_`
3. 从 `result_t_` 提取平移+旋转（四元数）
4. `tf_broadcaster_->sendTransform(transform_stamped)` — 广播 `map→odom` TF

#### 函数：`SmallGicpRelocalizationNode::initialPoseCallback(msg)`

**算法**：
1. 将 `/initialpose` 转换为 `map_to_robot_base` Isometry3d
2. TF 查找 `robot_base_frame_ → current_scan_frame_id_`（使用 `tf2::TimePointZero`）
3. 计算 `map_to_odom = map_to_robot_base * robot_base_to_odom`
4. `previous_result_t_ = result_t_ = map_to_odom`（重置 ICP 初值）

---

### 3.9 sensor_scan_generation — 传感器同步

#### 函数：`SensorScanGenerationNode::laserCloudAndOdometryHandler(...)` (151 行)

**签名**：`void laserCloudAndOdometryHandler(const Odometry::ConstSharedPtr& odometry_msg, const PointCloud2::ConstSharedPtr& pcd_msg)`

**被调用方式**：`message_filters::Synchronizer<ApproximateTime>`（队列 100），同步 `lidar_odometry` + `registered_scan`

**算法**：
1. 从里程计消息解析 `tf_odom_to_lidar`
2. TF 查找 `lidar_frame_ → robot_base_frame_`（缓存到 `tf_lidar_to_robot_base_`）
3. TF 查找 `lidar_frame_ → base_frame_`（chassis）
4. 计算 `tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis`
5. 计算 `tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_`
6. **广播 TF**：`odom → base_frame_`
7. **发布 odometry**：含有限差分速度计算（若 dt>0）
8. **变换点云**：`pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out)` → 发布 `/sensor_scan`

---

### 3.10 nav2_loopback_sim — 无硬件仿真闭环

#### 类：`LoopbackSimulator(Node)` (loopback_simulator.py:422)

**15 个 ROS 参数**：
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_duration` | 0.01s | 速度积分步长 |
| `base_frame_id` | `base_footprint` | 机器人基座 frame |
| `map_frame_id` | `map` | 地图 frame |
| `odom_frame_id` | `odom` | 里程计 frame |
| `scan_frame_id` | `base_scan` | 激光 frame |
| `scan_publish_dur` | 0.1s | 激光发布周期 |
| `scan_range_min/max` | 0.05/30.0 | 激光距离范围 |
| `scan_angle_increment` | 0.0261rad | 激光角分辨率 |

#### 函数：`LoopbackSimulator::timerCallback()` — cmd_vel→odom 积分

**签名**：`def timerCallback(self)`

**算法**：
1. 无 cmd_vel 或 cmd_vel 超过 1s → 清空速度，仅重发 TF
2. 有效 cmd_vel：
   - `dx = vx * dt`, `dy = vy * dt`, `dth = vyaw * dt`
   - 在机器人局部坐标系积分：`x += dx*cos(yaw) - dy*sin(yaw)`, `y += dx*sin(yaw) + dy*cos(yaw)`
   - `addYawToQuat(quat, dth)`
3. 发布 `odom→base_footprint` TF + Odometry 消息

#### 函数：`LoopbackSimulator::getLaserScan()` — 射线追踪假激光

**签名**：`def getLaserScan(self, base_to_laser, map_data)`

**算法**：
1. 计算激光在 map 帧的位姿 `(x0, y0, theta)`
2. 对约 240 条射线（`scan_angle_increment` 步进）：
   - 计算射线终点 `range_max` 处坐标
   - 用 `LineIterator`（步长 0.5 cell）沿射线步行
   - 检查每个 cell 的 `getMapOccupancy()` ≥ 60 即记录距离并跳出
   - 越界 → 不记录
3. 降级：无地图/无 initialpose/无激光位姿 → 全部 `inf` 或 `range_max-0.1`

#### TF 链：`map → odom → base_footprint → base_link → base_scan`
- `map→odom`：首个 `initialpose` 时设置；后续重置时重定位
- `odom→base_footprint`：由 cmd_vel 积分
- `base_footprint→base_link` + `base_link→base_scan`：静态 TF（来自 URDF + robot_state_publisher）

---

### 3.11 其他模块关键函数摘要

#### livox_ros_driver2 — LiDAR 驱动（47 文件）

**`DriverNode::PointCloudDataPollThread()`** (133 行)：3s 延迟后循环 `DistributePointCloudData()` → 等待 `exit_signal_`

**`Lddc::PublishPointcloud2()`** (544 行)：7 字段 PointCloud2（x,y,z,intensity,tag,line,timestamp），32 字节/点。支持 `multi_topic` 模式（per-IP topic）和共享模式。

**`PubHandler::OnLivoxLidarPointCloudCallback()`** (505 行)：SDK 静态回调，分离 IMU 数据（施加外参旋转）和点云数据（入 raw queue），通知 worker 线程。

#### ign_sim_pointcloud_tool — XYZ→XYZIRT 转换 (86 行)

**`PointCloudConverter::lidarHandle(msg)`**：
1. 遍历点，计算 `vertical_angle = atan2(z, hypot(x,y)) * 180/pi`
2. `ring = (vertical_angle + ang_bottom_) / ang_res_y_`
3. `time = (point_id % horizon_scan_) * 0.1 / horizon_scan_`
4. 发布自定义 `PointXYZIRT` 格式

#### pcd2pgm — 3D PCD→2D 栅格地图 (176 行)

**管线**：`loadPCDFile → applyTransform → passThroughFilter(Z) → radiusOutlierFilter → setMapTopicMsg`

**`setMapTopicMsg()`**：计算 XY 包围盒 → 栅格化（`ceil(width/res) × ceil(height/res)`）→ 点投影到 cell 设为 100（占据）→ 其余为 0

#### pointcloud_to_laserscan — 3D→2D 投影 (244 行)

**`PointCloudToLaserScanNode::cloudCallback(msg)`**：
1. 计算 `ranges_size = ceil((angle_max-angle_min) / angle_increment)`
2. 初始化：`use_inf_` → 全部 infinity，否则 `range_max + inf_epsilon_`
3. 对每点：**Z 过滤** → **强度过滤** → **距离过滤** → **角度过滤** → **分箱**（取最近点）
4. 发布 LaserScan

#### teleop_twist_joy — Xbox 手柄遥操作 (200 行)

**`joyCallback(msg)`**：
1. R1 按下（turbo_button）→ `sendCmdVelMsg(msg, "turbo")`
2. L1 按下（enable_button）或不需要 enable → `sendCmdVelMsg(msg, "normal")`
3. 两者都释放 → `sendZeroCommand()`（**死人开关**：只发一次零速度）
4. `inverted_reverse` 标志：倒车时翻转角速度方向

---

## 4. 全局变量 & 常量索引

### point_lio 全局变量

| 名称 | 类型 | 声明位置 | 用途 |
|------|------|----------|------|
| `kf_input` | `esekf<state_input>` | Estimator.cpp | 24 维输入模型 EKF |
| `kf_output` | `esekf<state_output>` | Estimator.cpp | 30 维输出模型 EKF |
| `ivox_` | `shared_ptr<IVoxType>` | Estimator.cpp | 增量体素空间地图 |
| `normvec` | `PointCloudXYZI::Ptr` | Estimator.cpp | 每点平面法向量存储 |
| `feats_down_body/world` | `PointCloudXYZI::Ptr` | Estimator.cpp | 降采样特征点云 |
| `time_seq` | `vector<int>` | Estimator.cpp | 时间压缩组大小 |
| `point_selected_surf[100000]` | `bool[]` | Estimator.cpp | 表面点有效性标记 |
| `effct_feat_num` | `int` | Estimator.cpp | 当前更新有效特征数 |
| `Lidar_T_wrt_IMU/R` | `V3D/M3D` | Estimator.cpp | LiDAR-IMU 外参 |
| `G_m_s2` | `double` (9.81) | Estimator.cpp | 重力大小 |

### terrain_analysis 全局变量

| 名称 | 类型 | 声明位置 | 用途 |
|------|------|----------|------|
| `terrainVoxelCloud[441]` | `PointCloud[]` | terrainAnalysis.cpp | 441 个 1m³ 粗体素 |
| `terrainVoxelUpdateNum[441]` | `int[]` | terrainAnalysis.cpp | 体素更新计数 |
| `planarVoxelElev[2601]` | `float[]` | terrainAnalysis.cpp | 细栅格地面高度 |
| `planarVoxelEdge[2601]` | `int[]` | terrainAnalysis.cpp | 边缘标记 |
| `planarVoxelDyObs[2601]` | `int[]` | terrainAnalysis.cpp | 动态障碍计数 |
| `noDataInited` | `int` (0) | terrainAnalysis.cpp | 无数据初始化状态 |

---

## 5. 类型/接口/枚举索引

### 自定义 ROS 2 消息

| 名称 | 类别 | 定义位置 | 字段 | 引用者 |
|------|------|----------|------|--------|
| `Gimbal` | msg | `interfaces/robot_interfaces/msg/Gimbal.msg` | pitch, yaw, pitch_range[2], yaw_range[2] | 云台控制 |
| `GimbalCmd` | msg | `interfaces/robot_interfaces/msg/GimbalCmd.msg` | header, control_type, position, velocity | 云台指令 |
| `Models` | msg | `interfaces/robot_interfaces/msg/Models.msg` | models[5] | 机器人标识 |
| `RobotStateInfo` | msg | `interfaces/robot_interfaces/msg/RobotStateInfo.msg` | header, robot_models | 状态聚合 |
| `CustomMsg` | msg | `livox_ros_driver2/msg/CustomMsg.msg` | header, timebase, point_num, lidar_id, points[] | Livox 驱动 |
| `CustomPoint` | msg | `livox_ros_driver2/msg/CustomPoint.msg` | offset_time, x, y, z, reflectivity, tag, line | Livox 单点 |
| `LocalSensorExternalTrigger` | msg | `point_lio/msg/LocalSensorExternalTrigger.msg` | header, trigger_id, event_id, timestamp_host | 外部触发 |

### 关键类

| 名称 | 类别 | 继承自 | 定义位置 |
|------|------|--------|----------|
| `OmniPidPursuitController` | Nav2 Controller 插件 | `nav2_core::Controller` | `omni_pid_pursuit_controller.hpp` |
| `PID` | 独立类 | 无 | `pid.hpp` |
| `SmallGicpRelocalizationNode` | ROS 2 组件节点 | `rclcpp::Node` | `small_gicp_relocalization.hpp` |
| `LoopbackSimulator` | ROS 2 Python 节点 | `rclcpp.node.Node` | `loopback_simulator.py` |
| `PointCloudConverter` | ROS 2 组件节点 | `rclcpp::Node` | `point_cloud_converter.hpp` |
| `IntensityVoxelLayer` | Nav2 代价地图层 | `nav2_costmap_2d::ObstacleLayer` | `intensity_voxel_layer.hpp` |
| `BackUpFreeSpace` | Nav2 行为插件 | `nav2_behaviors::DriveOnHeading` | `back_up_free_space.hpp` |
| `DriverNode` | ROS 2 组件节点 | `rclcpp::Node` | `driver_node.h` |
| `Lddc` | LiDAR 数据分发器 | 无 | `lddc.h` |
| `LdsLidar` | LiDAR 设备管理（单例） | `Lds` | `lds_lidar.h` |
| `IVox<3>` | 增量体素空间索引 | template | `ivox3d.h` |
| `LoamInterfaceNode` | 帧适配器 | `rclcpp::Node` | `loam_interface.hpp` |
| `SensorScanGenerationNode` | 传感器同步 | `rclcpp::Node` | `sensor_scan_generation.hpp` |

### IKFoM 状态流形

| 名称 | 维度 | 组成 |
|------|------|------|
| `state_input` | 24 | pos(3), rot(SO3), offset_R_L_I(SO3), offset_T_L_I(3), vel(3), bg(3), ba(3), gravity(3) |
| `state_output` | 30 | 上述 + omg(3), acc(3) |

### LiDAR 类型枚举 (point_lio)

| 值 | 名称 | 含义 |
|----|------|------|
| 1 | `AVIA` | Livox Avia/Horizon/Mid360 |
| 2 | `VELO16` | Velodyne VLP-16/32 |
| 3 | `OUST64` | Ouster OS1-64 |
| 4 | `HESAIxt32` | Hesai PandarXT-32 |

---

## 6. 依赖关系图谱

### 6.1 第三方依赖清单（核心）

| 依赖 | 用途 | 包 |
|------|------|-----|
| ROS 2 Humble (`rclcpp`, `rclpy`) | ROS 客户端库 | 全部 |
| Nav2 (`navigation2`, `nav2_core`, `nav2_costmap_2d`) | 导航栈 | nav_bringup, nav2_plugins, omni_pid |
| BehaviorTree.CPP 4.x | 行为树引擎 | nav2_plugins |
| PCL (Point Cloud Library) | 点云处理 | point_lio, terrain_*, pcd2pgm, loam_interface |
| IKFoM (Iterated Kalman Filters on Manifolds) | 流形 EKF | point_lio |
| small_gicp | GICP 配准 | small_gicp_relocalization |
| Livox SDK2 | LiDAR 硬件通信 | livox_ros_driver2 |
| SlamToolbox | 2D SLAM | nav_bringup (slam_launch.py) |
| Eigen3 | 线性代数 | point_lio, small_gicp |
| OpenMP | 并行计算 | point_lio, small_gicp |
| TF2 (`tf2_ros`, `tf2_geometry_msgs`) | 坐标变换 | 几乎全部 |
| RapidJSON | JSON 解析 | livox_ros_driver2 |
| rosbag2 | 数据录制 | rosbag2_composable_recorder |
| lasr_geometry | 激光投影 | pointcloud_to_laserscan |

### 6.2 内部模块依赖矩阵

```
                     interfaces  nav_bringup  nav2_plugins  point_lio  loam_if  sensor_scan  terrain  terrain_ext  omni_pid  small_gicp  pcl2scan  teleop  loopback  livox  pcd2pgm
nav_bringup              -           -             ✓            ✓         ✓          ✓           ✓         ✓           ✓          ✓           ✓        ✓        ✓       ✓       -
nav2_plugins             -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
point_lio                -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
loam_interface           -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
sensor_scan_generation   -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
terrain_analysis         -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
terrain_analysis_ext     -           -             -            -         -          -           ✓         -           -          -           -        -        -       -       -
omni_pid_pursuit         -           -             ✓            -         -          -           -         -           -          -           -        -        -       -       -
small_gicp_reloc         -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
pointcloud_to_laserscan  -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
teleop_twist_joy         -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
nav2_loopback_sim        -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
livox_ros_driver2        -           -             -            ✓         -          -           -         -           -          -           -        -        -       -       -
pcd2pgm                  -           -             -            -         -          -           -         -           -          -           -        -        -       -       -
```

### 6.3 关键调用链

**完整导航管线（实机模式）**：
```
livox_ros_driver2::DriverNode
  → /livox/lidar (CustomMsg/PointCloud2)
  → point_lio::laserMapping::main()
    → sync_packages() → Process() → ESIKF update → MapIncremental()
    → /aft_mapped_to_init (Odometry) + /cloud_registered (PointCloud2)
  → loam_interface::LoamInterfaceNode::odometryCallback()
    → frame_id: lidar_odom → odom (TF 变换)
    → /lidar_odometry + /registered_scan
  → sensor_scan_generation::SensorScanGenerationNode::laserCloudAndOdometryHandler()
    → ApproximateTime 同步 → TF broadcast odom→base_footprint
    → /odometry + /sensor_scan
  → terrain_analysis::terrainAnalysis::main()
    → 栅格化 + 分位数地面估计 + 高程计算
    → /terrain_map (PointCloud2, intensity=elevation)
  → terrain_analysis_ext::terrainAnalysisExt::main()
    → BFS 连通性 + 合并 /terrain_map
    → /terrain_map_ext
  → Nav2 costmaps (local: /terrain_map + global: /terrain_map_ext)
  → Nav2 Planner (SmacPlannerHybrid) → /plan (Path)
  → Nav2 Controller (OmniPidPursuitController) → /cmd_vel_nav2_result
  → velocity_smoother → /cmd_vel (Twist → 底盘)
```

**仿真闭环模式**：
```
fake_decision_sim_inputs → /initialpose
loopback_simulator → /odom + /scan + /clock (仿真时钟)
Nav2 costmaps (静态地图 + /scan)
Nav2 Planner → /plan
Nav2 Controller → /cmd_vel
loopback_simulator ← /cmd_vel (闭环回灌)
```

### 6.4 循环依赖标记

✅ **本项目无循环依赖**。模块间的依赖是严格的 DAG（有向无环图）：
- `nav_bringup` 依赖所有其他模块（launch 编排层），但无其他模块依赖它
- 核心算法模块（point_lio, terrain_*, omni_pid）之间零互相依赖
- 接口包 (`robot_interfaces`) 无任何内部依赖

---

## 7. 数据流

### 7.1 全局状态管理

- **point_lio 状态**：ESIKF 滤波器持有全部定位状态（位姿、速度、IMU bias、外参、重力），内部 `iVox` 地图使用 LRU 淘汰策略
- **terrain_analysis 状态**：441+2601 网格的体素和细栅格数组在 `main()` 函数作用域内，随车辆移动整体滚动
- **nav2_plugins BT 状态**：通过 BehaviorTree.CPP 黑板（`{decision_*}` 变量）共享
- **small_gicp 状态**：`result_t_` + `previous_result_t_` 的 `map→odom` 变换，跨 ICP 迭代保持

### 7.2 数据在模块间的流转路径

```
LiDAR 原始数据 (CustomMsg)
  → [时间戳归一化] Preprocess::avia_handler
  → [运动补偿] ESIKF 逐点状态更新
  → [坐标变换] loam_interface (lidar_odom → odom)
  → [时间同步] sensor_scan_generation (ApproximateTime)
  → [空间栅格化] terrain_analysis (1m 粗 + 0.2m 细)
  → [连通滤波] terrain_analysis_ext (BFS)
  → [代价注入] Nav2 local_costmap + global_costmap
  → [路径规划] SmacPlannerHybrid (Hybrid-A*)
  → [路径跟踪] OmniPidPursuitController (预瞄+PID)
  → [速度平滑] velocity_smoother
  → [执行] cmd_vel → 底盘
```

### 7.3 持久化方式 & 时机

| 数据类型 | 持久化方式 | 时机 |
|----------|-----------|------|
| 地图 (OccupancyGrid) | YAML + PGM 文件 | map_server 启动时加载 |
| PCD 点云地图 | .pcd 文件（ASCII/二进制） | pcd2pgm: 启动时加载；point_lio: pcd_save_en 时关停保存 |
| SLAM 地图 | slam_toolbox 序列化 | 建图完成后手动保存 |
| rosbag 录制 | SQLite3 数据库 | rosbag2_composable_recorder 按需录制 |
| 参数配置 | YAML 文件 | 启动时通过 launch 文件加载 |
| 运行时位置日志 | Log/mat_out.txt, Log/imu_pbp.txt | point_lio 运行时追加写入 |

### 7.4 异步/事件驱动模型

- **point_lio**：传感器回调（`imu_cbk`, `standard_pcl_cbk`）+ 主循环线程（500 Hz）+ 条件变量同步
- **sensor_scan_generation**：`message_filters::ApproximateTime` 同步器（队列 100）
- **Nav2**：ROS 2 action 服务器异步处理 `NavigateToPose`/`NavigateThroughPoses`
- **nav2_loopback_sim**：`create_timer` 定时器驱动（update_duration + scan_publish_dur）
- **teleop_twist_joy**：事件驱动（`joyCallback`）
- **livox_ros_driver2**：SDK 硬件回调 + 2 个 poll 线程（点云 + IMU）+ worker 线程 + 信号量同步

---

## 8. 配置 & 环境变量

### 8.1 仿真/实机参数对比（关键差异）

| 参数组 | 参数 | 实机值 | 仿真值 | 差异原因 |
|--------|------|--------|--------|----------|
| point_lio | `lid_topic` | `livox/lidar` | `velodyne_points` | 仿真使用通用 LiDAR |
| point_lio | `lidar_type` | 1 (Livox) | 2 (Velodyne) | 不同传感器 |
| point_lio | `filter_size_surf` | 0.05 | 0.2 | 仿真点云更稀疏 |
| point_lio | `acc_norm` | 1.0 (g) | 9.81 (m/s²) | IMU 单位不同 |
| point_lio | `gravity` | `[-0.145,-9.168,-3.407]` | `[4.774,-0.078,-8.501]` | 安装姿态不同 |
| point_lio | `satu_acc` | 6.0 | 30.0 | 仿真 IMU 无饱和 |
| controller | `v_linear_min/max` | -4.5/4.5 | -2.5/2.5 | 仿真更保守 |
| controller | `curvature_min/max` | 2.5/5.0 | 0.4/0.7 | 仿真半径更小 |
| local_costmap | `robot_radius` | 0.3 | 0.2 | 仿真模型不同 |
| global_costmap | `inflation_radius` | 0.4 | 0.7 | 仿真更保守 |
| velocity_smoother | `max_velocity` | `[3.5,3.5,5.0]` | `[2.5,2.5,3.0]` | 仿真速度限制 |
| terrain | `scanVoxelSize` | 0.02 | 0.05 | 实机需要更高精度 |
| small_gicp | `global_leaf_size` | 0.15 | 0.1 | 仿真地图精度不同 |

### 8.2 核心环境变量

| 变量 | 默认值 | 用途 | 影响范围 |
|------|--------|------|----------|
| `RCUTILS_LOGGING_BUFFERED_STREAM` | 1 | 缓冲日志输出 | nav_bringup launch |
| `RCUTILS_COLORIZED_OUTPUT` | 1 | 彩色终端日志 | nav_bringup launch |
| `use_sim_time` | false/true | 使用仿真时钟 | 全部节点 |
| `ROOT_DIR` | 编译时确定 | point_lio 运行时文件路径基础 | point_lio |
| `IVOX_NODE_TYPE_PHC` | 未定义 | 切换 iVox 节点类型（PHC/线性） | point_lio |
| `BUILDING_ROS2` | 编译时定义 | ROS2 构建标识 | livox_ros_driver2 |
| `MP_EN` / `MP_PROC_NUM` | 编译时检测 | OpenMP 并行开关/核数 | point_lio |

---

> **文档版本**：2026-07-04
> **生成工具**：Claude Code `/code-doc` skill
> **精读覆盖**：291 个文件全部逐行精读，5 个并行 Agent 分治完成
