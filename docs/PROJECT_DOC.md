# Basic Navigation Framework — 分层索引

> 生成时间: 2026-07-06 | 基线提交: `e61e9d6`
> 语言: C++ (85%) + Python (12%) + 其他 (3%) | 总文件数: 123 (不含外部依赖)
> 分层分布: A: 12 / B: 28 / C: 83 | 源码行数: ~8,500 (非空非注释)

---

> 本文档是分层导航索引，不替代源代码——源代码始终是唯一的真相来源。每个条目携带同步提交哈希，版本过期一目了然。

---

## Layer 1: 架构概览

### 1.1 摘要

- **技术栈**: ROS2 Humble / BehaviorTree.CPP / Nav2 / small_gicp / PCL / tf2
- **构建工具**: ament_cmake (C++) + setuptools (Python)
- **项目定位**: 面向全向移动机器人的自主导航框架，提供 Nav2 自定义插件（行为树节点、控制器、重定位）与仿真/实车部署的一体化方案。
- **作者**: Lihan Chen (lihanchen2004@163.com)

### 1.2 目录布局

```
Basic_Navigation_Framework/
├── src/
│   ├── interfaces/
│   │   └── robot_interfaces/          — 自定义 ROS2 消息定义
│   ├── navigation/
│   │   ├── nav_bringup/               — 🚀 总启动入口，编排所有节点
│   │   ├── nav2_plugins/              — 🧩 自定义行为树节点与代价地图插件
│   │   ├── omni_pid_pursuit_controller/ — 🎮 全向 PID 追踪控制器
│   │   ├── small_gicp_relocalization/ — 📍 基于 GICP 的全局重定位
│   │   ├── loam_interface/            — 🔗 LOAM 里程计桥接接口
│   │   ├── sensor_scan_generation/    — 📡 传感器扫描融合生成
│   │   ├── teleop_twist_joy/          — 🕹️ 手柄遥操作
│   │   ├── terrain_analysis/          — ⛰️ 地形分析
│   │   ├── terrain_analysis_ext/      — ⛰️ 地形分析扩展
│   │   ├── ign_sim_pointcloud_tool/   — ☁️ 点云格式转换工具
│   │   ├── pointcloud_to_laserscan/   — ☁️→📏 PointCloud 转 LaserScan
│   │   ├── livox_ros_driver2/         — [外部] Livox 激光雷达驱动
│   │   └── point_lio/                 — [外部] 激光惯性里程计
│   ├── simulation/
│   │   └── nav2_loopback_sim/         — 🔄 无物理回环仿真器
│   └── tools/
│       ├── pcd2pgm/                   — 🗺️ PCD 点云转栅格地图
│       └── rosbag2_composable_recorder/ — [外部] 可组合录制工具
├── src/dependencies/                  — [外部] 上游依赖（BT ROS2、JointStatePublisher、SDFormat Tools）
└── docs/                              — 本文档目录
```

### 1.3 模块依赖拓扑

```
                    ┌──────────────────────┐
                    │     nav_bringup      │  ← 总启动入口
                    └──────┬───────────────┘
           ┌───────────────┼───────────────────────┐
           │               │                       │
           ▼               ▼                       ▼
   ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐
   │  loam_interface│ │sensor_scan   │ │ terrain_analysis      │
   │  (里程计桥接)  │ │_generation   │ │ + terrain_analysis_ext│
   └──────┬───────┘ └──────┬───────┘ └──────────────────────┘
          │                │
          ▼                ▼
   ┌────────────────────────────────────────────────────────┐
   │                    Nav2 导航栈                           │
   │  ┌──────────┐ ┌──────────┐ ┌──────────────┐            │
   │  │planner   │→│controller│→│velocity      │            │
   │  │_server   │ │_server   │ │_smoother     │            │
   │  └──────────┘ └────┬─────┘ └──────────────┘            │
   │                    │                                     │
   │          ┌─────────▼──────────┐                          │
   │          │omni_pid_pursuit    │  ← 自定义控制器          │
   │          │_controller         │                          │
   │          └────────────────────┘                          │
   │                                                          │
   │  ┌──────────────────────┐  ┌──────────────────────┐     │
   │  │ bt_navigator         │  │ behavior_server      │     │
   │  │ + nav2_plugins (BT)  │  │ + nav2_plugins (层)  │     │
   │  └──────────────────────┘  └──────────────────────┘     │
   └────────────────────────────────────────────────────────┘
          │
          ▼
   ┌──────────────────────┐
   │ small_gicp           │  ← 全局重定位（按需触发）
   │ _relocalization      │
   └──────────────────────┘
          │
          ▼
   ┌──────────┐
   │ cmd_vel  │  → 机器人底盘
   └──────────┘

   teleop_twist_joy ──────> cmd_vel (遥操作直驱，绕过导航栈)
   nav2_loopback_sim ←─── cmd_vel (仿真模式替换真实底盘)
```

### 1.4 核心数据流

**自主导航流程:**
1. **传感器输入**: livox_ros_driver2 (LiDAR点云) + point_lio (里程计估计)
2. **接口桥接**: loam_interface → 里程计坐标系转换 → sensor_scan_generation → 点云+里程计同步融合 → 底盘里程计发布
3. **代价地图更新**: terrain_analysis / terrain_analysis_ext → 地形分析 → costmap 层
4. **全局规划**: planner_server (Nav2) → 生成全局路径
5. **行为决策**: bt_navigator (Nav2) + nav2_plugins 自定义BT节点 → 巡逻/定点/多航点决策
6. **局部控制**: controller_server → omni_pid_pursuit_controller → PID追踪 + 曲率限速
7. **安全平滑**: velocity_smoother → 速度平滑
8. **执行**: cmd_vel → 机器人底盘

**全局重定位流程:**
1. 载入先验全局地图 PCD → small_gicp_relocalization
2. 订阅实时点云 + 初始位姿估计 → GICP 配准
3. 发布 map→odom 校正变换

**回环仿真流程:**
1. nav2_loopback_sim 订阅 cmd_vel
2. 无物理积分 → 发布 odometry + TF
3. 基于静态地图生成虚拟 LaserScan → 替代真实传感器

### 1.5 入口点

| 场景 | 文件 | 说明 |
|---|---|---|
| 实车自主导航 | `src/navigation/nav_bringup/launch/navigation_launch.py` | 完整导航栈启动 |
| 仿真自主导航 | `src/navigation/nav_bringup/launch/simulation.launch.py` | 仿真环境导航 |
| SLAM 建图 | `src/navigation/nav_bringup/launch/slam_launch.py` | SLAM 模式 |
| 纯定位 | `src/navigation/nav_bringup/launch/localization_launch.py` | 已知地图定位 |
| 手柄遥控 | `src/navigation/nav_bringup/launch/joy_teleop_launch.py` | 手柄直驱 |
| 回环仿真 | `src/simulation/nav2_loopback_sim/launch/loopback_simulation.launch.py` | 无物理仿真 |
| 点云转地图 | `src/tools/pcd2pgm/launch/pcd2pgm_launch.py` | PCD→PGM 工具 |

### 1.6 模块目录

| 模块 | 类型 | 文件数 | 描述 |
|---|---|---|---|
| [nav_bringup](modules/nav_bringup.md) | 自定义 | 12 | 总启动编排 |
| [nav2_plugins](modules/nav2_plugins.md) | 自定义 | 35 | 行为树插件 + 代价地图层 |
| [omni_pid_pursuit_controller](modules/omni_pid_pursuit_controller.md) | 自定义 | 8 | 全向 PID 控制器 |
| [small_gicp_relocalization](modules/small_gicp_relocalization.md) | 自定义 | 6 | GICP 全局重定位 |
| [loam_interface](modules/loam_interface.md) | 自定义 | 5 | LOAM 桥接 |
| [sensor_scan_generation](modules/sensor_scan_generation.md) | 自定义 | 5 | 传感器融合 |
| [teleop_twist_joy](modules/teleop_twist_joy.md) | 自定义 | 7 | 手柄遥控 |
| [nav2_loopback_sim](modules/nav2_loopback_sim.md) | 自定义 | 20 | 回环仿真 |
| [tools](modules/tools.md) | 自定义 | 14 | 工具集(pcd2pgm) |
| [interfaces](modules/interfaces.md) | 自定义 | 6 | ROS2 消息定义 |
| [dependencies](modules/dependencies.md) | 外部 | 120+ | 第三方依赖汇总 |

---

## 阅读路径

- **调试 Bug**: 先看 [风险地图](risk-map.md) → 再查 [符号索引](symbol-index.md) → 最后在模块文档中追溯调用链
- **修改功能**: 先看 Layer 1.4 核心数据流 → 确认影响范围 → 再查模块文档中的"被依赖方"
- **新人上手**: 通读 Layer 1 → 再读 nav_bringup、nav2_plugins、omni_pid_pursuit_controller 三个核心模块
- **仿真测试**: 先看 nav2_loopback_sim → 再看 simulation.launch.py → 最后 nav2_params.simulation.yaml
