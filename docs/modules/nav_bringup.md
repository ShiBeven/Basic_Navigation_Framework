# 模块: nav_bringup

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 2 / B: 4 / C: 6

## 职责
总启动编排模块——负责将所有导航相关节点（自定义 + Nav2 标准 + 第三方）组合为完整的 launch 描述，支持独立进程模式和 ComposableNode 容器模式两种部署方式。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `launch/navigation_launch.py` | A | 主导航启动文件，编排所有导航节点 |
| `launch/simulation.launch.py` | A | 仿真环境启动入口 |
| `config/nav2_params.simulation.yaml` | B | 仿真环境 Nav2 参数配置 |
| `config/nav2_params.reality.yaml` | B | 实车环境 Nav2 参数配置 |
| `launch/slam_launch.py` | B | SLAM 建图启动 |
| `launch/localization_launch.py` | B | 纯定位模式启动 |
| `launch/joy_teleop_launch.py` | C | 手柄遥操作启动 |
| `launch/rviz_launch.py` | C | RViz 可视化启动 |
| `behavior_trees/*.xml` | C | 行为树 XML 定义 |
| `rviz/*.rviz` | C | RViz 配置文件 |
| `map/README.txt` | C | 地图目录说明 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | B | 包元数据（含所有 exec_depend） |

## 启动参数

`navigation_launch.py` 暴露的核心参数:

| 参数 | 默认值 | 说明 |
|---|---|---|
| `namespace` | `""` | 顶层命名空间 |
| `use_sim_time` | `"false"` | 是否使用仿真时钟 |
| `params_file` | `nav2_params.simulation.yaml` | Nav2 参数文件路径 |
| `autostart` | `"true"` | 是否自动激活生命周期节点 |
| `use_composition` | `"False"` | 是否使用 ComposableNode 容器 |
| `use_respawn` | `"False"` | 节点崩溃后是否自动重启 |
| `log_level` | `"info"` | 日志级别 |
| `use_sensor_scan` | `"true"` | 是否启动 sensor_scan_generation |

## 节点编排图

```
navigation_launch.py
├── terrain_analysis           (独立进程)
├── terrain_analysis_ext       (独立进程)
└── [GroupAction: 非组合模式]
    ├── loam_interface         — LOAM 里程计桥接
    ├── sensor_scan_generation — 传感器扫描融合 (条件: use_sensor_scan)
    ├── controller_server      — 路径跟踪控制器 (remap: cmd_vel→cmd_vel_nav2_result)
    ├── smoother_server        — 路径平滑器
    ├── planner_server         — 全局规划器
    ├── behavior_server        — 行为服务器 (恢复行为)
    ├── bt_navigator           — 行为树导航器 (remap: cmd_vel→cmd_vel_nav2_result)
    ├── waypoint_follower      — 航点跟随器
    ├── velocity_smoother      — 速度平滑器 (remap: cmd_vel_nav2_result→cmd_vel)
    └── lifecycle_manager      — 生命周期管理器
```

## 调用关系

**依赖**: nav2_plugins, teleop_twist_joy, omni_pid_pursuit_controller, small_gicp_relocalization, loam_interface, sensor_scan_generation, terrain_analysis, terrain_analysis_ext, livox_ros_driver2, point_lio, pointcloud_to_laserscan, slam_toolbox, navigation2

**被依赖方**: 无（顶层入口模块）

## 关键类型

| 类型 | 位置 | 用途 |
|---|---|---|
| 无自定义类型 | — | 本模块仅包含 launch 文件和配置文件，无 C++/Python 代码 |

## 注意事项

1. **cmd_vel 话题的重映射链**: controller_server → `cmd_vel_nav2_result` → velocity_smoother → `cmd_vel`。此重映射链确保速度平滑器能拦截原始控制指令。
2. **两种部署模式**: 独立进程模式（默认，`use_composition=False`）和 ComposableNode 容器模式（`use_composition=True`），后者将节点加载到同一进程以减少通信开销。
3. **terrain_analysis 始终启动**: 无论选择哪种部署模式，terrain_analysis 和 terrain_analysis_ext 始终作为独立 Node 启动，不参与组合。
