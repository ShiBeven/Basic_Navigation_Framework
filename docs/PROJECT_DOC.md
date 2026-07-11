# Basic Navigation Framework — 分层理解层文档

> 生成: 2026-07-11 | 基线: 非 git 仓库, 版本标记用日期
> 语言: C++ (~70%, h/hpp/cpp) + Python (~20%, launch/节点) + 配置 (~10%, yaml/xml)
> 包总数: 17 个 ROS2 package | 项目自研源码行数: ~5,900 (非空非注释) | 全量含第三方: 32,967
> 分析范围: 聚焦项目自研包; `livox_ros_driver2` (16.9k 行) 与 `point_lio` (8.7k 行) 为上游 vendored 第三方, 仅记职责与接口, 不逐函数深读。
> 本文档承载理解代码所需的语义(契约/数据流/不变量/意图)。落地编辑时回源码确认行号。
> 每个模块段落带同步日期标记, 过时立即可见。
> 2026-07-11 已做一轮"复用性整改"(修确定性 bug + 帧/参数硬编码), 详见 Layer 3 风险图状态列与各契约的"(2026-07-11 修复)"标注。

---

## Layer 1: 架构总览

### 1.1 概要
- 技术栈: ROS2 (Humble 系) + Nav2 导航栈 + PCL 点云处理 + Eigen + BehaviorTree.CPP (BT.ROS2) + small_gicp / Point-LIO 激光里程计。
- 构建: `ament_cmake` (C++ 包) + `ament_python` (部分仿真/GUI 包)。
- 一句话用途: 面向带 Livox 固态雷达的全向移动机器人的一套完整 3D 激光导航框架 —— 从雷达驱动、LIO 里程计、重定位、地形可通行性分析, 到 Nav2 规划/控制与巡逻决策行为树, 支持仿真与实车两套参数。

### 1.2 目录布局
```
src/
├── interfaces/robot_interfaces/        自定义消息 (Gimbal/GimbalCmd/Models/RobotStateInfo)
├── navigation/
│   ├── livox_ros_driver2/    [第三方] Livox MID360/HAP 雷达驱动
│   ├── point_lio/            [第三方] Point-LIO 紧耦合激光惯性里程计 (含 IKFoM)
│   ├── loam_interface/       LIO 输出适配: lidar_odom 系 → odom 系 (自研)
│   ├── sensor_scan_generation/  生成 base/robot_base 系点云与底盘里程计 + TF (自研)
│   ├── small_gicp_relocalization/  基于先验 PCD 地图的 GICP 重定位, 发 map→odom TF (自研)
│   ├── terrain_analysis/     近场地形可通行性分析 → terrain_map (自研/CMU 移植)
│   ├── terrain_analysis_ext/ 大尺度地形分析 + 连通性去天花板 → terrain_map_ext (自研/移植)
│   ├── nav2_plugins/         自定义 Nav2 插件: BT 决策节点/costmap 层/恢复行为 (自研)
│   ├── omni_pid_pursuit_controller/  全向 PID 纯追踪控制器插件 (自研)
│   ├── nav_bringup/          launch 与参数编排枢纽 (自研)
│   ├── pointcloud_to_laserscan/ [第三方] 点云转激光扫描
│   ├── ign_sim_pointcloud_tool/ 仿真点云格式转换 (加 ring/time 字段)
│   ├── teleop_twist_joy/     手柄遥控 → Twist (自研改版)
│   └── ign_sim_pointcloud_tool / livox ...
├── simulation/nav2_loopback_sim/  [第三方] Nav2 回环仿真 (免物理引擎里程计)
└── tools/
    ├── pcd2pgm/              PCD 点云地图 → 2D 占据栅格 PGM (自研)
    └── rosbag2_composable_recorder/  可组合 rosbag 录制节点 (第三方改版)
```

### 1.3 模块依赖拓扑
```
[硬件/仿真]
  livox_ros_driver2 ──┐
  ign_sim_pointcloud_tool ─┤ (原始点云 livox/lidar)
                          ▼
                    point_lio ──(cloud_registered + aft_mapped_to_init, lidar_odom 系)──┐
                                                                                        ▼
                                                                              loam_interface
                                                                (registered_scan + lidar_odometry, odom 系)
                                        ┌───────────────────────┬───────────────┴──────────────┐
                                        ▼                       ▼                              ▼
                            sensor_scan_generation      terrain_analysis            small_gicp_relocalization
                            (sensor_scan + odometry     (terrain_map)               (map→odom TF, 用先验 PCD)
                             + odom→base_footprint TF)          │
                                        │                       ▼
                                        │              terrain_analysis_ext (terrain_map_ext)
                                        ▼                       │
                              ┌─────────┴───────────────────────┘
                              ▼
                        Nav2 栈 (controller/planner/bt_navigator/behavior/waypoint/velocity_smoother)
                          ├─ costmap 层: nav2_plugins::IntensityVoxelLayer  ← 消费 terrain_map / terrain_map_ext
                          ├─ 控制器插件: OmniPidPursuitController (YAML 名与实际注册名不符, 见 Layer3 #30)
                          ├─ BT 节点: nav2_plugins::* (巡逻/决策/发布)
                          └─ 恢复行为: pb_nav2_behaviors::BackUpFreeSpace
                              ▼
                        cmd_vel (经 velocity_smoother 平滑) → 底盘

  teleop_twist_joy ─(cmd_vel, 手柄旁路)─▶ 底盘
  nav_bringup: 编排以上全部节点 (launch + nav2_params.{simulation,reality}.yaml)
```
> **循环/隐式说明:** 上图为静态数据流。Nav2 与所有 `nav2_plugins`/`omni_pid_pursuit_controller` 之间是 **pluginlib 运行时反射加载**, 无编译期依赖边 —— 真正的加载关系只在 `nav2_params.*.yaml` 与 BT XML 里以字符串出现 (见各模块"动态引用")。

### 1.4 核心数据流
1. **感知→里程计:** `livox_ros_driver2` (或仿真 `ign_sim_pointcloud_tool`) 出原始点云 → `point_lio` 紧耦合 LIO 产出 `cloud_registered` 点云与 `aft_mapped_to_init` 里程计 (均在内部 `lidar_odom` 系)。
2. **坐标系适配:** `loam_interface` 用 TF (`base_frame`→`lidar_frame`) 把 point_lio 的点云与里程计从 `lidar_odom` 系搬到真正的 `odom` 系, 发 `registered_scan` + `lidar_odometry`。
3. **扫描/TF 生成:** `sensor_scan_generation` 时间同步 `lidar_odometry`+`registered_scan`, 广播 `odom→base_footprint` TF, 发 `robot_base` 系里程计与 `sensor_scan`; 同时 `terrain_analysis(_ext)` 累积点云估计地面高程, 产 `terrain_map` / `terrain_map_ext` (每点 intensity = 相对地面高度代价)。
4. **重定位:** `small_gicp_relocalization` 用先验 PCD 地图做 GICP 配准, 以 2Hz 更新、20Hz 广播 `map→odom` TF (时间戳 +0.1s 前推)。
5. **规划→控制:** Nav2 costmap 的 `IntensityVoxelLayer` 消费 `terrain_map(_ext)` 标记障碍 → planner 出全局路径 → `OmniPidPursuitController` (FollowPath 插件) 纯追踪出全向 `cmd_vel` → `cmd_vel_nav2_result` → `velocity_smoother` 平滑 → `cmd_vel` → 底盘。
6. **决策/巡逻:** `bt_navigator` 加载 BT XML, 用 `nav2_plugins` 的巡逻/选路/发布节点驱动多点巡航与恢复。

> **cmd_vel 重映射链 (来自 navigation_launch.py, 关键不变量):** controller_server 与 bt_navigator 的 `cmd_vel` → `cmd_vel_nav2_result`; velocity_smoother 收 `cmd_vel_nav2_result`, 其 `cmd_vel_smoothed` → 最终 `cmd_vel`。改动控制链拓扑必须同步这三处 remap。

### 1.5 入口点
| 场景 | 文件 |
|---|---|
| 仿真总启动 | src/navigation/nav_bringup/launch/simulation.launch.py |
| 导航栈 (含 terrain/loam/sensor_scan + nav2) | src/navigation/nav_bringup/launch/navigation_launch.py |
| 定位 (含重定位/map_server) | src/navigation/nav_bringup/launch/localization_launch.py |
| SLAM 建图 | src/navigation/nav_bringup/launch/slam_launch.py |
| 手柄遥控 | src/navigation/nav_bringup/launch/joy_teleop_launch.py |
| RViz | src/navigation/nav_bringup/launch/rviz_launch.py |
| 仿真参数 / 实车参数 | config/nav2_params.simulation.yaml / nav2_params.reality.yaml |
| BT 行为树 | nav_bringup/behavior_trees/navigate_{to_pose,through_poses}_w_replanning_and_recovery.xml |

---

## 阅读路径
- **排查 Bug:** 先看 Layer 3 风险图定位热点 → 查附录符号索引跳到模块段 → 在 Layer 2 对应模块追调用/契约。
- **修改功能:** 先看 1.4 核心数据流与 cmd_vel 重映射链确认影响面 → 查 Layer 2 目标模块的"调用关系:被依赖" → 注意 pluginlib 插件的消费者在 yaml/BT XML 而非代码里。
- **新人上手:** 通读 Layer 1 → 看 loam_interface / sensor_scan_generation (最短, 理解坐标系流) → 再看 omni_pid_pursuit_controller 与 nav2_plugins。

## Layer 2: 模块契约

## 模块: loam_interface

> 同步: 2026-07-11 | Tier 分布: A: 1 / B: 0 / C: 2

### 职责
坐标系适配器: 把 point_lio 在内部 `lidar_odom` 系输出的点云与里程计, 变换到机器人真正的 `odom` 系, 对下游屏蔽 LIO 的内部坐标约定。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/loam_interface.cpp | A | 全部逻辑: 两个回调, 首帧初始化 TF, 变换并转发点云/里程计 |
| include/loam_interface/loam_interface.hpp | C | 节点类声明 |
| launch/loam_interface_launch.py | C | 独立启动 (通常经 navigation_launch composable 加载) |

### 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `pointCloudCallback` | `void(PointCloud2::ConstSharedPtr)` | 变换并转发点云 | 用 `tf_odom_to_lidar_odom_` 把点云从输入系变到 `odom_frame_` 后发 `registered_scan`; **未初始化 TF 前直接 return(2026-07-11 修复), 等 odometryCallback 首帧初始化后才转发** |
| `odometryCallback` | `void(Odometry::ConstSharedPtr)` | 变换并转发里程计 | 首帧 lookupTransform(`base_frame`→`lidar_frame`, 超时0.5s) 初始化 `tf_odom_to_lidar_odom_`, 失败则 WARN 并 return 等下一帧重试; 之后 `odom→lidar = base_to_lidar * lidar_odom→lidar`, 发 `lidar_odometry` (frame=`odom_frame_`, child=`lidar_frame_`) |
| 参数 | `state_estimation_topic / registered_scan_topic / odom_frame / base_frame / lidar_frame` | 话题与帧名 | 订阅话题名由参数给定 (仿真接 point_lio 的 `cloud_registered` / `aft_mapped_to_init`) |

### 调用关系
依赖: rclcpp, rclcpp_components, tf2/tf2_ros/tf2_geometry_msgs, pcl_ros (transformPointCloud), nav_msgs, sensor_msgs
被依赖: 由 navigation_launch.py 作为节点/composable `loam_interface` 加载; 输出被 sensor_scan_generation、terrain_analysis、small_gicp_relocalization 消费

### 动态引用
`RCLCPP_COMPONENTS_REGISTER_NODE` 注册为可组合节点, 运行期由容器按 plugin 名 `loam_interface::LoamInterfaceNode` 反射加载。

---

## 模块: sensor_scan_generation

> 同步: 2026-07-11 | Tier 分布: A: 1 / B: 0 / C: 2

### 职责
时间同步里程计与点云, 广播 `odom→base_footprint` TF, 并输出机器人底盘系的里程计(含数值微分速度)与 lidar 系的 `sensor_scan` 点云。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/sensor_scan_generation.cpp | A | 全部逻辑: message_filters 同步回调 + TF 广播 + 里程计发布 |
| include/sensor_scan_generation/sensor_scan_generation.hpp | C | 节点类与 SyncPolicy 声明 |
| launch/sensor_scan_generation.launch.py | C | 独立启动 |

### 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `laserCloudAndOdometryHandler` | `void(Odometry::ConstSharedPtr&, PointCloud2::ConstSharedPtr&)` | 同步回调 | 由 `message_filters::Synchronizer` (ApproximateTime, queue 100, BEST_EFFORT QoS) 触发; 广播 `odom→base_frame` TF, 发 robot_base 系 `odometry` 与 lidar 系 `sensor_scan` (把点云用 `tf_odom_to_lidar.inverse()` 变回 lidar 系) |
| `getTransform` | `tf2::Transform(target, source, time)` | 查 TF | **失败时返回单位变换 (仅 WARN), 不抛异常** —— TF 缺失会静默产出错误位姿 |
| `publishOdometry` | `void(transform, parent, child, stamp)` | 发里程计 | 速度由上一帧位姿做数值微分求得; 历史存于**成员变量** `previous_odom_*`/`has_previous_odom_`(2026-07-11 修复, 原为 static, 现支持多实例并随节点重建复位); 角速度取四元数差的轴角/dt |

### 调用关系
依赖: rclcpp, tf2/tf2_ros, pcl_ros, message_filters, nav_msgs/sensor_msgs
被依赖: navigation_launch.py 加载 (受 `use_sensor_scan` 开关); 其 `odom→base_footprint` TF 是全栈定位基础

### 动态引用
`RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)` 运行期反射加载。

---

## 模块: small_gicp_relocalization

> 同步: 2026-07-11 | Tier 分布: A: 1 / B: 0 / C: 2

### 职责
基于先验 PCD 点云地图, 用 small_gicp 做扫描-地图 GICP 配准估计 `map→odom` 变换, 提供全局重定位并周期性广播该 TF。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/small_gicp_relocalization.cpp | A | 全部逻辑: 加载/预处理先验地图, 累积扫描, 2Hz 配准, 20Hz 发 TF, 初始位姿注入 |
| include/small_gicp_relocalization/small_gicp_relocalization.hpp | C | 节点类声明 |
| launch/small_gicp_relocalization_launch.py | C | 独立启动 |

### 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `loadGlobalMap` | `void(file_name)` | 构造期加载先验 PCD | 读失败仅 ERROR 返回; 把地图从 `lidar_odom` 变到 `base`系, TF 查询最多重试 30 次(每次1s), 全失败则地图**留在原系不变换**(继续运行, 配准将偏) |
| `registeredPcdCallback` | `void(PointCloud2::SharedPtr)` | 累积扫描 | 把 `registered_scan` 累加进 `accumulated_cloud_`, 记录 stamp 与 frame_id; 不做配准 |
| `performRegistration` | `void()` (2Hz timer) | GICP 配准 | 前置: 累积点云非空; 下采样→估协方差→建KdTree→`align`(max 10 迭代, 以 `previous_result_t_` 为初值); 收敛才更新 `result_t_`, 否则 WARN 保留旧值; **每次结束清空累积云** |
| `publishTransform` | `void()` (20Hz timer) | 广播 map→odom | 时间戳 = `last_scan_time_ + 0.1s` (前推到未来避免 TF 外插失败); `result_t_` 为零则跳过 |
| `initialPoseCallback` | `void(PoseWithCovarianceStamped)` | RViz 初始位姿注入 | 从 `initialpose` 话题取, 结合 `robot_base→odom` TF 计算 `map→odom` 重置 `result_t_`/`previous_result_t_` |
| 参数 | `num_threads / num_neighbors / global_leaf_size / registered_leaf_size / max_dist_sq / *_frame / prior_pcd_file / init_pose[6]` | 配置 | `init_pose` = [x,y,z,roll,pitch,yaw]; 构造期若 size≥6 设为初值 |

### 调用关系
依赖: rclcpp/rclcpp_components, small_gicp (GICP/OMP), PCL, Eigen, tf2_eigen/tf2_ros, sensor_msgs/geometry_msgs
被依赖: localization_launch.py 加载; 广播的 `map→odom` TF 是 Nav2 全局定位基础

### 动态引用
`RCLCPP_COMPONENTS_REGISTER_NODE` 反射加载; 两个 wall_timer 与两个订阅回调在 executor 线程触发。

---

## 模块: omni_pid_pursuit_controller

> 同步: 2026-07-11 | Tier 分布: A: 2 / B: 2 / C: 2

### 职责
为全向底盘实现的 Nav2 `nav2_core::Controller` 插件, 采用纯追踪 (pure pursuit) 选取前瞻点, 再用两路独立 PID (平移/朝向) 结合曲率限速与靠近减速生成 base_link 系下的全向 `cmd_vel`。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/omni_pid_pursuit_controller.cpp | A | 控制器全部核心逻辑: 生命周期、前瞻计算、PID 调用、曲率/靠近限速、碰撞检测、动态参数 |
| include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp | A | 控制器类声明、成员参数、发布器、`nav2_core::Controller` 接口重写 |
| src/pid.cpp | B | 通用 PID `calculate` 实现 (P/I/D + 积分限幅 + 输出限幅) |
| include/pb_omni_pid_pursuit_controller/pid.hpp | B | `PID` 类声明 (增益、dt、max/min、内部误差状态) |
| pb_omni_pid_pursuit_controller.xml | C | pluginlib 清单, 将类导出为 `nav2_core::Controller` |
| CMakeLists.txt | C | ament_auto 构建, 导出插件描述文件 |

### 公共 API 契约
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `configure` | `void(WeakPtr&, name, tf_buffer, Costmap2DROS)` | 声明/读取全部参数, 建两路 PID 与三个发布器 | parent 须可 lock 否则抛 `PlannerException`; `control_duration_ = 1/controller_frequency` 传给 PID 作 dt; `max_robot_pose_search_dist_` 默认取 costmap 半幅 |
| `activate` / `deactivate` / `cleanup` | `void()` | 生命周期 | activate 须先 configure; activate 绑定 `dynamicParametersCallback`; cleanup 不重置 PID 指针 |
| `setPlan` | `void(const nav_msgs::msg::Path&)` | 存全局路径到 `global_plan_` | 必须在 `computeVelocityCommands` 前调用; 空路径会在后续 transform 阶段抛异常 |
| `computeVelocityCommands` | `TwistStamped(pose, velocity, GoalChecker*)` | 主控制回路 | 前置已 setPlan; 持 `mutex_` 与 costmap 锁; **`goal_checker` 参数被忽略**; 输出 `linear.x=lin_vel*cos(theta_dist)`, `linear.y=lin_vel*sin(theta_dist)`, `angular.z=angular_vel`; **速度分量按 base_link 系算, 但 `cmd_vel.header` 设为 `pose.header`(通常 global 系), 帧标注与数据不一致**; 检测到碰撞抛 `PlannerException` 停车; `use_rotate_to_heading_` 且末位姿朝向偏差 > 阈值时先原地转向 |
| `setSpeedLimit` | `void(const double&, const bool&)` | Nav2 限速接口 | **未实现**, 仅 `RCLCPP_WARN`, 忽略参数 |
| `getLookAheadDistance` | `double(const Twist& speed)` | 前瞻距离 | 速度缩放开时 `dist = hypot(vx,vy)*lookahead_time_` clamp 到 `[min,max]`; 否则用静态 `lookahead_dist_` |
| `getLookAheadPoint` | `PoseStamped(dist, Path)` | 前瞻点 | 取第一个距原点 ≥ lookahead 的位姿; 全在圈内取末点; 插值开时用圆-线段交点 |
| `PID::PID` | `PID(dt, max, min, kp, kd, ki)` | 构造 | **形参顺序 kp,kd,ki**; `pre_error_=0, integral_=0`; `integral_limit_` 默认 1.0 |
| `PID::calculate` | `double(set_point, pv)` | 单步输出 | 积分先 clamp 到 `[-integral_limit_, +integral_limit_]` **再**算 i_out(当拍即生效); 输出 clamp 到 `[min_,max_]`; 有状态 |
| `PID::setSumLimit` | `void(double)` | 设积分限幅 | limit>0 才生效; 由控制器用 `min_max_sum_error_` 调用(构造后 + 动态参数回调) |
| `PID::setSumError` | `void(double)` | 外部设积分 | 本模块内未使用(保留接口) |

### 调用关系
依赖: `nav2_core` (Controller/异常/GoalChecker)、`nav2_costmap_2d`、`nav2_util`、`tf2`+`tf2_geometry_msgs`、rclcpp/rclcpp_lifecycle、geometry_msgs/nav_msgs/visualization_msgs、pluginlib
被依赖: 作为 pluginlib 插件由 nav2 `controller_server` 运行时加载。**注意命名空间不一致**: 类实际注册为 `pb_omni_pid_pursuit_controller::OmniPidPursuitController` (见 `pb_omni_pid_pursuit_controller.xml` 与 `PLUGINLIB_EXPORT_CLASS`), 但 `nav2_params.{simulation,reality}.yaml` 里 `FollowPath: plugin:` 写的是 `omni_pid_pursuit_controller::OmniPidPursuitController` (无 `pb_` 前缀)。详见 Layer 3 #30。

### 动态引用
`PLUGINLIB_EXPORT_CLASS(pb_omni_pid_pursuit_controller::OmniPidPursuitController, nav2_core::Controller)` + `pb_omni_pid_pursuit_controller.xml` 注册。运行期由 controller_server 按 YAML 中的插件名字符串反射加载, 无编译期调用者。动态参数回调由 rclcpp 参数服务运行时触发。

### 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `OmniPidPursuitController` | omni_pid_pursuit_controller.hpp:21 | 控制器插件主类 |
| `PID` | pid.hpp:6 | 带积分/输出限幅的通用 PID, 平移与朝向各一实例 |

---

## 模块: nav2_plugins

> 同步: 2026-07-11 | Tier 分布: A: 3 / B: 12 / C: 5

### 职责
为 Navigation2 提供一组自定义扩展插件: 巡逻/定点决策类 BT 节点、恢复行为、速度/目标发布节点, 以及基于强度过滤的 3D 体素代价地图层与朝空旷方向后退的恢复行为, 均通过 pluginlib 在运行时被 nav2 各服务器加载。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| include/nav2_plugins/bt/nav_utils.hpp | A | 全部决策 BT 节点共享的纯函数工具库(黑板取 node、路径构建、巡逻状态机、距离/角度计算) |
| CMakeLists.txt | A | 声明 14 个共享库目标与 pluginlib 导出, 是插件注册的构建枢纽 |
| include/nav2_plugins/bt/custom_types.hpp | A | 为 `PoseStamped` 提供 BT 字符串反序列化模板特化, 被 XML 端口解析依赖 |
| src/behaviors/back_up_free_space.cpp | B | 恢复行为: 查代价地图找最宽空旷扇区并朝该方向后退 |
| src/layers/intensity_voxel_layer.cpp | B | 强度过滤 3D 体素代价地图层, 标记致命障碍 |
| src/bt/action/send_nav_through_poses.cpp | B | 异步调用 NavigateThroughPoses action, 含线程安全目标状态跟踪 |
| src/bt/action/send_nav2_goal.cpp | B | 基于 behaviortree_ros2 的 NavigateToPose action 客户端 |
| src/bt/action/publish_nav_goal.cpp | B | 向话题发目标位姿, 含去重与 burst 重发 |
| src/bt/action/select_patrol_path.cpp | B | 生成来回巡逻路径并输出下一游标/方向 |
| src/bt/action/select_fixed_path.cpp / select_path_goal_pose.cpp | B | 固定索引单点路径 / 取路径末点为目标 |
| src/bt/action/pub_twist.cpp / pub_spin_speed.cpp | B | 直接发 Twist / Float32 自旋速度 |
| src/bt/action/hold_stop_flag.cpp | B | 定时发 stop_flag 布尔量实现停留等待 |
| src/bt/condition/is_path_goal_reached.cpp | B | 判断当前位姿是否到达路径终点 |
| src/bt/control/recovery_node.cpp / decorator/rate_controller.cpp | C | nav2 移植的恢复控制/限频装饰节点 |

### 公共 API 契约
| 符号 | 端口/参数 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| `SendNav2Goal` | in `goal`(`x;y;yaw` 或 7 段), `goal_frame`(默认`map`) | 向 NavigateToPose 发目标 | frame 优先用入参 goal 自带的, 否则用 `goal_frame` 端口(2026-07-11 参数化, 原硬编码 `"map"`); 依赖 custom_types 解析; 派生自 behaviortree_ros2 RosActionNode |
| `SendNavThroughPoses` | in `path`, `goal_frame`(默认`map`); out `current_pose`,`goal_succeeded` | 异步走多点路径 | 异步 action + 回调状态机, 全程 mutex 保护; `goal_request_id_` 单调递增使旧回调失效; 同路径重复 tick 不重发; 默认 action `/navigate_through_poses`; 空 frame 填 `goal_frame`(2026-07-11 参数化) |
| `PublishNavGoal` | in `goal`, `topic_name`(默认`goal_pose`), `goal_frame`(默认`map`) | 直接向话题发目标 | **绕过 nav2 action**; 位姿与上次相近(位置 tol + yaw 0.05rad)则不重发仅 burst 补 1 次; 换 topic 重建 publisher 并清状态; 空 frame 填 `goal_frame`(2026-07-11 参数化) |
| `PublishTwist` / `PublishSpinSpeed` | in v_x/v_y/v_yaw / spin_speed | 直接发速度/自旋 | **绕过 nav2 控制器直接发话题**; halt 时发零 |
| `SelectFixedPath` | in `target_index`; out `path` | 单点路径 | 从参数 `nav.goal_points.x/y/z` 构建(构造期读取, 全程不变); 越界失败 |
| `SelectPatrolPath` | in `patrol_cursor`,`patrol_direction`; out `path`,`next_cursor`,`next_direction` | 往返巡逻 | 巡逻点构造期固化; 边界处反向(三角波); 游标/方向越界自动归位 |
| `IsPathGoalReached` | in `path`,`goal_succeeded`,`current_pose` | 到达判定 | `path_tolerance` 构造期读取; **`goal_succeeded` 端口声明但 tick 未使用**; 缺 path/pose 返回 FAILURE |
| `HoldStopFlag` | 无端口 | 停留计时发 stop_flag | 有状态; 发 `stop_flag`(Bool); `duration<=0` 立即成功; halt 补发 false 释放 |
| `RecoveryNode` | in `num_attempts`(默认 999) | 主+恢复循环 | **恰好 2 个子节点否则抛异常**; 仅首子成功才成功 |
| `pb_nav2_behaviors::BackUpFreeSpace` | `global_frame`,`max_radius`,`service_name`,`visualize` | 朝空旷方向后退 | 导出 `nav2_core::Behavior`; onRun **阻塞调用** `local_costmap/get_costmap` 服务; 扫 [-π,π] 找最宽安全扇区中点为后退方向; cost>=253 视为不安全 |
| `pb_nav2_costmap_2d::IntensityVoxelLayer` | `z_voxels`,`origin_z`,`min/max_obstacle_intensity`,`mark_threshold` 等 | 强度过滤体素障碍层 | 导出 `nav2_costmap_2d::Layer`(派生 ObstacleLayer); **`isClearable()` 恒 false —— 只标记不清除**, 原地修改 `costmap_`; 仅接受强度在 [min,max] 的点; 标为 `LETHAL_OBSTACLE` |

### 调用关系
依赖: rclcpp/rclcpp_action/rclcpp_lifecycle、behaviortree_cpp、behaviortree_ros2、nav2_behaviors、nav2_costmap_2d、nav2_voxel_grid、nav2_util、各 msgs、tf2、pluginlib; 内部头 `nav_utils.hpp`(几乎所有 BT 节点用) 与 `custom_types.hpp`(send_nav2_goal 用)
被依赖: **全部作为 pluginlib 插件** —— BT 节点经 `BT_REGISTER_NODES`/`CreateRosNodePlugin` 由 bt_navigator 加载; `IntensityVoxelLayer` 经 costmap_plugins.xml 由 costmap 加载; `BackUpFreeSpace` 经 behavior_plugin.xml 由 behavior_server 加载

### 动态引用
所有插件均通过 pluginlib + XML 清单在运行时按字符串名加载。**静态依赖图不显示谁加载它们** —— 注册名(如 `"SendNavThroughPoses"`、`"PublishTwist"`、`pb_nav2_costmap_2d::IntensityVoxelLayer`)只在 nav2 的 BT XML / costmap YAML / behavior_server 配置里以字符串出现。定位真正消费者须查 `nav2_params.*.yaml` 与 `behavior_trees/*.xml`。

### 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `convertFromString<PoseStamped>` (模板特化) | bt/custom_types.hpp:11 | 把 `x;y;yaw`(3段) 或 7 段字符串解析为 PoseStamped; 段数非 3/7 抛 RuntimeError |
| `getNodeFromBlackboard` | bt/nav_utils.hpp:23 | 从 BT 黑板取 `node` 句柄, 取不到抛 runtime_error |
| `computeNextPatrolState` | bt/nav_utils.hpp:144 | 巡逻三角波状态机 |
| `buildGoalPoints` / `buildPathFromIndices` / `makePoseStamped` | bt/nav_utils.hpp | 由点集与索引构建 Path, frame 默认 `"map"` |

---

## 模块: terrain_analysis

> 同步: 2026-07-11 | Tier 分布: A: 1 / B: 0 / C: 2

### 职责
订阅配准点云与里程计, 用滚动地形体素栈(21×21, 体素 1.0m)累积点云、逐平面体素(51×51, 0.2m)估计地面高程, 输出每点带相对地面高度(intensity)的近场可通行性地形代价点云 `terrain_map`。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/terrainAnalysis.cpp | A | 单文件节点: 全局参数、4 个回调、`main()` 主循环(体素滚动、下采样衰减、地面估计、动态障碍剔除、无数据障碍膨胀、发布) |
| CMakeLists.txt / package.xml | C | 构建 `terrainAnalysis` 可执行文件 / 依赖声明 |

### 公共 API 契约 (ROS 接口即 API)
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| 订阅 `lidar_odometry` | Odometry, q5 | 车体位姿 | 提供 vehicleXYZ+RPY; 被点云裁剪与体素索引依赖, 须先于点云到达 |
| 订阅 `registered_scan` | PointCloud2, q5 | 已配准点云(odom 系) | 裁剪到 `minRelZ..maxRelZ`(含 `disRatioZ*dis` 斜坡放宽)且 `dis < voxelSize*(halfWidth+1)`; `intensity` 被复用为相对起始时间戳 |
| 订阅 `joy` / `map_clearing` | Joy / Float32 | 手柄清云 / 外部清云半径 | `buttons[5]>0.5` 触发清云并重置 `noDataInited`; **未校验 buttons 长度** |
| 发布 `terrain_map` | PointCloud2, q2 | 地形代价点云 | frame_id 由参数 `mapFrameId`(默认 `"odom"`, 2026-07-11 参数化); 每点 `intensity`=相对局部地面的高度差 `disZ`(≥0), 保留条件 `disZ < vehicleHeight` 且体素点数 `≥ minBlockPointNum` |
| `main` | `int(int,char**)` | 100Hz spin_some | **单线程, 全局变量非线程安全**(回调与主循环同线程靠 spin_some 串行化) |
| 参数 | `quantileZ/useSorting/clearDyObs/considerDrop/noDataObstacle/vehicleHeight/minBlockPointNum/...` | 调参 | 地面高程取每平面体素点集的 `quantileZ` 分位数或最小 Z; `clearDyObs` 用车体系 VFOV 角度剔除动态障碍 |

### 调用关系
依赖: PCL (VoxelGrid/PointXYZI)、tf2(RPY)、rclcpp、各 msgs; (message_filters/pcl_ros 在 CMake 声明但源码未用)
被依赖: navigation_launch.py 作为节点 `terrainAnalysis` 启动; 输出 `terrain_map` 被 terrain_analysis_ext 与 costmap IntensityVoxelLayer 消费

### 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `terrainVoxelCloud[441]` (21×21, 1.0m) | 文件作用域全局 | 滚动累积的地形体素栈 |
| `planarVoxelElev/planarPointElev` (51×51, 0.2m) | 文件作用域全局 | 每平面体素的估计地面高度与点集 |

---

## 模块: terrain_analysis_ext

> 同步: 2026-07-11 | Tier 分布: A: 1 / B: 0 / C: 2

### 职责
在更大尺度(41×41, 体素 2.0m)累积地形并估计地面高程, 通过地形连通性 BFS 剔除天花板/悬空结构, 在 `localTerrainMapRadius` 外用自身估计、半径内直接融合 `terrain_map`, 发布扩展地形图 `terrain_map_ext`。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| src/terrainAnalysisExt.cpp | A | 单文件节点: 5 个回调 + `main()`(体素滚动、衰减、地面估计、连通性 BFS 去天花板、远近融合、发布) |
| CMakeLists.txt / package.xml | C | 构建 `terrainAnalysisExt` / 依赖 |

### 公共 API 契约 (ROS 接口即 API)
| 符号 | 签名 | 用途 | 契约 / 不变量 |
|---|---|---|---|
| 订阅 `registered_scan` | PointCloud2, q5 | 已配准点云(odom 系) | 裁剪到 `lowerBoundZ..upperBoundZ`(含放宽); intensity 复用为时间 |
| 订阅 `terrain_map` | PointCloud2, q2 | terrain_analysis 的近场地形图 | 在 `localTerrainMapRadius` 内被**原样融合**进输出(保留其精细代价) |
| 订阅 `lidar_odometry`/`joy`/`cloud_clearing` | Odometry/Joy/Float32 | 位姿/清云 | `buttons[5]` 未校验长度 |
| 发布 `terrain_map_ext` | PointCloud2, q2 | 扩展尺度地形图 | frame_id 由参数 `mapFrameId`(默认 `"odom"`, 2026-07-11 参数化); 远场点 `intensity`=`fabs(z-地面估计)`(连通性 `conn==2` 或未启检查时保留); 近场点来自 terrain_map 融合 |
| 参数 | `checkTerrainConn/terrainConnThre/ceilingFilteringThre/localTerrainMapRadius/...` | 调参 | 连通性 BFS 从车体中心体素起, `|Δelev|<terrainConnThre` 视为连通(conn=2), `>ceilingFilteringThre` 视为天花板(conn=-1) |

### 调用关系
依赖: PCL(VoxelGrid; kdtree 引入但未用)、tf2、rclcpp、std::queue(BFS)、各 msgs
被依赖: navigation_launch.py 作为节点 `terrainAnalysisExt` 启动; **拓扑耦合: 必须消费 terrain_analysis 的 terrain_map 才能填充近场, 否则输出中心空洞**; 输出被 costmap 的 global IntensityVoxelLayer 消费

### 关键类型
| 类型 | 定义位置 | 用途 |
|---|---|---|
| `terrainVoxelCloud` (41×41, 2.0m) | 文件作用域全局 | 大尺度滚动地形体素栈 |
| `planarVoxelConn` (101×101, 0.4m) | 文件作用域全局 | 连通性标记 (0/1/2/-1) |
| `planarVoxelQueue` | 文件作用域全局 | 连通性 BFS 队列 |

---

## 模块: nav_bringup

> 同步: 2026-07-11 | Tier 分布: A: 0 / B: 6 / C: 4

### 职责
**编排枢纽**: 无源码逻辑, 由 launch 文件与 `nav2_params.{simulation,reality}.yaml` 组织并配置全栈所有节点; `package.xml` 依赖列表即项目的功能全集。

### 关键文件
| 文件 | Tier | 职责 |
|---|---|---|
| launch/simulation.launch.py | B | 顶层仿真启动: loopback_sim + map_server + navigation + rviz, 加命名空间与 `/tf` remap |
| launch/navigation_launch.py | B | 导航栈: terrain_analysis(_ext) + loam_interface + sensor_scan + nav2 七个 lifecycle 节点; 支持普通/composable 两种加载 |
| launch/localization_launch.py / slam_launch.py | B | 定位(重定位+map_server) / SLAM 建图 |
| launch/joy_teleop_launch.py / rviz_launch.py | B | 手柄 / RViz |
| config/nav2_params.simulation.yaml | B | 全栈参数: point_lio、各帧名、costmap 层、FollowPath 控制器插件、BT 等 |
| config/nav2_params.reality.yaml | B | 实车参数 |
| behavior_trees/*.xml | C | Nav2 BT: navigate_to_pose / through_poses w/ replanning & recovery |

### 关键不变量 (来自 launch/yaml, 修改必读)
- **lifecycle 节点集** (navigation_launch.py:28): controller_server, smoother_server, planner_server, behavior_server, bt_navigator, waypoint_follower, velocity_smoother —— 由 `lifecycle_manager_navigation` 统一 autostart。
- **cmd_vel 重映射链**: 见 Layer 1 §1.4。
- **关键帧名** (sim yaml): `map`→`odom`(重定位广播)→`base_footprint`(sensor_scan 广播)→`front_mid360`(lidar)/`gimbal_yaw`(robot_base)。
- **costmap 插件链**: `["static_layer", "intensity_voxel_layer", "inflation_layer"]`; local 层 IntensityVoxelLayer 订 `terrain_map`, global 层订 `terrain_map_ext`。
- **控制器**: `FollowPath` → YAML 配 `omni_pid_pursuit_controller::OmniPidPursuitController` (实际注册名带 `pb_` 前缀, 命名不一致见 Layer3 #30), `controller_frequency: 20.0`。

### 调用关系
依赖 (package.xml, 即全项目功能清单): navigation2, nav2_smac_planner, nav2_plugins, teleop_twist_joy, livox_ros_driver2, point_lio, terrain_analysis(_ext), omni_pid_pursuit_controller, small_gicp_relocalization, pointcloud_to_laserscan, slam_toolbox, rviz2
被依赖: 顶层入口, 无

---

## 其余模块 (Tier C 概览)

| 模块 | 归属 | 职责 (一句话) |
|---|---|---|
| `robot_interfaces` | interfaces | 自定义消息定义: `Gimbal`/`GimbalCmd`(云台角度/速度指令)、`Models`(string[5])、`RobotStateInfo`; 无代码逻辑, rosidl 生成 |
| `livox_ros_driver2` | 第三方 | Livox MID360/HAP 固态雷达 ROS2 驱动 (含 Livox-SDK2、rapidjson); 出原始点云与 IMU。项目消费其输出, 一般不改。 |
| `point_lio` | 第三方 | Point-LIO 紧耦合激光惯性里程计 (含 IKFoM 流形卡尔曼滤波); 出 `cloud_registered` + `aft_mapped_to_init`(lidar_odom 系)。核心算法在 laserMapping.cpp/Estimator.cpp。 |
| `pointcloud_to_laserscan` | 第三方 | 点云↔激光扫描双向转换 (Willow Garage BSD); 为 slam_toolbox 等 2D 组件提供 LaserScan。 |
| `nav2_loopback_sim` | 第三方 | Nav2 官方回环仿真: 免物理引擎, 直接把 cmd_vel 积分为里程计与 TF; 供 simulation.launch.py 用。 |
| `ign_sim_pointcloud_tool` | 自研 | 仿真点云格式转换: 为 Gazebo/Ignition 点云补 ring/time 字段以适配 point_lio (参数 n_scan/horizon_scan/ang_*)。 |
| `teleop_twist_joy` | 自研改版 | 手柄 → Twist 遥控, 支持 enable 按钮、stamped twist、robot_base_frame 变换。 |
| `pcd2pgm` | 自研 | 离线工具: PCD 点云地图 → 2D 占据栅格 PGM (含 RadiusOutlierRemoval 滤波); transient_local QoS 发 map。 |
| `rosbag2_composable_recorder` | 第三方改版 | 可组合的 rosbag2 录制节点, 支持进程内组合与自动时间戳命名。 |




---

## Layer 3: 风险图

> 以下标注基于代码的结构特征, 不断言"这是 bug", 每条需人工确认。
> 置信度: 高 = 可量化事实; 中 = 模式匹配需复核; 低 = 大概率无害但列出以求完整。
> 范围: 仅项目自研包 (第三方 livox/point_lio 未做风险扫描)。
>
> **2026-07-11 复用性整改 (已修):** 以复用率为核心修复了一批确定性 bug 与硬编码。状态列 ✅=已修 / ⬜=待办。
> 已修: #1 loam TF 未初始化+提前使用 bug; #2 死参数 min_max_sum_error 已接入 PID; #3 PID 积分限幅顺序;
> #14/#23 全部硬编码 map/odom 帧改为参数(terrain 的 `mapFrameId`、BT 节点的 `goal_frame` 端口、teleop 的 `global_frame`); #27 sensor_scan static 变量改成员。

### 摘要 (共 30 条; 其中 ✅ 已修 6 条 / ⬜ 待办 24 条)
| 类别 | 数量 |
|---|---|
| 复杂度热点 | 6 |
| 高耦合 / 拓扑耦合 | 3 |
| 隐式依赖 | 8 |
| 异常处理缺口 | 6 |
| 理解缺口 | 6 |
| 硬编码常量 | 7 |
| 配置不一致 | 1 |

### 注解
| # | 状态 | 文件:行 | 类型 | 置信度 | 说明 / 建议 |
|---|---|---|---|---|---|
| 1 | ⬜ | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:262 | 隐式依赖 | 高 | `cmd_vel.header` 设为 `pose.header`(global 系), 但 twist 分量按 base_link 系计算, 帧标注与数据不一致。下游若按 header 帧解读速度会出错, 需确认约定。(Nav2 内部当前不读此 header, 故暂未致故障) |
| 2 | ✅ | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:161 | 理解缺口 | 高 | 已修: `min_max_sum_error_` 现经新增 `PID::setSumLimit()` 接入两路 PID(构造后与动态参数回调均调用), 参数生效。 |
| 3 | ✅ | omni_pid_pursuit_controller/src/pid.cpp:20 | 理解缺口 | 中 | 已修: 积分限幅(L21-25)移到 `i_out`(L26) 计算之前, 当拍输出即反映 anti-windup; 限幅上限改为可配 `integral_limit_`。 |
| 4 | ⬜ | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:213 | 复杂度热点 | 高 | `computeVelocityCommands` 串联 transform/前瞻/双PID/曲率限速/靠近减速/碰撞检测多职责。重构前补特征测试。 |
| 5 | ⬜ | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:445 | 异常处理缺口 | 中 | `isCollisionDetected` 中位姿超出 costmap 时直接 return false(视为无碰撞), 告警被注释, 边界外静默放行。 |
| 6 | ⬜ | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp:276 | 理解缺口 | 中 | `setSpeedLimit` 未实现(仅 WARN), 上层若下发限速将被忽略。 |
| 30 | ⬜ | nav2_params.{simulation,reality}.yaml:339/332 vs pb_omni_pid_pursuit_controller.xml:3 | 配置不一致 | 高 | YAML 里 `FollowPath.plugin = "omni_pid_pursuit_controller::OmniPidPursuitController"`, 但插件实际注册名为 `pb_omni_pid_pursuit_controller::OmniPidPursuitController`(带 `pb_` 前缀)。pluginlib 按字符串精确匹配, 名称不符通常导致控制器加载失败。**需在实机/仿真确认**: 若当前能跑, 说明另有别名或我理解有误; 若加载报 "does not exist" 类错误, 把 YAML 改为带 `pb_` 前缀即可。 |
| 7 | ⬜ | nav2_plugins/src/behaviors/back_up_free_space.cpp:45 | 高耦合 | 中 | onRun 在 behavior_server 线程内**同步阻塞**等 `local_costmap/get_costmap` 服务(1s 超时硬编码), 服务不可用会阻塞恢复行为。 |
| 8 | ⬜ | nav2_plugins/src/behaviors/back_up_free_space.cpp:148 | 复杂度热点 | 中 | `findBestDirection`(L148) 双重循环 + 多状态标志的扇区扫描逻辑, 分支交织。 |
| 9 | ⬜ | nav2_plugins/src/behaviors/back_up_free_space.cpp:222 | 理解缺口 | 低 | `gatherFreePoints` 定义但本文件内未见调用, 疑死代码。 |
| 10 | ⬜ | nav2_plugins/src/layers/intensity_voxel_layer.cpp:161 | 隐式依赖 | 中 | 直接按 getIndex 下标原地写 `costmap_` 并标 LETHAL; `isClearable` 恒 false 意味只增不清, 依赖父类内存布局。 |
| 11 | ⬜ | nav2_plugins/src/bt/action/send_nav_through_poses.cpp:34 | 复杂度热点 | 中 | `tick()`(L34)内多 mutex 段 + 多状态组合的并发状态机, 复杂易错。 |
| 12 | ⬜ | nav2_plugins/src/bt/condition/is_path_goal_reached.cpp:32 | 理解缺口 | 中 | 声明 `goal_succeeded` 输入端口但 tickCondition 未读取, 契约与实现不一致。 |
| 13 | ⬜ | nav2_plugins/include/nav2_plugins/bt/custom_types.hpp:11 | 隐式依赖 | 低 | 头文件中定义非 inline 的模板全特化函数, 多 TU 包含可能 ODR 冲突。 |
| 14 | ✅ | nav2_plugins/src/bt/action/{send_nav2_goal,publish_nav_goal,send_nav_through_poses}.cpp | 硬编码常量 | 低 | 已修: goal frame 改为 BT 输入端口 `goal_frame`(默认 `"map"`), 且优先采用入参已带的 frame_id。action 名 `/navigate_through_poses` 仍可经参数覆盖(原已支持)。 |
| 15 | ⬜ | nav2_plugins/src/bt/action/hold_stop_flag.cpp:14 | 硬编码常量 | 低 | 话题名 `stop_flag` 硬编码。 |
| 16 | ⬜ | terrain_analysis/src/terrainAnalysis.cpp:19 | 隐式依赖 | 高 | ~60 个文件作用域全局变量(自 L19 起)承载全部状态, 回调与主循环靠单线程 spin_some 隐式串行, 非线程安全。 |
| 17 | ⬜ | terrain_analysis_ext/src/terrainAnalysisExt.cpp:20 | 隐式依赖 | 高 | ~50 个文件作用域全局变量(自 L20 起)承载全部状态, 非线程安全。 |
| 18 | ⬜ | terrain_analysis/src/terrainAnalysis.cpp:187 | 复杂度热点 | 高 | `main()` 约 500 行含多阶段处理, 单函数职责过重。 |
| 19 | ⬜ | terrain_analysis_ext/src/terrainAnalysisExt.cpp:163 | 复杂度热点 | 高 | `main()` 约 400 行含体素/连通性 BFS/融合多阶段。 |
| 20 | ⬜ | terrain_analysis/src/terrainAnalysis.cpp:174; terrainAnalysisExt.cpp:152 | 异常处理缺口 | 中 | `joy->buttons[5]` 未校验数组长度, 空/短 Joy 消息越界。 |
| 21 | ⬜ | terrain_analysis*/两节点 | 隐式依赖 | 中 | 输出点 `intensity` 字段被重载为高度代价, 与输入 intensity=时间戳语义冲突, 属非显然约定。 |
| 22 | ⬜ | terrain_analysis_ext/src/terrainAnalysisExt.cpp:87 | 理解缺口 | 中 | `kdtree`(KdTreeFLANN, L87)及搜索缓冲(L230)声明但从未使用, 残留死代码易误导。 |
| 23 | ✅ (帧) | terrain_analysis*/两节点 | 硬编码常量 | 中 | 已修帧: `frame_id="odom"` 改为参数 `mapFrameId`(默认 `"odom"`)。⬜ 其余魔数(体素尺寸/窗口/`minZ`/`*1e9`)仍待参数化。 |
| 24 | ⬜ | terrain_analysis_ext (拓扑) | 拓扑耦合 | 中 | ext 必须消费 terrain_analysis 的 terrain_map 才能填充 localTerrainMapRadius 内近场, 否则输出中心空洞。 |
| 25 | ✅ | loam_interface/src/loam_interface.cpp:42 | 隐式依赖→bug | 高 | 已修: `tf_odom_to_lidar_odom_` 头文件初始化为 `getIdentity()`, 且 pointCloudCallback 加 `if(!initialized) return;` guard。原为未初始化内存被提前使用的确定性 bug。 |
| 26 | ⬜ | sensor_scan_generation/src/sensor_scan_generation.cpp:87 | 异常处理缺口 | 中 | `getTransform` 失败返回单位变换(L87, 仅 WARN), TF 缺失静默产出错误位姿。 |
| 27 | ✅ | sensor_scan_generation/src/sensor_scan_generation.cpp:103 | 隐式依赖 | 中 | 已修: `publishOdometry`(L103) 速度微分的 `static` 历史改为成员 `previous_odom_*` + `has_previous_odom_`(L120 起), 支持多实例且节点重建时复位。 |
| 28 | ⬜ | small_gicp_relocalization/src/small_gicp_relocalization.cpp:97 | 异常处理缺口 | 中 | PCD 读取失败仅 ERROR 返回, 后续以空地图继续跑; TF 30 次重试全失败则地图留原系不变换, 配准会偏。 |
| 29 | ⬜ | small_gicp_relocalization/src/small_gicp_relocalization.cpp:190 | 理解缺口 | 低 | 发布 TF 时间戳 +0.1s 前推(有注释与出处), 属有意设计, 记录以备排查 TF 外插问题。 |

---

## 附录: 符号索引

> 字母序 (按符号名)。用 `grep <符号名>` 或搜模块名跳到对应段落。仅收录自研包关键符号。

| 符号 | 种类 | 文件 | 行 | 模块 |
|---|---|---|---|---|
| `BackUpFreeSpace` | class | nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp | 26 | nav2_plugins |
| `buildGoalPoints` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 53 | nav2_plugins |
| `buildPathFromIndices` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 80 | nav2_plugins |
| `computeNextPatrolState` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 144 | nav2_plugins |
| `computeVelocityCommands` | method | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 213 | omni_pid_pursuit_controller |
| `configure` | method | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 21 | omni_pid_pursuit_controller |
| `convertFromString<PoseStamped>` | function | nav2_plugins/include/nav2_plugins/bt/custom_types.hpp | 11 | nav2_plugins |
| `findBestDirection` | function | nav2_plugins/src/behaviors/back_up_free_space.cpp | 148 | nav2_plugins |
| `findNearestIndex` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 121 | nav2_plugins |
| `getLookAheadDistance` | method | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 466 | omni_pid_pursuit_controller |
| `getLookAheadPoint` | method | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 357 | omni_pid_pursuit_controller |
| `getNodeFromBlackboard` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 23 | nav2_plugins |
| `getTransform` | method | sensor_scan_generation/src/sensor_scan_generation.cpp | 76 | sensor_scan_generation |
| `HoldStopFlagAction` | class | nav2_plugins/include/nav2_plugins/bt/action/hold_stop_flag.hpp | 13 | nav2_plugins |
| `IntensityVoxelLayer` | class | nav2_plugins/include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp | 24 | nav2_plugins |
| `IsPathGoalReachedCondition` | class | nav2_plugins/include/nav2_plugins/bt/condition/is_path_goal_reached.hpp | 14 | nav2_plugins |
| `laserCloudAndOdometryHandler` | method | sensor_scan_generation/src/sensor_scan_generation.cpp | 50 | sensor_scan_generation |
| `laserCloudHandler` | function | terrain_analysis/src/terrainAnalysis.cpp | 135 | terrain_analysis |
| `loadGlobalMap` | method | small_gicp_relocalization/src/small_gicp_relocalization.cpp | 95 | small_gicp_relocalization |
| `LoamInterfaceNode` | class | loam_interface/include/loam_interface/loam_interface.hpp | 17 | loam_interface |
| `main` | function | terrain_analysis/src/terrainAnalysis.cpp | 187 | terrain_analysis |
| `main` | function | terrain_analysis_ext/src/terrainAnalysisExt.cpp | 163 | terrain_analysis_ext |
| `makePoseStamped` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 40 | nav2_plugins |
| `normalizeAngle` | function | nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp | 104 | nav2_plugins |
| `odometryCallback` | method | loam_interface/src/loam_interface.cpp | 60 | loam_interface |
| `OmniPidPursuitController` | class | omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp | 21 | omni_pid_pursuit_controller |
| `PID` | class | omni_pid_pursuit_controller/include/pb_omni_pid_pursuit_controller/pid.hpp | 6 | omni_pid_pursuit_controller |
| `PID::calculate` | method | omni_pid_pursuit_controller/src/pid.cpp | 10 | omni_pid_pursuit_controller |
| `pointCloudCallback` | method | loam_interface/src/loam_interface.cpp | 42 | loam_interface |
| `PublishNavGoalAction` | class | nav2_plugins/include/nav2_plugins/bt/action/publish_nav_goal.hpp | 14 | nav2_plugins |
| `publishOdometry` | method | sensor_scan_generation/src/sensor_scan_generation.cpp | 103 | sensor_scan_generation |
| `PublishSpinSpeedAction` | class | nav2_plugins/include/nav2_plugins/bt/action/pub_spin_speed.hpp | 13 | nav2_plugins |
| `publishTransform` | method | small_gicp_relocalization/src/small_gicp_relocalization.cpp | 182 | small_gicp_relocalization |
| `PublishTwistAction` | class | nav2_plugins/include/nav2_plugins/bt/action/pub_twist.hpp | 12 | nav2_plugins |
| `performRegistration` | method | small_gicp_relocalization/src/small_gicp_relocalization.cpp | 147 | small_gicp_relocalization |
| `RateController` | class | nav2_plugins/include/nav2_plugins/bt/decorator/rate_controller.hpp | 15 | nav2_plugins |
| `RecoveryNode` | class | nav2_plugins/include/nav2_plugins/bt/control/recovery_node.hpp | 22 | nav2_plugins |
| `registeredPcdCallback` | method | small_gicp_relocalization/src/small_gicp_relocalization.cpp | 136 | small_gicp_relocalization |
| `SelectFixedPathAction` | class | nav2_plugins/include/nav2_plugins/bt/action/select_fixed_path.hpp | 15 | nav2_plugins |
| `SelectPathGoalPoseAction` | class | nav2_plugins/include/nav2_plugins/bt/action/select_path_goal_pose.hpp | 14 | nav2_plugins |
| `SelectPatrolPathAction` | class | nav2_plugins/include/nav2_plugins/bt/action/select_patrol_path.hpp | 15 | nav2_plugins |
| `SendNav2GoalAction` | class | nav2_plugins/include/nav2_plugins/bt/action/send_nav2_goal.hpp | 12 | nav2_plugins |
| `SendNavThroughPosesAction` | class | nav2_plugins/include/nav2_plugins/bt/action/send_nav_through_poses.hpp | 19 | nav2_plugins |
| `SensorScanGenerationNode` | class | sensor_scan_generation/include/sensor_scan_generation/sensor_scan_generation.hpp | 23 | sensor_scan_generation |
| `setPlan` | method | omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp | 274 | omni_pid_pursuit_controller |
| `SmallGicpRelocalizationNode` | class | small_gicp_relocalization/include/small_gicp_relocalization/small_gicp_relocalization.hpp | 26 | small_gicp_relocalization |
| `TeleopTwistJoyNode` | class | teleop_twist_joy/include/teleop_twist_joy/teleop_twist_joy.hpp | 23 | teleop_twist_joy |

