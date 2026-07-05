# 模块: nav2_plugins

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 5 / B: 8 / C: 22

## 职责
Nav2 的自定义插件扩展库——提供行为树（BT）动作/条件/装饰器/控制节点、代价地图插件层、以及导航工具函数。覆盖巡逻、定点导航、路径选择等高级决策逻辑。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/nav2_plugins/bt/nav_utils.hpp` | A | 核心工具函数集（路径构建、距离计算、索引验证、巡逻状态机） |
| `include/nav2_plugins/bt/custom_types.hpp` | A | BT 自定义类型转换器（PoseStamped 字符串解析） |
| `src/bt/action/select_patrol_path.cpp` | A | 巡逻路径选择——根据巡逻游标和方向生成预览路径 |
| `include/nav2_plugins/bt/action/send_nav2_goal.hpp` | A | 向 Nav2 发送导航目标的 BT ROS 动作节点 |
| `include/nav2_plugins/bt/control/recovery_node.hpp` | A | 自定义恢复控制节点 |
| `include/nav2_plugins/bt/action/publish_nav_goal.hpp` | B | 发布导航目标点 |
| `include/nav2_plugins/bt/action/select_fixed_path.hpp` | B | 固定索引路径选择 |
| `include/nav2_plugins/bt/action/select_path_goal_pose.hpp` | B | 基于目标位姿的路径选择 |
| `include/nav2_plugins/bt/action/send_nav_through_poses.hpp` | B | 通过多航点导航 |
| `include/nav2_plugins/bt/action/hold_stop_flag.hpp` | B | 保持停止标志 |
| `include/nav2_plugins/bt/action/pub_spin_speed.hpp` | B | 发布旋转速度 |
| `include/nav2_plugins/bt/action/pub_twist.hpp` | B | 发布 Twist 速度指令 |
| `include/nav2_plugins/bt/condition/is_path_goal_reached.hpp` | B | 路径目标到达判断 |
| `include/nav2_plugins/bt/decorator/rate_controller.hpp` | C | 速率控制装饰器 |
| `include/pb_nav2_plugins/behaviors/back_up_free_space.hpp` | B | 空闲空间倒退行为 |
| `include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp` | B | 强度体素代价地图层 |
| `src/bt/action/*.cpp` | C | 各 BT 动作节点的实现 |
| `src/layers/intensity_voxel_layer.cpp` | C | 强度体素层实现 |
| `behavior_plugin.xml` | C | BT 插件注册清单 |
| `costmap_plugins.xml` | C | 代价地图插件注册清单 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | B | 包元数据 |

## 公共 API 参考

### 核心工具函数 (`nav_utils.hpp`)

| 符号 | 签名 | 用途 |
|---|---|---|
| `getNodeFromBlackboard` | `rclcpp::Node::SharedPtr (const BT::TreeNode &)` | 从 BT 黑板获取 ROS 节点句柄 |
| `makePoseStamped` | `geometry_msgs::msg::PoseStamped (const Point &, const string & frame_id)` | 从点构造带位姿的消息 |
| `buildGoalPoints` | `vector<Point> (const vector<double> &xs, ys, zs)` | 从三组坐标构建目标点列表，含长度一致性检查 |
| `validateIndex` | `size_t (size_t index, size_t total, const char * label)` | 索引范围验证，越界抛异常 |
| `buildPathFromIndices` | `nav_msgs::msg::Path (const vector<Point> &, const vector<size_t> &, const string &)` | 按索引列表组装 Path 消息 |
| `squaredDistance` | `double (const Point &, const Point &)` | 两点欧氏距离平方 |
| `normalizeAngle` | `double (double angle)` | 角度归一化到 [-π, π] |
| `isPathGoalReached` | `bool (const PoseStamped &, const Path &, double tolerance)` | 判断是否到达路径终点 |
| `findNearestIndex` | `size_t (const PoseStamped &, const vector<Point> &, const vector<size_t> &)` | 在候选索引中找最近目标点 |
| `computeNextPatrolState` | `pair<int,int> (int cursor, int direction, size_t count)` | 巡逻往复状态机——到达两端自动反转方向 |
| `pathEquivalent` | `bool (const Path &, const Path &, double tolerance)` | 判断两路径是否等价 |
| `toSizeIndices` | `vector<size_t> (const vector<int64_t> &, size_t total, const char *)` | 有符号索引转 size_t，含负值检查 |

### BT 动作节点

| 符号 | 类型 | 用途 |
|---|---|---|
| `SelectPatrolPath` | BT::SyncActionNode | 巡逻路径选择——生成含预览点的往复式巡逻路线 |
| `SelectFixedPath` | BT::SyncActionNode | 固定路径选择——按参数中给定的目标索引生成路径 |
| `SelectPathGoalPose` | BT::SyncActionNode | 目标位姿路径选择 |
| `SendNav2Goal` | BT::RosActionNode\<NavigateToPose\> | 向 Nav2 发送导航目标（ROS 动作） |
| `SendNavThroughPoses` | BT::RosActionNode | 通过多航点导航（ROS 动作） |
| `PublishNavGoal` | BT::SyncActionNode | 将导航目标发布到话题 |
| `HoldStopFlag` | BT::SyncActionNode | 保持停止标志位 |
| `PubSpinSpeed` | BT::SyncActionNode | 发布原地旋转速度指令 |
| `PubTwist` | BT::SyncActionNode | 发布通用 Twist 速度指令 |

### BT 条件节点

| 符号 | 类型 | 用途 |
|---|---|---|
| `IsPathGoalReached` | BT::ConditionNode | 判断当前位姿是否到达路径目标（容差可配） |

### BT 控制/装饰器节点

| 符号 | 类型 | 用途 |
|---|---|---|
| `RecoveryNode` | BT::ControlNode | 自定义恢复控制流节点 |
| `RateController` | BT::DecoratorNode | 频率控制装饰器 |

### 代价地图插件

| 符号 | 类型 | 用途 |
|---|---|---|
| `IntensityVoxelLayer` | nav2_costmap_2d::Layer | 基于点云强度的体素代价地图层 |
| `BackUpFreeSpace` | nav2_core::Behavior | 空闲空间倒退恢复行为 |

## 调用关系

**依赖**: BehaviorTree.CPP, behaviortree_ros2, nav2_behavior_tree, nav2_core, nav2_costmap_2d, nav2_behaviors, nav2_util, nav2_msgs, nav_msgs, geometry_msgs, tf2

**被依赖方**: nav_bringup（启动文件中配置为 bt_navigator 和 behavior_server 的插件）

## 关键类型

| 类型 | 位置 | 用途 |
|---|---|---|
| `geometry_msgs::msg::PoseStamped` | BT custom_types.hpp:11 | BT 字符串到 PoseStamped 的转换（支持 7 元组 x;y;z;qx;qy;qz;qw 和 3 元组 x;y;yaw 两种格式） |

## 设计要点

1. **巡逻往复算法** (`computeNextPatrolState`): 游标在巡逻索引列表两端自动反转方向。单点巡逻时返回固定状态 `{0, 1}`。
2. **路径预览机制** (`SelectPatrolPath`): 生成 `patrol_preview_points` 个预览点，而非仅返回单个目标点，使规划器和控制器能提前看到后续路径。
3. **黑板参数通信**: 所有 BT 节点通过 BehaviorTree 黑板（`{decision_path}`, `{decision_patrol_cursor}` 等）传递状态，无全局变量耦合。
4. **ROS 节点延迟获取**: `getNodeFromBlackboard` 从 BT 根黑板动态获取节点句柄，避免编译期耦合。
