# 模块: omni_pid_pursuit_controller

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 2 / B: 2 / C: 4

## 职责
面向全向移动机器人的 Nav2 控制器插件——基于纯追踪（Pure Pursuit）前视点算法 + 双 PID 控制（平移 + 航向），集成曲率限速、接近减速、碰撞检测等安全机制。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/pb_omni_pid_pursuit_controller/omni_pid_pursuit_controller.hpp` | A | 控制器类声明——完整的 public/protected/private API |
| `src/omni_pid_pursuit_controller.cpp` | A | 控制器核心实现（759 行） |
| `include/pb_omni_pid_pursuit_controller/pid.hpp` | B | PID 控制器封装 |
| `src/pid.cpp` | B | PID 控制器实现 |
| `pb_omni_pid_pursuit_controller.xml` | C | 插件注册清单 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | C | 包元数据 |
| `README.md` | C | 说明文档 |

## 公共 API 参考

| 符号 | 签名 | 用途 |
|---|---|---|
| `OmniPidPursuitController` | `class : public nav2_core::Controller` | 全向 PID 追踪控制器主类 |
| `configure` | `void (WeakPtr, string, shared_ptr<Buffer>, shared_ptr<Costmap2DROS>)` | 配置控制器，初始化 PID 和参数 |
| `computeVelocityCommands` | `TwistStamped (const PoseStamped &, const Twist &, GoalChecker *)` | 主控制循环——计算速度指令 |
| `setPlan` | `void (const Path &)` | 设置全局路径 |
| `setSpeedLimit` | `void (const double &, const bool &)` | 速度限制（⚠️ 未实现，仅打印警告） |
| `cleanup/activate/deactivate` | `void ()` | 生命周期管理 |

## 控制算法流程

```
computeVelocityCommands(pose, velocity)
│
├─ 1. transformGlobalPlan(pose)         — 将全局路径转换到机器人坐标系
│    ├── 找最近路径点
│    ├── 裁剪已通过部分
│    └── 坐标变换到 base_frame
│
├─ 2. getLookAheadDistance(velocity)     — 计算前视距离
│    └── 支持速度缩放: dist = speed * lookahead_time (clamp到 [min, max])
│
├─ 3. getLookAheadPoint(dist, plan)      — 计算前视点
│    ├── 找距离≥lookahead_dist的第一个路径点
│    └── 可选: 圆-线段交点插值（精确到亚路径点级别）
│
├─ 4. PID 计算
│    ├── move_pid_.calculate(lin_dist, 0) → 线速度
│    └── heading_pid_.calculate(angle_to_goal, 0) → 角速度
│
├─ 5. applyCurvatureLimitation()         — 曲率限速
│    ├── 三点圆拟合法计算曲率半径
│    ├── curvature ∈ [min, max] 区间内线性插值减速比例
│    └── 速率变化受 max_velocity_scaling_factor_rate 约束（防突变）
│
├─ 6. applyApproachVelocityScaling()     — 接近终点减速
│    └── 剩余距离 < approach_velocity_scaling_dist 时线性缩放
│
├─ 7. isCollisionDetected()              — 碰撞检测
│    └── 采样10个路径点检查代价地图 → 碰到障碍则抛异常
│
└─ 8. 返回 TwistStamped (linear.x, linear.y, angular.z)
```

## 关键参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `translation_kp/ki/kd` | 3.0/0.1/0.3 | 平移 PID 增益 |
| `rotation_kp/ki/kd` | 3.0/0.1/0.3 | 旋转 PID 增益 |
| `enable_rotation` | true | 是否启用旋转控制 |
| `lookahead_dist` | 0.3 m | 静态前视距离 |
| `use_velocity_scaled_lookahead_dist` | true | 启用速度缩放前视距离 |
| `min/max_lookahead_dist` | 0.2/1.0 m | 前视距离边界 |
| `lookahead_time` | 1.0 s | 前视时间系数 |
| `v_linear_min/max` | -3.0/3.0 m/s | 线速度边界 |
| `v_angular_min/max` | -3.0/3.0 rad/s | 角速度边界 |
| `curvature_min/max` | 0.4/0.7 | 曲率阈值范围（开始减速 ↔ 最大减速） |
| `reduction_ratio_at_high_curvature` | 0.5 | 高曲率时减速到原速的50% |
| `approach_velocity_scaling_dist` | 0.6 m | 接近减速起始距离 |
| `min_approach_linear_velocity` | 0.05 m/s | 最小接近速度（防止完全停止） |
| `use_interpolation` | true | 前视点插值（精确圆-线段交点） |
| `use_rotate_to_heading` | true | 接近目标时先旋转对准 |

## 调用关系

**依赖**: nav2_core, nav2_costmap_2d, nav2_util, tf2, pluginlib

**被依赖方**: nav_bringup（作为 controller_server 的插件加载）

## 注意事项

1. **全向支持**: 同时输出 `linear.x` 和 `linear.y`，支持全向底盘的横向移动。
2. **碰撞后抛异常**: `isCollisionDetected` 返回 true 时抛出 `PlannerException`，而非温和减速。由上层 recovery 机制接管。
3. **setSpeedLimit 未实现**: 速度限制接口只打印警告日志，不接受外部限速指令。
4. **动态参数**: 大部分控制参数支持运行时通过 ROS2 参数回调动态更新（`dynamicParametersCallback`）。
5. **曲率限速的速率平滑**: 通过 `max_velocity_scaling_factor_rate` 和 `last_velocity_scaling_factor_` 实现一阶低通滤波效果，避免速度指令突变。
