# 模块: nav2_loopback_sim

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 1 / B: 3 / C: 16

## 职责
无物理回环仿真器——替代 Gazebo/Ignition 等物理引擎，提供一个无摩擦、无惯量、无碰撞的简化仿真环境。接收 cmd_vel 指令，积分生成里程计和 TF，基于静态地图射线投射生成虚拟 LaserScan。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `nav2_loopback_sim/loopback_simulator.py` | A | 仿真器主类（423行）——完整的仿真循环 |
| `nav2_loopback_sim/utils.py` | B | 工具函数（坐标变换矩阵运算、地图查询） |
| `nav2_loopback_sim/tf_compat.py` | B | NumPy 2.x 兼容层——修复 tf_transformations 的依赖问题 |
| `launch/*.py` | C | 各类启动文件 |
| `maps/*.pgm` + `*.yaml` | C | 静态地图文件（depot, warehouse, tb3_sandbox） |
| `params/nav2_params.yaml` | C | Nav2 参数 |
| `setup.py` / `setup.cfg` | C | Python 包配置 |

## 公共 API 参考

| 符号 | 类型 | 用途 |
|---|---|---|
| `LoopbackSimulator` | `class : public rclcpp.Node` | 仿真器主节点 |
| `cmdVelCallback` / `cmdVelStampedCallback` | `void (Twist)` / `void (TwistStamped)` | 接收速度指令 |
| `initialPoseCallback` | `void (PoseWithCovarianceStamped)` | 设置初始位姿（首次=初始化，后续=重定位） |
| `timerCallback` | `void ()` | 主仿真循环——积分速度→更新 odom→发布 TF |
| `publishLaserScan` | `void ()` | 栅格化地图生成虚拟 LaserScan |
| `getLaserScan` | `void (int num_samples)` | 射线投射核心——逐角度遍历地图栅格 |
| `getMap` | `void ()` | 从 map_server 获取静态地图 |

## 仿真循环

```
timerCallback (频率=1/update_dur, 默认100Hz)
│
├─ 检查 cmd_vel 是否超时（>1秒视为停止）
│
├─ 速度积分:
│   dx = vx * dt, dy = vy * dt, dth = vth * dt
│   odom→base_link 变换累加 (带 yaw 旋转分解)
│
├─ publishTransforms(map→odom, odom→base_link)
└─ publishOdometry(odom→base_link)

publishLaserScan (独立定时器)
│
└─ getLaserScan():
    对每个角度射线:
      Bresenham 直线遍历地图栅格
      → 碰到障碍(occupancy≥60) → 记录距离
      → 无碰撞 → range = inf (或 max_range)
```

## 设计特点

1. **NumPy 2.x 兼容**: `tf_compat.py` 在 `import tf_transformations` 之前 monkey-patch NumPy 已移除的 `np.float` / `np.maximum_sctype` 符号，使仿真器能在较新的 Python 环境中运行。
2. **map→odom 重定位**: `initialPoseCallback` 第二次调用时，通过矩阵运算反算新的 `map→odom` 变换，保持 `odom→base_link` 不变，模拟 AMCL 重定位行为。
3. **无物理特性**: 不存在加速度、摩擦力、碰撞响应和惯量——速度指令直接积分，适合快速验证导航算法逻辑。
4. **虚拟 LaserScan**: 基于静态地图做射线投射，代价 < 60 的栅格视为空闲，≥60 视为障碍。

## 调用关系

**依赖**: nav2_simple_commander, tf_transformations, geometry_msgs, nav_msgs, sensor_msgs, tf2

**被依赖方**: 独立启动，替代真实仿真器

## 注意事项

1. `setupTimer` 在初始位姿到达前预热系统（发布 identity TF + 获取 base→laser 变换）。
2. 地图获取失败时退化为全 `inf`（或 `max_range - 0.1`）的扫描数据，不会崩溃。
3. `mat_base_to_laser` 在 startup 期间持续重试直到 Static TF 就绪。
