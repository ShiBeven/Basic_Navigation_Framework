# 模块: sensor_scan_generation

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 1 / B: 1 / C: 3

## 职责
传感器扫描融合生成——将 LiDAR 里程计（odometry）与点云（PointCloud2）进行时间同步融合，发布变换后的点云和底盘里程计，为 Nav2 代价地图提供正确的传感器输入。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/sensor_scan_generation/sensor_scan_generation.hpp` | A | 节点类声明——消息过滤器同步策略 |
| `src/sensor_scan_generation.cpp` | B | 节点核心实现 |
| `launch/sensor_scan_generation.launch.py` | C | 启动文件 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | C | 包元数据 |

## 公共 API 参考

| 符号 | 类型 | 用途 |
|---|---|---|
| `SensorScanGenerationNode` | `class : public rclcpp::Node` | 传感器融合节点主类 |
| `laserCloudAndOdometryHandler` | `void (Odometry::SharedPtr, PointCloud2::SharedPtr)` | 同步回调——处理一对时间对齐的odom+点云 |
| `getTransform` | `tf2::Transform (const string &target, const string &source, const Time &)` | TF 查询封装 |
| `publishTransform` | `void (const Transform &, const string &parent, const string &child, const Time &)` | 发布 TF 变换 |
| `publishOdometry` | `void (const Transform &, string parent, string child, const Time &)` | 发布里程计消息 |

## 数据流

```
Odometry (LiDAR里程计) ──┐
                          ├── message_filters::ApproximateTime 同步
PointCloud2 (LiDAR点云) ──┘
                          │
                          ▼
              laserCloudAndOdometryHandler()
                          │
         ┌────────────────┼────────────────┐
         ▼                ▼                ▼
   查询TF变换      发布变换点云       发布底盘里程计
   (lidar→base)   (pub_laser_cloud_)  (pub_chassis_odometry_)
```

## 调用关系

**依赖**: message_filters (ApproximateTime), tf2, nav_msgs, sensor_msgs

**被依赖方**: nav_bringup（有条件启动，由 `use_sensor_scan` 参数控制）

## 关键类型

| 类型 | 位置 | 用途 |
|---|---|---|
| `SyncPolicy` | sensor_scan_generation.hpp:57 | `ApproximateTime<Odometry, PointCloud2>` 同步策略 |
| `tf_lidar_to_robot_base_` | sensor_scan_generation.hpp:61 | LiDAR 到机器人底盘的静态 TF |

## 注意事项

1. **ApproximateTime 同步**: 使用 message_filters 的近似时间同步策略，不要求 odom 和点云有完全相同的时间戳。
2. **TF 缓存独立**: 使用自有的 `tf_buffer_` / `tf_listener_` 实例，避免与其他节点的 TF 竞争。
3. **坐标系补偿**: `tf_lidar_to_robot_base_` 补偿 LiDAR 在机器人上的安装偏移，输出的是机器人底盘坐标系下的数据。
