# 模块: loam_interface

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: B: 2 / C: 3

## 职责
LOAM（Lidar Odometry and Mapping）里程计桥接接口——订阅 point_lio 等 SLAM/里程计算法发布的点云和里程计话题，进行坐标系变换后重新发布到标准化话题名，解耦上游算法与下游导航栈。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/loam_interface/loam_interface.hpp` | B | 节点类声明 |
| `src/loam_interface.cpp` | B | 节点核心实现 |
| `launch/loam_interface_launch.py` | C | 启动文件 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | C | 包元数据 |

## 公共 API 参考

| 符号 | 类型 | 用途 |
|---|---|---|
| `LoamInterfaceNode` | `class : public rclcpp::Node` | 接口桥接节点 |
| `pointCloudCallback` | `void (PointCloud2::SharedPtr)` | 接收上游点云 → TF 变换 → 重新发布 |
| `odometryCallback` | `void (Odometry::SharedPtr)` | 接收上游里程计 → TF 变换 → 重新发布 |

## 数据流

```
point_lio (或其他里程计)
│
├── /state_estimation (Odometry)
│   └── odometryCallback() → TF变换(lidar_odom→odom) → /odom (标准化)
│
└── /registered_scan (PointCloud2)
    └── pointCloudCallback() → TF变换 → /scan (标准化)
```

## 调用关系

**依赖**: tf2, nav_msgs, sensor_msgs

**被依赖方**: nav_bringup（作为独立/组合节点启动）

## 设计意图

此模块的存在是为了**解耦**——上游里程计算法（point_lio 等）发布的话题名和坐标系可能不同，loam_interface 将它们统一转换为 Nav2 期望的标准话题名和坐标系，使更换里程计算法时只需修改此模块的配置。
