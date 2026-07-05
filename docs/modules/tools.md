# 模块: tools（工具集）

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: B: 3 / C: 11

## 内容

### pcd2pgm — PCD 点云转栅格地图

#### 职责
将 PCD 格式的三维点云转换为 ROS2 OccupancyGrid 栅格地图，支持直通滤波、半径离群点剔除和坐标系变换。

#### 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/pcd2pgm/pcd2pgm.hpp` | B | 节点类声明 |
| `src/pcd2pgm.cpp` | B | 核心实现 |
| `launch/pcd2pgm_launch.py` | C | 启动文件 |
| `config/pcd2pgm.yaml` | C | 参数配置 |

#### 处理流水线

```
PCD文件 → passThroughFilter(z_min, z_max) → radiusOutlierFilter(radius, count)
       → applyTransform(odom→lidar_odom)  → setMapTopicMsg(cloud → OccupancyGrid)
       → publishCallback (定时发布)
```

#### 关键参数

| 参数 | 用途 |
|---|---|
| `thre_z_min_/thre_z_max_` | 直通滤波 Z 轴范围 |
| `thre_radius_` / `thres_point_count_` | 半径离群点剔除参数 |
| `map_resolution_` | 输出栅格地图分辨率 |
| `odom_to_lidar_odom_` | 里程计到 LiDAR 里程计的坐标变换 |

### rosbag2_composable_recorder — 可组合录制工具

**外部依赖**（Bernd Pfrommer）。支持作为 ComposableNode 加载的 ROS2 bag 录制器。包含 `composable_recorder.cpp`、`composable_recorder_node.cpp` 和 Python 录制脚本 `start_recording.py`。

| 文件 | 层级 |
|---|---|
| `include/rosbag2_composable_recorder/composable_recorder.hpp` | C |
| `src/composable_recorder.cpp` | C |
| `src/composable_recorder_node.cpp` | C |
| `src/start_recording.py` | C |

## 调用关系

**依赖 (pcd2pgm)**: PCL, nav_msgs, sensor_msgs

**被依赖方**: 独立工具，不参与导航运行时
