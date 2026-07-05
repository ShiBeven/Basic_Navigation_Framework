# 模块: small_gicp_relocalization

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: A: 1 / B: 2 / C: 3

## 职责
基于 small_gicp 库的全局重定位节点——在已知全局地图（PCD）的情况下，将当前激光雷达扫描与全局地图进行 GICP 配准，发布 map→odom 校正变换，实现累计漂移的消除。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/small_gicp_relocalization/small_gicp_relocalization.hpp` | A | 节点类声明——完整的成员变量和方法签名 |
| `src/small_gicp_relocalization.cpp` | B | 节点核心实现 |
| `launch/small_gicp_relocalization_launch.py` | B | 启动文件 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | C | 包元数据 |
| `README.md` | C | 说明文档 |

## 公共 API 参考

| 符号 | 类型 | 用途 |
|---|---|---|
| `SmallGicpRelocalizationNode` | `class : public rclcpp::Node` | 重定位节点主类 |
| `registeredPcdCallback` | `void (PointCloud2::SharedPtr)` | 接收实时点云，累积并触发配准 |
| `loadGlobalMap` | `void (const string & file_name)` | 从 PCD 文件加载全局地图 |
| `performRegistration` | `void ()` | 执行 GICP 配准 |
| `publishTransform` | `void ()` | 发布校正后的 odom→map 变换 |
| `initialPoseCallback` | `void (PoseWithCovarianceStamped::SharedPtr)` | 接收初始位姿估计 |

## 算法流程

```
1. 节点初始化 → loadGlobalMap(PCD文件) → 下采样(global_leaf_size)
2. 订阅 registered_scan (PointCloud2) → 累积点云 → 下采样(registered_leaf_size)
3. 订阅 initial_pose → 作为 GICP 配准的初始猜测
4. 定时器触发(register_timer_) → performRegistration():
   ├── 构建 source/target 协方差点云 (GICPFactor)
   ├── 构建 KdTreeOMP (并行最近邻搜索)
   ├── 执行 Registration (GICP + ParallelReductionOMP)
   └── 获得 result_t_ (Eigen::Isometry3d)
5. 定时器触发(transform_timer_) → publishTransform():
   └── 发布 map→odom TF 变换
```

## 关键参数

| 参数 | 用途 |
|---|---|
| `num_threads_` | 并行线程数 |
| `num_neighbors_` | KdTree 最近邻数量 |
| `global_leaf_size_` | 全局地图下采样体素尺寸 |
| `registered_leaf_size_` | 实时扫描下采样体素尺寸 |
| `max_dist_sq_` | GICP 对应点最大距离平方 |
| `prior_pcd_file_` | 先验全局地图 PCD 文件路径 |
| `map_frame_` / `odom_frame_` / `lidar_frame_` / `base_frame_` / `robot_base_frame_` | TF 坐标系配置 |

## 调用关系

**依赖**: small_gicp (GICPFactor, KdTreeOMP, ParallelReductionOMP, Registration), PCL, tf2, geometry_msgs, sensor_msgs

**被依赖方**: nav_bringup（作为独立节点启动）

## 注意事项

1. **累积式点云**: `accumulated_cloud_` 持续收集扫描帧，而非逐帧配准——减少配准次数同时提高稳定性。
2. **定时器解耦**: 点云接收（回调）与配准执行（定时器）异步分离，避免回调阻塞。
3. **TF 线程安全**: 使用独立的 `tf_buffer_` / `tf_listener_`，不共享外部 TF 对象。
4. **结果保持**: `previous_result_t_` 保存上次配准结果，用于检测配准是否收敛或需要重置。
