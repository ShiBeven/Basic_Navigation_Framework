# 模块: dependencies（第三方依赖汇总）

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: C: 120+（全部为外部/第三方代码）

## 内容

本模块汇总 `src/navigation/`、`src/tools/` 和 `src/dependencies/` 下的第三方/外部依赖包。这些包非项目作者开发，在此仅记录路径和用途。**详细分析留待阅读上游文档。**

### 导航依赖

| 包名 | 路径 | 作者 | 用途 |
|---|---|---|---|
| `livox_ros_driver2` | `src/navigation/livox_ros_driver2/` | Livox (feng) | 览沃激光雷达 ROS2 驱动，含 SDK2 和 rapidjson |
| `point_lio` | `src/navigation/point_lio/` | claydergc | 激光惯性里程计（LIO），含 IKFoM 卡尔曼滤波工具箱 |
| `pointcloud_to_laserscan` | `src/navigation/pointcloud_to_laserscan/` | Paul Bovbel | PointCloud2 ↔ LaserScan 相互转换 |
| `terrain_analysis` | `src/navigation/terrain_analysis/` | Ji Zhang | 地形可通行性分析（4 文件，含 .launch 启动） |
| `terrain_analysis_ext` | `src/navigation/terrain_analysis_ext/` | Ji Zhang | 地形分析扩展（4 文件，含 .launch 启动） |
| `ign_sim_pointcloud_tool` | `src/navigation/ign_sim_pointcloud_tool/` | Lihan Chen | Ignition Gazebo 仿真点云转换工具 |

### 工具依赖

| 包名 | 路径 | 作者 | 用途 |
|---|---|---|---|
| `rosbag2_composable_recorder` | `src/tools/rosbag2_composable_recorder/` | Bernd Pfrommer | 可组合节点形式的 bag 录制器 |

### 上游依赖

| 包名 | 路径 | 用途 |
|---|---|---|
| `BehaviorTree.ROS2` | `src/dependencies/BehaviorTree.ROS2/` | BT ROS2 包装层（bt_action_node, bt_service_node, tree_execution_server） |
| `joint_state_publisher` | `src/dependencies/joint_state_publisher/` | ROS2 关节状态发布器 + GUI |
| `sdformat_tools` | `src/dependencies/sdformat_tools/` | SDFormat 格式转换工具（sdf2urdf, xmacro） |

### 关键文件速查（仅记录入口点）

| 文件 | 包 | 用途 |
|---|---|---|
| `livox_ros_driver2/src/livox_ros_driver2.cpp` | livox | LiDAR 驱动主入口 |
| `point_lio/src/laserMapping.cpp` | point_lio | LIO 建图主循环 |
| `point_lio/src/Estimator.cpp` | point_lio | 误差状态卡尔曼滤波估计器 |
| `terrain_analysis/src/terrainAnalysis.cpp` | terrain | 地形分析主节点 |
| `terrain_analysis_ext/src/terrainAnalysisExt.cpp` | terrain_ext | 扩展地形分析主节点 |
| `btcpp_ros2_samples/src/sample_bt_executor.cpp` | BT.ROS2 | BT 执行器示例 |

## 注意事项

1. **terrain_analysis 作者为 Ji Zhang**（LIO-SAM 作者），代码高度专业化，修改需谨慎。
2. **point_lio 使用 IKFoM**（迭代卡尔曼滤波 on Manifold），是 LIO 领域的先进算法实现。
3. **livox_ros_driver2 捆绑了 rapidjson**（`3rdparty/rapidjson/`），升级时需确认与系统版本的兼容性。
4. **sdformat_tools 提供 `sdf2urdf.py`** 和 `xmacro4sdf.py`，可在 URDF/SDF 格式间转换。
