# 风险地图

> [返回 PROJECT_DOC.md](../PROJECT_DOC.md)
> 以下标注基于代码的结构特征分析，不断言"这是 Bug"。每一项都需要人工确认。
> 置信度: **高** = 可量化事实；**中** = 模式匹配需人工复核；**低** = 可能无害但标记以便完整覆盖。

---

## 汇总

| 类别 | 数量 |
|---|---|
| 复杂度热点 | 2 |
| 高耦合 | 1 |
| 隐式依赖 | 1 |
| 异常处理缺陷 | 3 |
| 理解缺口 | 2 |
| 硬编码密钥 | 0 |

---

## 复杂度热点（行数 >300 / 嵌套深度 >4 / 参数 >5）

### 标注 #1
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp](../modules/omni_pid_pursuit_controller.md):1-763 |
| 类型 | 复杂度热点 — 763 行；函数 `computeVelocityCommands` 含 7 级子调用链 |
| 置信度 | **高**（可量化结构指标） |
| 影响 | 控制器是自主导航的核心执行环节，Bug 直接影响机器人安全。`dynamicParametersCallback` 含 20+ 个 `if-else` 分支手工匹配参数名，新增参数极易遗漏。 |
| 建议 | 将参数更新逻辑重构为查找表+循环；将 `computeVelocityCommands` 的曲率限速、接近减速、碰撞检测三个子阶段抽取为独立的策略类。 |

### 标注 #2
| 字段 | 详情 |
|---|---|
| 文件 | [src/simulation/nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py](../modules/nav2_loopback_sim.md):57-423 |
| 类型 | 复杂度热点 — 423 行单文件；类 `LoopbackSimulator` 同时承担 cmd_vel 处理、TF 管理、LaserScan 生成、地图获取四类职责 |
| 置信度 | **高**（可量化结构指标） |
| 影响 | 修改变换逻辑可能意外影响 LaserScan 生成；无单元测试覆盖（仅存在 flake8/copyright/pep257 元测试）。 |
| 建议 | 将 `LaserScan` 生成逻辑抽取为独立的 `LaserScanGenerator` 类；添加仿真精度回归测试（至少验证直线行驶 1m 后期望位置偏差）。 |

---

## 高耦合（被 ≥10 个模块引用）

### 标注 #3
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/nav2_plugins/include/nav2_plugins/bt/nav_utils.hpp](../modules/nav2_plugins.md) |
| 类型 | 高耦合 — 所有 BT 动作节点（9+ 个）均依赖此文件中的工具函数 |
| 置信度 | **高**（统计事实） |
| 影响 | `nav_utils.hpp` 的修改会级联影响所有 BT 节点。该文件全为 `inline` 函数，修改后需重新编译整个 `nav2_plugins` 包。 |
| 建议 | 修改此文件前运行完整构建；考虑将部分纯数学函数（`squaredDistance`, `normalizeAngle`）下沉到独立的数学工具头文件。 |

---

## 隐式依赖（全局变量 / 模块级可变状态）

### 标注 #4
| 字段 | 详情 |
|---|---|
| 文件 | [src/simulation/nav2_loopback_sim/nav2_loopback_sim/loopback_simulator.py](../modules/nav2_loopback_sim.md):23-41 |
| 类型 | 隐式依赖 — `import tf_transformations` 前 monkey-patch NumPy 全局命名空间（`np.float`, `np.maximum_sctype`） |
| 置信度 | **中**（无法静态确定所有受影响的 NumPy 下游调用方） |
| 建议 | 此 patch 是为了 NumPy 2.x 兼容性，但全局修改可能影响同一进程中其他使用 NumPy 的节点。评估迁移到 `scipy.spatial.transform` 的可行性；若无法迁移，至少在 patch 前后记录日志。 |

---

## 异常处理缺陷

### 高置信度（确定性缺陷）

#### 标注 #5
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp](../modules/omni_pid_pursuit_controller.md):264 |
| 类型 | 异常处理缺陷 — 碰撞检测触发时直接 `throw PlannerException`，但上层调用方无针对碰撞的恢复策略选择逻辑 |
| 置信度 | **高**（碰撞后机器人完全停止，无降级策略） |
| 建议 | 确认 Nav2 的 behavior_server 是否能正确捕获此异常并触发 recovery 行为。若不能，考虑改为返回零速 + 发布碰撞警告话题，而非抛异常。 |

#### 标注 #6
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp](../modules/omni_pid_pursuit_controller.md):452-456 |
| 类型 | 异常处理缺陷 — 路径点在代价地图外时 `isCollisionDetected` 返回 `false`（原注释掉的警告已说明风险），可能让机器人在无代价地图覆盖的区域盲目行驶 |
| 置信度 | **高**（静默跳过碰撞检测） |
| 建议 | 当路径点不在代价地图中时，至少发出 WARN 级别日志。考虑增加参数 `allow_out_of_costmap_motion` 让用户显式选择行为。 |

### 低置信度（可能无害）

#### 标注 #7
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp](../modules/omni_pid_pursuit_controller.md):275 |
| 类型 | 异常处理缺陷 — `setSpeedLimit` 未实现，仅打印 WARN 日志 |
| 置信度 | **低**（项目可能有外部速度限制机制；当前未实现可能是设计选择而非遗漏） |
| 建议 | 若外部无其他速度限制机制，确认此设计的充分性。否则实现实际的速度限制逻辑。 |

---

## 理解缺口（意图无法确定的代码段）

### 标注 #8
| 字段 | 详情 |
|---|---|
| 文件 | [src/navigation/nav2_plugins/src/bt/action/select_patrol_path.cpp](../modules/nav2_plugins.md):62 |
| 类型 | 理解缺口 — `patrol_preview_points_` 被 `max(2, patrol_preview_points_)` 强制下限为 2，即使配置文件中设为 0 或 1。无法确定这是有意的最小预览逻辑还是参数验证不完整。 |
| 置信度 | **高**（代码行为与配置意图可能存在分歧） |
| 建议 | 查阅原始设计文档或 `git blame` 确认此 `max(2,...)` 的业务原因。若为参数校验，应加注释说明。 |

### 标注 #9
| 字段 | 详情 |
|---|---|
| 文件 | [src/interfaces/robot_interfaces/msg/Models.msg](../modules/interfaces.md) |
| 类型 | 理解缺口 — `string[5] models` 固定数组长度 5，但项目中无文档解释为何是 5 种型号、是哪 5 种。当前 `RobotStateInfo` 引用此消息但代码库中未检测到发布/订阅方。 |
| 置信度 | **高**（消息定义与其实际使用之间的关联不明确） |
| 建议 | 确认 `robot_interfaces` 的生产者和消费者。若此接口尚未投入使用，考虑标记为 `@deprecated` 或补充使用文档。 |
