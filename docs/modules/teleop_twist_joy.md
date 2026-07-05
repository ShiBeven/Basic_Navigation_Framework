# 模块: teleop_twist_joy

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: B: 2 / C: 5

## 职责
手柄遥操作节点——将游戏手柄（joystick）的轴/按钮输入映射为机器人底盘速度指令（Twist），支持多种控制模式，提供使能按钮安全保障。

## 关键文件

| 文件 | 层级 | 职责 |
|---|---|---|
| `include/teleop_twist_joy/teleop_twist_joy.hpp` | B | 节点类声明——完整的参数和回调签名 |
| `src/pb_teleop_twist_joy.cpp` | B | 节点核心实现 |
| `config/xbox.config.yaml` | C | Xbox 手柄按键映射配置 |
| `launch/pb_teleop_twist_joy_launch.py` | C | 启动文件 |
| `CMakeLists.txt` | C | 构建配置 |
| `package.xml` | C | 包元数据 |
| `README.md` | C | 说明文档 |

## 公共 API 参考

| 符号 | 类型 | 用途 |
|---|---|---|
| `TeleopTwistJoyNode` | `class : public rclcpp::Node` | 手柄遥操作节点 |
| `joyCallback` | `void (Joy::SharedPtr)` | 手柄消息回调——分发到控制逻辑 |
| `sendCmdVelMsg` | `void (Joy::SharedPtr, const string & which_map)` | 发送 Twist 速度指令 |
| `sendGoalPoseAction` | `void (Joy::SharedPtr, const string & which_map)` | 通过 Nav2 Action 发送导航目标 |
| `sendZeroCommand` | `void ()` | 发送零速指令（安全停止） |

## 安全机制

1. **使能按钮** (`enable_button_`): 必须按住指定按钮才发送控制指令，松开自动发零速。
2. **Turbo 按钮** (`enable_turbo_button_`): 按住时切换到高速比例映射。
3. **防重复零速** (`sent_disable_msg_`): 松开使能按钮后只发送一次零速，避免持续发布。
4. **反向控制** (`inverted_reverse_`): 支持配置反向控制（如推杆向前=机器人后退）。

## 调用关系

**依赖**: rclcpp_action, geometry_msgs, sensor_msgs, nav2_msgs, tf2

**被依赖方**: nav_bringup（通过 joy_teleop_launch.py 启动）

## 注意事项

1. `sendGoalPoseAction` 支持通过手柄按钮直接向 Nav2 发送导航目标，用于「一键导航到预设点」场景。
2. 支持同时发布 `Twist` 和 `TwistStamped` 两种格式（`publish_stamped_twist_` 参数控制）。
