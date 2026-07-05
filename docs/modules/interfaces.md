# 模块: robot_interfaces（自定义消息接口）

> 父文档: [PROJECT_DOC.md](../PROJECT_DOC.md)
> 最后同步: 提交 `e61e9d6` — 2026-07-04
> 分层分布: C: 6（全部为消息定义文件，无控制流）

## 职责
定义项目自定义的 ROS2 消息类型，用于机器人状态信息传递。

## 文件清单

| 文件 | 层级 | 职责 |
|---|---|---|
| `msg/Gimbal.msg` | C | 云台状态消息定义 |
| `msg/GimbalCmd.msg` | C | 云台控制指令消息定义 |
| `msg/Models.msg` | C | 机器人型号列表（`string[5] models`） |
| `msg/RobotStateInfo.msg` | C | 综合机器人状态信息（Header + Models） |
| `CMakeLists.txt` | C | 构建配置（rosidl_generate_interfaces） |
| `package.xml` | C | 包元数据 |

## 消息结构

| 消息类型 | 字段 | 用途 |
|---|---|---|
| `Models` | `string[5] models` | 固定5槽机器人型号标识 |
| `RobotStateInfo` | `Header header` + `Models robot_models` | 带时间戳的机器人型号状态 |
| `Gimbal` | *(待确认)* | 云台状态 |
| `GimbalCmd` | *(待确认)* | 云台控制指令 |

## 调用关系

**被依赖方**: 可能被上层决策/监控节点使用（当前代码库中未检测到显式引用）

## 注意事项

1. `Models.msg` 使用固定大小的 `string[5]` 数组，不支持动态数量的机器人型号。
2. 当前接口定义较简单，可能处于早期开发阶段——实际使用范围需要确认。
