# Claude Workspace Guide

## Scope

This repository is a ROS 2 Humble workspace for robot navigation. Most tasks should stay inside one package unless the request is explicitly cross-package.

Primary packages:

- `src/navigation/nav_bringup`: launch entrypoints and navigation configuration
- `src/navigation/nav2_plugins`: Nav2 costmap layers, behaviors, and BT nodes
- `src/navigation/omni_pid_pursuit_controller`: omnidirectional Nav2 controller plugin
- `src/navigation/point_lio`: LiDAR-inertial odometry
- `src/navigation/livox_ros_driver2`: Livox LiDAR ROS2 driver
- `src/navigation/loam_interface`: LiDAR odometry frame adapter
- `src/navigation/sensor_scan_generation`: sensor scan synchronization
- `src/navigation/small_gicp_relocalization`: global relocalization
- `src/navigation/pointcloud_to_laserscan`: pointcloud-to-laserscan conversion
- `src/navigation/terrain_analysis`: terrain ground estimation
- `src/navigation/terrain_analysis_ext`: extended terrain mapping
- `src/navigation/ign_sim_pointcloud_tool`: simulation pointcloud format converter
- `src/navigation/teleop_twist_joy`: joystick teleop for chassis control
- `src/simulation/nav2_loopback_sim`: Nav2 loopback simulator
- `src/interfaces/robot_interfaces`: custom ROS interfaces (gimbal, robot state)
- `src/tools/pcd2pgm`: PCD to PGM map conversion
- `src/tools/rosbag2_composable_recorder`: composable rosbag recorder

Generated or noisy directories:

- `build/`
- `install/`
- `log/`
- `logs/`

Do not search those directories unless the user explicitly asks for logs or generated artifacts.

## Default Search Strategy

1. Start from the narrowest package path possible.
2. Read the nearest `README.md` before making assumptions.
3. Prefer package-level build or test commands over workspace-wide rebuilds.

## Task Intake Contract

Ask the user to provide tasks in this shape when possible:

- Goal: exactly what should change
- Scope: allowed package or file range
- Constraints: API, style, performance, hardware, or dependency limits
- Validation: the command or behavior that proves success
- Stop condition: what is out of scope

If the request mixes multiple goals, split them and complete one objective per session.

## Context Control

- One conversation should map to one objective.
- If the objective changes, start a new conversation with a short handoff summary instead of reusing a long thread.
- Do not paste large logs or full source files unless a specific excerpt is needed.
- Prefer path references, error signatures, and reproduction steps over broad background dumps.

## Verification Policy

- For navigation packages, prefer package-scoped build first.
- Verify the touched package before attempting workspace-wide validation.
- If verification requires unavailable hardware, validate what can be checked statically and state the remaining gap.
