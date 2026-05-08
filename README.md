# PX4 Robotics Lab Homework 3 — Fly Your Drone

GitHub repository: https://github.com/K-ishu/kishu_squad

## Project Contents

This repository contains the implementation for Robotics Lab Homework 3.

### 1. Custom Multi-Rotor UAV

A custom UAV model named `kishu_quad` was created for PX4/Gazebo simulation.

Files:
- `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/kishu_quad/`
- `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4010_gazebo-classic_kishu_quad`

The UAV was launched in PX4 SITL and actuator outputs were extracted from PX4 ULog.

Plot:
- `plots/kishu_quad_actuator_outputs.png`

### 2. Force Land Node Modification

The `force_land_node.cpp` was modified so that once the landing procedure is triggered above 20 m, it cannot be triggered again until the vehicle has actually landed.

The implementation uses:

- `VehicleLocalPosition`
- `VehicleLandDetected`
- `VehicleCommand`

Relevant package:
- `ros2_ws/src/aerial_robotics/ros2_ws/src/force_land`

Plot:
- `plots/force_land_manual_vs_altitude.png`
- `plots/force_land_altitude_land_detected.png`

### 3. Offboard Trajectory Planner

An offboard trajectory planner was implemented using 7+ waypoints.

Features:
- Offboard mode activation
- Arm command
- Smooth Catmull-Rom trajectory
- Position, velocity, acceleration, and yaw setpoints
- No stop at intermediate waypoints
- Zero velocity only at the final waypoint

Relevant package:
- `ros2_ws/src/aerial_robotics/ros2_ws/src/offboard_rl`

Plots:
- `plots/offboard_xy_trajectory.png`
- `plots/offboard_altitude.png`
- `plots/offboard_yaw.png`
- `plots/offboard_velocity.png`
- `plots/offboard_acceleration.png`

## Main Commands

Build ROS2 workspace:

```bash
cd ~/robotics_hw3_workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
#Run custom UAV in PX4 SITL:
```cd ~/robotics_hw3_workspace/PX4-Autopilot
PX4_SYS_AUTOSTART=4010 make px4_sitl gazebo-classic_kishu_quad
```
#Run force land node:
```cd ~/robotics_hw3_workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run force_land force_land
```
#Run offboard trajectory planner:
```cd ~/robotics_hw3_workspace/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run offboard_rl go_to_point
```
