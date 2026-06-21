# Robotics Lab 2026 – Homework 3
**Course:** Robotics Lab 2026
**Platform:** PX4 SITL + ROS 2 Humble + Gazebo Classic
---

# Project Overview

This project implements the required tasks for Homework 3 using PX4 SITL, ROS 2, and Gazebo.

Implemented components:

- Custom quadrotor validation
- Force-land safety mechanism
- Offboard trajectory planner
- ROS 2 bag recording
- PX4 ULog analysis
- Flight-data visualization

---

# Repository Structure

```text
ros2_ws/
├── src/
│   ├── force_land
│   ├── offboard_rl
│   └── read_rpy
│
plots/
bags/


#Main ROS 2 packages:


- force_land – Modified force-land safety node.

- offboard_rl– Offboard trajectory planner.

- read_rpy – Attitude and orientation utilities.



-------------



#Custom UAV Validation



A custom quadrotor UAV was successfully simulated in PX4 SITL.



Vehicle behavior was validated through actuator output analysis during:



* Arming

* Takeoff

* Hover

* Waypoint tracking

* Landing



The generated actuator plots confirm stable motor commands and proper thrust allocation throughout the mission.



---



#Force-Land Safety Mechanism



A safety mechanism was implemented to automatically trigger landing whenever the vehicle altitude exceeds 20 meters.



The node continuously monitors the vehicle position using PX4 telemetry messages.



When the altitude threshold is exceeded:



1. A landing command is sent to PX4.

2. The vehicle enters the landing state.

3. Landing completion is verified using touchdown detection.



The implementation uses:



* VehicleLocalPosition

* VehicleLandDetected

* VehicleCommand

* ManualControlSetpoint



This approach prevents repeated landing commands while the vehicle is already descending.



---



#Offboard Trajectory Planner



An autonomous offboard controller was developed using ROS 2.



The planner commands the UAV through a trajectory containing more than seven waypoints while maintaining continuous motion throughout the mission.



The controller publishes:



* Position setpoints

* Velocity setpoints

* Acceleration setpoints

* Yaw setpoints

* Yaw-rate setpoints



The vehicle is not allowed to stop at intermediate waypoints. Zero velocity is commanded only at the final waypoint.



This behavior satisfies the assignment requirements while ensuring smooth trajectory tracking.



---



#Build Instructions



```bash

cd ~/robotics_hw3_workspace/ros2_ws



source /opt/ros/humble/setup.bash



colcon build --symlink-install



source install/setup.bash

```



---



# Start PX4 SITL



```bash

cd ~/robotics_hw3_workspace/PX4-Autopilot



pkill -9 px4

pkill -9 gzserver

pkill -9 gzclient



PX4_SIMULATOR=gazebo-classic make px4_sitl gazebo-classic_iris

```



PX4 shell configuration:



```text

param set COM_ARM_WO_GPS 1

param set COM_RC_IN_MODE 4

param set CBRK_SUPPLY_CHK 894281



commander arm

commander takeoff

```



---



# Start XRCE-DDS Agent



```bash

MicroXRCEAgent udp4 -p 8888

```



---



# Run Force-Land Node



```bash

source ~/robotics_hw3_workspace/ros2_ws/install/setup.bash



ros2 run force_land force_land

```



---



# Run Offboard Mission



```bash

source ~/robotics_hw3_workspace/ros2_ws/install/setup.bash



ros2 run offboard_rl go_to_point

```



---



# Publish Manual Control Input



```bash

ros2 topic pub /fmu/in/manual_control_input px4_msgs/msg/ManualControlSetpoint \

"{valid: true, data_source: 1, throttle: 0.7}" \

-r 20

```



---



# Record Force-Land Experiment



```bash

ros2 bag record -o hw3_force_land_clean_v2 \

/fmu/in/manual_control_input \

/fmu/out/vehicle_local_position_v1 \

/fmu/out/vehicle_land_detected

```



---



# Record Offboard Mission



```bash

ros2 bag record -o hw3_offboard_landing_final \

/fmu/out/vehicle_local_position_v1 \

/fmu/out/vehicle_attitude \

/fmu/out/vehicle_odometry \

/fmu/out/vehicle_land_detected \

/fmu/in/trajectory_setpoint

```



---



# Generate Plots



```bash

python3 plot_force_land_clean.py



python3 plot_offboard_bag.py



python3 plot_ulog_hw3.py

```



Generated outputs:



```text

plots/kishu_quad_actuator_outputs.png

plots/force_land_altitude_land_detected.png

plots/force_land_manual_control.png

plots/force_land_manual_vs_altitude.png

plots/offboard_xy_trajectory.png

plots/offboard_altitude.png

plots/offboard_velocity.png

plots/offboard_acceleration.png

plots/offboard_yaw.png

```



---



# Results



The completed experiments demonstrate:



* Successful UAV validation in PX4 SITL.

* Stable actuator behavior during all flight phases.

* Reliable force-land activation above the safety threshold.

* Correct touchdown detection.

* Smooth multi-waypoint offboard navigation.

* Stable trajectory tracking and orientation control.

* Successful ROS 2 and PX4 integration.



The generated plots confirm that all required project objectives were achieved.



---



# Conclusion



Homework 3 was successfully completed using PX4 SITL, Gazebo Classic, and ROS 2 Humble.



The custom UAV platform, force-land safety mechanism, and offboard trajectory planner were implemented and validated through simulation experiments. Flight data were recorded using ROS 2 bags and PX4 logs, and the resulting analyses confirmed correct system behavior and successful completion of all assignment requirements.
