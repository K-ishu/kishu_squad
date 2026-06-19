ROBOTICS LAB 2025 – HOMEWORK 3

Student: Kishu

==================================================
1. PROJECT OVERVIEW
==================================================

This homework focuses on PX4-based UAV simulation using ROS 2 and Gazebo.

The project includes:

- PX4 SITL simulation
- Custom UAV validation
- Force-Land safety implementation
- Offboard waypoint navigation
- ROS 2 bag recording
- Flight data analysis
- Result visualization

Software stack:

- PX4 Autopilot
- ROS 2 Humble
- Gazebo Classic
- Micro XRCE-DDS Agent
- Python
- Matplotlib
- ROS 2 Bag

==================================================
2. REPOSITORY STRUCTURE
==================================================

PX4-Autopilot/

ros2_ws/
 └── src/
     ├── force_land/
     ├── offboard_rl/
     └── read_rpy/

plots/

bags/

Main ROS 2 packages:

- force_land: modified force-land FSM node
- offboard_rl: offboard trajectory planner
- read_rpy: attitude reader and logging utilities

==================================================
3. CUSTOM UAV VALIDATION
==================================================

The custom quadrotor vehicle was successfully simulated inside PX4 SITL.

Actuator outputs were extracted from PX4 logs and analyzed to verify motor behavior during:

- Arming
- Takeoff
- Hover
- Waypoint tracking
- Landing

Figure 1 – Actuator Outputs

File:
plots/kishu_quad_actuator_outputs.png

The actuator outputs show normal motor arming behavior and stable thrust generation during flight. Motor commands increase during takeoff, remain stable during waypoint tracking, and decrease during landing.

==================================================
4. FORCE-LAND SAFETY LOGIC
==================================================

A safety mechanism was implemented to automatically trigger landing when the UAV altitude exceeds 20 meters.

Condition:

Altitude > 20 m

When this condition becomes true:

- Force-land mode is activated
- Landing sequence starts automatically
- Vehicle descends until touchdown

Figure 2 – Force Land Altitude and Landing Detection

File:
plots/force_land_altitude_land_detected.png

The UAV climbs above the 20 m threshold and the force-land logic is triggered. The landing detector confirms successful touchdown at the end of the mission.

Figure 3 – Manual Control Input

File:
plots/force_land_manual_control.png

The recorded manual control input remains constant during the experiment. Throttle is maintained while roll, pitch, and yaw remain close to zero.

Figure 4 – Manual Input vs Altitude

File:
plots/force_land_manual_vs_altitude.png

This figure compares throttle input and altitude evolution. The altitude increases until the safety threshold is reached, after which the force-land procedure starts.

==================================================
5. OFFBOARD TRAJECTORY PLANNING
==================================================

An autonomous offboard controller was implemented using ROS 2.

The UAV follows a trajectory containing more than seven waypoints while maintaining continuous motion throughout the path.

The trajectory planner uses Catmull-Rom spline interpolation to guarantee smooth transitions between waypoints.

Position, velocity, acceleration, yaw, and yaw rate references are continuously generated and transmitted to PX4 through the Offboard interface.

Waypoint sequence:

(0,0,5)
(5,0,5)
(7,4,6)
(3,8,6)
(-2,7,5)
(-6,3,5)
(-4,-3,6)
(0,0,5)

Figure 5 – XY Trajectory

File:
plots/offboard_xy_trajectory.png

The UAV successfully follows the desired waypoint sequence and returns near the starting location.

Figure 6 – Altitude Tracking

File:
plots/offboard_altitude.png

The altitude remains close to the desired flight level during navigation and decreases smoothly during landing.

Figure 7 – Velocity Profile

File:
plots/offboard_velocity.png

Velocity components vary according to waypoint transitions. Peaks correspond to trajectory changes and turning maneuvers.

Figure 8 – Acceleration Profile

File:
plots/offboard_acceleration.png

Acceleration remains bounded during most of the mission. Short peaks appear during aggressive trajectory transitions and landing initiation.

Figure 9 – Yaw Evolution

File:
plots/offboard_yaw.png

Yaw changes continuously during the mission to align the UAV heading with the desired trajectory direction.

==================================================
6. EXECUTION COMMANDS
==================================================

Build Workspace:

cd ~/robotics_hw3_workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

Run PX4 SITL:

cd ~/robotics_hw3_workspace/PX4-Autopilot

pkill -9 px4
pkill -9 gzserver
pkill -9 gzclient

make px4_sitl gazebo-classic

Run Force-Land:

source ~/robotics_hw3_workspace/ros2_ws/install/setup.bash

ros2 run force_land force_land

Run Offboard Mission:

source ~/robotics_hw3_workspace/ros2_ws/install/setup.bash

ros2 run offboard_rl go_to_point

Record Offboard Bag:

ros2 bag record -o hw3_offboard_landing_final \
/fmu/out/vehicle_local_position_v1 \
/fmu/out/vehicle_attitude \
/fmu/out/vehicle_odometry \
/fmu/out/vehicle_land_detected \
/fmu/in/trajectory_setpoint

Publish Manual Input:

ros2 topic pub /fmu/in/manual_control_input px4_msgs/msg/ManualControlSetpoint \
"{valid: true, data_source: 1, throttle: 0.7}" -r 20

Record Force-Land Bag:

ros2 bag record -o hw3_force_land_clean_v2 \
/fmu/in/manual_control_input \
/fmu/out/vehicle_local_position_v1 \
/fmu/out/vehicle_land_detected

Generate Plots:

python3 plot_offboard_bag.py
python3 plot_force_land_clean.py
python3 plot_ulog_hw3.py

==================================================
7. RESULTS ANALYSIS
==================================================

Analysis of the generated plots shows that the UAV maintained stable flight during all mission phases.

The actuator outputs remained balanced, indicating proper control allocation and thrust generation.

The force-land experiment confirms that the safety logic is triggered when the altitude threshold is exceeded.

The landing detector correctly identifies touchdown and terminates the landing sequence.

The trajectory tracking results show smooth position evolution, bounded acceleration, continuous velocity profiles, and gradual yaw transitions.

The UAV never stops at intermediate waypoints, satisfying the assignment requirements.

==================================================
8. CONCLUSION
==================================================

Homework 3 was successfully completed using PX4 SITL, Gazebo, and ROS 2.

A multi-waypoint offboard trajectory planner was implemented and validated.

The UAV autonomously tracked the desired trajectory while maintaining smooth velocity and acceleration profiles.

Additionally, the force-land safety mechanism was modified and tested.

Experimental results confirmed correct activation of the landing procedure and proper touchdown detection.

All requested plots were generated from recorded flight data and demonstrate successful completion of the assignment objectives.

