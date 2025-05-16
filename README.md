# Fastening Project

# Installation
## Prerequisites
- Ubuntu 20.04 or 22.04 (recommended)
- ROS2 (Foxy, Galactic, Humble, or Rolling)
- C++17 compiler
- Python 3.8+ with pip

## System Dependencies
Install ROS2 and its dependencies by following the official ROS2 installation guide:

https://docs.ros.org/en/rolling/Installation.html

Make sure rosdep is initialized and updated:

```bash
sudo rosdep init
rosdep update
```
Install OMPL ROS2 package and other dependencies:
```bash
sudo apt install ros-<your_ros2_distro>-ompl ros-<your_ros2_distro>-rclcpp ros-<your_ros2_distro>-tf2-ros
```
Replace <your_ros2_distro> with your ROS2 distribution name (e.g., humble).


## Python Dependencies
Install Python packages required for simulation and visualization:
```bash
pip install -r requirements.txt
```

## System Design

ROS2 Nodes & Packages
- State Estimation & Kinematics
Uses tf2_ros for coordinate transformations.
Integrates FK/IK solvers (e.g., trac_ik, ik_fast).

- Motion Planning Module
Global Planning: OMPL-based path planning.
(TBD) Local Planning: Custom controllers for reactive adjustments.
(TBD) Constraint Handling: Ensures collision-free and feasible paths

- Controller 
Executes the motion by publishing joint commands

- Simulator Integration
MuJoCo ROS2 bridge for real-time physics simulation.
Synchronizes with ROS2 topics for visualization.

# Build & Run Instructions
## Build
From the root of your ROS2 workspace:

```bash
cd robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Source your workspace
```bash
source install/setup.bash
```
## Run Motion Planning Nodes
Open one terminal:
```bash
ros2 run planning motion_server
```
Open another terminal:
```bash
ros2 run planning motion_client
```