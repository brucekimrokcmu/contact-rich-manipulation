# Fastening Project

## Installation

To install the required dependencies, run the following command:

```bash
pip install -r requirements.txt
```

## Requirements

The `requirements.txt` file includes the following dependencies:
- **MuJoCo Simulation**: A physics engine for simulating robots.
- **Viewer Module**: For visualizing the simulation.
- **Robot Descriptions**: Definitions and models of the robots used in the simulation.
- **Inverse Kinematics**: Algorithms based on the MuJoCo physics engine to control robot movements.

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

## Motion Planning Workflow
```
brucekimrok@brucekimrok-gram:~/projects/Fastening/robot_ws/src$ tree -L 3
.
├── control
│   ├── CMakeLists.txt
│   └── package.xml
├── localization
│   ├── CMakeLists.txt
│   └── package.xml
├── mapping
│   ├── CMakeLists.txt
│   └── package.xml
├── msgs
│   ├── CMakeLists.txt
│   ├── msg
│   ├── package.xml
│   └── srv
│       └── PlanMotion.srv
└── planning
    ├── CMakeLists.txt
    ├── include
    │   └── planning
    ├── LICENSE
    ├── package.xml
    └── src
        ├── motion_client.cpp
        ├── motion_server.cpp
        └── simple_motion_planner.cpp

```

## Build & Run
Build
```
cd robot_ws 
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Open a terminal
```
source install/setup.bash
ros2 run planning motion_server
```
Open another terminal
```
source install/setup.bash
ros2 run planning motion_client
```