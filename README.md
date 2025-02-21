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

- Trajectory Optimization: 
(TBD) Generates smooth joint-space trajectories. 

- Controller 
Executes the motion by publishing joint commands

- Simulator Integration
MuJoCo ROS2 bridge for real-time physics simulation.
Synchronizes with ROS2 topics for visualization.

3. Motion Planning Workflow
(base) brucekimrok@brucekimrok-gram:~/projects/Fastening/robot_ws$ tree -L 3
.
├── build
├── install
├── log
└── src
    ├── control
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── motion_executor_node.cpp  # Executes planned trajectories
    ├── msgs
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── msg
    │   │   ├── Trajectory.msg        # A sequence of waypoints for execution
    │   ├── srv
    │       ├── PlannerRequest.srv    # Service request for motion planning
    └── planning
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/planning
        │   ├── robot_motion_panner.hpp # Planning logic (header)
        ├── src
            ├── motion_planner_node.cpp   # Global planner using OMPL
            ├── robot_motion_planner.cpp  # Planning logic (implementation)
        ├── config
            ├── planning.yaml         # Planner settings (OMPL, constraints)
