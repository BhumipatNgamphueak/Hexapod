# FRA333 Hexapod Kinematic Project

A ROS 2 Humble implementation of a hexapod robot with inverse kinematics, trajectory planning, gait generation, and Joint Trajectory Controllers in Gazebo simulation.

## Table of Contents
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Testing Trajectory Controllers](#testing-trajectory-controllers)
- [Project Structure](#project-structure)

## Features

- **Inverse Kinematics**: Numerical IK solver for 6-legged hexapod robot
- **Trajectory Planning**: Smooth trajectory generation with cubic interpolation
- **Gait Generation**: Multiple gait patterns (tripod, wave, ripple)
- **ROS 2 Joint Trajectory Controllers**: Standard position control interface
- **Gazebo Simulation**: Full physics simulation with mesh rendering
- **RViz Visualization**: Real-time robot state visualization

## System Architecture

### Joint Trajectory Controllers (simulation-trajectory.launch.py)
```
Gait Planning → Trajectory Generation → IK → Bridge → Joint Trajectory Controller → Gazebo
     ↓                   ↓               ↓       ↓              ↓
(Kinematic)        (Kinematic)     (Kinematic) (Bridge)    (Control)
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Sim (Harmonic)
- Python 3.10+
- NumPy, SciPy

## Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/BhumipatNgamphueak/FRA333_Kinematic_Project.git
   cd FRA333_Kinematic_Project
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Usage

Launch the simulation with ROS 2 Joint Trajectory Controllers:

```bash
source install/setup.bash
ros2 launch hexapod_simulation simulation-trajectory.launch.py
```

**Send velocity commands:**
```bash
# Move forward with tripod gait
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Strafe right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Testing Trajectory Controllers

### 1. Check Active Controllers

```bash
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster          [joint_state_broadcaster/JointStateBroadcaster]          active
joint_trajectory_controller_leg_1 [joint_trajectory_controller/JointTrajectoryController]  active
joint_trajectory_controller_leg_2 [joint_trajectory_controller/JointTrajectoryController]  active
...
```

### 2. Send Single-Point Trajectory (Leg 1)

```bash
ros2 topic pub --once /joint_trajectory_controller_leg_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['hip_joint_1', 'knee_joint_1', 'ankle_joint_1'],
  points: [
    {
      positions: [0.5, 0.5, 0.5],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

### 3. Send Multi-Point Trajectory (Leg 1)

```bash
ros2 topic pub --once /joint_trajectory_controller_leg_1/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['hip_joint_1', 'knee_joint_1', 'ankle_joint_1'],
  points: [
    {
      positions: [0.0, 0.0, 0.0],
      time_from_start: {sec: 1, nanosec: 0}
    },
    {
      positions: [0.5, 0.5, 0.5],
      time_from_start: {sec: 3, nanosec: 0}
    },
    {
      positions: [-0.5, -0.5, -0.5],
      time_from_start: {sec: 5, nanosec: 0}
    },
    {
      positions: [0.0, 0.0, 0.0],
      time_from_start: {sec: 7, nanosec: 0}
    }
  ]
}"
```

### 4. Send Trajectories to All Legs Simultaneously

```bash
for leg_id in {1..6}; do
  ros2 topic pub --once /joint_trajectory_controller_leg_${leg_id}/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
    joint_names: ['hip_joint_${leg_id}', 'knee_joint_${leg_id}', 'ankle_joint_${leg_id}'],
    points: [
      {
        positions: [0.3, 0.5, 0.5],
        time_from_start: {sec: 2, nanosec: 0}
      }
    ]
  }" &
done
```

### 5. Monitor Joint States

```bash
# Check all joint positions
ros2 topic echo /joint_states

# Monitor trajectory controller state (Leg 1)
ros2 topic echo /joint_trajectory_controller_leg_1/state
```

### 6. Test with Gait Planner

Send velocity commands to test the full kinematic chain:

```bash
# Move forward with tripod gait
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The trajectory flow:
1. Gait planner generates end-effector setpoints
2. Trajectory planner creates smooth trajectories
3. IK converts to joint positions
4. Bridge node converts to JointTrajectory messages
5. Trajectory controllers execute the motion

## Project Structure

```
FRA333_Kinematic_Project/
├── src/
│   ├── hexapod/
│   │   ├── scripts/
│   │   │   ├── gait_planning.py                    # Gait pattern generation
│   │   │   ├── trajectory_planning.py              # Smooth trajectory generation
│   │   │   ├── inverse_position_kinematic.py       # IK solver
│   │   │   ├── trajectory_controller_bridge.py     # IK → JointTrajectory bridge
│   │   │   └── joint_state_splitter.py             # Split joint states per leg
│   │   └── CMakeLists.txt
│   │
│   ├── hexapod_description/
│   │   ├── meshes/                                 # STL mesh files
│   │   └── robot/visual/
│   │       ├── hexapod_trajectory.xacro            # URDF for trajectory controllers
│   │       ├── gazebo_trajectory.xacro             # Gazebo config (position control)
│   │       ├── hexapod_params.xacro                # Robot parameters
│   │       └── robot_material.xacro                # Visual materials
│   │
│   └── hexapod_simulation/
│       ├── config/
│       │   └── controller_trajectory.yaml          # Trajectory controller config
│       ├── launch/
│       │   └── simulation-trajectory.launch.py     # Launch with trajectory controllers
│       └── rviz/
│           └── display.rviz                        # RViz configuration
│
└── README.md
```

## Contributors

- Project maintained by FRA333 Robotics Course
- Trajectory controller implementation with Claude Code assistance

## License

This project is for educational purposes as part of FRA333 Robotics Course.

---

**Generated with Claude Code**
