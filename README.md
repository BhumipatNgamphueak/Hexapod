# FRA333 Hexapod Kinematic Project

A ROS 2 Humble implementation of a hexapod robot with inverse kinematics, trajectory planning, gait generation, and control using both custom PID controllers and ROS 2 Joint Trajectory Controllers in Gazebo simulation.

## Table of Contents
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Simulation with Custom PID Controllers](#simulation-with-custom-pid-controllers)
  - [Simulation with Joint Trajectory Controllers](#simulation-with-joint-trajectory-controllers)
- [Testing Trajectory Controllers](#testing-trajectory-controllers)
- [Project Structure](#project-structure)
- [Controllers Comparison](#controllers-comparison)
- [Troubleshooting](#troubleshooting)

## Features

- **Inverse Kinematics**: Numerical IK solver for 6-legged hexapod robot
- **Trajectory Planning**: Smooth trajectory generation with cubic interpolation
- **Gait Generation**: Multiple gait patterns (tripod, wave, ripple)
- **Dual Control Modes**:
  - Custom cascaded PID controllers (position + velocity control with effort commands)
  - ROS 2 Joint Trajectory Controllers (position control)
- **Gazebo Simulation**: Full physics simulation with mesh rendering
- **RViz Visualization**: Real-time robot state visualization

## System Architecture

### Architecture 1: Custom PID Controllers (simulation-full.launch.py)
```
Gait Planning → Trajectory Generation → IK → Position PID → Velocity PID → Effort Controller → Gazebo
     ↓                   ↓               ↓        ↓              ↓               ↓
(Kinematic)        (Kinematic)     (Kinematic) (Control)     (Control)      (Simulation)
```

### Architecture 2: Joint Trajectory Controllers (simulation-trajectory.launch.py)
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

### Simulation with Custom PID Controllers

Launch the full simulation with custom cascaded PID controllers:

```bash
source install/setup.bash
ros2 launch hexapod_simulation simulation-full.launch.py
```

**Features:**
- Custom position and velocity PID controllers
- Effort command interface
- Gravity compensation
- Robot is fixed in air for testing (world link enabled)

**Send velocity commands:**
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

# Strafe right
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Simulation with Joint Trajectory Controllers

Launch the simulation with ROS 2 Joint Trajectory Controllers:

```bash
source install/setup.bash
ros2 launch hexapod_simulation simulation-trajectory.launch.py
```

**Features:**
- Standard ROS 2 Joint Trajectory Controllers
- Position command interface
- Spline interpolation for smooth trajectories
- Robot is fixed in air (world link enabled for mesh rendering)

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
│   │   │   ├── inverse_velocity_kinematic.py       # Velocity IK (Jacobian)
│   │   │   ├── pid_position_controller.py          # Position PID controller
│   │   │   ├── pid_velocity_controller.py          # Velocity PID controller
│   │   │   ├── trajectory_controller_bridge.py     # IK → JointTrajectory bridge
│   │   │   ├── gravity_compensation.py             # Gravity compensation
│   │   │   └── joint_state_splitter.py             # Split joint states per leg
│   │   └── CMakeLists.txt
│   │
│   ├── hexapod_description/
│   │   ├── meshes/                                 # STL mesh files
│   │   └── robot/visual/
│   │       ├── hexapod.xacro                       # URDF for PID controllers
│   │       ├── hexapod_trajectory.xacro            # URDF for trajectory controllers
│   │       ├── gazebo_full.xacro                   # Gazebo config (effort control)
│   │       ├── gazebo_trajectory.xacro             # Gazebo config (position control)
│   │       ├── hexapod_params.xacro                # Robot parameters
│   │       └── robot_material.xacro                # Visual materials
│   │
│   └── hexapod_simulation/
│       ├── config/
│       │   ├── controller_full.yaml                # Effort controller config
│       │   └── controller_trajectory.yaml          # Trajectory controller config
│       ├── launch/
│       │   ├── simulation-full.launch.py           # Launch with PID controllers
│       │   └── simulation-trajectory.launch.py     # Launch with trajectory controllers
│       └── rviz/
│           └── display.rviz                        # RViz configuration
│
└── README.md
```

## Controllers Comparison

| Feature | Custom PID Controllers | Joint Trajectory Controllers |
|---------|----------------------|---------------------------|
| **Command Interface** | Effort | Position |
| **Control Layers** | Position PID + Velocity PID | Single layer (built-in) |
| **Interpolation** | Custom cubic splines | ROS 2 spline interpolation |
| **Gravity Compensation** | Manual implementation | Not included |
| **Multi-point Trajectories** | Via trajectory planner | Native support |
| **Tuning Complexity** | High (2 PID layers) | Low (built-in) |
| **Launch File** | `simulation-full.launch.py` | `simulation-trajectory.launch.py` |

## Troubleshooting

### Mesh Not Rendering in Gazebo

**Problem:** Robot appears without visual meshes (only collision shapes visible)

**Solution:**
- Ensure `GZ_SIM_RESOURCE_PATH` includes hexapod_description package
- Verify mesh files exist in `install/hexapod_description/share/hexapod_description/meshes/`
- Check that `gazebo_trajectory.xacro` includes the Sensors plugin

### Trajectory Controller Not Accepting Commands

**Problem:** Sending trajectories doesn't move the robot

**Symptoms:**
```bash
ros2 topic pub /joint_trajectory_controller_leg_1/joint_trajectory ...
# No movement observed
```

**Solutions:**
1. Check controller is active:
   ```bash
   ros2 control list_controllers
   ```

2. Verify URDF uses position command interface (not effort)

3. Check controller configuration in `controller_trajectory.yaml`

4. Ensure robot is not in contact with ground (if world link is disabled)

### Legs Detached from Base

**Problem:** Legs appear disconnected from robot base

**Solution:**
- Hip joint positions must match between URDF and IK calculations
- Verify `hexapod_trajectory.xacro` uses hexagonal layout (not rectangular)
- Correct positions are in the URDF at lines 271-293

### Build Errors

**Problem:** Symlink installation fails

**Solution:**
```bash
rm -rf build/ install/ log/
colcon build --symlink-install
```

## Contributors

- Project maintained by FRA333 Robotics Course
- Trajectory controller implementation with Claude Code assistance

## License

This project is for educational purposes as part of FRA333 Robotics Course.

---

**Generated with Claude Code**
