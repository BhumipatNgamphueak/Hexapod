# Hexapod Robot Control System

## 🤖 Overview
A complete hierarchical control system for a 6-legged hexapod robot with **ROS2** and **Gazebo** simulation. This project implements advanced kinematics, multi-gait locomotion, and cascaded PID control for stable walking on a hexapod robot platform.

**Status**: ✅ **Fully Functional** - Robot walks successfully with stable tripod gait

**Course**: FRA333 Robotics
**Framework**: ROS2 Humble
**Simulation**: Gazebo Classic

---

## ✨ Features

- 🎯 **Multiple Gait Patterns**: Tripod, Wave, and Ripple gaits
- 🔧 **Complete Kinematics**: Forward and inverse position/velocity kinematics
- 🎮 **Cascaded Control**: Position PID → Velocity PID with feedforward
- 📊 **Hierarchical Architecture**: 45 nodes coordinated at different frequencies
- 🌐 **Full Simulation**: Gazebo physics with realistic robot model
- 📈 **Real-time Monitoring**: RViz visualization and data logging

---

## 📋 Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Humble Hawksbill
- **Python**: 3.10+
- **Gazebo**: Gazebo Classic 11

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  python3-colcon-common-extensions
```

### Python Dependencies
```bash
pip3 install numpy scipy
```

---

## 📊 System Architecture

### Node Structure
- **3 Global Nodes** - Coordinate all legs
- **7 Per-Leg Nodes** × 6 legs = **42 nodes**
- **Total**: 45 nodes running simultaneously

### Control Frequencies
- **Planning**: 10 Hz (Gait selection)
- **Coordination**: 50 Hz (Phase management, setpoint generation)
- **Control**: 100 Hz (IK, PID, trajectory)

---

## 🔄 Data Flow

```
User Input (/cmd_vel)
    ↓
[Gait Planner] (10 Hz)
    ↓ gait_parameters, body_velocity
[State Machine] (50 Hz)
    ↓ phase_info (per leg)
[Set Point Generator] (50 Hz)
    ↓ end_effector_setpoint
[Trajectory Generator] (100 Hz)
    ↓ target + velocity
    ├─→ [IK] ──→ joint_position_target ──→ [Position PID] ──┐
    └─→ [IVK] ─→ velocity_feedforward ─────────────────────┤
                                                            ↓
                                            [Velocity PID] ──→ effort ──→ Gazebo
```

---

## 📁 Node Details

### Global Nodes

#### 1. Gait Planner (`gait_planning.py`)
- **Frequency**: 10 Hz
- **Input**: `/cmd_vel` (user velocity commands)
- **Output**:
  - `/hexapod/gait_parameters` - [gait_type, step_height, step_length, cycle_time, duty_factor]
  - `/hexapod/body_velocity` - filtered velocity
- **Function**: Selects gait pattern (tripod/wave/ripple) based on velocity

#### 2. State Controller (`state_controller.py`)
- **Frequency**: 50 Hz
- **Input**: `gait_parameters`, `body_velocity`
- **Output**: `/hexapod/leg_{1-6}/phase_info` - [phase_type, progress, leg_phase]
- **Function**: Manages gait cycle and calculates phase for each leg

#### 3. Joint State Splitter (`joint_state_splitter.py`)
- **Frequency**: Callback-based
- **Input**: `/joint_states` (all joints from Gazebo)
- **Output**: `/hexapod/leg_{1-6}/joint_states` (per leg)
- **Function**: Distributes global joint states to individual legs

---

### Per-Leg Nodes (7 nodes × 6 legs)

#### 4. Set Point Generator (`set_point.py`)
- **Frequency**: 50 Hz
- **Input**: `phase_info`, `body_velocity`
- **Output**: `end_effector_setpoint` (discrete foot waypoints)
- **Function**: Generates swing/stance trajectory waypoints

#### 5. Trajectory Generator (`trajectory_planning.py`)
- **Frequency**: 100 Hz
- **Input**: `end_effector_setpoint`
- **Output**:
  - `end_effector_target` (smooth position)
  - `end_effector_velocity` (for feedforward)
- **Function**: Cubic spline interpolation for smooth motion

#### 6. Inverse Position Kinematics (`inverse_position_kinematic.py`)
- **Frequency**: Callback-based
- **Model**: **PDF model with URDF offset correction**
- **Input**: `end_effector_target`
- **Output**: `joint_position_target`
- **Function**: Analytical IK solution (geometric approach)

#### 7. Inverse Velocity Kinematics (`inverse_velocity_kinematic.py`)
- **Frequency**: 100 Hz
- **Model**: **PDF Jacobian (numerical differentiation)**
- **Input**: `end_effector_velocity`, `joint_states`
- **Output**: `joint_velocity_feedforward`
- **Function**: Damped least squares Jacobian inverse
- **Damping**: 0.05

#### 8. Position PID Controller (`pid_position_controller.py`)
- **Frequency**: 100 Hz
- **Input**: `joint_states`, `joint_position_target`
- **Output**: `joint_velocity_target`
- **Function**: Position control with angle wrap-around
- **Features**:
  - Angle wrap-around for error computation
  - Joint limit enforcement
  - Anti-windup with limit awareness

#### 9. Velocity PID Controller (`pid_velocity_controller.py`)
- **Frequency**: 100 Hz
- **Input**:
  - `joint_velocity_target` (from position PID)
  - `joint_velocity_feedforward` (from IVK)
  - `joint_states` (current state)
- **Output**: `/effort_controller_leg_X/commands` (torque to Gazebo)
- **Function**: Velocity control with feedforward + feedback
- **Features**: Gravity compensation, feedforward

#### 10. Forward Kinematics (`forward_position_kinematic.py`)
- **Frequency**: 100 Hz
- **Model**: **PDF model with URDF offset correction**
- **Input**: `joint_states`
- **Output**: `end_effector_position` (computed foot position)
- **Function**: Verification and monitoring

---

## 🚀 Quick Start

### 1. Clone and Build
```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd FRA333_Kinematic_Project

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Launch Simulation
```bash
# Terminal 1: Start Gazebo simulation
ros2 launch hexapod_simulation simulation-full.launch.py

# Terminal 2: Start control nodes (wait 10 seconds for Gazebo)
source install/setup.bash
ros2 launch hexapod simple.launch.py

# Terminal 3: Send velocity command (wait 4 seconds for nodes)
source install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

### 3. Control
```bash
# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

## 🎮 Gait Patterns

### Tripod Gait (Default, gait_type=0)
- **Group 1**: Legs 1, 3, 5 (phase 0.0)
- **Group 2**: Legs 2, 4, 6 (phase 0.5)
- **Duty Factor**: 0.5
- **Speed**: Fast, most stable

### Wave Gait (gait_type=1)
- Sequential leg lifting: 1→2→3→4→5→6
- **Duty Factor**: 0.83
- **Speed**: Slow, very stable

### Ripple Gait (gait_type=2)
- Three groups of paired legs
- **Duty Factor**: 0.67
- **Speed**: Medium

---

## 📊 Performance Tuning

### Current Parameters (Optimized)
```python
# Gait Parameters
step_height = 0.02          # Lower = more stable
step_length_scale = 0.3     # Shorter steps = less sway
cycle_time = 2.0            # Faster = better balance

# Position PID
Kp = [12.5, 12.5, 10.0]
Ki = [0.5, 0.5, 1.0]
Kd = [0.5, 0.5, 0.75]

# Velocity PID
Kp = [5.0, 5.0, 5.0]
Ki = [0.5, 0.5, 0.5]
Kd = [0.0, 0.0, 0.0]

# IVK Damping
damping_factor = 0.05       # Higher = smoother, less oscillation
```

---

## 🤖 Robot Geometry

### Physical Specifications
- **Total Legs**: 6 (hexagonal arrangement)
- **Joints per Leg**: 3 (Hip, Knee, Ankle)
- **Total DOF**: 18

### Link Lengths
- **L1 (Hip)**: 0.0785 m
- **L2 (Knee)**: 0.12 m
- **L3 (Ankle)**: 0.1899 m

### Leg Arrangement
```
      Front
    1       2
3               4
    5       6
      Rear
```

### Coordinate Frames
- **Body Frame**: Center of hexagon base
- **Leg Frame**: Hip joint attachment point
- **Foot Frame**: End effector position

---

## ⚙️ Configuration Files

### Main Configuration
**File**: `src/hexapod/config/hexapod_simplified_params.yaml`

Key parameters:
```yaml
gait_planner:
  cycle_time: 2.0
  step_height: 0.02
  step_length_scale: 0.3

position_controller:
  p_gains: [12.5, 12.5, 10.0]
  i_gains: [0.5, 0.5, 1.0]
  d_gains: [0.5, 0.5, 0.75]

velocity_controller:
  p_gains: [5.0, 5.0, 5.0]
  i_gains: [0.5, 0.5, 0.5]
  d_gains: [0.0, 0.0, 0.0]

inverse_velocity_kinematic:
  damping_factor: 0.05
```

### Gazebo Controllers
**File**: `src/hexapod_simulation/config/controller_full.yaml`

- 6 effort controllers (one per leg)
- Update rate: 100 Hz
- ROS2 control manager configuration

---

## 🔧 Troubleshooting

### Common Issues

#### 1. Robot Falls Immediately
**Symptoms**: Robot collapses when control starts
**Solutions**:
- Increase `cycle_time` to 2.5 or 3.0 (slower movements)
- Reduce `step_height` to 0.015 (smaller steps)
- Check if all 45 nodes are running: `ros2 node list`

#### 2. Oscillation/Shaking
**Symptoms**: Robot vibrates or shakes during stance phase
**Solutions**:
- Increase IVK `damping_factor` to 0.1
- Reduce position PID Kp gains
- Lower velocity command magnitude

#### 3. Nodes Not Starting
**Symptoms**: Launch file completes but robot doesn't move
**Solutions**:
```bash
# Check if Gazebo is ready (wait 10 seconds)
ros2 topic list | grep joint_states

# Verify control nodes are publishing
ros2 topic echo /hexapod/leg_1/end_effector_target
```

#### 4. Build Errors
**Symptoms**: `colcon build` fails
**Solutions**:
```bash
# Clean build
rm -rf build install log
colcon build

# Check Python path
which python3
# Should be /usr/bin/python3
```

#### 5. Robot Drifts Sideways
**Symptoms**: Robot walks in circles or sideways
**Solutions**:
- Verify leg phases in `state_controller.py`
- Check IK offset corrections in `fk_pdf_model.py`
- Ensure symmetrical gait parameters

### Debug Commands

```bash
# View all active nodes
ros2 node list

# Monitor specific topic
ros2 topic echo /hexapod/gait_parameters

# Check node frequency
ros2 topic hz /hexapod/leg_1/end_effector_target

# View TF tree
ros2 run tf2_tools view_frames

# Check joint states
ros2 topic echo /joint_states
```

---

## 📚 References

### Research Papers
1. **Gurel, C. S.** - "Hexapod Modelling, Path Planning and Control"
   - PDF kinematic model implementation
   - Gait pattern analysis
   - Jacobian-based velocity control

### ROS2 Documentation
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [ros2_control](https://control.ros.org/humble/index.html)
- [Gazebo Classic](http://gazebosim.org/tutorials)

### Key Concepts Implemented
- **Tripod Gait**: Statically stable locomotion
- **Inverse Kinematics**: Geometric analytical solution
- **Jacobian Inverse**: Damped least squares method
- **Cascaded PID Control**: Position → Velocity control
- **Phase-based Coordination**: Distributed gait timing

---

## 🗂️ Repository Structure

```
FRA333_Kinematic_Project/
├── src/
│   ├── hexapod/                    # Main control package
│   │   ├── scripts/
│   │   │   ├── gait_planning.py
│   │   │   ├── state_controller.py
│   │   │   ├── set_point.py
│   │   │   ├── trajectory_planning.py
│   │   │   ├── inverse_position_kinematic.py
│   │   │   ├── inverse_velocity_kinematic.py
│   │   │   ├── pid_position_controller.py
│   │   │   ├── pid_velocity_controller.py
│   │   │   ├── forward_position_kinematic.py
│   │   │   ├── joint_state_splitter.py
│   │   │   ├── fk_pdf_model.py         # PDF kinematic library
│   │   │   └── data_logger.py          # Data logging utility
│   │   └── launch/
│   │       └── simple.launch.py        # Main control launch file
│   ├── hexapod_description/        # Robot URDF/xacro
│   │   └── robot/visual/
│   │       ├── hexapod.xacro
│   │       ├── gazebo_full.xacro
│   │       └── hexapod_params.xacro
│   └── hexapod_simulation/         # Gazebo simulation
│       └── launch/
│           └── simulation-full.launch.py
├── README.md
└── rosgraph.png
```

---

## 🧪 Testing and Validation

### Verification Methods

#### 1. Kinematic Accuracy
```bash
# Monitor FK vs IK consistency
ros2 topic echo /hexapod/leg_1/end_effector_position
ros2 topic echo /hexapod/leg_1/end_effector_target
```

#### 2. Controller Performance
```bash
# Log data for analysis
ros2 run hexapod data_logger.py

# Check for steady-state error
ros2 topic echo /hexapod/leg_1/joint_states
```

#### 3. Gait Stability
- **Visual Check**: Robot should maintain body height
- **Phase Check**: Legs should lift in correct sequence
- **Balance Test**: Robot should not tip over during motion

### Performance Metrics
- **Walking Speed**: ~0.1 m/s forward (tripod gait)
- **Turning Rate**: ~0.5 rad/s
- **Stability**: Stable at cycle_time ≥ 2.0 seconds
- **Position Error**: < 2cm end effector tracking error

---

## ⚠️ Known Limitations

1. **Terrain**: Currently optimized for flat surfaces only
2. **Speed**: Limited by PID tuning (max ~0.15 m/s)
3. **Obstacle Avoidance**: No collision detection implemented
4. **Dynamic Stability**: Uses quasi-static gait (no running)
5. **Sensor Feedback**: No IMU or force sensors integrated

---

## 🔮 Future Improvements

- [ ] Implement terrain adaptation
- [ ] Add force/torque sensing
- [ ] Integrate IMU for body stabilization
- [ ] Optimize gait parameters automatically
- [ ] Add path planning and obstacle avoidance
- [ ] Implement dynamic gaits (running, jumping)
- [ ] Multi-terrain testing (slopes, stairs)

---

## 👥 Contributors

**Development Team**: FRA333 Robotics Students

**Acknowledgments**:
- Based on research by **Canberk Suat Gurel** - "Hexapod Modelling, Path Planning and Control"
- ROS2 community for excellent documentation
- Gazebo simulation framework

---

## 📄 License

Educational project for **FRA333 Robotics** course

This project is developed for educational purposes. Feel free to use and modify for learning and research.

---

## 📞 Support

For issues and questions:
1. Check the [Troubleshooting](#-troubleshooting) section
2. Review ROS2 logs: `ros2 topic list`, `ros2 node list`
3. Open an issue on the repository

---

## 🙏 Acknowledgments

Special thanks to:
- FRA333 course instructors
- ROS2 and Gazebo communities
- All contributors and testers

---


