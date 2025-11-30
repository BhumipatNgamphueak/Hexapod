# Hexapod Robot Control System

## 🤖 Overview
A complete hierarchical control system for a 6-legged hexapod robot with **ROS2** and **Gazebo** simulation.

**Status**: ✅ **Fully Functional** - Robot walks successfully with stable tripod gait

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

### 1. Build
```bash
cd ~/Desktop/FRA333_Kinematic_Project
colcon build
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

## 👥 Contributors

- Development Team
- Based on research: Canberk Suat Gurel

---

## 📄 License

Educational project for FRA333 Robotics course

---


