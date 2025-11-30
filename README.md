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
- **✅ Fixed**: Corrected tripod gait pattern (legs 1,3,5 and 2,4,6)

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
- **✅ Includes**: PDF_TO_URDF_OFFSET = [0.18, -0.1985, 0.1399]

#### 7. Inverse Velocity Kinematics (`inverse_velocity_kinematic.py`)
- **Frequency**: 100 Hz
- **Model**: **PDF Jacobian (numerical differentiation)**
- **Input**: `end_effector_velocity`, `joint_states`
- **Output**: `joint_velocity_feedforward`
- **Function**: Damped least squares Jacobian inverse
- **✅ Fixed**: Added proper mathematical explanation for offset handling
- **Damping**: 0.05 (increased from 0.01 for stability)

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

## 🛠️ Recent Fixes & Improvements

### PID_tuning Branch (Latest)

#### ✅ Critical Stability Fixes:
1. **Fixed Tripod Gait Pattern** (`state_controller.py`)
   - Corrected from `[1,4,5]` to `[1,3,5]` for Group 1
   - Proper alternating pattern for stable walking

2. **Removed Floating Robot Issue** (`hexapod.xacro`)
   - Removed fixed `world_to_base` joint that locked robot at z=0.1
   - Robot now properly interacts with physics

3. **Fixed IVK Consistency** (`inverse_velocity_kinematic.py`)
   - Added PDF_TO_URDF_OFFSET documentation
   - Mathematical proof that constant offset doesn't affect velocity
   - Increased damping factor: 0.01 → 0.05

4. **Optimized Gait Parameters** (`simple.launch.py`)
   - step_height: 0.03 → 0.02 (more stable)
   - step_length_scale: 0.5 → 0.3 (shorter steps)
   - cycle_time: 20.0 → 2.0 (faster, better balance)

5. **Cleaned Up Launch Files**
   - Removed duplicate `robot_state_publisher` from `simple.launch.py`
   - Removed extra pose publisher plugin from `gazebo_full.xacro`
   - Removed `/model/hexapod/pose` bridge from `simulation-full.launch.py`

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

## 📐 Kinematic Models

### PDF Model (Denavit-Hartenberg)
Based on research: *Canberk Suat Gurel — Hexapod Modelling, Path Planning and Control*

**DH Parameters**:
- Joint 1 (Hip): a=0.0785m, α=+90°, θ_offset=+90°
- Joint 2 (Knee): a=0.12m, α=0°, θ_offset=0°
- Joint 3 (Ankle): a=0.1899m, α=-90°, θ_offset=-90°

**URDF Offset Correction**:
```python
PDF_TO_URDF_OFFSET = [0.18, -0.1985, 0.1399]
```

### Coordinate Frames
- **Base Frame**: Robot body center
- **Leg Frame**: Hip joint (6 frames, one per leg)
- **PDF Frame**: DH convention frame
- **URDF Frame**: Actual robot geometry

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

## 🐛 Known Issues & Solutions

### Issue: Robot floating in air
**Solution**: ✅ Fixed - Removed world frame fixed joint

### Issue: Robot walking unstable/tilting
**Solutions**:
- ✅ Fixed gait pattern
- ✅ Optimized gait parameters
- ✅ Increased IVK damping

### Issue: Authentication failed for git push
**Solution**: Use Personal Access Token instead of password
```bash
git config --global credential.helper store
git push -u origin <branch>
# Enter username and token (not password)
```

---

## 🔧 Troubleshooting

### Robot not moving
1. Check all nodes are running: `ros2 node list`
2. Check topics: `ros2 topic list`
3. Verify joint controllers loaded: `ros2 control list_controllers`

### Robot falling over
1. Reduce step_height: `0.02 → 0.015`
2. Reduce step_length_scale: `0.3 → 0.2`
3. Increase cycle_time: `2.0 → 2.5`

### Oscillation/shaking
1. Increase IVK damping: `0.05 → 0.1`
2. Increase PID Kd values
3. Reduce velocity commands

---

## 📚 Documentation

- **GAIT_FRAMEWORK_ARCHITECTURE.md** - Detailed gait planning architecture
- **PDF_TEST_RESULTS_TH.md** - PDF model test results (Thai)
- **REFACTORING_SUMMARY.md** - Code refactoring summary
- **DATA_LOGGER_USAGE.md** - Data logging guide

---

## 👥 Contributors

- Development Team
- Based on research: Canberk Suat Gurel

---

## 📄 License

Educational project for FRA333 Robotics course

---

## 🚧 Future Improvements

- [ ] Implement terrain adaptation
- [ ] Add obstacle avoidance
- [ ] Optimize PID auto-tuning
- [ ] Add teleoperation interface
- [ ] Implement vision-based navigation

---

**Last Updated**: December 2025
**Branch**: PID_tuning
**Status**: ✅ Stable and functional
