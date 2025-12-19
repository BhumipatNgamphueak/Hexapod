# Hexapod Robot Control System

## 🤖 Overview
A hierarchical control system for a 6-legged hexapod robot implementing **tripod gait for straight-line walking** using **ROS2** and **Gazebo** simulation.

**Scope**: This project focuses on tripod gait pattern with straight trajectory control only.

**Status**:
- ✅ Robot successfully moves with tripod gait and follows trajectory
- ⚠️ **Known Issue**: Robot doesn't move perfectly straight due to **RTF (Real Time Factor)** problems
  - Hardware limitations cause Gazebo feedback to not run in real-time
  - Non-realtime feedback affects control loop synchronization

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

### Prerequisites
- **ROS2** (Humble or later)
- **Gazebo** (Harmonic recommended)
- **Python 3** with numpy
- **ROS2 packages**: `ros_gz_sim`, `ros_gz_bridge`, `controller_manager`, `effort_controllers`

### 1. Clone Repository
```bash
git clone https://github.com/BhumipatNgamphueak/Hexapod.git
cd Hexapod
```

### 2. Build
```bash
colcon build
source install/setup.bash
```

### 3. Launch Simulation
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

### 4. Control
```bash
# Forward (Straight-line motion - Primary functionality)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# Slower forward (Recommended for testing)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

**Note**: This project implements straight-line walking only. Turning commands are not part of this implementation.

---

## 🎮 Gait Patterns

### Tripod Gait (Implemented - gait_type=0)
- **Group 1**: Legs 1, 3, 5 (phase 0.0)
- **Group 2**: Legs 2, 4, 6 (phase 0.5)
- **Duty Factor**: 0.5
- **Movement**: Straight-line trajectory only
- **Status**: ✅ Functional - follows trajectory but may drift due to RTF issues

### Other Gaits (Not Implemented)
The codebase contains logic for wave and ripple gaits, but **this project focuses exclusively on tripod gait with straight-line motion**:
- Wave Gait (gait_type=1) - Sequential leg lifting
- Ripple Gait (gait_type=2) - Three groups of paired legs

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
kinematic_project_ws/
├── src/
│   ├── hexapod/                         # Main control package (228 KB)
│   │   ├── scripts/                     # 11 Python control nodes
│   │   │   ├── gait_planning.py         # Global: Gait pattern selection
│   │   │   ├── state_controller.py      # Global: Phase management
│   │   │   ├── joint_state_splitter.py  # Global: Joint state distribution
│   │   │   ├── set_point.py             # Per-leg: Waypoint generation
│   │   │   ├── trajectory_planning.py   # Per-leg: Cubic spline interpolation
│   │   │   ├── inverse_position_kinematic.py  # Per-leg: IK solver
│   │   │   ├── inverse_velocity_kinematic.py  # Per-leg: Jacobian-based IVK
│   │   │   ├── pid_position_controller.py     # Per-leg: Position PID
│   │   │   ├── pid_velocity_controller.py     # Per-leg: Velocity PID
│   │   │   ├── forward_position_kinematic.py  # Per-leg: FK verification
│   │   │   ├── fk_pdf_model.py          # PDF kinematic library (417 lines)
│   │   │   └── data_logger.py           # Data logging utility
│   │   ├── launch/
│   │   │   └── simple.launch.py         # Main control launch file
│   │   ├── config/
│   │   │   └── hexapod_simplified_params.yaml
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── hexapod_description/             # Robot model package (20 MB)
│   │   ├── robot/visual/
│   │   │   ├── hexapod.xacro            # Main robot definition
│   │   │   ├── hexapod_params.xacro     # Robot parameters
│   │   │   └── gazebo_full.xacro        # Gazebo plugins & controllers
│   │   ├── meshes/                      # 6 STL mesh files (CAD models)
│   │   ├── config/
│   │   │   └── display.rviz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── hexapod_simulation/              # Simulation package (84 KB)
│   │   ├── launch/
│   │   │   └── simulation-full.launch.py  # Gazebo + RViz launcher
│   │   ├── config/
│   │   │   └── controller_full.yaml     # Effort controller config (100 Hz)
│   │   ├── rviz/
│   │   │   ├── display.rviz
│   │   │   └── config.rviz
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── controller/                      # Alternative controllers (48 KB)
│       ├── scripts/                     # Experimental/alternative implementations
│       ├── CMakeLists.txt
│       └── package.xml
│
├── README.md
└── .git/                                # Git repository
```

---

## 🔧 Technical Details

### Kinematic Model
- **Based on**: Canberk Suat Gurel's hexapod research paper
- **Method**: Denavit-Hartenberg (DH) parameters
- **Approach**: Analytical solutions for Forward/Inverse Kinematics
- **URDF Compensation**: Automatic offset correction between PDF frame and URDF geometry
- **Jacobian**: Numerical differentiation for velocity control
- **Singularity Handling**: Damped least squares (damping factor: 0.05)

### Robot Specifications
- **Legs**: 6 (numbered 1-6)
- **Joints per leg**: 3 (hip, knee, ankle)
- **Total DOF**: 18
- **Control Interface**: Effort (torque) control via `effort_controllers`

### Software Stack
- **Framework**: ROS2 (Robot Operating System 2)
- **Simulation**: Gazebo Harmonic
- **Bridge**: `ros_gz_bridge` for ROS2 ↔ Gazebo communication
- **Visualization**: RViz2
- **Programming**: Python 3 with NumPy
- **Build System**: CMake + ament_cmake

### Key Features
✅ Hierarchical multi-rate control (10/50/100 Hz)
✅ Cascaded PID control with feedforward
✅ Gravity compensation
✅ Multiple gait patterns (tripod/wave/ripple)
✅ Synchronized 45-node distributed system
✅ Real-time trajectory generation with cubic splines
✅ Analytical IK with singularity avoidance

---

## 📦 Package Dependencies

### ROS2 Packages (in package.xml)
- `ament_cmake` / `ament_cmake_python` - Build tools
- `rclcpp` / `rclpy` - ROS2 client libraries
- `geometry_msgs` / `std_msgs` - Message types
- `sensor_msgs` - Joint state messages
- `rosidl_default_generators` - Custom message generation

### Runtime Dependencies
- `robot_state_publisher` - URDF/TF broadcasting
- `ros_gz_sim` - Gazebo Harmonic integration
- `ros_gz_bridge` - ROS2 ↔ Gazebo communication
- `controller_manager` - Controller lifecycle
- `joint_state_broadcaster` - Joint state publishing
- `effort_controllers` - Torque control interface
- `rviz2` - 3D visualization

### Python Dependencies
- `numpy` - Numerical computations
- `rclpy` - ROS2 Python API

---

## 🐛 Troubleshooting

### Robot doesn't move
- **Wait 10 seconds** after launching Gazebo before starting control nodes
- **Wait 4 seconds** after control nodes start before sending velocity commands
- Check if all 45 nodes are running: `ros2 node list | wc -l`
- Verify Gazebo controllers: `ros2 control list_controllers`

### Robot doesn't move straight (Known Issue)
- **Root Cause**: Real Time Factor (RTF) < 1.0 due to hardware limitations
- **Symptom**: Robot follows trajectory but drifts laterally or rotates unintentionally
- **Explanation**: When Gazebo runs slower than real-time, feedback delays cause control loop desynchronization
- **Workarounds**:
  - Use a more powerful computer to maintain RTF ≈ 1.0
  - Reduce simulation complexity (disable shadows, reduce physics update rate)
  - Lower control frequencies if RTF consistently < 0.5
  - Accept minor drift as limitation of current hardware setup

### Robot falls or unstable
- Reduce velocity: Use `x: 0.05` instead of `x: 0.1`
- Lower step height in [gait_planning.py](src/hexapod/scripts/gait_planning.py)
- Increase cycle time for slower, more stable gait
- Check PID gains in position/velocity controllers

### Build errors
```bash
# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Missing dependencies
```bash
# Install ROS2 packages
sudo apt install ros-humble-ros-gz ros-humble-controller-manager \
  ros-humble-effort-controllers ros-humble-joint-state-broadcaster

# Install Python packages
pip install numpy
```

---

## 👥 Contributors

- Development Team (Prime, Athit, Oat)
- Based on research: Canberk Suat Gurel

---

## 📄 License

Educational project for FRA333 Robotics course

---


