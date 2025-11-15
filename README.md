# 🤖 FRA333 Hexapod Kinematic & Cascade Control Project

Hexapod robot control system with optimized kinematics and cascade PID control.

## ✨ Features

- ✅ **Optimized Inverse Kinematics** (1mm accuracy, 507 Hz)
- ✅ **Cascade PID Control** (Position + Velocity loops)
- ✅ **Feedforward Compensation** (Jacobian-based)
- ✅ **ROS2 Integration** (Humble compatible)
- ✅ **Gazebo Simulation** support
- ✅ **Complete Testing Suite**

---

## 🚀 Quick Start

### **1. Build Workspace**
```bash
colcon build --symlink-install
source install/setup.bash
```

### **2. Test Single Leg**
```bash
# Terminal 1: Launch control system
./test_leg1.sh

# Terminal 2: Send test target
./send_target.sh 0.15 -0.1 -0.05
```

### **3. Launch Full System (6 legs)**
```bash
ros2 launch hexapod simple.launch.py
```

### **4. With Gazebo Simulation**
```bash
# Terminal 1: Gazebo
ros2 launch hexapod_simulation simulation-full.launch.py

# Terminal 2: Control
ros2 launch hexapod simple.launch.py
```

---

## 📊 Performance

| Component | Metric | Value |
|-----------|--------|-------|
| **IK Speed** | Computation time | 1.97ms |
| **IK Accuracy** | Mean error | 1.03mm |
| **IK Frequency** | Max capable | 507 Hz |
| **Jacobian** | Error | 0.006mm |
| **Control Loop** | Rate | 100 Hz |
| **Total Latency** | End-to-end | ~2.7ms |

---

## 📁 Project Structure

```
.
├── src/
│   ├── hexapod/              # Main control package
│   │   ├── scripts/          # Python nodes
│   │   │   ├── forward_position_kinematic.py
│   │   │   ├── inverse_position_kinematic.py
│   │   │   ├── inverse_velocity_kinematic.py
│   │   │   ├── pid_position_controller.py
│   │   │   ├── pid_velocity_controller.py
│   │   │   └── joint_state_splitter.py
│   │   └── launch/
│   │       └── simple.launch.py
│   ├── hexapod_description/  # URDF/xacro models
│   └── hexapod_simulation/   # Gazebo launch files
│
├── test_leg1.sh              # Quick test script
├── send_target.sh            # Send target helper
├── test_kinematics_validation.py
├── test_optimized_ik.py
├── test_cascade_control.py
│
├── CASCADE_CONTROL_SUMMARY.md    # Detailed documentation
├── TESTING_GUIDE.md              # Testing instructions
└── README.md                     # This file
```

---

## 🎯 System Architecture

```
Target Position (x,y,z)
    ↓
[Inverse Position Kinematics] → Joint Angles [θ₁, θ₂, θ₃]
    ↓
[Position PID Controller] → Velocity Commands
    ↓                            ↓
[Velocity PID] ← [IVK Feedforward]
    ↓
Joint Efforts/Torques
    ↓
[Robot / Gazebo Simulation]
    ↓
Feedback (joint states)
```

---

## 🧪 Testing

### **Validate Kinematics:**
```bash
python3 test_kinematics_validation.py
python3 test_optimized_ik.py
```

### **Test Cascade Control:**
```bash
python3 test_cascade_control.py
```

### **Live Testing:**
```bash
# See TESTING_GUIDE.md for detailed instructions
./test_leg1.sh
./send_target.sh
```

---

## 📚 Documentation

- **CASCADE_CONTROL_SUMMARY.md** - Complete system documentation
- **TESTING_GUIDE.md** - Step-by-step testing guide
- **src/hexapod/README.md** - Package details

---

## 🛠️ Dependencies

- ROS2 Humble
- Python 3.10+
- NumPy
- Gazebo Harmonic/Garden (optional, for simulation)
- RViz2 (optional, for visualization)

---

## 🎓 Key Components

### **1. Inverse Kinematics (Optimized)**
- **Algorithm:** Numerical Jacobian + Levenberg-Marquardt
- **Optimizations:** Reduced iterations, adaptive tolerance
- **Performance:** 2.13x faster than baseline

### **2. Cascade PID Control**
- **Outer Loop:** Position PID (converts position error → velocity)
- **Inner Loop:** Velocity PID (converts velocity error → torque)
- **Enhancement:** Feedforward from Inverse Velocity Kinematics

### **3. Feedforward Compensation**
- **Method:** Jacobian-based velocity computation
- **Benefit:** Reduced tracking error, faster response
- **Accuracy:** 0.006mm Jacobian error

---

## 📖 Usage Examples

### **Example 1: Move leg to position**
```bash
ros2 topic pub --once /hexapod/leg_1/end_effector_target \
  geometry_msgs/msg/PointStamped \
  "{point: {x: 0.15, y: -0.1, z: -0.05}}"
```

### **Example 2: Monitor control loop**
```bash
ros2 topic echo /hexapod/leg_1/end_effector_position
ros2 topic echo /effort_controller_leg_1/commands
```

### **Example 3: Visualize**
```bash
rqt_plot /hexapod/leg_1/end_effector_position/point/x:y:z
```

---

## 🤝 Contributing

This is a course project for FRA333 - Kinematics.

---

## 📝 License

Educational project - FRA333 Institute of Field Robotics, KMUTT

---

## 🎉 Acknowledgments

- **Kinematics equations** from course materials
- **URDF model** from hexapod_description package
- **Control theory** based on cascade PID principles

---

## 📞 Support

For issues or questions:
1. Check **TESTING_GUIDE.md**
2. Check **CASCADE_CONTROL_SUMMARY.md**
3. Review test scripts and examples

---

**Built with ❤️ using ROS2 and Python**

**Status:** ✅ Ready for Testing & Deployment
