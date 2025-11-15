#!/bin/bash
# Test Script for Hexapod Leg 1 - Cascade Control System
# This script launches all necessary nodes for testing a single leg

echo "======================================================"
echo "  Hexapod Leg 1 - Cascade Control Test"
echo "======================================================"
echo ""

# Source the workspace
source install/setup.bash

echo "Launching nodes..."
echo ""

# 1. Joint State Splitter (splits /joint_states into per-leg topics)
echo "[1/6] Starting Joint State Splitter..."
ros2 run hexapod joint_state_splitter.py --ros-args \
  -r leg_1/joint_states:=/hexapod/leg_1/joint_states \
  -r leg_2/joint_states:=/hexapod/leg_2/joint_states \
  -r leg_3/joint_states:=/hexapod/leg_3/joint_states \
  -r leg_4/joint_states:=/hexapod/leg_4/joint_states \
  -r leg_5/joint_states:=/hexapod/leg_5/joint_states \
  -r leg_6/joint_states:=/hexapod/leg_6/joint_states &
SPLITTER_PID=$!
sleep 2

# 2. Inverse Position Kinematics (converts target position to joint angles)
echo "[2/6] Starting Inverse Position Kinematics..."
ros2 run hexapod inverse_position_kinematic.py --ros-args -p leg_id:=1 -r __ns:=/hexapod/leg_1 &
IK_PID=$!
sleep 1

# 3. Inverse Velocity Kinematics (computes feedforward velocities)
echo "[3/6] Starting Inverse Velocity Kinematics (Feedforward)..."
ros2 run hexapod inverse_velocity_kinematic.py --ros-args -p leg_id:=1 -r __ns:=/hexapod/leg_1 &
IVK_PID=$!
sleep 1

# 4. Position PID Controller (outer loop)
echo "[4/6] Starting Position PID Controller (Outer Loop)..."
ros2 run hexapod pid_position_controller.py --ros-args -p leg_id:=1 -r __ns:=/hexapod/leg_1 &
POS_PID=$!
sleep 1

# 5. Velocity PID Controller (inner loop with feedforward)
echo "[5/6] Starting Velocity PID Controller (Inner Loop)..."
ros2 run hexapod pid_velocity_controller.py --ros-args -p leg_id:=1 -r __ns:=/hexapod/leg_1 &
VEL_PID=$!
sleep 1

# 6. Forward Kinematics (for monitoring/verification)
echo "[6/6] Starting Forward Kinematics (Monitor)..."
ros2 run hexapod forward_position_kinematic.py --ros-args -p leg_id:=1 -r __ns:=/hexapod/leg_1 &
FK_PID=$!

echo ""
echo "======================================================"
echo "  ✅ All nodes launched successfully!"
echo "======================================================"
echo ""
echo "Control Architecture:"
echo "  Target → [IK] → [Position PID] → [Velocity PID + FF] → Effort"
echo ""
echo "Available Topics:"
echo "  Input:  /hexapod/leg_1/end_effector_target"
echo "  Output: /effort_controller_leg_1/commands"
echo "  Monitor: /hexapod/leg_1/end_effector_position"
echo ""
echo "Test Command (in another terminal):"
echo "  ros2 topic pub --once /hexapod/leg_1/end_effector_target \\"
echo "    geometry_msgs/msg/PointStamped \\"
echo "    \"{point: {x: 0.15, y: -0.1, z: -0.05}}\""
echo ""
echo "Press Ctrl+C to stop all nodes..."
echo "======================================================"

# Trap Ctrl+C and kill all background processes
trap 'echo ""; echo "Stopping all nodes..."; kill $SPLITTER_PID $IK_PID $IVK_PID $POS_PID $VEL_PID $FK_PID 2>/dev/null; exit 0' INT

# Wait for all processes
wait
