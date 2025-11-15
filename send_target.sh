#!/bin/bash
# Helper script to send target positions to leg 1

source install/setup.bash

echo "======================================================"
echo "  Sending Target Position to Hexapod Leg 1"
echo "======================================================"

# Default values (can be overridden by command line arguments)
X=${1:-0.15}
Y=${2:--0.1}
Z=${3:--0.05}

echo ""
echo "Target Position:"
echo "  X: $X m"
echo "  Y: $Y m"
echo "  Z: $Z m"
echo ""

ros2 topic pub --once /hexapod/leg_1/end_effector_target \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: 'base_link'}, point: {x: $X, y: $Y, z: $Z}}"

echo ""
echo "✅ Target sent!"
echo ""
echo "Monitor result with:"
echo "  ros2 topic echo /hexapod/leg_1/end_effector_position"
echo "  ros2 topic echo /hexapod/leg_1/joint_position_target"
echo "  ros2 topic echo /effort_controller_leg_1/commands"
echo ""
echo "======================================================"
