#!/bin/bash

# Test script to verify the TF tree has proper world frame
# This script checks that the world->base_link transform exists

echo "======================================"
echo "Testing TF Tree with World Frame"
echo "======================================"
echo ""

# Source the workspace
source install/setup.bash

echo "1. Checking TF tree structure..."
echo "   Expected: world -> base_link -> [leg joints...]"
echo ""

# Wait for TF to be available
sleep 2

echo "2. Listing all TF frames:"
ros2 run tf2_ros tf2_echo world base_link &
TF_PID=$!

sleep 3
kill $TF_PID 2>/dev/null

echo ""
echo "3. Visualizing TF tree (generating PDF)..."
ros2 run tf2_tools view_frames

echo ""
echo "4. Check the generated 'frames.pdf' file to verify:"
echo "   - 'world' is the root frame"
echo "   - 'base_link' is connected to 'world'"
echo "   - All leg links are connected properly"
echo ""
echo "======================================"
echo "TF Tree Test Complete"
echo "======================================"
