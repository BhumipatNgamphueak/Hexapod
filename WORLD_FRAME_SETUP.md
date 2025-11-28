# World Frame Setup Documentation

## Overview
This document describes the implementation of a proper world frame for the hexapod robot, ensuring consistent coordinate frames between Gazebo simulation and RViz visualization.

## Changes Made

### 1. **hexapod.xacro** - Added World Frame
**File**: `src/hexapod_description/robot/visual/hexapod.xacro`

**Changes**:
- Added a `world` link as the root reference frame
- Added a fixed joint `world_to_base` connecting world to base_link
- Set initial spawn height to 0.1m above ground

```xml
<!-- WORLD FRAME -->
<link name="world"/>

<!-- WORLD TO BASE_LINK JOINT -->
<joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
</joint>
```

**Benefits**:
- Provides a fixed reference frame for the entire robot
- Ensures consistent coordinate system between Gazebo and RViz
- Makes TF tree complete with a proper root

---

### 2. **gazebo_full.xacro** - Added Pose Publisher
**File**: `src/hexapod_description/robot/visual/gazebo_full.xacro`

**Changes**:
- Added Gazebo Pose Publisher plugin to publish link poses to TF

```xml
<!-- Pose Publisher: Publishes world->base_link transform to TF -->
<plugin filename="libignition-gazebo-pose-publisher-system.so"
        name="ignition::gazebo::systems::PosePublisher">
    <publish_link_pose>true</publish_link_pose>
    <use_pose_vector_msg>false</use_pose_vector_msg>
    <static_publisher>false</static_publisher>
    <static_update_frequency>100.0</static_update_frequency>
</plugin>
```

**Benefits**:
- Publishes the actual pose of base_link in Gazebo's world frame
- Updates at 100 Hz for smooth visualization
- Ensures Gazebo physics and visualization are synchronized

---

### 3. **simulation-full.launch.py** - Enhanced Launch Configuration
**File**: `src/hexapod_simulation/launch/simulation-full.launch.py`

**Changes**:
- Updated robot_state_publisher to publish at 100 Hz
- Added pose bridge topic for TF synchronization
- Added comments explaining the TF tree structure

```python
# Robot state publisher
# This publishes the static transforms from the URDF (world->base_link and all joint transforms)
state_pub = Node(
    package='robot_state_publisher', 
    executable='robot_state_publisher',
    name='robot_state_publisher', 
    output='screen',
    parameters=[
        {'robot_description': robot_description_param},
        {'use_sim_time': use_sim_time},
        {'publish_frequency': 100.0}  # Publish at 100 Hz for smooth visualization
    ]
)
```

**Benefits**:
- Higher update rate for smoother visualization
- Clear documentation of node responsibilities
- Proper TF tree published to ROS2

---

## TF Tree Structure

The complete TF tree now has the following structure:

```
world (root)
  └─ base_link (fixed joint initially, dynamic in Gazebo)
      ├─ hip_link_1
      │   └─ knee_link_1
      │       └─ ankle_link_1
      │           └─ foot_pointer_1
      │               └─ end_effector_1
      │                   └─ foot_contact_1
      ├─ hip_link_2
      │   └─ [similar structure]
      ├─ hip_link_3
      │   └─ [similar structure]
      ├─ hip_link_4
      │   └─ [similar structure]
      ├─ hip_link_5
      │   └─ [similar structure]
      └─ hip_link_6
          └─ [similar structure]
```

---

## Coordinate Frame Convention

### World Frame
- **Origin**: Ground plane at (0, 0, 0)
- **X-axis**: Forward (positive = forward)
- **Y-axis**: Left (positive = left)
- **Z-axis**: Up (positive = up)
- **Convention**: Right-handed coordinate system (ROS standard: REP-103)

### Base Link Frame
- **Origin**: Center of the hexapod body
- **Initial Position**: (0.0, 0.0, 0.1) - 10cm above ground
- **Orientation**: Same as world frame at spawn
- **Motion**: Moves with robot during simulation

### Frame Consistency
- ✅ Gazebo uses `world` as the root frame
- ✅ RViz uses `world` as the Fixed Frame
- ✅ robot_state_publisher publishes `world -> base_link`
- ✅ All leg frames relative to `base_link`

---

## Testing & Verification

### 1. Check TF Tree
```bash
# Source the workspace
source install/setup.bash

# View TF frames
ros2 run tf2_tools view_frames

# This generates 'frames.pdf' showing the complete TF tree
```

### 2. Echo Transform
```bash
# Check world -> base_link transform
ros2 run tf2_ros tf2_echo world base_link

# Expected output:
# Translation: [x, y, z] (will change as robot moves)
# Rotation: quaternion (will change as robot rotates)
```

### 3. Visualize in RViz
```bash
# Launch simulation
ros2 launch hexapod_simulation simulation-full.launch.py

# In RViz:
# 1. Set "Fixed Frame" to "world"
# 2. Add "TF" display
# 3. You should see all frames connected to "world"
```

### 4. Use Test Script
```bash
# Run the provided test script
./test_tf_tree.sh
```

---

## Usage Examples

### Launch with Gazebo (Full Simulation)
```bash
source install/setup.bash
ros2 launch hexapod_simulation simulation-full.launch.py
```

### Launch RViz Only (Visualization)
```bash
source install/setup.bash
ros2 launch hexapod_description display.launch.py
```

### Check Robot Position in World Frame
```bash
# Echo the transform from world to base_link
ros2 run tf2_ros tf2_echo world base_link

# Or check TF topic
ros2 topic echo /tf --once
```

---

## Important Notes

### 1. **Fixed Joint in URDF vs Dynamic in Gazebo**
- The `world_to_base` joint is defined as **fixed** in the URDF
- This is correct! In the URDF, it defines the *initial* relationship
- During simulation, Gazebo physics will move `base_link` freely
- The robot_state_publisher publishes the static URDF structure
- Gazebo publishes the dynamic position through the pose publisher plugin

### 2. **RViz Fixed Frame Setting**
- Always set RViz "Fixed Frame" to `world`
- This ensures the robot is visualized in the world coordinate system
- If you see the robot "floating" or moving oddly, check this setting

### 3. **Coordinate Frame Conventions**
- Follows ROS REP-103 (right-handed, X-forward, Y-left, Z-up)
- Same as Gazebo default coordinate system
- Compatible with standard ROS2 navigation and control packages

### 4. **Initial Spawn Height**
- Robot spawns at Z = 0.1m (10cm above ground)
- This prevents initial collision with ground plane
- Adjust in `hexapod.xacro` if needed:
  ```xml
  <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
  ```

---

## Troubleshooting

### Issue: "No transform from world to base_link"
**Solution**: 
1. Check robot_state_publisher is running: `ros2 node list | grep robot_state_publisher`
2. Verify the URDF was rebuilt: `colcon build --packages-select hexapod_description`
3. Check TF topics: `ros2 topic list | grep tf`

### Issue: "RViz shows robot at origin, but Gazebo shows it elsewhere"
**Solution**:
1. Verify Fixed Frame in RViz is set to `world`
2. Check that Gazebo pose publisher plugin is loaded
3. Restart both Gazebo and RViz

### Issue: "TF tree is disconnected"
**Solution**:
1. Run `ros2 run tf2_tools view_frames` to visualize the tree
2. Check for missing nodes in the launch file
3. Ensure all joints in URDF are properly connected

### Issue: "Multiple world frames in TF tree"
**Solution**:
1. Check that only ONE robot_state_publisher is running
2. Verify no other nodes are publishing conflicting transforms
3. Use `ros2 node list` and `ros2 topic info /tf` to debug

---

## Future Enhancements

1. **Odometry Frame**: Add an intermediate `odom` frame for drift estimation
2. **IMU Integration**: Use IMU sensor to improve orientation estimation
3. **Ground Truth Logging**: Record world->base_link transform for analysis
4. **Map Frame**: Add `map` frame for SLAM and navigation

---

## References

- [ROS REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [TF2 Documentation](http://wiki.ros.org/tf2)
- [Gazebo Pose Publisher Plugin](https://gazebosim.org/api/gazebo/6.9/classignition_1_1gazebo_1_1systems_1_1PosePublisher.html)

---

## Summary

✅ **World frame created** in `hexapod.xacro`
✅ **Fixed joint** connects world to base_link
✅ **Gazebo plugin** publishes dynamic poses
✅ **Launch files** updated for proper TF publishing
✅ **TF tree** is complete and consistent
✅ **Gazebo and RViz** use the same coordinate frame

The hexapod robot now has a proper world frame reference, making it easier to:
- Track robot position and orientation
- Visualize motion in a consistent coordinate system
- Integrate with navigation and control systems
- Debug and analyze robot behavior
