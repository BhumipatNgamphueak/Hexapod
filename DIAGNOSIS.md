# Joint States Publishing Issue - Diagnosis Report

## Problem Statement
The cascade PID control system cannot work because `/joint_states` is only publishing at **0.2 Hz** instead of the expected **100 Hz**.

## Investigation Results

### Test 1: Verified Publishing Rate
```
Command: python3 test_joint_states_rate.py
Result: 2 messages in 10 seconds = 0.2 Hz ❌
Expected: ~1000 messages in 10 seconds = 100 Hz ✅
```

### Test 2: Configuration Check
```
controller_manager update_rate: 100 Hz ✅
joint_state_broadcaster update_rate: 0 Hz (uses controller_manager rate)
joint_state_broadcaster status: ACTIVE ✅
```

### Test 3: Effort Command Test
```
Sent effort commands at 10 Hz
Result: Still only latched message received ❌
Conclusion: controller_manager update loop NOT running
```

## Root Cause

**The `gz_ros2_control` plugin's update loop is not being triggered by Gazebo.**

In ros2_control + Gazebo integration:
1. Gazebo physics engine runs simulation steps
2. On each physics step, `gz_ros2_control` plugin is called
3. Plugin calls `controller_manager->update()`
4. controller_manager reads hardware (joint states) and writes commands
5. `joint_state_broadcaster` publishes updated joint states

**Current behavior**: Only steps 1-2 happen, step 3+ don't execute at 100 Hz

## Possible Causes

1. **Gazebo empty.sdf world** - No physics configuration
2. **Plugin not properly connected** to simulation loop
3. **Hold joints parameter** - Might prevent updates when joints don't move
4. **Gazebo-ROS2 bridge** misconfiguration

## Log Evidence

```
[WARN] [gz_ros2_control]: Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
```
This shows the plugin IS loaded and configured, but may not be triggering updates properly.

```
[INFO] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
```
Broadcaster is configured to publish all joints.

## Impact on Cascade Control

```
Current Flow:
Target → [IK] ✅ → [Position PID] ✅ → [Velocity PID] ❌ BLOCKED

Velocity PID Status:
- Receives velocity targets: ✅ (hundreds of messages)
- Receives joint_states feedback: ❌ (only 1 latched message)
- Cannot compute velocity error without continuous feedback
- Cannot publish effort commands
```

## Recommended Solutions

### Option 1: Fix Gazebo World Configuration (RECOMMENDED)
Create a proper world file with physics enabled:
- Set physics update rate to match or exceed controller rate (≥100 Hz)
- Enable real-time factor
- Proper gravity and solver settings

### Option 2: Alternative Feedback Source
- Use `/dynamic_joint_states` directly from Gazebo bridge
- Bypass joint_state_broadcaster
- May require modifying velocity PID subscriber

### Option 3: Standalone Controller Manager
- Run controller_manager as separate node with explicit timer
- Not ideal for simulation - may cause sync issues

## Next Steps

1. Create proper Gazebo world file with physics configuration
2. Test if this enables 100 Hz publishing
3. If not, investigate gz_ros2_control plugin version/compatibility
4. Consider filing bug report if configuration is correct but still fails
