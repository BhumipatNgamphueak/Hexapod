# Data Logger Usage Guide

## Overview
The data logger records all hexapod data to CSV files for analysis.

## What Gets Logged

### Task 1: Per-Leg Data (6 CSV files)
Each leg gets its own CSV file: `leg_1_YYYYMMDD_HHMMSS.csv` to `leg_6_YYYYMMDD_HHMMSS.csv`

**Columns:**
- `timestamp` - Time in seconds
- `joint_hip_pos`, `joint_knee_pos`, `joint_ankle_pos` - Joint positions (rad)
- `joint_hip_vel`, `joint_knee_vel`, `joint_ankle_vel` - Joint velocities (rad/s)
- `joint_hip_target`, `joint_knee_target`, `joint_ankle_target` - Joint target positions (rad)
- `joint_hip_target_vel`, `joint_knee_target_vel`, `joint_ankle_target_vel` - Joint target velocities (rad/s)
- `ee_pos_x`, `ee_pos_y`, `ee_pos_z` - End effector position (m)
- `ee_vel_x`, `ee_vel_y`, `ee_vel_z` - End effector velocity (m/s)
- `ee_target_x`, `ee_target_y`, `ee_target_z` - End effector target position (m)

### Task 2: Base Link Data (1 CSV file)
File: `base_link_YYYYMMDD_HHMMSS.csv`

**Columns:**
- `timestamp` - Time in seconds
- `pos_x`, `pos_y`, `pos_z` - Position relative to reference (0, 0, 0.5) in meters
- `vel_x`, `vel_y`, `vel_z` - Velocity (m/s)
- `distance_from_ref` - Euclidean distance from reference point (m)

## Usage

### Method 1: Run Standalone (Recommended)

```bash
# Source your workspace
source install/setup.bash

# Run the data logger
ros2 run hexapod data_logger.py

# Stop logging with Ctrl+C
```

**Parameters:**
```bash
# Change log rate (default 50 Hz)
ros2 run hexapod data_logger.py --ros-args -p log_rate:=100.0

# Change log directory (default ~/hexapod_logs)
ros2 run hexapod data_logger.py --ros-args -p log_directory:=/path/to/logs

# Change reference position (default [0.0, 0.0, 0.5])
ros2 run hexapod data_logger.py --ros-args -p reference_position:=[0.0, 0.0, 1.0]
```

### Method 2: Add to Launch File

Add to your `simple.launch.py`:

```python
# Data Logger (optional - for data collection)
data_logger = Node(
    package='hexapod',
    executable='data_logger.py',
    name='data_logger',
    namespace='hexapod',
    output='screen',
    parameters=[{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'num_legs': 6,
        'log_rate': 50.0,
        'log_directory': os.path.expanduser('~/hexapod_logs'),
        'reference_position': [0.0, 0.0, 0.5]
    }]
)

nodes_to_launch.append(TimerAction(
    period=5.0,  # Start after other nodes
    actions=[data_logger]
))
```

## Output Location

By default, CSV files are saved to:
```
~/hexapod_logs/
├── leg_1_20251128_143025.csv
├── leg_2_20251128_143025.csv
├── leg_3_20251128_143025.csv
├── leg_4_20251128_143025.csv
├── leg_5_20251128_143025.csv
├── leg_6_20251128_143025.csv
└── base_link_20251128_143025.csv
```

## Tips

1. **Start logging AFTER simulation is stable** (2-3 seconds)
2. **Stop with Ctrl+C** - files are automatically flushed and closed
3. **Large files** - Logging at 50 Hz for 60 seconds = ~3000 rows per file
4. **Analysis** - Use Python pandas, MATLAB, or Excel to analyze CSV files

## Example Analysis (Python)

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load leg 1 data
df = pd.read_csv('~/hexapod_logs/leg_1_20251128_143025.csv')

# Plot hip joint tracking
plt.figure()
plt.plot(df['timestamp'], df['joint_hip_target'], label='Target')
plt.plot(df['timestamp'], df['joint_hip_pos'], label='Actual')
plt.xlabel('Time (s)')
plt.ylabel('Hip Position (rad)')
plt.legend()
plt.title('Leg 1 Hip Joint Tracking')
plt.show()

# Calculate tracking error
df['hip_error'] = df['joint_hip_target'] - df['joint_hip_pos']
print(f"RMS Error: {(df['hip_error']**2).mean()**0.5:.4f} rad")
```

## Troubleshooting

**No data logged:**
- Check that all nodes are running: `ros2 node list`
- Check topics exist: `ros2 topic list | grep leg_1`
- Verify TF is working: `ros2 run tf2_ros tf2_echo world base_link`

**Missing columns (zeros):**
- `joint_targets` requires PID controller running
- `ee_position` requires IK/FK nodes running
- `base_link` data requires TF tree (world → base_link)

**Permission denied:**
- Make sure log directory is writable
- Default `~/hexapod_logs` should work automatically
