# Cylinder Platform for Hexapod Robot

## Overview
A cylindrical platform has been added to the Gazebo simulation for the hexapod robot to walk on.

## Platform Specifications

### Dimensions
- **Radius**: 1.0 meter (2.0m diameter)
- **Height**: 0.1 meter (10cm thick)
- **Position**: Center at (0, 0, 0.05) - platform top at z=0.1m

### Properties
- **Static**: Yes (platform doesn't move)
- **Friction coefficient**: 1.0 (good grip for walking)
- **Material**: Gray metallic appearance

### Robot Spawn Position
- Robot spawns at **z=0.15m** (on top of platform)
- Platform provides a stable walking surface

## Files Created/Modified

### New Files
1. `src/hexapod_simulation/models/cylinder_platform/model.config`
   - Model metadata and description

2. `src/hexapod_simulation/models/cylinder_platform/model.sdf`
   - SDF definition with geometry, physics, and visual properties

### Modified Files
1. `src/hexapod_simulation/launch/simulation-full.launch.py`
   - Added `spawn_platform` node to spawn the cylinder before the robot
   - Changed robot spawn height from z=0.1 to z=0.15

2. `src/hexapod_simulation/CMakeLists.txt`
   - Added `models` directory to install list

## Usage

Launch the simulation as normal:
```bash
source install/setup.bash
ros2 launch hexapod_simulation simulation-full.launch.py
```

The cylinder platform will automatically spawn, and the robot will be placed on top of it.

## Customization

### Change Platform Size
Edit `src/hexapod_simulation/models/cylinder_platform/model.sdf`:
```xml
<cylinder>
  <radius>1.5</radius>  <!-- Change this (in meters) -->
  <length>0.2</length>  <!-- Change height (in meters) -->
</cylinder>
```

### Change Platform Color
Edit the `<material>` section in model.sdf:
```xml
<material>
  <ambient>R G B A</ambient>   <!-- RGBA values 0-1 -->
  <diffuse>R G B A</diffuse>
  <specular>R G B A</specular>
</material>
```

Examples:
- Red: `<ambient>1.0 0.0 0.0 1</ambient>`
- Blue: `<ambient>0.0 0.0 1.0 1</ambient>`
- Green: `<ambient>0.0 1.0 0.0 1</ambient>`

### Change Platform Position
Edit launch file `simulation-full.launch.py`:
```python
arguments=[
    '-file', os.path.join(sim_pkg, 'models', 'cylinder_platform', 'model.sdf'),
    '-name', 'cylinder_platform',
    '-allow_renaming', 'true',
    '-x', '0.0',    # X position
    '-y', '0.0',    # Y position
    '-z', '0.0'     # Z position (bottom of platform)
]
```

### Adjust Robot Height
If you change platform height, adjust robot spawn height accordingly:
```python
'-x', '0.0', '-y', '0.0', '-z', '0.15'  # Should be platform_top + 0.05
```

## Technical Details

### Collision Properties
- Uses ODE friction model
- Friction coefficients (mu, mu2) = 1.0 for good traction

### Visual Properties
- Gray metallic appearance
- Ambient: (0.5, 0.5, 0.5)
- Diffuse: (0.7, 0.7, 0.7)
- Specular: (0.3, 0.3, 0.3)

## Troubleshooting

**Platform not visible:**
- Check that models directory was installed: `ls install/hexapod_simulation/share/hexapod_simulation/models/`
- Verify GZ_SIM_RESOURCE_PATH includes the models directory

**Robot falls through platform:**
- Ensure platform spawns before robot (check launch order)
- Verify robot z-position is above platform top (>0.1m)
- Check collision geometry is properly defined

**Need different platform shape:**
- Replace `<cylinder>` with `<box>` or `<sphere>` in model.sdf
- Adjust dimensions accordingly

## Future Enhancements

Possible additions:
- Textured surface
- Multiple platforms at different heights
- Movable/dynamic platforms
- Stairs or ramps
- Obstacles for navigation testing
