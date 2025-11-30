# Tripod Gait Only - Changes Summary

## Date: November 29, 2025

## Overview
Simplified the gait system to use **TRIPOD GAIT ONLY**, removing wave and ripple gait types.

## Tripod Gait Pattern
- **Group 1 (phase 0.0)**: Legs 1, 3, 5 (alternating around hexagon)
- **Group 2 (phase 0.5)**: Legs 2, 4, 6 (opposite alternating pattern)
- **Duty factor**: 0.5 (50% stance, 50% swing)

This creates the checkerboard pattern where:
- When legs 1, 3, 5 are in swing, legs 2, 4, 6 are in stance
- When legs 1, 3, 5 are in stance, legs 2, 4, 6 are in swing

## Files Modified

### 1. `src/hexapod/scripts/state_controller.py`
**Changes:**
- Removed wave gait (type 1) logic
- Removed ripple gait (type 2) logic
- Kept only tripod gait (type 0)
- Simplified `_get_leg_phase_offset()` to always return tripod offsets
- Updated gait_configs to only include tripod
- Updated file header

**Phase offset logic:**
```python
if leg_id in [1, 3, 5]:
    return 0.0  # Group 1
else:
    return 0.5  # Group 2 (legs 2, 4, 6)
```

### 2. `src/hexapod/scripts/gait_planning.py`
**Changes:**
- Removed wave and ripple from gait_configs
- Simplified `_select_gait_type()` to always return 0 (tripod)
- Removed wave and ripple logic from `_get_leg_phase_offset()`
- Updated parameter comment
- Updated file header

## Expected Behavior

### Leg Swing Pattern
All runs should now show:
- **Legs 1, 3, 5** swinging together (synchronized)
- **Legs 2, 4, 6** swinging together (synchronized, offset by half cycle)

### Timeline Example (2s cycle time)
- t=0-1s: Legs 1,3,5 swing, Legs 2,4,6 stance
- t=1-2s: Legs 1,3,5 stance, Legs 2,4,6 swing
- t=2-3s: Legs 1,3,5 swing, Legs 2,4,6 stance
- (repeats...)

## Testing

To verify proper tripod gait:

1. **Source and launch:**
   ```bash
   source install/setup.bash
   ros2 launch hexapod_simulation simple.launch.py
   ```

2. **Monitor gait parameters:**
   ```bash
   ros2 topic echo /hexapod/gait_parameters
   # Should show: data: [0.0, ...]  (gait_type=0)
   ```

3. **Check phase info for legs:**
   ```bash
   ros2 topic echo /hexapod/leg_1/phase_info
   ros2 topic echo /hexapod/leg_2/phase_info
   # Leg 1 should have phase ~0.0
   # Leg 2 should have phase ~0.5
   ```

4. **Plot end effector targets:**
   - Legs 1, 3, 5 should show synchronized swing arcs
   - Legs 2, 4, 6 should show synchronized swing arcs offset by half cycle

## Benefits of Tripod-Only

1. **Simplicity**: Easier to debug and tune
2. **Stability**: Most stable gait for hexapods
3. **Speed**: Fastest gait pattern
4. **Code clarity**: Removed unused/complex logic

## Rebuild
Package rebuilt successfully:
```bash
colcon build --packages-select hexapod
```

## Next Steps

If you need to add other gaits back later:
1. Add gait type to `gait_configs` dictionary
2. Add case in `_get_leg_phase_offset()` with appropriate phase offsets
3. Update `_select_gait_type()` logic in gait_planning.py
