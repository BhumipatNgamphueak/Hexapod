#!/usr/bin/env python3
"""
Analyze gait pattern from end_effector_target z-position
This script helps diagnose the actual gait pattern being executed
"""

import numpy as np
import matplotlib.pyplot as plt

# Sample data from your graphs (approximate peak times for each leg swing)
# Format: [leg_id, swing_start_time, swing_end_time]
swing_events = [
    # First graph data (t=2.5-7)
    [1, 2.5, 3.5],
    [2, 2.5, 3.5],
    [3, 4.0, 5.5],
    [4, 4.0, 5.5],
    [5, 6.0, 7.0],
    [6, 6.0, 7.0],
    # Second graph data (t=8-13)
    [1, 8.5, 9.5],
    [2, 8.5, 9.5],
    [3, 10.0, 11.5],
    [4, 10.0, 11.5],
    [5, 12.0, 13.0],
    [6, 12.0, 13.0],
]

print("="*60)
print("GAIT PATTERN ANALYSIS")
print("="*60)

# Analyze phase relationships
print("\nSwing Event Timeline:")
for event in swing_events:
    leg_id, start, end = event
    mid = (start + end) / 2
    print(f"  Leg {leg_id}: swing at t={start:.1f}-{end:.1f}s (mid={mid:.1f}s)")

# Calculate cycle time (time between same leg swings)
print("\nCycle Time Analysis:")
leg_swings = {}
for event in swing_events:
    leg_id, start, end = event
    if leg_id not in leg_swings:
        leg_swings[leg_id] = []
    leg_swings[leg_id].append((start + end) / 2)

for leg_id in sorted(leg_swings.keys()):
    swings = leg_swings[leg_id]
    if len(swings) >= 2:
        cycle_time = swings[1] - swings[0]
        print(f"  Leg {leg_id}: cycle time ≈ {cycle_time:.1f}s")

# Calculate relative phase offsets (using first swing of each leg)
print("\nPhase Offset Analysis:")
first_swings = {}
for event in swing_events:
    leg_id, start, end = event
    if leg_id not in first_swings:
        first_swings[leg_id] = (start + end) / 2

# Use leg 1 as reference (phase 0)
ref_time = first_swings[1]
estimated_cycle = 6.0  # From observation

print(f"  Reference: Leg 1 at t={ref_time:.1f}s (phase 0.0)")
for leg_id in sorted(first_swings.keys()):
    swing_time = first_swings[leg_id]
    time_offset = swing_time - ref_time
    phase_offset = (time_offset / estimated_cycle) % 1.0
    print(f"  Leg {leg_id}: t={swing_time:.1f}s, offset={time_offset:.1f}s, phase≈{phase_offset:.3f}")

# Determine pattern
print("\n" + "="*60)
print("PATTERN IDENTIFICATION")
print("="*60)

# Group legs by phase
phase_groups = {}
for leg_id in sorted(first_swings.keys()):
    swing_time = first_swings[leg_id]
    time_offset = swing_time - ref_time
    phase_offset = round((time_offset / estimated_cycle) * 3) / 3  # Round to nearest 1/3
    if phase_offset not in phase_groups:
        phase_groups[phase_offset] = []
    phase_groups[phase_offset].append(leg_id)

print("\nGrouping by phase:")
for phase in sorted(phase_groups.keys()):
    legs = phase_groups[phase]
    print(f"  Phase {phase:.3f}: Legs {legs}")

# Compare with standard gaits
print("\n" + "-"*60)
print("COMPARISON WITH STANDARD GAITS:")
print("-"*60)

print("\nTripod (expected):")
print("  Phase 0.0: Legs [1, 3, 5]")
print("  Phase 0.5: Legs [2, 4, 6]")

print("\nWave (expected):")
print("  Phase 0.000: Leg 1")
print("  Phase 0.167: Leg 2")
print("  Phase 0.333: Leg 3")
print("  Phase 0.500: Leg 4")
print("  Phase 0.667: Leg 5")
print("  Phase 0.833: Leg 6")

print("\nRipple (expected):")
print("  Phase 0.000: Legs [1, 6]")
print("  Phase 0.333: Legs [2, 5]")
print("  Phase 0.667: Legs [3, 4]")

print("\nACTUAL OBSERVED:")
for phase in sorted(phase_groups.keys()):
    legs = phase_groups[phase]
    print(f"  Phase {phase:.3f}: Legs {legs}")

# Diagnosis
print("\n" + "="*60)
print("DIAGNOSIS:")
print("="*60)

observed_pattern = str([phase_groups[p] for p in sorted(phase_groups.keys())])
if observed_pattern == "[[1, 2], [3, 4], [5, 6]]":
    print("❌ ISSUE: Legs are swinging in PAIRS (1,2), (3,4), (5,6)")
    print("   This is NOT standard tripod, wave, or ripple gait.")
    print("   Possible causes:")
    print("   1. Custom/modified phase offsets in launch file")
    print("   2. Bug in phase offset calculation")
    print("   3. Wrong gait type being used")
    print("\n   Estimated phase offsets: [0.0, 0.0, 0.33, 0.33, 0.67, 0.67]")
else:
    print(f"Pattern: {observed_pattern}")

print("\nRECOMMENDATION:")
print("  Run: ros2 topic echo /hexapod/gait_parameters")
print("  Run: ros2 topic echo /hexapod/leg_1/phase_info")
print("  Check the launch file for custom phase offset parameters")
print("="*60)
