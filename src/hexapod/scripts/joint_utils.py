#!/usr/bin/env python3
"""
Joint Utilities - Angle wrap-around and joint limit enforcement

CRITICAL: These functions prevent catastrophic PID errors and joint damage!

Example of wrap-around problem:
  target = -0.1 rad, current = 6.2 rad
  WITHOUT wrap: error = -6.3 rad → HUGE PID output!
  WITH wrap:    error = -0.083 rad → correct small adjustment
"""

import numpy as np


# Joint limits from hexapod_params.xacro
JOINT_LIMITS = {
    'hip': {'lower': -1.57, 'upper': 1.57},    # ±90°
    'knee': {'lower': -2.0, 'upper': 2.0},     # ±115°
    'ankle': {'lower': -1.57, 'upper': 1.57}   # ±90°
}


def wrap_angle(angle):
    """
    Wrap angle to [-π, π] range
    
    Critical for PID controllers to avoid huge errors from angle wrap-around!
    
    Examples:
        wrap_angle(3.5) → -2.78 (equivalent angle)
        wrap_angle(-3.5) → 2.78
        wrap_angle(6.28) → 0.0 (full rotation)
    
    Args:
        angle: Angle in radians (can be any value)
    
    Returns:
        Wrapped angle in [-π, π]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def wrap_angle_array(angles):
    """
    Wrap array of angles to [-π, π] range
    
    Args:
        angles: Array of angles in radians
    
    Returns:
        Array of wrapped angles in [-π, π]
    """
    return np.arctan2(np.sin(angles), np.cos(angles))


def angle_difference(target, current):
    """
    Compute shortest angular difference: target - current
    
    CRITICAL for PID error computation!
    
    Examples:
        angle_difference(-0.1, 6.2) → -0.083 (NOT -6.3!)
        angle_difference(3.0, -3.0) → -0.283 (shortest path)
    
    Args:
        target: Target angle (rad)
        current: Current angle (rad)
    
    Returns:
        Shortest angular difference in [-π, π]
    """
    diff = target - current
    return wrap_angle(diff)


def angle_difference_array(targets, currents):
    """
    Compute shortest angular differences for arrays
    
    Args:
        targets: Array of target angles (rad)
        currents: Array of current angles (rad)
    
    Returns:
        Array of shortest differences in [-π, π]
    """
    diffs = targets - currents
    return wrap_angle_array(diffs)


def enforce_joint_limits(angles, joint_names=['hip', 'knee', 'ankle']):
    """
    Clamp joint angles to their physical limits
    
    Args:
        angles: Array of joint angles [hip, knee, ankle]
        joint_names: Names of joints (default: ['hip', 'knee', 'ankle'])
    
    Returns:
        Clamped angles within limits
        Boolean array indicating if any limit was hit
    """
    clamped = np.copy(angles)
    limits_hit = np.zeros(len(angles), dtype=bool)
    
    for i, name in enumerate(joint_names):
        if name in JOINT_LIMITS:
            lower = JOINT_LIMITS[name]['lower']
            upper = JOINT_LIMITS[name]['upper']
            
            if clamped[i] < lower:
                clamped[i] = lower
                limits_hit[i] = True
            elif clamped[i] > upper:
                clamped[i] = upper
                limits_hit[i] = True
    
    return clamped, limits_hit


def check_joint_limits(angles, joint_names=['hip', 'knee', 'ankle'], tolerance=0.01):
    """
    Check if joint angles are within limits (with tolerance)
    
    Args:
        angles: Array of joint angles
        joint_names: Names of joints
        tolerance: Safety margin (rad)
    
    Returns:
        True if all joints within limits, False otherwise
        Array of booleans for each joint
    """
    within_limits = np.ones(len(angles), dtype=bool)
    
    for i, name in enumerate(joint_names):
        if name in JOINT_LIMITS:
            lower = JOINT_LIMITS[name]['lower'] + tolerance
            upper = JOINT_LIMITS[name]['upper'] - tolerance
            
            if angles[i] < lower or angles[i] > upper:
                within_limits[i] = False
    
    return np.all(within_limits), within_limits


def get_joint_limits_array(joint_names=['hip', 'knee', 'ankle']):
    """
    Get joint limits as arrays
    
    Returns:
        lower_limits: Array of lower limits
        upper_limits: Array of upper limits
    """
    lower = np.array([JOINT_LIMITS[name]['lower'] for name in joint_names])
    upper = np.array([JOINT_LIMITS[name]['upper'] for name in joint_names])
    return lower, upper


def compute_limit_margin(angles, joint_names=['hip', 'knee', 'ankle']):
    """
    Compute how close angles are to limits
    
    Returns:
        Array of margins (positive = within limits, negative = outside)
        Minimum margin across all joints
    """
    margins = np.zeros(len(angles))
    
    for i, name in enumerate(joint_names):
        if name in JOINT_LIMITS:
            lower = JOINT_LIMITS[name]['lower']
            upper = JOINT_LIMITS[name]['upper']
            
            margin_lower = angles[i] - lower
            margin_upper = upper - angles[i]
            margins[i] = min(margin_lower, margin_upper)
    
    return margins, np.min(margins)


# Example usage and testing
if __name__ == "__main__":
    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║                 Joint Utilities - Test & Examples                        ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝")
    print()
    
    # Test angle wrapping
    print("1. Angle Wrapping Tests:")
    print("-" * 70)
    test_angles = [3.5, -3.5, 6.28, -6.28, 0.1, -0.1]
    for ang in test_angles:
        wrapped = wrap_angle(ang)
        print(f"   wrap_angle({ang:6.2f}) = {wrapped:6.2f} rad ({np.degrees(wrapped):6.1f}°)")
    print()
    
    # Test angle difference (CRITICAL for PID!)
    print("2. Angle Difference Tests (Critical for PID):")
    print("-" * 70)
    test_cases = [
        (-0.1, 6.2, "Wrap-around case"),
        (3.0, -3.0, "Across zero"),
        (0.5, 0.4, "Normal case"),
        (-1.57, 1.57, "±90° case")
    ]
    for target, current, desc in test_cases:
        diff = angle_difference(target, current)
        naive_diff = target - current
        print(f"   {desc}:")
        print(f"      target={target:6.2f}, current={current:6.2f}")
        print(f"      Correct (wrapped): {diff:6.2f} rad")
        print(f"      WRONG (naive):     {naive_diff:6.2f} rad ❌")
        print()
    
    # Test joint limits
    print("3. Joint Limit Tests:")
    print("-" * 70)
    test_angles_array = np.array([
        [0.0, 0.0, 0.0],      # Within limits
        [1.6, 0.0, 0.0],      # Hip exceeds
        [0.0, -2.1, 0.0],     # Knee exceeds
        [-1.6, 2.1, 1.6]      # All exceed
    ])
    
    for angles in test_angles_array:
        clamped, hit = enforce_joint_limits(angles)
        within, _ = check_joint_limits(angles)
        print(f"   Original: [{angles[0]:6.2f}, {angles[1]:6.2f}, {angles[2]:6.2f}]")
        print(f"   Clamped:  [{clamped[0]:6.2f}, {clamped[1]:6.2f}, {clamped[2]:6.2f}]")
        print(f"   Limits hit: {hit}, Within limits: {within}")
        print()
    
    print("╔══════════════════════════════════════════════════════════════════════════╗")
    print("║                         Tests Complete!                                  ║")
    print("╚══════════════════════════════════════════════════════════════════════════╝")
