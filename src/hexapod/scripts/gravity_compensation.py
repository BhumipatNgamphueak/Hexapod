#!/usr/bin/env python3
"""
Gravity Compensation Module using Pinocchio

This module computes gravity compensation torques for the hexapod robot
using the Pinocchio library for rigid body dynamics.

Key Features:
- Robust joint indexing using Pinocchio's model.getJointId()
- Proper configuration space (q) and velocity space (v) handling
- Support for all 6 legs with consistent behavior
- Clear error messages for debugging

Author: Generated for FRA333 Kinematic Project
"""

import pinocchio as pin
import numpy as np
from typing import Optional
from pathlib import Path
import os


class GravityCompensation:
    """
    Compute gravity compensation torques for a hexapod leg using Pinocchio.

    The hexapod has 6 legs, each with 3 revolute joints:
    - hip_joint_X   (X ∈ {1,2,3,4,5,6})
    - knee_joint_X
    - ankle_joint_X

    This class loads the full robot URDF, extracts the joint indices for
    the specified leg, and computes gravity torques using Pinocchio's
    computeGeneralizedGravity() function.
    """

    def __init__(self, urdf_path: Optional[str] = None, leg_id: int = 1):
        """
        Initialize gravity compensation for a single leg.

        Args:
            urdf_path: Absolute path to URDF file. If None, auto-detect from
                      common ROS2 workspace locations.
            leg_id: Leg ID from 1 to 6.

        Raises:
            ValueError: If leg_id is not in valid range [1, 6]
            FileNotFoundError: If URDF file cannot be found
            RuntimeError: If URDF loading fails or joint names not found
        """
        # Validate leg_id
        if not isinstance(leg_id, int) or leg_id < 1 or leg_id > 6:
            raise ValueError(
                f"Invalid leg_id={leg_id}. Must be an integer in range [1, 6]."
            )

        self.leg_id = leg_id

        # Auto-detect URDF path if not provided
        if urdf_path is None:
            urdf_path = self._find_urdf()

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(
                f"URDF file not found: {urdf_path}\n"
                f"Please provide a valid urdf_path or place the URDF in a standard location."
            )

        # Load robot model using Pinocchio
        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
        except Exception as e:
            raise RuntimeError(f"Failed to load URDF '{urdf_path}': {e}")

        # Define joint names for this leg (based on URDF convention)
        self.joint_names = [
            f'hip_joint_{leg_id}',
            f'knee_joint_{leg_id}',
            f'ankle_joint_{leg_id}'
        ]

        # Get joint indices in configuration (q) and velocity (v) spaces
        # For standard revolute joints: idx_q == idx_v
        self.joint_indices_q, self.joint_indices_v = self._get_joint_indices()

        # Initialize neutral configuration (all joints at zero)
        self.q0 = pin.neutral(self.model)

        # Gravity vector (standard: pointing down along -Z axis)
        # Pinocchio convention: gravity = [0, 0, -9.81]
        self.model.gravity.linear = np.array([0.0, 0.0, -9.81])

        # Print initialization summary
        print(f"✓ Gravity Compensation initialized for Leg {leg_id}")
        print(f"  URDF: {urdf_path}")
        print(f"  Model DOF (nv): {self.model.nv}, Config dim (nq): {self.model.nq}")
        print(f"  Joint names: {self.joint_names}")
        print(f"  Joint indices (q): {self.joint_indices_q}")
        print(f"  Joint indices (v): {self.joint_indices_v}")

    def _find_urdf(self) -> str:
        """
        Auto-detect URDF file location from common ROS2 workspace paths.

        Returns:
            str: Absolute path to URDF file

        Raises:
            FileNotFoundError: If URDF cannot be found in any standard location
        """
        # Try common locations in order of preference
        possible_paths = [
            # Installed package (after colcon build)
            "/home/oat/Desktop/FRA333_Kinematic_Project/install/hexapod_description/share/hexapod_description/robot/visual/hexapod.urdf",
            # Source package
            "/home/oat/Desktop/FRA333_Kinematic_Project/src/hexapod_description/robot/visual/hexapod.urdf",
            # Home directory relative path
            str(Path.home() / "Desktop/FRA333_Kinematic_Project/src/hexapod_description/robot/visual/hexapod.urdf"),
        ]

        for path in possible_paths:
            if os.path.exists(path):
                return path

        # If not found, provide helpful error message
        raise FileNotFoundError(
            "Could not auto-detect URDF file. Searched locations:\n" +
            "\n".join(f"  - {p}" for p in possible_paths) +
            "\n\nPlease provide urdf_path explicitly to GravityCompensation()."
        )

    def _get_joint_indices(self):
        """
        Get configuration (q) and velocity (v) indices for the leg's 3 joints.

        For standard revolute joints, idx_q == idx_v. For more complex joint
        types (e.g., floating base), these can differ.

        Returns:
            tuple: (indices_q, indices_v) where each is a list of 3 integers

        Raises:
            RuntimeError: If any joint name is not found in the model
        """
        indices_q = []
        indices_v = []

        for joint_name in self.joint_names:
            # Check if joint exists in model
            if not self.model.existJointName(joint_name):
                available_joints = [self.model.names[i] for i in range(1, self.model.njoints)]
                raise RuntimeError(
                    f"Joint '{joint_name}' not found in URDF model.\n"
                    f"Available joints: {available_joints}\n"
                    f"Please check your URDF and ensure joint naming follows the pattern:\n"
                    f"  hip_joint_{{leg_id}}, knee_joint_{{leg_id}}, ankle_joint_{{leg_id}}"
                )

            # Get joint ID (1-indexed in Pinocchio, 0 is universe)
            joint_id = self.model.getJointId(joint_name)
            joint = self.model.joints[joint_id]

            # Extract indices
            # idx_q: starting index in configuration vector q
            # idx_v: starting index in velocity vector v
            # For revolute joints: nq = nv = 1
            indices_q.append(joint.idx_q)
            indices_v.append(joint.idx_v)

        return indices_q, indices_v

    def compute_gravity_torques(self, joint_positions: np.ndarray) -> np.ndarray:
        """
        Compute gravity compensation torques for the leg's 3 joints.

        Args:
            joint_positions: numpy array of shape (3,) with joint angles in radians
                            Order: [hip, knee, ankle]

        Returns:
            numpy array of shape (3,) with gravity torques in Nm
            Order: [tau_hip, tau_knee, tau_ankle]

            Sign convention: Torque that would counteract gravity
            (i.e., add these torques to hold the leg in place against gravity)

        Raises:
            ValueError: If joint_positions does not have exactly 3 elements
        """
        # Validate input
        joint_positions = np.asarray(joint_positions, dtype=float)
        if joint_positions.shape != (3,):
            raise ValueError(
                f"Expected joint_positions with shape (3,), got {joint_positions.shape}. "
                f"Input must be [hip_angle, knee_angle, ankle_angle] in radians."
            )

        # Create full robot configuration vector
        # Start from neutral configuration (all zeros)
        q = self.q0.copy()

        # Insert this leg's joint positions into the correct indices
        for i, idx_q in enumerate(self.joint_indices_q):
            q[idx_q] = joint_positions[i]

        # Compute generalized gravity vector using Pinocchio
        # This computes: g(q) = C(q, 0, 0) where C is the nonlinear dynamics
        # In other words: the torques needed to counteract gravity at config q
        pin.computeGeneralizedGravity(self.model, self.data, q)

        # Extract gravity torques for this leg's joints
        # self.data.g is the full generalized gravity vector (size = model.nv)
        gravity_torques = np.array([
            self.data.g[idx_v] for idx_v in self.joint_indices_v
        ], dtype=float)

        return gravity_torques

    def test_computation(self, test_positions: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Test gravity compensation computation with a given configuration.

        Useful for debugging and validation.

        Args:
            test_positions: Joint positions [hip, knee, ankle].
                           Defaults to [0, 0, 0] if None.

        Returns:
            numpy array of shape (3,) with computed gravity torques
        """
        if test_positions is None:
            test_positions = np.zeros(3)

        print("\n" + "=" * 70)
        print("  GRAVITY COMPENSATION TEST")
        print("=" * 70)
        print(f"  Leg ID:          {self.leg_id}")
        print(f"  Joint positions: {test_positions} rad")
        print(f"  Joint positions: {np.rad2deg(test_positions)} deg")
        print("-" * 70)

        tau_g = self.compute_gravity_torques(test_positions)

        print("  Gravity Torques (Nm):")
        joint_labels = ['Hip  ', 'Knee ', 'Ankle']
        for label, torque in zip(joint_labels, tau_g):
            print(f"    {label}: {torque:8.4f}")
        print("=" * 70)

        return tau_g


class MultiLegGravityCompensation:
    """
    Gravity compensation for all 6 legs of the hexapod.

    This is a convenience wrapper that creates 6 GravityCompensation instances,
    one for each leg, and provides batch computation methods.
    """

    def __init__(self, urdf_path: Optional[str] = None):
        """
        Initialize gravity compensation for all 6 legs.

        Args:
            urdf_path: Path to URDF file (if None, auto-detect)
        """
        self.compensators = {}

        print("\n" + "=" * 70)
        print("  Initializing Multi-Leg Gravity Compensation")
        print("=" * 70)

        # Create compensator for each leg
        for leg_id in range(1, 7):
            try:
                self.compensators[leg_id] = GravityCompensation(
                    urdf_path=urdf_path,
                    leg_id=leg_id
                )
                print(f"✓ Leg {leg_id} initialized successfully")
            except Exception as e:
                print(f"✗ Failed to initialize Leg {leg_id}: {e}")
                raise

        print("=" * 70)
        print(f"✓ All {len(self.compensators)} legs initialized")
        print("=" * 70 + "\n")

    def compute_all_legs(self, all_joint_positions: dict) -> dict:
        """
        Compute gravity compensation for multiple legs.

        Args:
            all_joint_positions: dict mapping leg_id → np.array([hip, knee, ankle])
                                Example: {1: np.array([0.1, 0.2, 0.3]),
                                          2: np.array([0.0, 0.5, -0.2])}

        Returns:
            dict mapping leg_id → np.array([tau_hip, tau_knee, tau_ankle])
        """
        gravity_torques = {}

        for leg_id, positions in all_joint_positions.items():
            if leg_id in self.compensators:
                gravity_torques[leg_id] = self.compensators[leg_id].compute_gravity_torques(positions)
            else:
                print(f"⚠ Warning: No compensator for leg {leg_id}, skipping")

        return gravity_torques

    def compute_single_leg(self, leg_id: int, joint_positions: np.ndarray) -> np.ndarray:
        """
        Compute gravity compensation for a single leg.

        Args:
            leg_id: Leg ID (1-6)
            joint_positions: np.array([hip, knee, ankle])

        Returns:
            np.array([tau_hip, tau_knee, tau_ankle])

        Raises:
            ValueError: If leg_id is not initialized
        """
        if leg_id not in self.compensators:
            raise ValueError(
                f"No compensator for leg_id={leg_id}. "
                f"Available legs: {list(self.compensators.keys())}"
            )

        return self.compensators[leg_id].compute_gravity_torques(joint_positions)


def main():
    """
    Test script for gravity compensation module.

    Usage:
        python3 gravity_compensation.py --leg 1 --knee 0.5
        python3 gravity_compensation.py --urdf /path/to/hexapod.urdf
    """
    import argparse

    parser = argparse.ArgumentParser(
        description='Test Pinocchio-based Gravity Compensation for Hexapod'
    )
    parser.add_argument('--leg', type=int, default=1,
                       help='Leg ID (1-6), default=1')
    parser.add_argument('--urdf', type=str, default=None,
                       help='Path to URDF file (auto-detect if not provided)')
    parser.add_argument('--hip', type=float, default=0.0,
                       help='Hip joint position in radians (default=0)')
    parser.add_argument('--knee', type=float, default=0.0,
                       help='Knee joint position in radians (default=0)')
    parser.add_argument('--ankle', type=float, default=0.0,
                       help='Ankle joint position in radians (default=0)')
    parser.add_argument('--test-all-legs', action='store_true',
                       help='Test all 6 legs at zero configuration')

    args = parser.parse_args()

    print("\n" + "=" * 70)
    print("  PINOCCHIO GRAVITY COMPENSATION TEST")
    print("=" * 70 + "\n")

    try:
        if args.test_all_legs:
            # Test all legs
            print("Testing all 6 legs at zero configuration...\n")
            multi_gc = MultiLegGravityCompensation(urdf_path=args.urdf)

            test_config = {leg_id: np.zeros(3) for leg_id in range(1, 7)}
            all_torques = multi_gc.compute_all_legs(test_config)

            print("\nResults:")
            print("-" * 70)
            for leg_id, torques in all_torques.items():
                print(f"Leg {leg_id}: τ = [{torques[0]:7.4f}, {torques[1]:7.4f}, {torques[2]:7.4f}] Nm")
            print("-" * 70)

        else:
            # Test single leg
            gc = GravityCompensation(urdf_path=args.urdf, leg_id=args.leg)

            # Test with user-provided positions
            test_pos = np.array([args.hip, args.knee, args.ankle])
            gc.test_computation(test_pos)

            # Test multiple configurations
            print("\n" + "=" * 70)
            print("  TESTING MULTIPLE CONFIGURATIONS")
            print("=" * 70 + "\n")

            test_configs = [
                ("Zero position",      np.array([0.0, 0.0, 0.0])),
                ("Knee bent forward",  np.array([0.0, 0.5, 0.0])),
                ("Knee bent backward", np.array([0.0, -0.5, 0.0])),
                ("Leg extended down",  np.array([0.0, -0.5, 0.5])),
                ("Hip rotated",        np.array([0.5, 0.0, 0.0])),
                ("Suspended config",   np.array([0.0, 2.0, -1.57])),
            ]

            for name, pos in test_configs:
                tau = gc.compute_gravity_torques(pos)
                print(f"{name:20s} q={pos}  →  τ=[{tau[0]:7.4f}, {tau[1]:7.4f}, {tau[2]:7.4f}] Nm")

        print("\n" + "=" * 70)
        print("✓ Gravity compensation module working correctly!")
        print("=" * 70 + "\n")

        return 0

    except Exception as e:
        print("\n" + "=" * 70)
        print("❌ ERROR")
        print("=" * 70)
        print(f"{e}\n")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
