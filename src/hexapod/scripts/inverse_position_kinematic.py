#!/usr/bin/env python3
"""
Inverse Kinematics - FIXED WITH JOINT LIMIT ENFORCEMENT (ONE PER LEG)

CRITICAL FIXES:
1. Check IK solutions against joint limits
2. Reject unreachable targets (beyond limits)
3. Option to clamp solutions to limits
4. Warn user when targets are unreachable

INPUT: /hexapod/leg_X/end_effector_target (PointStamped - desired foot position)
OUTPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target joint angles)
Frequency: Callback-based
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


# Joint limits from hexapod_params.xacro
JOINT_LIMITS = {
    0: {'lower': -1.57, 'upper': 1.57, 'name': 'hip'},     # ±90°
    1: {'lower': -2.0, 'upper': 2.0, 'name': 'knee'},      # ±115°
    2: {'lower': -1.57, 'upper': 1.57, 'name': 'ankle'}    # ±90°
}


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('max_iterations', 15)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('use_numerical_ik', True)
        self.declare_parameter('enforce_joint_limits', True)  # NEW
        self.declare_parameter('clamp_to_limits', False)  # NEW: clamp vs reject

        # Link lengths
        self.declare_parameter('knee_joint_x', 0.078424)
        self.declare_parameter('knee_joint_y', -0.0031746)
        self.declare_parameter('knee_joint_z', 0.0010006)
        self.declare_parameter('ankle_joint_x', -0.087752)
        self.declare_parameter('ankle_joint_y', -0.081834)
        self.declare_parameter('ankle_joint_z', 0.0)
        self.declare_parameter('foot_pointer_joint_x', 0.18098)
        self.declare_parameter('foot_pointer_joint_y', -0.022156)
        self.declare_parameter('end_effector_joint_x', 0.0075528)
        self.declare_parameter('end_effector_joint_y', -0.00094278)
        
        # Hip attachment point (leg-specific!)
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)
        
        # Hip RPY orientation (leg-specific!)
        self.declare_parameter('hip_rpy_roll', 0.0)
        self.declare_parameter('hip_rpy_pitch', 0.0)
        self.declare_parameter('hip_rpy_yaw', -1.5708)

        leg_id = self.get_parameter('leg_id').value
        self.use_numerical_ik = self.get_parameter('use_numerical_ik').value
        self.enforce_limits = self.get_parameter('enforce_joint_limits').value
        self.clamp_to_limits = self.get_parameter('clamp_to_limits').value

        # Get link offsets
        self.knee_offset = np.array([
            self.get_parameter('knee_joint_x').value,
            self.get_parameter('knee_joint_y').value,
            self.get_parameter('knee_joint_z').value
        ])

        self.ankle_offset = np.array([
            self.get_parameter('ankle_joint_x').value,
            self.get_parameter('ankle_joint_y').value,
            self.get_parameter('ankle_joint_z').value
        ])

        self.foot_offset = np.array([
            self.get_parameter('foot_pointer_joint_x').value + self.get_parameter('end_effector_joint_x').value,
            self.get_parameter('foot_pointer_joint_y').value + self.get_parameter('end_effector_joint_y').value,
            0.0
        ])

        # Calculate link lengths
        self.L1 = np.linalg.norm(self.knee_offset)
        self.L2 = np.linalg.norm(self.ankle_offset)
        self.L3 = np.linalg.norm(self.foot_offset)

        # URDF RPY offsets (leg-specific hip orientation!)
        self.hip_rpy = np.array([
            self.get_parameter('hip_rpy_roll').value,
            self.get_parameter('hip_rpy_pitch').value,
            self.get_parameter('hip_rpy_yaw').value
        ])
        self.knee_rpy = np.array([1.5708, 1.5708, 3.142])
        
        # Hip attachment point from parameters (leg-specific)
        self.hip_xyz = np.array([
            self.get_parameter('hip_xyz_x').value,
            self.get_parameter('hip_xyz_y').value,
            self.get_parameter('hip_xyz_z').value
        ])

        # Subscribers & Publishers
        self.target_sub = self.create_subscription(
            PointStamped, 'end_effector_target', self.target_callback, 10)
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'joint_position_target', 10)

        self.get_logger().info(f'IK (with joint limit enforcement) initialized for leg {leg_id}')
        self.get_logger().info(f'Link lengths: L1={self.L1:.4f}, L2={self.L2:.4f}, L3={self.L3:.4f}')
        self.get_logger().info(f'Joint limit enforcement: {self.enforce_limits}')
        self.get_logger().info(f'Clamp to limits: {self.clamp_to_limits}')

    @staticmethod
    def rotation_x(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def rotation_y(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def rotation_z(theta):
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def rpy_to_matrix(self, roll, pitch, yaw):
        return self.rotation_z(yaw) @ self.rotation_y(pitch) @ self.rotation_x(roll)

    def forward_kinematics(self, theta1, theta2, theta3):
        """Forward kinematics (URDF-aware)"""
        R_hip_static = self.rpy_to_matrix(*self.hip_rpy)
        R1 = R_hip_static @ self.rotation_z(theta1)
        T1 = self.hip_xyz + R1 @ self.knee_offset

        R_knee_static = self.rpy_to_matrix(1.5708, 1.5708, 3.142)
        R2 = R1 @ R_knee_static @ self.rotation_z(theta2)
        T2 = T1 + R2 @ self.ankle_offset

        R3 = R2 @ self.rotation_z(theta3)
        T3 = T2 + R3 @ self.foot_offset

        return T3

    def compute_jacobian(self, theta):
        """Numerical Jacobian"""
        epsilon = 1e-6
        J = np.zeros((3, 3))
        pos = self.forward_kinematics(*theta)

        for j in range(3):
            theta_plus = theta.copy()
            theta_plus[j] += epsilon
            pos_plus = self.forward_kinematics(*theta_plus)
            J[:, j] = (pos_plus - pos) / epsilon

        return J

    def check_joint_limits(self, angles, tolerance=0.01):
        """
        Check if angles are within joint limits
        
        Returns:
            within_limits (bool): True if all within limits
            violations (list): List of (joint_idx, angle, limit) for violations
        """
        violations = []
        
        for i, limits in JOINT_LIMITS.items():
            lower = limits['lower'] + tolerance
            upper = limits['upper'] - tolerance
            
            if angles[i] < lower:
                violations.append((i, angles[i], 'lower', lower))
            elif angles[i] > upper:
                violations.append((i, angles[i], 'upper', upper))
        
        return len(violations) == 0, violations

    def enforce_joint_limits_clamp(self, angles):
        """Clamp angles to joint limits"""
        clamped = np.copy(angles)
        
        for i, limits in JOINT_LIMITS.items():
            clamped[i] = np.clip(clamped[i], limits['lower'], limits['upper'])
        
        return clamped

    def inverse_kinematics_numerical(self, target):
        """Numerical IK with joint limit awareness"""
        # Initial guess (downward-pointing stance)
        # For leg to point down (negative Z), joints 2 and 3 should be negative
        theta = np.array([0.0, -np.pi/4, -np.pi/4])
        
        max_iterations = 30  # Increased from 15
        tolerance = 0.0001  # Tightened from 0.001 (1mm -> 0.1mm)
        lambda_damping = 0.005

        for iteration in range(max_iterations):
            current_pos = self.forward_kinematics(*theta)
            error = target - current_pos
            error_norm = np.linalg.norm(error)

            if error_norm < tolerance:
                break  # Converged

            J = self.compute_jacobian(theta)
            JtJ = J.T @ J
            Jt_error = J.T @ error

            try:
                delta_theta = np.linalg.solve(JtJ + lambda_damping * np.eye(3), Jt_error)
            except np.linalg.LinAlgError:
                delta_theta = np.linalg.pinv(J) @ error

            # Line search with limit checking
            alpha = 1.0
            for _ in range(5):
                theta_new = theta + alpha * delta_theta
                
                # Check if new angles would violate limits
                if self.enforce_limits:
                    within_limits, _ = self.check_joint_limits(theta_new, tolerance=0.0)
                    if not within_limits:
                        # Try smaller step
                        alpha *= 0.5
                        continue
                
                new_pos = self.forward_kinematics(*theta_new)
                new_error_norm = np.linalg.norm(target - new_pos)

                if new_error_norm < error_norm:
                    theta = theta_new
                    break
                else:
                    alpha *= 0.5

            if abs(new_error_norm - error_norm) < 1e-6:
                break

            if new_error_norm < error_norm:
                lambda_damping *= 0.85
            else:
                lambda_damping *= 1.5

        return theta

    def target_callback(self, msg):
        """INPUT: Receive target position and compute IK"""
        target_base = np.array([msg.point.x, msg.point.y, msg.point.z])

        try:
            # Compute IK solution
            theta = self.inverse_kinematics_numerical(target_base)

            # Check joint limits
            if self.enforce_limits:
                within_limits, violations = self.check_joint_limits(theta, tolerance=0.01)
                
                if not within_limits:
                    if self.clamp_to_limits:
                        # Clamp to limits and warn
                        theta = self.enforce_joint_limits_clamp(theta)
                        self.get_logger().warn(
                            f'IK solution clamped to limits: {[v[0] for v in violations]}',
                            throttle_duration_sec=1.0
                        )
                    else:
                        # Reject solution
                        self.get_logger().warn(
                            f'Target unreachable (violates joint limits): '
                            f'{[(JOINT_LIMITS[v[0]]["name"], np.degrees(v[1])) for v in violations]}',
                            throttle_duration_sec=1.0
                        )
                        return  # Don't publish invalid solution

            # Publish joint targets
            joint_msg = Float64MultiArray()
            joint_msg.data = [float(theta[0]), float(theta[1]), float(theta[2])]
            self.joint_target_pub.publish(joint_msg)

        except Exception as e:
            self.get_logger().error(f'IK computation failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()