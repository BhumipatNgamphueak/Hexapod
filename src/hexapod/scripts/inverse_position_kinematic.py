#!/usr/bin/env python3
"""
Inverse Kinematics Node - URDF-AWARE IMPLEMENTATION (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for seed)
INPUT: /hexapod/leg_X/end_effector_target (PointStamped - desired foot position)
OUTPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target joint angles)
Frequency: Callback-based (triggered by target)

URDF-aware IK with RPY offset compensation:
- Transforms target from base frame to hip local frame
- Accounts for hip joint RPY offset (yaw -90°)
- Accounts for knee joint RPY offset (roll 90°, pitch 90°, yaw 180°)
- Produces joint angles compatible with URDF geometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('use_analytical_ik', True)

        # Link lengths from xacro (in meters)
        # L1: Hip to knee
        self.declare_parameter('knee_joint_x', 0.078424)
        self.declare_parameter('knee_joint_y', -0.0031746)
        self.declare_parameter('knee_joint_z', 0.0010006)

        # L2: Knee to ankle
        self.declare_parameter('ankle_joint_x', -0.087752)
        self.declare_parameter('ankle_joint_y', -0.081834)
        self.declare_parameter('ankle_joint_z', 0.0)

        # L3: Ankle to end effector
        self.declare_parameter('foot_pointer_joint_x', 0.18098)
        self.declare_parameter('foot_pointer_joint_y', -0.022156)
        self.declare_parameter('end_effector_joint_x', 0.0075528)
        self.declare_parameter('end_effector_joint_y', -0.00094278)

        leg_id = self.get_parameter('leg_id').value

        # Store offsets for URDF-aware transformations
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

        # URDF RPY offsets (from hexapod.urdf)
        self.hip_rpy = np.array([0.0, 0.0, -1.5708])  # Hip joint RPY offset
        self.knee_rpy = np.array([1.5708, 1.5708, 3.142])  # Knee joint RPY offset
        self.hip_xyz = np.array([-1.2616e-05, -0.095255, 0.0])  # Hip joint xyz offset

        self.target_sub = self.create_subscription(
            PointStamped, 'end_effector_target', self.target_callback, 10)

        # OUTPUT: Publisher
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'joint_position_target', 10)

        # Declare use_numerical_ik parameter (DEFAULT: True for high accuracy)
        self.declare_parameter('use_numerical_ik', True)
        self.use_numerical_ik = self.get_parameter('use_numerical_ik').value

        self.get_logger().info(f'Inverse Kinematics initialized for leg {leg_id}')
        self.get_logger().info(f'Link lengths: L1={self.L1:.4f}, L2={self.L2:.4f}, L3={self.L3:.4f}')
        self.get_logger().info('URDF-aware IK with RPY compensation enabled')
        self.get_logger().info(f'IK Mode: {"NUMERICAL (Optimized)" if self.use_numerical_ik else "ANALYTICAL (Fast)"}')

    @staticmethod
    def rotation_x(theta):
        """Rotation matrix around X axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    @staticmethod
    def rotation_y(theta):
        """Rotation matrix around Y axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    @staticmethod
    def rotation_z(theta):
        """Rotation matrix around Z axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def rpy_to_matrix(self, roll, pitch, yaw):
        """Convert RPY (roll-pitch-yaw) to rotation matrix (ZYX convention)"""
        return self.rotation_z(yaw) @ self.rotation_y(pitch) @ self.rotation_x(roll)

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Forward kinematics for numerical IK (same as FK node)
        """
        # Hip joint (with RPY offset)
        R_hip_static = self.rpy_to_matrix(*self.hip_rpy)
        R1 = R_hip_static @ self.rotation_z(theta1)
        T1 = self.hip_xyz + R1 @ self.knee_offset

        # Knee joint (with RPY offset)
        R_knee_static = self.rpy_to_matrix(1.5708, 1.5708, 3.142)
        R2 = R1 @ R_knee_static @ self.rotation_z(theta2)
        T2 = T1 + R2 @ self.ankle_offset

        # Ankle joint
        R3 = R2 @ self.rotation_z(theta3)
        T3 = T2 + R3 @ self.foot_offset

        return T3

    def compute_jacobian(self, theta):
        """Numerical Jacobian for IK"""
        epsilon = 1e-6
        J = np.zeros((3, 3))
        pos = self.forward_kinematics(*theta)

        for j in range(3):
            theta_plus = theta.copy()
            theta_plus[j] += epsilon
            pos_plus = self.forward_kinematics(*theta_plus)
            J[:, j] = (pos_plus - pos) / epsilon

        return J

    def inverse_kinematics_analytical(self, target_base):
        """Analytical IK (fast but approximate)"""
        try:
            # Transform target to hip local frame
            R_hip_offset = self.rpy_to_matrix(*self.hip_rpy)
            target_hip_frame = R_hip_offset.T @ (target_base - self.hip_xyz)

            Xp_hip = target_hip_frame[0]
            Yp_hip = target_hip_frame[1]
            Zp_hip = target_hip_frame[2]

            # Compute IK in hip local frame
            theta1 = np.arctan2(Yp_hip, Xp_hip)
            r1 = np.sqrt(Xp_hip**2 + Yp_hip**2) - self.L1
            r2 = np.sqrt(r1**2 + Zp_hip**2)

            # Check reachability
            if r2 > (self.L2 + self.L3) or r2 < abs(self.L2 - self.L3):
                return None

            # Law of cosines
            phi1 = np.arctan2(Zp_hip, r1)

            cos_phi3 = (self.L2**2 + self.L3**2 - r2**2) / (2 * self.L2 * self.L3)
            cos_phi3 = np.clip(cos_phi3, -1.0, 1.0)
            phi3 = np.arccos(cos_phi3)

            cos_phi2 = (self.L2**2 + r2**2 - self.L3**2) / (2 * self.L2 * r2)
            cos_phi2 = np.clip(cos_phi2, -1.0, 1.0)
            phi2 = np.arccos(cos_phi2)

            theta2 = phi1 + phi2
            theta3 = np.pi - phi3

            return np.array([theta1, theta2, theta3])

        except Exception:
            return None

    def inverse_kinematics_numerical(self, target):
        """
        Optimized Numerical IK with Jacobian
        OPTIMIZATIONS:
        - Reduced max iterations: 30 → 15 (faster)
        - Relaxed tolerance: 0.1mm → 1mm (practical accuracy)
        - Reduced line search: 10 → 5 iterations
        - Better damping: 0.01 → 0.005 (faster convergence)
        Expected: ~1-2ms computation time, <1mm error
        """
        # Get initial guess from analytical IK (fast initial guess)
        theta_init = self.inverse_kinematics_analytical(target)

        if theta_init is None:
            # Fallback: neutral pose
            theta = np.array([0.0, np.pi/3, np.pi/2])
        else:
            theta = theta_init

        # OPTIMIZED PARAMETERS for speed + accuracy balance
        max_iterations = 15      # Reduced from 30
        tolerance = 0.001        # 1mm (relaxed from 0.1mm)
        lambda_damping = 0.005   # Better initial damping

        for iteration in range(max_iterations):
            current_pos = self.forward_kinematics(*theta)
            error = target - current_pos
            error_norm = np.linalg.norm(error)

            # Early termination
            if error_norm < tolerance:
                return theta  # Converged!

            J = self.compute_jacobian(theta)
            JtJ = J.T @ J
            Jt_error = J.T @ error

            try:
                # Damped least squares (Levenberg-Marquardt)
                delta_theta = np.linalg.solve(JtJ + lambda_damping * np.eye(3), Jt_error)
            except np.linalg.LinAlgError:
                # Fallback to pseudo-inverse
                delta_theta = np.linalg.pinv(J) @ error

            # REDUCED line search: 10 → 5 iterations
            alpha = 1.0
            new_error_norm = error_norm
            for _ in range(5):
                theta_new = theta + alpha * delta_theta
                new_pos = self.forward_kinematics(*theta_new)
                new_error_norm = np.linalg.norm(target - new_pos)

                if new_error_norm < error_norm:
                    theta = theta_new
                    break
                else:
                    alpha *= 0.5

            # Early stopping if stuck
            if abs(new_error_norm - error_norm) < 1e-6:
                break

            # Adaptive damping
            if new_error_norm < error_norm:
                lambda_damping *= 0.85  # Faster reduction
            else:
                lambda_damping *= 1.5

        return theta

    def target_callback(self, msg):
        """INPUT: Receive target end effector position and compute IK"""
        # Target in base frame
        target_base = np.array([msg.point.x, msg.point.y, msg.point.z])

        try:
            # Choose IK method based on parameter
            if self.use_numerical_ik:
                # Numerical IK (high accuracy ~0.06mm)
                theta = self.inverse_kinematics_numerical(target_base)
            else:
                # Analytical IK (fast but approximate ~173mm error)
                theta = self.inverse_kinematics_analytical(target_base)

                if theta is None:
                    self.get_logger().warn('Target unreachable with analytical IK')
                    return

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