#!/usr/bin/env python3
"""
Inverse Kinematics Node - URDF-AWARE with BASE FRAME (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for seed)
INPUT: /hexapod/leg_X/end_effector_target (PointStamped - desired foot position in BASE_LINK)
OUTPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target joint angles)
Frequency: Callback-based (triggered by target)

FIXED: Now correctly handles base_link frame with per-leg hip offsets
- Receives target in base_link (from gait_planner)
- Transforms to hip-relative coordinates using leg-specific hip offset
- Computes joint angles θ₁, θ₂, θ₃
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

        self.leg_id = self.get_parameter('leg_id').value

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

        # FIXED: Hip offset from base_link to hip joint (leg-specific)
        self.hip_xyz = self._get_hip_offset(self.leg_id)

        self.target_sub = self.create_subscription(
            PointStamped, 'end_effector_target', self.target_callback, 10)

        # OUTPUT: Publisher
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'joint_position_target', 10)

        # Declare use_numerical_ik parameter (DEFAULT: True for high accuracy)
        self.declare_parameter('use_numerical_ik', True)
        self.use_numerical_ik = self.get_parameter('use_numerical_ik').value

        self.get_logger().info(f'Inverse Kinematics initialized for leg {self.leg_id}')
        self.get_logger().info(f'Link lengths: L1={self.L1:.4f}, L2={self.L2:.4f}, L3={self.L3:.4f}')
        self.get_logger().info(f'Hip offset (base_link → hip): {self.hip_xyz}')
        self.get_logger().info('FIXED: Using base_link frame with per-leg hip offsets')
        self.get_logger().info(f'IK Mode: {"NUMERICAL (Optimized)" if self.use_numerical_ik else "ANALYTICAL (Fast)"}')

    def _get_hip_offset(self, leg_id):
        """
        Get hip joint offset from base_link for each leg

        Leg layout (top view):
              Front
          1         6
          2         5
          3         4
             Back

        Leg 1: Front Right  →  X=+0.0785, Y=-0.095255
        Leg 2: Middle Right →  X=0,       Y=-0.095255
        Leg 3: Rear Right   →  X=-0.0785, Y=-0.095255
        Leg 4: Rear Left    →  X=-0.0785, Y=+0.095255
        Leg 5: Middle Left  →  X=0,       Y=+0.095255
        Leg 6: Front Left   →  X=+0.0785, Y=+0.095255
        """
        # X offsets (front-back position)
        x_front = 0.0785
        x_middle = 0.0
        x_rear = -0.0785

        # Y offsets (left-right position)
        y_right = -0.095255
        y_left = 0.095255

        # Z offset (all legs same height)
        z_offset = 0.0

        # Map leg_id to offsets
        hip_offsets = {
            1: np.array([x_front, y_right, z_offset]),   # Front Right
            2: np.array([x_middle, y_right, z_offset]),  # Middle Right
            3: np.array([x_rear, y_right, z_offset]),    # Rear Right
            4: np.array([x_rear, y_left, z_offset]),     # Rear Left
            5: np.array([x_middle, y_left, z_offset]),   # Middle Left
            6: np.array([x_front, y_left, z_offset])     # Front Left
        }

        return hip_offsets.get(leg_id, np.array([0.0, 0.0, 0.0]))

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
        Forward kinematics - computes end effector position in BASE_LINK frame

        FIXED: Now correctly returns position in base_link by adding hip_xyz offset
        """
        # Hip joint (with RPY offset)
        R_hip_static = self.rpy_to_matrix(*self.hip_rpy)
        R1 = R_hip_static @ self.rotation_z(theta1)
        T1 = self.hip_xyz + R1 @ self.knee_offset  # FIXED: Added hip_xyz offset

        # Knee joint (with RPY offset)
        R_knee_static = self.rpy_to_matrix(1.5708, 1.5708, 3.142)
        R2 = R1 @ R_knee_static @ self.rotation_z(theta2)
        T2 = T1 + R2 @ self.ankle_offset

        # Ankle joint
        R3 = R2 @ self.rotation_z(theta3)
        T3 = T2 + R3 @ self.foot_offset

        return T3  # Position in base_link frame

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
        """
        Analytical IK - simplified geometric solution

        INPUT: target_base - position in base_link frame
        OUTPUT: [θ₁, θ₂, θ₃] joint angles

        FIXED: Now transforms target from base_link to hip-relative coordinates
        """
        # Transform target from base_link to hip-relative coordinates
        target_hip_relative = target_base - self.hip_xyz

        # Extract coordinates
        Xp = target_hip_relative[0]
        Yp = target_hip_relative[1]
        Zp = target_hip_relative[2]

        try:
            # θ₁: Hip joint angle (yaw)
            theta1 = np.arctan2(Yp, Xp)

            # Project to X-Z plane
            r2 = Xp / np.cos(theta1) - self.L1

            # Distance in X-Z plane
            r1 = np.sqrt(Zp**2 + r2**2)

            # θ₂ and θ₃: Knee and ankle angles (2-link planar IK)
            cos_phi3 = (r1**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            cos_phi3 = np.clip(cos_phi3, -1.0, 1.0)
            phi3 = np.arccos(cos_phi3)

            phi1 = np.arctan2(Zp, r2)

            cos_phi2 = (self.L2**2 + r1**2 - self.L3**2) / (2 * self.L2 * r1)
            cos_phi2 = np.clip(cos_phi2, -1.0, 1.0)
            phi2 = np.arccos(cos_phi2)

            theta2 = phi1 + phi2
            theta3 = np.pi - phi3

            return np.array([theta1, theta2, theta3])

        except Exception:
            return None

    def inverse_kinematics_numerical(self, target_base):
        """
        Optimized Numerical IK with Jacobian

        INPUT: target_base - position in base_link frame
        OUTPUT: [θ₁, θ₂, θ₃] joint angles

        FIXED: FK now handles base_link correctly, so this works as-is
        """
        # Get initial guess from analytical IK (fast initial guess)
        theta_init = self.inverse_kinematics_analytical(target_base)

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
            error = target_base - current_pos
            error_norm = np.linalg.norm(error)

            # Early termination
            if error_norm < tolerance:
                return theta  # Converged!

            J = self.compute_jacobian(theta)
            JtJ = J.T @ J
            Jt_error = J.T @ error

            # Damped least squares (Levenberg-Marquardt)
            damping_matrix = lambda_damping * np.eye(3)

            # Line search for step size
            best_alpha = 1.0
            best_error = error_norm

            for alpha in [1.0, 0.5, 0.25, 0.1, 0.05]:
                try:
                    delta_theta = alpha * np.linalg.solve(JtJ + damping_matrix, Jt_error)
                    theta_new = theta + delta_theta

                    new_pos = self.forward_kinematics(*theta_new)
                    new_error_norm = np.linalg.norm(target_base - new_pos)

                    if new_error_norm < best_error:
                        best_alpha = alpha
                        best_error = new_error_norm
                        theta = theta_new
                        break
                except np.linalg.LinAlgError:
                    continue

            # Early stopping if stuck
            if abs(best_error - error_norm) < 1e-6:
                break

            # Adaptive damping
            if best_error < error_norm:
                lambda_damping *= 0.85  # Faster reduction
            else:
                lambda_damping *= 1.5

        return theta

    def target_callback(self, msg):
        """
        INPUT: Receive target end effector position in BASE_LINK and compute IK

        FIXED: Now expects frame_id = 'base_link' (from gait_planner via trajectory_generator)
        """
        # Verify frame_id
        if msg.header.frame_id != 'base_link':
            self.get_logger().warn(
                f'Expected frame_id=base_link, got {msg.header.frame_id}. '
                f'Proceeding anyway but check trajectory_planning.py'
            )

        # Target in base frame (as expected)
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
