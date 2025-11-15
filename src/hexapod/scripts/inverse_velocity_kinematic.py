#!/usr/bin/env python3
"""
Inverse Velocity Kinematics Node - IMPLEMENTED (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joints for Jacobian)
INPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - desired foot velocity)
OUTPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - joint velocities)
Frequency: 100 Hz

Based on equations from PDF (page 7):
- θ̇ = J⁻¹(θ)v
- Uses damped least squares (Levenberg-Marquardt) for numerical stability
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class InverseVelocityKinematics(Node):
    def __init__(self):
        super().__init__('inverse_velocity_kinematics')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('damping_factor', 0.01)

        # Link lengths from xacro (in meters)
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

        update_rate = self.get_parameter('update_rate').value
        leg_id = self.get_parameter('leg_id').value
        self.damping = self.get_parameter('damping_factor').value

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

        # State variables
        self.joint_angles = np.zeros(3)
        self.ee_velocity = np.zeros(3)
        self.joint_data_received = False
        self.ee_vel_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        self.ee_vel_sub = self.create_subscription(
            Vector3Stamped, 'end_effector_velocity', self.ee_velocity_callback, 10)

        # OUTPUT: Publisher
        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_feedforward', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_velocity)

        self.get_logger().info(f'Inverse Velocity Kinematics initialized for leg {leg_id}')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint positions"""
        if len(msg.position) >= 3:
            self.joint_angles = np.array(msg.position[:3])
            self.joint_data_received = True

    def ee_velocity_callback(self, msg):
        """INPUT: Receive desired end effector velocity"""
        self.ee_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.ee_vel_received = True

    def compute_velocity(self):
        """OUTPUT: Compute joint velocities using Jacobian - 100 Hz"""
        if not self.joint_data_received or not self.ee_vel_received:
            return

        theta1, theta2, theta3 = self.joint_angles

        # Compute Jacobian matrix (3x3)
        J = self.compute_jacobian(theta1, theta2, theta3)

        # Solve for joint velocities using damped least squares
        # θ̇ = J^T(JJ^T + λ²I)^(-1)v
        # This is more stable than direct inverse when J is near singular
        JJT = J @ J.T
        damping_matrix = self.damping**2 * np.eye(3)

        try:
            joint_velocities = J.T @ np.linalg.inv(JJT + damping_matrix) @ self.ee_velocity
        except np.linalg.LinAlgError:
            self.get_logger().warn('Jacobian inversion failed, using pseudo-inverse')
            joint_velocities = np.linalg.pinv(J) @ self.ee_velocity

        # Publish joint velocities
        msg = Float64MultiArray()
        msg.data = [float(v) for v in joint_velocities]
        self.joint_vel_pub.publish(msg)

    def compute_jacobian(self, theta1, theta2, theta3):
        """
        Compute Jacobian matrix analytically
        J[i,j] = ∂position_i/∂theta_j
        """
        # Build rotation matrices
        R1 = self.rotation_z(theta1)
        R2 = R1 @ self.rotation_y(theta2)
        R3 = R2 @ self.rotation_y(theta3)

        # Compute positions of each joint
        p1 = R1 @ self.knee_offset
        p2 = p1 + R2 @ self.ankle_offset
        p3 = p2 + R3 @ self.foot_offset  # End effector position

        # Joint axes in world frame
        z0 = np.array([0, 0, 1])  # Hip joint axis (Z)
        z1 = R1 @ np.array([0, 1, 0])  # Knee joint axis (Y in local frame)
        z2 = R2 @ np.array([0, 1, 0])  # Ankle joint axis (Y in local frame)

        # Jacobian columns: J_i = z_i × (p_e - p_i)
        # For revolute joints: v = ω × r
        J = np.zeros((3, 3))

        # Column 1: Effect of θ₁ on end effector
        J[:, 0] = np.cross(z0, p3)

        # Column 2: Effect of θ₂ on end effector
        J[:, 1] = np.cross(z1, p3 - p1)

        # Column 3: Effect of θ₃ on end effector
        J[:, 2] = np.cross(z2, p3 - p2)

        return J

    @staticmethod
    def rotation_z(theta):
        """Rotation matrix around Z axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])

    @staticmethod
    def rotation_y(theta):
        """Rotation matrix around Y axis"""
        c, s = np.cos(theta), np.sin(theta)
        return np.array([
            [ c, 0, s],
            [ 0, 1, 0],
            [-s, 0, c]
        ])


def main(args=None):
    rclpy.init(args=args)
    node = InverseVelocityKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()