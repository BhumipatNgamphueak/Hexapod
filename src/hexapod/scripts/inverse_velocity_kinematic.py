#!/usr/bin/env python3
"""
Inverse Velocity Kinematic Node - PDF MODEL WITH URDF CORRECTION
INPUT:
  - /hexapod/leg_X/joint_states (JointState - current joint positions for Jacobian)
  - /hexapod/leg_X/end_effector_velocity (Vector3Stamped - desired velocity in BASE frame)
OUTPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - feedforward joint velocities)
Frequency: 100 Hz

CRITICAL: Jacobian is computed in PDF frame, so we use it directly.
The PDF_TO_URDF_OFFSET is a CONSTANT offset, so its derivative is ZERO.
Therefore: v_urdf = v_pdf, and we can use J_pdf directly!

Match FK/IK: Both use PDF model with offset correction.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from fk_pdf_model import compute_jacobian_pdf

# PDF to URDF offset (CONSTANT - derivative is zero!)
PDF_TO_URDF_OFFSET = np.array([0.18, -0.1985, 0.1399])


class InverseVelocityKinematic(Node):
    def __init__(self):
        super().__init__('inverse_velocity_kinematic')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)
        self.declare_parameter('damping_factor', 0.01)

        # Transformation from base to leg frame (leg-specific!)
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)
        self.declare_parameter('hip_rpy_roll', 0.0)
        self.declare_parameter('hip_rpy_pitch', 0.0)
        self.declare_parameter('hip_rpy_yaw', -1.5708)

        leg_id = self.get_parameter('leg_id').value
        update_rate = self.get_parameter('update_rate').value
        self.damping = self.get_parameter('damping_factor').value

        # Build transformation matrix from base to leg frame
        self.hip_xyz = np.array([
            self.get_parameter('hip_xyz_x').value,
            self.get_parameter('hip_xyz_y').value,
            self.get_parameter('hip_xyz_z').value
        ])
        self.hip_rpy = np.array([
            self.get_parameter('hip_rpy_roll').value,
            self.get_parameter('hip_rpy_pitch').value,
            self.get_parameter('hip_rpy_yaw').value
        ])

        # Create transformation matrix T_base_leg (for transforming velocities)
        R = self.rpy_to_matrix(*self.hip_rpy)
        self.T_base_leg = np.eye(4)
        self.T_base_leg[0:3, 0:3] = R
        self.T_base_leg[0:3, 3] = self.hip_xyz

        # Invert to get T_leg_base
        self.T_leg_base = np.linalg.inv(self.T_base_leg)
        self.R_leg_base = self.T_leg_base[0:3, 0:3]  # Rotation only

        # Store current joint angles (needed for Jacobian)
        self.joint_angles = np.zeros(3)
        self.ee_velocity_base = np.zeros(3)
        self.joint_data_received = False
        self.ee_vel_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.ee_vel_sub = self.create_subscription(
            Vector3Stamped, 'end_effector_velocity', self.ee_velocity_callback, 10)

        # Publisher
        self.joint_velocity_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_feedforward', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_velocity)

        self.get_logger().info(f'Inverse Velocity Kinematics (PDF model) initialized for leg {leg_id}')

    def joint_state_callback(self, msg):
        """Store current joint angles"""
        if len(msg.position) >= 3:
            # Store joint angles (offsets are in DH matrix)
            self.joint_angles = np.array(msg.position[:3])
            self.joint_data_received = True

    def ee_velocity_callback(self, msg):
        """Store desired end-effector velocity (in base frame)"""
        self.ee_velocity_base = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.ee_vel_received = True

    def compute_velocity(self):
        """
        Compute inverse velocity kinematics using PDF Jacobian

        MATH:
        Position: p_urdf = p_pdf + OFFSET  (OFFSET is constant)
        Velocity: v_urdf = d/dt(p_pdf + OFFSET) = v_pdf + 0 = v_pdf

        Therefore, we can use PDF Jacobian directly:
          v_pdf = J_pdf * q_dot
          q_dot = J_pdf^(-1) * v_pdf

        This matches FK/IK which use PDF model with offset correction.
        """
        if not self.joint_data_received or not self.ee_vel_received:
            return

        try:
            # Transform desired velocity from base frame to leg frame (URDF frame)
            # Since v_urdf = v_pdf (constant offset), this is correct
            v_ee_leg = self.R_leg_base @ self.ee_velocity_base

            # Compute Jacobian at current joint configuration (PDF frame)
            # This gives us: v_pdf = J_pdf * q_dot
            J = compute_jacobian_pdf(*self.joint_angles)

            # Damped least squares inverse (prevents singularities)
            # q_dot = (J^T J + λI)^(-1) J^T * v
            JtJ = J.T @ J
            J_inv = np.linalg.solve(JtJ + self.damping * np.eye(3), J.T)

            # Compute joint velocities
            q_dot = J_inv @ v_ee_leg

            # Publish feedforward joint velocities
            joint_vel_msg = Float64MultiArray()
            joint_vel_msg.data = [float(q_dot[0]), float(q_dot[1]), float(q_dot[2])]
            self.joint_velocity_pub.publish(joint_vel_msg)

        except Exception as e:
            self.get_logger().error(f'IVK computation failed: {str(e)}')

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
        """Convert RPY to rotation matrix (ZYX convention)"""
        Rx = self.rotation_x(roll)
        Ry = self.rotation_y(pitch)
        Rz = self.rotation_z(yaw)
        return Rz @ Ry @ Rx


def main(args=None):
    rclpy.init(args=args)
    node = InverseVelocityKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
