#!/usr/bin/env python3
"""
Forward Position Kinematic Node - PDF MODEL WITH URDF CORRECTION
INPUT: /hexapod/leg_X/joint_states (JointState - joint positions)
OUTPUT: /hexapod/leg_X/end_effector_position (PointStamped - computed foot position)
Frequency: 100 Hz

CORRECTION: PDF frame offset to match URDF geometry
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np
from fk_pdf_model import fk_pdf

# PDF to URDF offset correction (in leg frame)
# This compensates for the difference between PDF DH frame and URDF geometry
PDF_TO_URDF_OFFSET = np.array([0.18, -0.1985, 0.1399])


class ForwardPositionKinematic(Node):
    def __init__(self):
        super().__init__('forward_position_kinematic')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)

        # Transformation from base to leg frame (leg-specific!)
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)
        self.declare_parameter('hip_rpy_roll', 0.0)
        self.declare_parameter('hip_rpy_pitch', 0.0)
        self.declare_parameter('hip_rpy_yaw', -1.5708)

        update_rate = self.get_parameter('update_rate').value
        leg_id = self.get_parameter('leg_id').value

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

        # Create transformation matrix T_base_leg
        R = self.rpy_to_matrix(*self.hip_rpy)
        self.T_base_leg = np.eye(4)
        self.T_base_leg[0:3, 0:3] = R
        self.T_base_leg[0:3, 3] = self.hip_xyz

        # Store joint angles [hip, knee, ankle]
        self.joint_angles = np.zeros(3)
        self.joint_data_received = False

        # INPUT: Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)

        # OUTPUT: Publisher
        self.ee_position_pub = self.create_publisher(
            PointStamped, 'end_effector_position', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.compute_fk)

        self.get_logger().info(f'Forward Kinematics (PDF model) initialized for leg {leg_id}')

    def joint_state_callback(self, msg):
        """INPUT: Receive joint positions"""
        if len(msg.position) >= 3:
            self.joint_angles = np.array(msg.position[:3])  # [hip, knee, ankle]
            self.joint_data_received = True

    def compute_fk(self):
        """OUTPUT: Compute forward kinematics using PDF model - 100 Hz"""
        if not self.joint_data_received:
            return

        q1, q2, q3 = self.joint_angles

        # PDF FK in leg frame (pure DH frame)
        p_leg_pdf = fk_pdf(q1, q2, q3)

        # Apply PDF to URDF correction (in leg frame)
        p_leg = p_leg_pdf + PDF_TO_URDF_OFFSET

        # Transform to base frame
        p_leg_h = np.array([p_leg[0], p_leg[1], p_leg[2], 1.0])
        p_base = self.T_base_leg @ p_leg_h
        ee_position = p_base[0:3]

        # Publish result
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = float(ee_position[0])
        msg.point.y = float(ee_position[1])
        msg.point.z = float(ee_position[2])

        self.ee_position_pub.publish(msg)

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

    @staticmethod
    def rpy_to_matrix(roll, pitch, yaw):
        """Convert RPY to rotation matrix (ZYX convention)"""
        Rx = ForwardPositionKinematic.rotation_x(roll)
        Ry = ForwardPositionKinematic.rotation_y(pitch)
        Rz = ForwardPositionKinematic.rotation_z(yaw)
        return Rz @ Ry @ Rx


def main(args=None):
    rclpy.init(args=args)
    node = ForwardPositionKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
