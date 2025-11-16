#!/usr/bin/env python3
"""
Forward Kinematics Node - IMPLEMENTED (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - joint positions)
OUTPUT: /hexapod/leg_X/end_effector_position (PointStamped - computed foot position)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
import numpy as np


class ForwardKinematics(Node):
    def __init__(self):
        super().__init__('forward_kinematics')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('update_rate', 100.0)

        # Link lengths from xacro (in meters)
        # Hip to knee offset
        self.declare_parameter('knee_joint_x', 0.078424)
        self.declare_parameter('knee_joint_y', -0.0031746)
        self.declare_parameter('knee_joint_z', 0.0010006)

        # Knee to ankle offset
        self.declare_parameter('ankle_joint_x', -0.087752)
        self.declare_parameter('ankle_joint_y', -0.081834)
        self.declare_parameter('ankle_joint_z', 0.0)

        # Ankle to foot pointer offset
        self.declare_parameter('foot_pointer_joint_x', 0.18098)
        self.declare_parameter('foot_pointer_joint_y', -0.022156)

        # Foot pointer to end effector offset
        self.declare_parameter('end_effector_joint_x', 0.0075528)
        self.declare_parameter('end_effector_joint_y', -0.00094278)

        # URDF RPY offsets (from hexapod.urdf)
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)

        update_rate = self.get_parameter('update_rate').value
        leg_id = self.get_parameter('leg_id').value

        # Get link parameters
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

        # Total foot offset (foot pointer + end effector)
        self.foot_offset = np.array([
            self.get_parameter('foot_pointer_joint_x').value + self.get_parameter('end_effector_joint_x').value,
            self.get_parameter('foot_pointer_joint_y').value + self.get_parameter('end_effector_joint_y').value,
            0.0
        ])

        # URDF offsets
        self.hip_xyz = np.array([
            self.get_parameter('hip_xyz_x').value,
            self.get_parameter('hip_xyz_y').value,
            self.get_parameter('hip_xyz_z').value
        ])
        self.hip_rpy = np.array([0.0, 0.0, -1.5708])  # From URDF

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

        self.get_logger().info(f'Forward Kinematics initialized for leg {leg_id}')

    def joint_state_callback(self, msg):
        """INPUT: Receive joint positions"""
        if len(msg.position) >= 3:
            self.joint_angles = np.array(msg.position[:3])  # [hip, knee, ankle]
            self.joint_data_received = True

    def compute_fk(self):
        """OUTPUT: Compute forward kinematics using URDF-aware transformations - 100 Hz"""
        if not self.joint_data_received:
            return

        theta1, theta2, theta3 = self.joint_angles

        # URDF-aware FK: Apply static RPY before each joint rotation

        # Frame 0 -> Frame 1 (Hip joint with RPY offset)
        # URDF: xyz="... -0.095255 ..." rpy="0 0 -1.5708"
        R_hip_static = self.rpy_to_matrix(*self.hip_rpy)  # Static RPY from URDF
        R1 = R_hip_static @ self.rotation_z(theta1)  # Apply joint rotation in joint frame
        T1 = self.hip_xyz + R1 @ self.knee_offset  # Position in base frame

        # Frame 1 -> Frame 2 (Knee joint with RPY offset from URDF)
        # URDF: rpy="1.5708 1.5708 3.142" (90°, 90°, 180°)
        R_knee_static = self.rpy_to_matrix(1.5708, 1.5708, 3.142)
        R_knee_joint = self.rotation_z(theta2)  # Joint rotates around Z in joint frame
        R2 = R1 @ R_knee_static @ R_knee_joint
        T2 = T1 + R2 @ self.ankle_offset

        # Frame 2 -> Frame 3 (Ankle joint, no RPY offset)
        # URDF: rpy="0 0 0", axis="0 0 1"
        R_ankle_joint = self.rotation_z(theta3)  # Joint rotates around Z in joint frame
        R3 = R2 @ R_ankle_joint
        T3 = T2 + R3 @ self.foot_offset

        # T3 is already in base frame
        ee_position = T3

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
        return np.array([
            [1,  0,  0],
            [0,  c, -s],
            [0,  s,  c]
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
    def rpy_to_matrix(roll, pitch, yaw):
        """
        Convert RPY (roll-pitch-yaw) to rotation matrix
        Uses ZYX convention (yaw-pitch-roll order)
        """
        Rx = ForwardKinematics.rotation_x(roll)
        Ry = ForwardKinematics.rotation_y(pitch)
        Rz = ForwardKinematics.rotation_z(yaw)
        return Rz @ Ry @ Rx


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()