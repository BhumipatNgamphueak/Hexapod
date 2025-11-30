#!/usr/bin/env python3
"""
Inverse Position Kinematic Node - PDF MODEL WITH URDF CORRECTION
INPUT: /hexapod/leg_X/end_effector_target (PointStamped - desired foot position)
OUTPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target joint angles)
Frequency: Callback-based

CORRECTION: PDF frame offset to match URDF geometry
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from fk_pdf_model import ik_pdf_analytical

# PDF to URDF offset correction (in leg frame)
# This compensates for the difference between PDF DH frame and URDF geometry
PDF_TO_URDF_OFFSET = np.array([0.18, -0.1985, 0.1399])


class InversePositionKinematic(Node):
    def __init__(self):
        super().__init__('inverse_position_kinematic')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('elbow_up', True)  # Default to elbow-up solution

        # Transformation from base to leg frame (leg-specific!)
        self.declare_parameter('hip_xyz_x', -1.2616e-05)
        self.declare_parameter('hip_xyz_y', -0.095255)
        self.declare_parameter('hip_xyz_z', 0.0)
        self.declare_parameter('hip_rpy_roll', 0.0)
        self.declare_parameter('hip_rpy_pitch', 0.0)
        self.declare_parameter('hip_rpy_yaw', -1.5708)

        leg_id = self.get_parameter('leg_id').value
        self.elbow_up = self.get_parameter('elbow_up').value

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

        # Invert to get T_leg_base (for transforming target from base to leg frame)
        self.T_leg_base = np.linalg.inv(self.T_base_leg)

        # Subscribers & Publishers
        self.target_sub = self.create_subscription(
            PointStamped, 'end_effector_target', self.target_callback, 10)
        self.joint_target_pub = self.create_publisher(
            Float64MultiArray, 'joint_position_target', 10)

        self.get_logger().info(f'IK (PDF model with Float64MultiArray) initialized for leg {leg_id}')
        self.get_logger().info(f'Default solution: {"elbow-up" if self.elbow_up else "elbow-down"}')

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

    def target_callback(self, msg):
        """INPUT: Receive target position and compute IK"""
        # Target in base frame
        p_base = np.array([msg.point.x, msg.point.y, msg.point.z, 1.0])

        # Transform to leg frame (URDF frame)
        p_leg_urdf = self.T_leg_base @ p_base

        # Remove PDF to URDF offset to get position in PDF frame
        p_leg_pdf = p_leg_urdf[:3] - PDF_TO_URDF_OFFSET
        px, py, pz = p_leg_pdf[0], p_leg_pdf[1], p_leg_pdf[2]

        try:
            # Try primary solution (elbow-up or elbow-down based on parameter)
            sol = ik_pdf_analytical(px, py, pz, elbow_up=self.elbow_up)

            # If failed, try alternative solution
            if sol is None:
                sol = ik_pdf_analytical(px, py, pz, elbow_up=not self.elbow_up)

            if sol is not None:
                # Publish joint targets
                joint_msg = Float64MultiArray()
                joint_msg.data = [float(sol[0]), float(sol[1]), float(sol[2])]
                self.joint_target_pub.publish(joint_msg)
            else:
                self.get_logger().warn(
                    f'IK failed for target PDF({px:.3f}, {py:.3f}, {pz:.3f}) '
                    f'URDF({p_leg_urdf[0]:.3f}, {p_leg_urdf[1]:.3f}, {p_leg_urdf[2]:.3f}) - unreachable',
                    throttle_duration_sec=1.0
                )

        except Exception as e:
            self.get_logger().error(f'IK computation error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = InversePositionKinematic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
