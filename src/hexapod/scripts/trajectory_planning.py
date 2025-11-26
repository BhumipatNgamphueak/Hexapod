#!/usr/bin/env python3
"""
Trajectory Generator Node - Exponential Smoothing (ONE PER LEG)
INPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete position)
OUTPUT: /hexapod/leg_X/end_effector_target (PointStamped - smooth interpolated position)
OUTPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - velocity)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
import numpy as np


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('trajectory_rate', 100.0)
        self.declare_parameter('smoothing_alpha', 0.1)  # Lower = smoother
        
        self.leg_id = self.get_parameter('leg_id').value
        trajectory_rate = self.get_parameter('trajectory_rate').value
        self.alpha = self.get_parameter('smoothing_alpha').value
        
        self.dt = 1.0 / trajectory_rate
        
        # State variables
        self.current_setpoint = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.previous_position = np.array([0.0, 0.0, 0.0])
        self.initialized = False
        
        # INPUT: Subscribers
        self.setpoint_sub = self.create_subscription(
            PointStamped, 'end_effector_setpoint', self.setpoint_callback, 10)
        
        # OUTPUT: Publishers
        self.target_pub = self.create_publisher(
            PointStamped, 'end_effector_target', 10)
        self.velocity_pub = self.create_publisher(
            Vector3Stamped, 'end_effector_velocity', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.generate_trajectory)
        
        self.get_logger().info(
            f'Trajectory Generator (Exponential Smoothing) for leg {self.leg_id} initialized'
            f' with alpha={self.alpha}'
        )
    
    def setpoint_callback(self, msg):
        """INPUT: Receive discrete setpoint"""
        self.current_setpoint = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        # Initialize position on first setpoint
        if not self.initialized:
            self.current_position = self.current_setpoint.copy()
            self.previous_position = self.current_setpoint.copy()
            self.initialized = True
            self.get_logger().info(f'Initialized at position: {self.current_position}')
    
    def generate_trajectory(self):
        """OUTPUT: Generate smooth trajectory using exponential smoothing - 100 Hz"""
        if not self.initialized:
            return
        
        # Store previous position for velocity calculation
        self.previous_position = self.current_position.copy()
        
        # Exponential smoothing: position = alpha * setpoint + (1 - alpha) * previous_position
        # This acts as a low-pass filter, removing high-frequency noise
        self.current_position = (
            self.alpha * self.current_setpoint + 
            (1.0 - self.alpha) * self.current_position
        )
        
        # Calculate velocity using finite difference
        self.current_velocity = (self.current_position - self.previous_position) / self.dt
        
        # Publish smoothed target position
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'base_link'
        target_msg.point.x = float(self.current_position[0])
        target_msg.point.y = float(self.current_position[1])
        target_msg.point.z = float(self.current_position[2])
        self.target_pub.publish(target_msg)
        
        # Publish velocity
        vel_msg = Vector3Stamped()
        vel_msg.header.stamp = target_msg.header.stamp
        vel_msg.header.frame_id = target_msg.header.frame_id
        vel_msg.vector.x = float(self.current_velocity[0])
        vel_msg.vector.y = float(self.current_velocity[1])
        vel_msg.vector.z = float(self.current_velocity[2])
        self.velocity_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()