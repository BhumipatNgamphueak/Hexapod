#!/usr/bin/env python3
"""
Simple Odometry Velocity Tracker
Directly subscribes to /model/hexapod/odometry and displays velocity
WAITS PATIENTLY for Gazebo to start publishing
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np


class SimpleOdomTracker(Node):
    def __init__(self):
        super().__init__('simple_odom_tracker')
        
        # Parameters
        self.declare_parameter('display_rate', 2.0)
        self.declare_parameter('reference_position', [0.0, 0.0, 0.5])
        
        self.display_rate = self.get_parameter('display_rate').value
        self.ref_pos = np.array(self.get_parameter('reference_position').value)
        
        # State
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.data_received = False
        self.message_count = 0
        
        # Statistics
        self.max_speed = 0.0
        self.avg_speed = 0.0
        self.sample_count = 0
        
        # Subscribe directly to odometry
        self.create_subscription(
            Odometry,
            '/model/hexapod/odometry',
            self.odom_callback,
            10
        )
        
        # Display timer
        self.timer = self.create_timer(1.0 / self.display_rate, self.display_info)
        
        self.get_logger().info('Simple Odometry Tracker initialized')
        self.get_logger().info('Subscribing to: /model/hexapod/odometry')
        self.get_logger().info(f'Reference position: {self.ref_pos}')
        self.get_logger().info('Waiting for Gazebo to publish odometry...')
        self.get_logger().info('(This can take 10-30 seconds after robot spawns)')
    
    def odom_callback(self, msg):
        """Receive odometry data directly from Gazebo"""
        if not self.data_received:
            self.get_logger().info('✅ ODOMETRY DATA RECEIVED!')
            self.data_received = True
        
        self.message_count += 1
        
        # Extract position
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Extract velocity (directly from odometry - no calculation needed!)
        self.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Extract angular velocity
        self.angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
        
        # Update statistics
        speed = np.linalg.norm(self.velocity)
        self.max_speed = max(self.max_speed, speed)
        self.sample_count += 1
        self.avg_speed = ((self.avg_speed * (self.sample_count - 1)) + speed) / self.sample_count
    
    def display_info(self):
        """Display velocity information"""
        if not self.data_received:
            elapsed = self.get_clock().now().nanoseconds / 1e9
            if elapsed > 30:
                self.get_logger().warn('Still waiting... (30+ seconds)')
                self.get_logger().warn('Topic /model/hexapod/odometry may not be publishing')
                self.get_logger().warn('Try: ros2 topic echo /model/hexapod/odometry')
            else:
                self.get_logger().info(f'Waiting for odometry... ({int(elapsed)}s)')
            return
        
        # Position relative to reference
        pos_rel = self.position - self.ref_pos
        distance = np.linalg.norm(pos_rel)
        
        # Velocity magnitude
        speed = np.linalg.norm(self.velocity)
        ang_speed = np.linalg.norm(self.angular_velocity)
        
        # Display
        print('\n' + '='*70)
        print('BASE LINK VELOCITY TRACKING (Direct Odometry)')
        print('='*70)
        print(f'Position (world): [{self.position[0]:7.4f}, {self.position[1]:7.4f}, {self.position[2]:7.4f}] m')
        print(f'Position (rel):   [{pos_rel[0]:7.4f}, {pos_rel[1]:7.4f}, {pos_rel[2]:7.4f}] m')
        print(f'Distance from ref: {distance:.4f} m')
        print('-'*70)
        print(f'Linear Velocity:  [{self.velocity[0]:7.4f}, {self.velocity[1]:7.4f}, {self.velocity[2]:7.4f}] m/s')
        print(f'Speed (magnitude): {speed:.4f} m/s')
        print(f'Angular Velocity: [{self.angular_velocity[0]:7.4f}, {self.angular_velocity[1]:7.4f}, {self.angular_velocity[2]:7.4f}] rad/s')
        print(f'Ang Speed:         {ang_speed:.4f} rad/s')
        print('-'*70)
        print(f'Max Speed:   {self.max_speed:.4f} m/s')
        print(f'Avg Speed:   {self.avg_speed:.4f} m/s')
        print(f'Messages:    {self.message_count}')
        print('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
