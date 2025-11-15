#!/usr/bin/env python3
"""
Set Point Generator Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
INPUT: /hexapod/body_velocity (Twist - body velocity)
OUTPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete foot position)
Frequency: 50 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64MultiArray


class SetPointGenerator(Node):
    def __init__(self):
        super().__init__('set_point_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('stance_x', 0.15)
        self.declare_parameter('stance_y', 0.0)
        self.declare_parameter('stance_z', -0.05)
        self.declare_parameter('update_rate', 50.0)
        
        update_rate = self.get_parameter('update_rate').value
        
        # INPUT: Subscribers
        self.phase_sub = self.create_subscription(
            Float64MultiArray, 'phase_info', self.phase_callback, 10)
        self.body_vel_sub = self.create_subscription(
            Twist, 'body_velocity', self.body_velocity_callback, 10)
        
        # OUTPUT: Publisher - setpoint for THIS leg only
        self.setpoint_pub = self.create_publisher(
            PointStamped, 'end_effector_setpoint', 10)
        
        # Timer - 50 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.generate_setpoint)
        
        self.get_logger().info('Set Point Generator initialized')
    
    def phase_callback(self, msg):
        """INPUT: Receive phase information"""
        # TODO: Store phase [type, progress, time]
        pass
    
    def body_velocity_callback(self, msg):
        """INPUT: Receive body velocity"""
        # TODO: Store body velocity
        pass
    
    def generate_setpoint(self):
        """OUTPUT: Generate setpoint - 50 Hz"""
        # TODO: Generate stance or swing trajectory based on phase
        # TODO: Publish end effector setpoint
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SetPointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()