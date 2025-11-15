#!/usr/bin/env python3
"""
Gait Planner Node - SKELETON
INPUT: /cmd_vel (Twist - velocity commands)
OUTPUT: /hexapod/gait_parameters (Float64MultiArray - [gait_type, step_height, step_length, cycle_time, duty_factor])
OUTPUT: /hexapod/body_velocity (Twist - filtered velocity)
Frequency: 10 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')
        
        # Parameters
        self.declare_parameter('default_gait_type', 0)
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('step_height', 0.03)
        self.declare_parameter('step_length_scale', 0.05)
        self.declare_parameter('cycle_time', 1.0)
        
        # INPUT: Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # OUTPUT: Publishers
        self.gait_params_pub = self.create_publisher(
            Float64MultiArray, 'gait_parameters', 10)
        self.body_vel_pub = self.create_publisher(
            Twist, 'body_velocity', 10)
        
        # Timer - 10 Hz
        self.timer = self.create_timer(0.1, self.publish_gait)
        
        self.get_logger().info('Gait Planner initialized')
    
    def cmd_vel_callback(self, msg):
        """INPUT: Receive velocity commands"""
        # TODO: Store and clip velocity
        pass
    
    def publish_gait(self):
        """OUTPUT: Publish gait parameters - 10 Hz"""
        # TODO: Select gait based on velocity
        # TODO: Calculate step parameters
        # TODO: Publish gait_parameters [gait_type, step_height, step_length, cycle_time, duty_factor]
        # TODO: Publish body_velocity
        pass


def main(args=None):
    rclpy.init(args=args)
    node = GaitPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()