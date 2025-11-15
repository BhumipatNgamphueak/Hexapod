#!/usr/bin/env python3
"""
State Machine Node - SKELETON
INPUT: /hexapod/gait_parameters (Float64MultiArray - gait configuration)
INPUT: /hexapod/body_velocity (Twist - body velocity)
OUTPUT: /hexapod/leg_{1-6}/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
Frequency: 50 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        for i in range(1, 7):
            self.declare_parameter(f'leg_{i}_phase_offset', 0.0)
        
        control_rate = self.get_parameter('control_rate').value
        
        # INPUT: Subscribers
        self.gait_params_sub = self.create_subscription(
            Float64MultiArray, 'gait_parameters', self.gait_params_callback, 10)
        self.body_vel_sub = self.create_subscription(
            Twist, 'body_velocity', self.body_velocity_callback, 10)
        
        # OUTPUT: Publishers - phase info (one per leg)
        self.phase_pubs = {}
        for i in range(1, 7):
            self.phase_pubs[i] = self.create_publisher(
                Float64MultiArray, f'leg_{i}/phase_info', 10)
        
        # Timer - 50 Hz
        self.timer = self.create_timer(1.0 / control_rate, self.update_state)
        
        self.get_logger().info('State Machine initialized')
    
    def gait_params_callback(self, msg):
        """INPUT: Receive gait parameters"""
        # TODO: Store gait parameters
        pass
    
    def body_velocity_callback(self, msg):
        """INPUT: Receive body velocity"""
        # TODO: Store body velocity
        pass
    
    def update_state(self):
        """OUTPUT: Update gait cycle and phases - 50 Hz"""
        # TODO: Track gait time
        # TODO: Calculate phase for each leg
        # TODO: Publish phase_info [phase_type, progress, leg_phase] for each leg
        pass


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()