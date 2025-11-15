#!/usr/bin/env python3
"""
Trajectory Generator Node - SKELETON (ONE PER LEG)
INPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete position)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - for timing)
OUTPUT: /hexapod/leg_X/end_effector_target (PointStamped - smooth interpolated position)
OUTPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - velocity)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('interpolation_method', 'cubic')
        self.declare_parameter('trajectory_rate', 100.0)
        self.declare_parameter('swing_clearance', 0.03)
        
        trajectory_rate = self.get_parameter('trajectory_rate').value
        
        # INPUT: Subscribers
        self.setpoint_sub = self.create_subscription(
            PointStamped, 'end_effector_setpoint', self.setpoint_callback, 10)
        
        # OUTPUT: Publishers
        self.target_pub = self.create_publisher(
            PointStamped, 'end_effector_target', 10)
        self.velocity_pub = self.create_publisher(
            Vector3Stamped, 'end_effector_velocity', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(1.0 / trajectory_rate, self.generate_trajectory)
        
        self.get_logger().info('Trajectory Generator initialized')
    
    def setpoint_callback(self, msg):
        """INPUT: Receive discrete setpoint"""
        # TODO: Store setpoint
        pass
    
    def generate_trajectory(self):
        """OUTPUT: Generate smooth trajectory - 100 Hz"""
        # TODO: Interpolate between current and target
        # TODO: Publish interpolated target
        # TODO: Publish velocity
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()