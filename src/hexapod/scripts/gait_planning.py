#!/usr/bin/env python3
"""
Gait Planner Node - Tripod Gait Only
INPUT: /cmd_vel (Twist - velocity commands)
OUTPUT: /hexapod/gait_parameters (Float64MultiArray - [gait_type, step_height, step_length, cycle_time, duty_factor])
OUTPUT: /hexapod/body_velocity (Twist - filtered velocity)
Frequency: 10 Hz

Tripod Gait Pattern:
- Group 1 (phase 0.0): Legs 1, 3, 5
- Group 2 (phase 0.5): Legs 2, 4, 6
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')
        
        # Parameters
        self.declare_parameter('default_gait_type', 0)  # 0: tripod only
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('step_height', 0.03)
        self.declare_parameter('step_length_scale', 0.75)
        self.declare_parameter('min_cycle_time', 0.5)
        self.declare_parameter('max_cycle_time', 2.0)
        self.declare_parameter('num_legs', 6)
        # self.declare_parameter('num_waypoints', 8)
        self.declare_parameter('num_waypoints', 16)
        
        # Default cmd_vel values (when no external command is received)
        self.declare_parameter('default_linear_x', 0.0)
        self.declare_parameter('default_linear_y', 0.0)
        self.declare_parameter('default_angular_z', 0.0)
        
        # Get parameters
        self.default_gait = self.get_parameter('default_gait_type').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_linear_y = self.get_parameter('max_linear_y').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.step_height = self.get_parameter('step_height').value
        self.step_length_scale = self.get_parameter('step_length_scale').value
        self.min_cycle_time = self.get_parameter('min_cycle_time').value
        self.max_cycle_time = self.get_parameter('max_cycle_time').value
        self.num_legs = self.get_parameter('num_legs').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        
        # Get default cmd_vel values
        default_linear_x = self.get_parameter('default_linear_x').value
        default_linear_y = self.get_parameter('default_linear_y').value
        default_angular_z = self.get_parameter('default_angular_z').value
        
        # State variables - initialize with default velocity
        self.current_cmd_vel = Twist()
        self.current_cmd_vel.linear.x = default_linear_x
        self.current_cmd_vel.linear.y = default_linear_y
        self.current_cmd_vel.angular.z = default_angular_z
        self.filtered_velocity = Twist()
        self.current_gait_type = self.default_gait
        self.gait_phase = 0.0  # Phase tracker [0, 1]
        self.dt = 0.1  # 10 Hz
        
        # Gait definitions - Tripod only
        self.gait_configs = {
            0: {'name': 'tripod', 'duty_factor': 0.5, 'legs_per_group': 3}
        }
        
        # INPUT: Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # OUTPUT: Publishers (only gait parameters and body velocity)
        self.gait_params_pub = self.create_publisher(
            Float64MultiArray, '/hexapod/gait_parameters', 10)
        self.body_vel_pub = self.create_publisher(
            Twist, '/hexapod/body_velocity', 10)
        
        # Timer - 10 Hz
        self.timer = self.create_timer(self.dt, self.publish_gait)
        
        self.get_logger().info(f'Gait Planner initialized with {self.num_legs} legs')
    
    def cmd_vel_callback(self, msg):
        """INPUT: Receive velocity commands"""
        # Store and clip velocity to limits
        self.current_cmd_vel.linear.x = np.clip(
            msg.linear.x, -self.max_linear_x, self.max_linear_x)
        self.current_cmd_vel.linear.y = np.clip(
            msg.linear.y, -self.max_linear_y, self.max_linear_y)
        self.current_cmd_vel.angular.z = np.clip(
            msg.angular.z, -self.max_angular_z, self.max_angular_z)
    
    def _select_gait_type(self, velocity_magnitude):
        """Select gait type - always return tripod (type 0)"""
        return 0  # Tripod only
    
    def _filter_velocity(self, alpha=0.3):
        """Apply exponential smoothing to velocity commands"""
        self.filtered_velocity.linear.x = (alpha * self.current_cmd_vel.linear.x + 
                                          (1 - alpha) * self.filtered_velocity.linear.x)
        self.filtered_velocity.linear.y = (alpha * self.current_cmd_vel.linear.y + 
                                          (1 - alpha) * self.filtered_velocity.linear.y)
        self.filtered_velocity.angular.z = (alpha * self.current_cmd_vel.angular.z + 
                                           (1 - alpha) * self.filtered_velocity.angular.z)
    
    def _calculate_step_length(self, velocity, cycle_time):
        """Calculate step length based on velocity and cycle time"""
        vx = velocity.linear.x
        vy = velocity.linear.y
        
        # Step length proportional to velocity * time
        step_length_x = vx * cycle_time * self.step_length_scale
        step_length_y = vy * cycle_time * self.step_length_scale
        
        step_length = np.sqrt(step_length_x**2 + step_length_y**2)
        return max(step_length, 0.01)  # Minimum step length
    
    def _generate_gait_path(self, step_length, step_height, num_waypoints=8):
        """Generate a walking gait path (stance + swing) similar to cubic.py"""
        waypoints = []
        
        # Stance phase: straight line on ground (moving forward)
        x_start = -step_length / 2.0
        waypoints.append([x_start, 0.0, 0.0])
        waypoints.append([x_start + step_length, 0.0, 0.0])
        
        # Swing phase: arc in the air moving backward to starting position
        x_swing_start = x_start + step_length
        x_swing_end = x_start
        
        # Create arc waypoints using parabolic shape
        for i in range(1, num_waypoints):
            t = i / num_waypoints
            x = x_swing_start - t * step_length  # Move backward
            # Parabolic arc: peaks at middle
            z = step_height * 4 * t * (1 - t)  # peaks at t=0.5
            waypoints.append([x, 0.0, z])
        
        # End of swing phase back on ground (closes the loop)
        waypoints.append([x_swing_end, 0.0, 0.0])
        
        return np.array(waypoints)
    
    def _get_leg_phase_offset(self, leg_id, gait_type):
        """Calculate phase offset for each leg - Tripod only"""
        # Leg arrangement (hexagon - clockwise from right):
        #        leg3
        #   leg4    leg2
        #        
        #   leg5    leg1
        #        leg6
        
        # Tripod gait only
        # Group 1 (phase 0.0): Legs 1, 3, 5 (alternating pattern)
        # Group 2 (phase 0.5): Legs 2, 4, 6 (alternating pattern)
        if leg_id in [1, 3, 5]:
            return 0.0
        else:
            return 0.5
    
    def publish_gait(self):
        """OUTPUT: Publish gait parameters and body velocity - 10 Hz"""
        # Filter velocity
        self._filter_velocity()
        
        # Calculate velocity magnitude
        vel_mag = np.sqrt(self.filtered_velocity.linear.x**2 + 
                         self.filtered_velocity.linear.y**2)
        
        # Select gait type
        self.current_gait_type = self._select_gait_type(vel_mag)
        gait_config = self.gait_configs[self.current_gait_type]
        
        # Calculate cycle time (slower for slower velocities)
        if vel_mag > 0.01:
            cycle_time = np.clip(1.0 / vel_mag, self.min_cycle_time, self.max_cycle_time)
        else:
            cycle_time = self.max_cycle_time
        
        # Calculate step parameters
        step_length = self._calculate_step_length(self.filtered_velocity, cycle_time)
        duty_factor = gait_config['duty_factor']
        
        # Publish gait parameters
        gait_msg = Float64MultiArray()
        gait_msg.data = [
            float(self.current_gait_type),
            float(self.step_height),
            float(step_length),
            float(cycle_time),
            float(duty_factor)
        ]
        self.gait_params_pub.publish(gait_msg)
        
        # Publish filtered body velocity
        self.body_vel_pub.publish(self.filtered_velocity)
        
        self.get_logger().debug(
            f'Gait: {gait_config["name"]}, step_length: {step_length:.3f}, '
            f'cycle_time: {cycle_time:.3f}, duty_factor: {duty_factor:.3f}'
        )
    
def main(args=None):
    rclpy.init(args=args)
    node = GaitPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()