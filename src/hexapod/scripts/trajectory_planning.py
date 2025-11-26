#!/usr/bin/env python3
"""
Trajectory Generator Node - Phase-Based Trajectory (ONE PER LEG)
INPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete position)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
OUTPUT: /hexapod/leg_X/end_effector_target (PointStamped - smooth interpolated position)
OUTPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - velocity)
Frequency: 100 Hz

Trajectory Strategy:
- STANCE: Linear interpolation between start and end (2 points only)
- SWING: Sinusoidal trajectory for smooth motion
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('trajectory_rate', 100.0)
        
        self.leg_id = self.get_parameter('leg_id').value
        trajectory_rate = self.get_parameter('trajectory_rate').value
        
        self.dt = 1.0 / trajectory_rate
        
        # State variables
        self.current_setpoint = None
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.phase_type = 0  # 0: stance, 1: swing
        self.progress = 0.0  # Phase progress [0, 1]
        self.previous_phase_type = 0
        self.initialized = False
        
        # Phase trajectory state
        self.phase_start_position = np.array([0.0, 0.0, 0.0])
        self.phase_end_position = np.array([0.0, 0.0, 0.0])
        self.previous_position = np.array([0.0, 0.0, 0.0])
        
        # Velocity filtering (increase for smoother velocity)
        self.velocity_filter_alpha = 0.15  # Lower = smoother (15% new, 85% old)
        self.filtered_velocity = np.array([0.0, 0.0, 0.0])
        
        # INPUT: Subscribers
        self.setpoint_sub = self.create_subscription(
            PointStamped, f'/hexapod/leg_{self.leg_id}/end_effector_setpoint', 
            self.setpoint_callback, 10)
        self.phase_sub = self.create_subscription(
            Float64MultiArray, f'/hexapod/leg_{self.leg_id}/phase_info',
            self.phase_callback, 10)
        
        # OUTPUT: Publishers
        self.target_pub = self.create_publisher(
            PointStamped, f'/hexapod/leg_{self.leg_id}/end_effector_target', 10)
        self.velocity_pub = self.create_publisher(
            Vector3Stamped, f'/hexapod/leg_{self.leg_id}/end_effector_velocity', 10)
        
        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.generate_trajectory)
        
        self.get_logger().info(f'Trajectory Generator for leg {self.leg_id} initialized')
    
    def phase_callback(self, msg):
        """INPUT: Receive phase information"""
        # msg.data = [phase_type, progress, leg_phase]
        # phase_type: 0 = stance, 1 = swing
        new_phase_type = int(msg.data[0])
        self.progress = float(msg.data[1])
        
        # Detect phase transition
        if new_phase_type != self.previous_phase_type and self.initialized:
            self.phase_start_position = self.current_position.copy()
            if self.current_setpoint is not None:
                self.phase_end_position = self.current_setpoint.copy()
            self.get_logger().info(
                f'Phase transition: {self.previous_phase_type} -> {new_phase_type}'
            )
        
        self.previous_phase_type = self.phase_type
        self.phase_type = new_phase_type
    
    def setpoint_callback(self, msg):
        """INPUT: Receive discrete setpoint - used as phase end target"""
        new_setpoint = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        # Initialize position on first setpoint
        if not self.initialized:
            self.current_position = new_setpoint.copy()
            self.previous_position = new_setpoint.copy()
            self.current_setpoint = new_setpoint
            self.phase_start_position = new_setpoint.copy()
            self.phase_end_position = new_setpoint.copy()
            self.initialized = True
            self.get_logger().info(f'Initialized at position: {new_setpoint}')
            return
        
        # Update end position for current phase
        self.current_setpoint = new_setpoint
        self.phase_end_position = new_setpoint
    
    def _stance_trajectory(self, progress):
        """Trapezoidal velocity profile for stance phase (straight line path)"""
        # Clamp progress to [0, 1]
        t = np.clip(progress, 0.0, 1.0)
        
        # Trapezoidal profile parameters
        t1 = 0.25  # End of acceleration (25% of phase)
        t2 = 0.75  # Start of deceleration (75% of phase)
        vmax = 2.0  # Peak velocity (normalized)
        
        # Calculate position and velocity based on current progress
        if t <= t1:
            # Acceleration phase
            s = 0.5 * (vmax / t1) * t * t
            v = (vmax / t1) * t
        elif t <= t2:
            # Constant velocity phase
            s1 = 0.5 * vmax * t1
            s = s1 + vmax * (t - t1)
            v = vmax
        else:
            # Deceleration phase
            s1 = 0.5 * vmax * t1
            s2 = s1 + vmax * (t2 - t1)
            t_dec = t - t2
            t_dec_total = 1.0 - t2
            s = s2 + vmax * t_dec - 0.5 * (vmax / t_dec_total) * t_dec * t_dec
            v = vmax - (vmax / t_dec_total) * t_dec
        
        # Normalize position to [0, 1]
        s_total = 0.5 * vmax * t1 + vmax * (t2 - t1) + 0.5 * vmax * (1.0 - t2)
        s_normalized = s / s_total if s_total > 0 else 0
        
        # Calculate actual position and velocity (straight line)
        position = self.phase_start_position + s_normalized * (self.phase_end_position - self.phase_start_position)
        velocity = (v / s_total) * (self.phase_end_position - self.phase_start_position) if s_total > 0 else np.array([0.0, 0.0, 0.0])
        
        return position, velocity
    
    def _swing_trajectory(self, progress):
        """Sinusoidal trajectory for swing phase - smooth curved path"""
        # Clamp progress to [0, 1]
        t = np.clip(progress, 0.0, 1.0)
        
        # Sinusoidal interpolation: s(t) = 0.5 * (1 - cos(π*t))
        # This gives smooth start (v=0) and smooth end (v=0)
        s = 0.5 * (1.0 - np.cos(np.pi * t))
        
        # Position: interpolate using sinusoidal profile
        position = self.phase_start_position + s * (self.phase_end_position - self.phase_start_position)
        
        # Velocity: derivative of position
        # ds/dt = 0.5 * π * sin(π*t)
        # Since progress goes from 0 to 1 over the phase duration
        if t > 0 and t < 1.0:
            ds_dt = 0.5 * np.pi * np.sin(np.pi * t)
            velocity = ds_dt * (self.phase_end_position - self.phase_start_position)
        else:
            velocity = np.array([0.0, 0.0, 0.0])
        
        return position, velocity
    
    def generate_trajectory(self):
        """OUTPUT: Generate smooth trajectory - 100 Hz"""
        # Don't publish until properly initialized
        if not self.initialized:
            return
        
        # Generate position and velocity based on phase type
        if self.phase_type == 0:  # Stance phase
            self.current_position, velocity = self._stance_trajectory(self.progress)
        else:  # Swing phase
            self.current_position, velocity = self._swing_trajectory(self.progress)
        
        # Calculate velocity using numerical differentiation for accuracy
        numerical_velocity = (self.current_position - self.previous_position) / self.dt
        
        # Apply exponential smoothing filter to reduce noise
        self.filtered_velocity = (self.velocity_filter_alpha * numerical_velocity + 
                                  (1 - self.velocity_filter_alpha) * self.filtered_velocity)
        
        self.current_velocity = self.filtered_velocity.copy()
        
        # Update previous position
        self.previous_position = self.current_position.copy()
        
        # CRITICAL: Force Z velocity to zero during stance phase (foot on ground)
        if self.phase_type == 0:  # Stance phase
            self.current_velocity[2] = 0.0  # Zero Z velocity during stance
        
        # Publish interpolated target position
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = f'leg_{self.leg_id}_base'
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
