#!/usr/bin/env python3
"""
Position PID Controller - FIXED WITH ANGLE WRAP-AROUND (ONE PER LEG)

CRITICAL FIXES:
1. Angle wrap-around for error computation (prevents huge errors)
2. Joint limit enforcement (prevents damage)
3. Anti-windup with limit awareness

INPUT: /hexapod/leg_X/joint_states (JointState - current joint positions)
INPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target from IK)
OUTPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - to velocity PID)
Frequency: 100 Hz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


# Joint limits from hexapod_params.xacro
JOINT_LIMITS = {
    0: {'lower': -1.57, 'upper': 1.57, 'name': 'hip'},     # ±90°
    1: {'lower': -2.0, 'upper': 2.0, 'name': 'knee'},      # ±115°
    2: {'lower': -1.57, 'upper': 1.57, 'name': 'ankle'}    # ±90°
}


def wrap_angle(angle):
    """Wrap angle to [-π, π]"""
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_difference(target, current):
    """Compute shortest angular difference (CRITICAL for PID!)"""
    return wrap_angle(target - current)


class PositionPIDController(Node):
    def __init__(self):
        super().__init__('position_pid_controller')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('position_kp', [30.0, 30.0, 30.0])
        self.declare_parameter('position_ki', [0.5, 0.5, 0.5])
        self.declare_parameter('position_kd', [3.0, 3.0, 3.0])
        self.declare_parameter('velocity_limit', 10.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('integral_limit', 2.0)
        self.declare_parameter('use_angle_wrapping', True)  # NEW
        self.declare_parameter('enforce_joint_limits', True)  # NEW

        leg_id = self.get_parameter('leg_id').value
        control_rate = self.get_parameter('control_rate').value

        # Get parameters
        self.kp = np.array(self.get_parameter('position_kp').value)
        self.ki = np.array(self.get_parameter('position_ki').value)
        self.kd = np.array(self.get_parameter('position_kd').value)
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.use_angle_wrapping = self.get_parameter('use_angle_wrapping').value
        self.enforce_limits = self.get_parameter('enforce_joint_limits').value

        self.dt = 1.0 / control_rate

        # State variables (3 joints)
        self.current_position = np.zeros(3)
        self.target_position = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.integral_error = np.zeros(3)

        # Flags
        self.position_received = False
        self.target_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_position_target', self.target_callback, 10)

        # Publishers
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_target', 10)
        
        # NEW: Publish joint targets for logging
        self.joint_targets_pub = self.create_publisher(
            Float64MultiArray, 'joint_targets', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Position PID Controller (angle-wrap aware) initialized for leg {leg_id}')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Angle wrapping: {self.use_angle_wrapping}')
        self.get_logger().info(f'Joint limit enforcement: {self.enforce_limits}')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint positions"""
        if len(msg.position) >= 3:
            self.current_position = np.array(msg.position[:3])
            self.position_received = True

    def target_callback(self, msg):
        """INPUT: Receive target joint positions from IK"""
        if len(msg.data) >= 3:
            target = np.array(msg.data[:3])
            
            # Enforce joint limits on target if enabled
            if self.enforce_limits:
                target_clamped, limits_hit = self.enforce_joint_limits(target)
                if np.any(limits_hit):
                    self.get_logger().warn(
                        f'Target clamped to limits: {limits_hit}',
                        throttle_duration_sec=1.0
                    )
                self.target_position = target_clamped
            else:
                self.target_position = target
            
            self.target_received = True

    def enforce_joint_limits(self, angles):
        """Clamp angles to joint limits"""
        clamped = np.copy(angles)
        limits_hit = np.zeros(3, dtype=bool)
        
        for i, limits in JOINT_LIMITS.items():
            if clamped[i] < limits['lower']:
                clamped[i] = limits['lower']
                limits_hit[i] = True
            elif clamped[i] > limits['upper']:
                clamped[i] = limits['upper']
                limits_hit[i] = True
        
        return clamped, limits_hit

    def control_loop(self):
        """
        OUTPUT: PID control with angle wrap-around - 100 Hz
        
        CRITICAL: Uses angle_difference() to handle wrap-around!
        
        Example of the problem this fixes:
          target = -0.1 rad, current = 6.2 rad
          WITHOUT wrap: error = -6.3 rad → HUGE velocity command!
          WITH wrap:    error = -0.083 rad → correct small adjustment
        """
        if not self.position_received or not self.target_received:
            return

        # Compute position error WITH ANGLE WRAP-AROUND
        if self.use_angle_wrapping:
            # CORRECT: Shortest angular distance
            error = np.array([
                angle_difference(self.target_position[i], self.current_position[i])
                for i in range(3)
            ])
        else:
            # WRONG: Naive difference (can be huge!)
            error = self.target_position - self.current_position

        # Integral term (with anti-windup)
        self.integral_error += error * self.dt
        
        # Anti-windup: Reset integral if we're at limits
        if self.enforce_limits:
            _, limits_hit = self.enforce_joint_limits(self.current_position)
            for i in range(3):
                if limits_hit[i]:
                    # At limit, reset integral for this joint
                    self.integral_error[i] = 0.0
        
        # Clamp integral
        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit
        )

        # Derivative term
        derivative = (error - self.previous_error) / self.dt

        # PID output (velocity command)
        velocity_command = (
            self.kp * error +
            self.ki * self.integral_error +
            self.kd * derivative
        )

        # Apply velocity limits
        velocity_command = np.clip(
            velocity_command,
            -self.velocity_limit,
            self.velocity_limit
        )

        # Update previous error
        self.previous_error = error.copy()

        # Publish velocity command
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocity_command]
        self.velocity_pub.publish(msg)
        
        # NEW: Publish joint targets for logging (position + velocity)
        target_msg = Float64MultiArray()
        target_msg.data = [
            float(self.target_position[0]),
            float(self.target_position[1]),
            float(self.target_position[2]),
            float(velocity_command[0]),
            float(velocity_command[1]),
            float(velocity_command[2])
        ]
        self.joint_targets_pub.publish(target_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()