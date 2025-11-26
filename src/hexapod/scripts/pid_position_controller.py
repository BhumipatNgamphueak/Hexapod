#!/usr/bin/env python3
"""
Position PID Controller Node - OUTER LOOP of CASCADE CONTROL (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joint positions)
INPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - target positions from IK)
OUTPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - velocity commands to inner loop)
Frequency: 100 Hz

CASCADE CONTROL:
  Target Position → [Position PID] → Target Velocity → [Velocity PID] → Effort
                      (THIS NODE)         ↓               (Inner Loop)
                                    (To velocity controller)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class PositionPIDController(Node):
    def __init__(self):
        super().__init__('position_pid_controller')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('position_kp', [10.0, 10.0, 10.0])
        self.declare_parameter('position_ki', [0.1, 0.1, 0.1])
        self.declare_parameter('position_kd', [1.0, 1.0, 1.0])
        self.declare_parameter('velocity_limit', 5.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('integral_limit', 2.0)  # Anti-windup
        self.declare_parameter('derivative_filter_alpha', 0.3)  # Derivative filter

        leg_id = self.get_parameter('leg_id').value
        control_rate = self.get_parameter('control_rate').value

        # Get PID gains (3 joints: hip, knee, ankle)
        self.kp = np.array(self.get_parameter('position_kp').value)
        self.ki = np.array(self.get_parameter('position_ki').value)
        self.kd = np.array(self.get_parameter('position_kd').value)
        self.velocity_limit = self.get_parameter('velocity_limit').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.derivative_alpha = self.get_parameter('derivative_filter_alpha').value

        # Control loop timing
        self.dt = 1.0 / control_rate

        # State variables (3 joints)
        self.current_position = np.zeros(3)
        self.target_position = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.integral_error = np.zeros(3)
        self.filtered_derivative = np.zeros(3)  # Filtered derivative term

        # Flags
        self.position_received = False
        self.target_received = False

        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_position_target', self.target_callback, 10)

        # OUTPUT: Publisher
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, 'joint_velocity_target', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Position PID Controller initialized for leg {leg_id}')
        self.get_logger().info(f'Control rate: {control_rate} Hz (dt={self.dt*1000:.2f}ms)')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Velocity limit: ±{self.velocity_limit} rad/s')
        self.get_logger().info(f'Derivative filter alpha: {self.derivative_alpha}')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint positions"""
        if len(msg.position) >= 3:
            self.current_position = np.array(msg.position[:3])
            self.position_received = True

    def target_callback(self, msg):
        """INPUT: Receive target joint positions from IK"""
        if len(msg.data) >= 3:
            self.target_position = np.array(msg.data[:3])
            self.target_received = True

    def control_loop(self):
        """
        OUTPUT: Main PID control loop - 100 Hz

        PID Control Law:
          error = target - current
          integral += error * dt
          derivative = (error - prev_error) / dt
          output = Kp * error + Ki * integral + Kd * derivative
        """
        if not self.position_received or not self.target_received:
            return

        # Compute position error for each joint
        error = self.target_position - self.current_position

        # Integral term (with anti-windup)
        self.integral_error += error * self.dt
        # Clamp integral to prevent windup
        self.integral_error = np.clip(self.integral_error,
                                      -self.integral_limit,
                                      self.integral_limit)

        # Derivative term with low-pass filter to reduce noise
        derivative_raw = (error - self.previous_error) / self.dt
        self.filtered_derivative = (self.derivative_alpha * derivative_raw +
                                   (1 - self.derivative_alpha) * self.filtered_derivative)

        # PID output (velocity command)
        velocity_command = (self.kp * error +
                           self.ki * self.integral_error +
                           self.kd * self.filtered_derivative)

        # Apply velocity limits (safety)
        velocity_command = np.clip(velocity_command,
                                   -self.velocity_limit,
                                   self.velocity_limit)

        # Update previous error for next iteration
        self.previous_error = error.copy()

        # Publish velocity target to inner loop (Velocity PID)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in velocity_command]
        self.velocity_pub.publish(msg)

    def reset_controller(self):
        """Reset PID state (useful when target changes drastically)"""
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.filtered_derivative = np.zeros(3)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()