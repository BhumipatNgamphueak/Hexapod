#!/usr/bin/env python3
"""
Velocity PID Controller Node - INNER LOOP of CASCADE CONTROL (ONE PER LEG)
INPUT: /hexapod/leg_X/joint_states (JointState - current joint velocities)
INPUT: /hexapod/leg_X/joint_velocity_target (Float64MultiArray - target from position PID)
INPUT: /hexapod/leg_X/joint_velocity_feedforward (Float64MultiArray - from IVK)
OUTPUT: /effort_controller_leg_X/commands (Float64MultiArray - torques to Gazebo)
Frequency: 100 Hz

CASCADE CONTROL WITH FEEDFORWARD:
  Position PID → Velocity Target ──┬──→ [Velocity PID] → Effort → Robot
                                    │      (THIS NODE)
  IVK Jacobian → Feedforward ───────┘

  Desired Velocity = Target + Feedforward
  Error = Desired - Current
  Torque = PID(Error)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np


class VelocityPIDController(Node):
    def __init__(self):
        super().__init__('velocity_pid_controller')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('velocity_kp', [5.0, 5.0, 5.0])
        self.declare_parameter('velocity_ki', [0.5, 0.5, 0.5])
        self.declare_parameter('velocity_kd', [0.1, 0.1, 0.1])
        self.declare_parameter('effort_limit', 10.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('integral_limit', 1.0)  # Anti-windup
        self.declare_parameter('use_feedforward', True)  # Enable/disable feedforward

        leg_id = self.get_parameter('leg_id').value
        control_rate = self.get_parameter('control_rate').value

        # Get PID gains
        self.kp = np.array(self.get_parameter('velocity_kp').value)
        self.ki = np.array(self.get_parameter('velocity_ki').value)
        self.kd = np.array(self.get_parameter('velocity_kd').value)
        self.effort_limit = self.get_parameter('effort_limit').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.use_feedforward = self.get_parameter('use_feedforward').value

        # Control loop timing
        self.dt = 1.0 / control_rate

        # State variables (3 joints)
        self.current_velocity = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.feedforward_velocity = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.integral_error = np.zeros(3)

        # Flags
        self.velocity_received = False
        self.target_received = False
        self.feedforward_received = False

        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_target', self.target_callback, 10)
        self.feedforward_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_feedforward', self.feedforward_callback, 10)

        # OUTPUT: Publisher to Gazebo
        self.effort_pub = self.create_publisher(Float64MultiArray, 'commands', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Velocity PID Controller initialized for leg {leg_id}')
        self.get_logger().info(f'Control rate: {control_rate} Hz (dt={self.dt*1000:.2f}ms)')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Effort limit: ±{self.effort_limit} Nm')
        self.get_logger().info(f'Feedforward: {"ENABLED" if self.use_feedforward else "DISABLED"}')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint velocities"""
        if len(msg.velocity) >= 3:
            self.current_velocity = np.array(msg.velocity[:3])
            self.velocity_received = True

    def target_callback(self, msg):
        """INPUT: Receive target velocities from Position PID"""
        if len(msg.data) >= 3:
            self.target_velocity = np.array(msg.data[:3])
            self.target_received = True

    def feedforward_callback(self, msg):
        """INPUT: Receive feedforward velocities from IVK"""
        if len(msg.data) >= 3:
            self.feedforward_velocity = np.array(msg.data[:3])
            self.feedforward_received = True

    def control_loop(self):
        """
        OUTPUT: Main PID control loop with feedforward - 100 Hz

        Cascade Control with Feedforward:
          desired_vel = target_vel + feedforward_vel
          error = desired_vel - current_vel
          torque = Kp*error + Ki*∫error + Kd*d(error)/dt

        Benefits of Feedforward:
          - Faster response (predictive component)
          - Reduced tracking error
          - Better disturbance rejection
        """
        if not self.velocity_received or not self.target_received:
            return

        # Combine target velocity with feedforward (if enabled)
        if self.use_feedforward and self.feedforward_received:
            desired_velocity = self.target_velocity + self.feedforward_velocity
        else:
            desired_velocity = self.target_velocity

        # Compute velocity error
        error = desired_velocity - self.current_velocity

        # Integral term (with anti-windup)
        self.integral_error += error * self.dt
        # Clamp integral to prevent windup
        self.integral_error = np.clip(self.integral_error,
                                      -self.integral_limit,
                                      self.integral_limit)

        # Derivative term
        derivative = (error - self.previous_error) / self.dt

        # PID output (effort/torque command)
        effort_command = (self.kp * error +
                         self.ki * self.integral_error +
                         self.kd * derivative)

        # Apply effort limits (safety)
        effort_command = np.clip(effort_command,
                                -self.effort_limit,
                                self.effort_limit)

        # Update previous error
        self.previous_error = error.copy()

        # Publish effort to Gazebo
        msg = Float64MultiArray()
        msg.data = [float(e) for e in effort_command]
        self.effort_pub.publish(msg)

    def reset_controller(self):
        """Reset PID state"""
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()