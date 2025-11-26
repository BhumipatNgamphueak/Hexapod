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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import sys
import os

# Add path for gravity compensation module
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

try:
    from gravity_compensation import GravityCompensation
    GRAVITY_COMP_AVAILABLE = True
except ImportError as e:
    print(f"⚠ Gravity compensation not available: {e}")
    GRAVITY_COMP_AVAILABLE = False


class VelocityPIDController(Node):
    def __init__(self):
        super().__init__('velocity_pid_controller')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('velocity_kp', [1.2, 1.2, 1.2])
        self.declare_parameter('velocity_ki', [0.0, 0.0, 0.0])
        self.declare_parameter('velocity_kd', [0.0, 0.0, 0.0])
        self.declare_parameter('effort_limit', 20.0)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('integral_limit', 1.0)  # Anti-windup
        self.declare_parameter('use_feedforward', True)  # Enable/disable feedforward
        self.declare_parameter('derivative_filter_alpha', 0.1)  # Derivative filter (lower = more filtering)
        self.declare_parameter('use_gravity_compensation', True)  # Enable/disable gravity compensation
        self.declare_parameter('urdf_path', '')  # Path to URDF (empty = auto-detect)

        # Advanced control parameters
        self.declare_parameter('damping_coefficient', [0.3, 0.3, 0.3])  # Viscous damping B
        self.declare_parameter('effort_rate_limit', 100.0)  # Max torque change per second (Nm/s)
        self.declare_parameter('antiwindup_gain', 1.0)  # Back-calculation gain Kb
        self.declare_parameter('gravity_filter_alpha', 0.05)  # Gravity compensation filter

        leg_id = self.get_parameter('leg_id').value
        control_rate = self.get_parameter('control_rate').value

        # Get PID gains
        self.kp = np.array(self.get_parameter('velocity_kp').value)
        self.ki = np.array(self.get_parameter('velocity_ki').value)
        self.kd = np.array(self.get_parameter('velocity_kd').value)
        self.effort_limit = self.get_parameter('effort_limit').value
        self.integral_limit = self.get_parameter('integral_limit').value
        self.use_feedforward = self.get_parameter('use_feedforward').value
        self.derivative_alpha = self.get_parameter('derivative_filter_alpha').value
        self.use_gravity_comp = self.get_parameter('use_gravity_compensation').value
        urdf_path = self.get_parameter('urdf_path').value
        if urdf_path == '':
            urdf_path = None

        # Advanced control parameters
        self.damping_coeff = np.array(self.get_parameter('damping_coefficient').value)
        self.effort_rate_limit = self.get_parameter('effort_rate_limit').value
        self.antiwindup_gain = self.get_parameter('antiwindup_gain').value
        self.gravity_alpha = self.get_parameter('gravity_filter_alpha').value

        # Control loop timing
        self.dt = 1.0 / control_rate

        # SIGN CORRECTION: REMOVED - Testing showed Knee joint has NORMAL mapping!
        # Previous assumption was WRONG - debug logs proved: +effort → +velocity
        # The RPY transform does NOT cause effort-to-velocity inversion
        # All joints have normal positive effort-to-velocity mapping
        self.effort_sign_correction = np.array([1.0, 1.0, 1.0])  # NO inversion needed!

        # State variables (3 joints)
        self.current_velocity = np.zeros(3)
        self.current_position = np.zeros(3)  # For gravity compensation
        self.target_velocity = np.zeros(3)
        self.feedforward_velocity = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.integral_error = np.zeros(3)
        self.filtered_derivative = np.zeros(3)  # Filtered derivative term
        self.previous_effort = np.zeros(3)  # For effort rate limiting
        self.filtered_gravity = np.zeros(3)  # Filtered gravity compensation

        # Initialize gravity compensation
        self.gravity_comp = None
        if self.use_gravity_comp and GRAVITY_COMP_AVAILABLE:
            try:
                self.gravity_comp = GravityCompensation(urdf_path=urdf_path, leg_id=leg_id)
                self.get_logger().info('✓ Gravity compensation ENABLED')
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize gravity compensation: {e}')
                self.get_logger().warn('  Continuing without gravity compensation')
                self.use_gravity_comp = False
        elif self.use_gravity_comp and not GRAVITY_COMP_AVAILABLE:
            self.get_logger().warn('Gravity compensation requested but Pinocchio not available')
            self.use_gravity_comp = False

        # Flags
        self.velocity_received = False
        self.target_received = False
        self.feedforward_received = False

        # QoS Profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # INPUT: Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, qos_profile)
        self.target_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_target', self.target_callback, qos_profile)
        self.feedforward_sub = self.create_subscription(
            Float64MultiArray, 'joint_velocity_feedforward', self.feedforward_callback, qos_profile)

        # OUTPUT: Publisher to Gazebo
        self.effort_pub = self.create_publisher(Float64MultiArray, 'commands', qos_profile)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(f'Velocity PID Controller initialized for leg {leg_id}')
        self.get_logger().info(f'Control rate: {control_rate} Hz (dt={self.dt*1000:.2f}ms)')
        self.get_logger().info(f'PID Gains - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')
        self.get_logger().info(f'Damping Coefficient: {self.damping_coeff}')
        self.get_logger().info(f'Effort limit: ±{self.effort_limit} Nm')
        self.get_logger().info(f'Effort rate limit: {self.effort_rate_limit} Nm/s')
        self.get_logger().info(f'Anti-windup gain (Kb): {self.antiwindup_gain}')
        self.get_logger().info(f'Feedforward: {"ENABLED" if self.use_feedforward else "DISABLED"}')
        self.get_logger().info(f'Gravity Comp: {"ENABLED" if self.use_gravity_comp else "DISABLED"}')
        self.get_logger().info(f'Derivative filter alpha: {self.derivative_alpha}')
        self.get_logger().info(f'Gravity filter alpha: {self.gravity_alpha}')
        self.get_logger().info(f'⚠ Sign correction: [Hip, Knee, Ankle] = {self.effort_sign_correction}')

    def joint_state_callback(self, msg):
        """INPUT: Receive current joint velocities and positions"""
        if len(msg.velocity) >= 3:
            self.current_velocity = np.array(msg.velocity[:3])
            self.velocity_received = True

        # Also get positions for gravity compensation
        if len(msg.position) >= 3:
            self.current_position = np.array(msg.position[:3])

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
        if not self.velocity_received or not self.target_received:
            return

        # ============================
        # STEP 1: Error
        # ============================
        error = self.target_velocity - self.current_velocity

        # DEBUG: Log every 50th iteration (~ 2 Hz @ 100 Hz)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f'[KNEE] Target={self.target_velocity[1]:.3f}, Current={self.current_velocity[1]:.3f}, Error={error[1]:.3f}')

        # ============================
        # STEP 2: Update Integral FIRST
        # ============================
        self.integral_error += error * self.dt

        # Limit integral to prevent runaway (basic anti-windup)
        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit
        )

        # ============================
        # STEP 3: PID
        # ============================
        P_term = self.kp * error
        I_term = self.ki * self.integral_error

        # Derivative (filtered)
        derivative_raw = (error - self.previous_error) / self.dt
        self.filtered_derivative = (
            self.derivative_alpha * derivative_raw +
            (1 - self.derivative_alpha) * self.filtered_derivative
        )
        D_term = self.kd * self.filtered_derivative

        # ============================
        # STEP 4: Damping Injection
        # ============================
        damping_term = -self.damping_coeff * self.current_velocity

        # ============================
        # STEP 5: Feedforward Torque (CORRECT IMPLEMENTATION)
        # ============================
        # TRUE Feedforward for velocity control: feedforward = kV * target_velocity
        # This provides the baseline effort needed to maintain the desired velocity
        # Reference: WPILib - "For velocity control, a feedforward controller
        # must be combined with a feedback controller"
        feedforward_torque = np.zeros(3)
        if self.use_feedforward:
            # Use target velocity (not error!) for feedforward
            # kV (velocity feedforward gain) is estimated from system characteristics
            # Knee needs higher kV since it does most of the work
            kV = np.array([0.3, 1.0, 0.5])  # [Hip, Knee, Ankle] - Knee increased to 1.0
            # Feedforward: Provide baseline effort needed to maintain target velocity
            feedforward_torque = kV * self.target_velocity

        # ============================
        # STEP 6: Gravity Compensation
        # ============================
        gravity_torque = np.zeros(3)
        if self.use_gravity_comp and self.gravity_comp is not None:
            try:
                raw_gravity = self.gravity_comp.compute_gravity_torques(self.current_position)
                self.filtered_gravity = (
                    self.gravity_alpha * raw_gravity +
                    (1 - self.gravity_alpha) * self.filtered_gravity
                )
                gravity_torque = self.filtered_gravity
                # Hip joint (index 0) rotates around Z-axis (vertical)
                # Gravity acts along Z → no torque on Hip joint
                gravity_torque[0] = 0.0
            except Exception:
                pass

        # ============================
        # STEP 7: Combine Effort
        # ============================
        effort_unlimited = (
            P_term + I_term + D_term +
            damping_term + feedforward_torque + gravity_torque
        )

        # ============================
        # STEP 8: Slew Rate Limiting
        # ============================
        max_delta = self.effort_rate_limit * self.dt
        delta_effort = effort_unlimited - self.previous_effort
        delta_limited = np.clip(delta_effort, -max_delta, max_delta)
        effort_rate_limited = self.previous_effort + delta_limited

        # ============================
        # STEP 9: Saturation
        # ============================
        effort_final = np.clip(
            effort_rate_limited,
            -self.effort_limit,
            self.effort_limit
        )

        # ============================
        # STEP 10: Back-Calculation Anti-Windup
        # ============================
        if self.antiwindup_gain > 0:
            saturation_error = effort_unlimited - effort_final
            self.integral_error -= (
                saturation_error / (self.antiwindup_gain + 1e-6)
            ) * self.dt

            # Re-limit after back-calc
            self.integral_error = np.clip(
                self.integral_error,
                -self.integral_limit,
                self.integral_limit
            )

        # ============================
        # STEP 11: Update State
        # ============================
        self.previous_error = error.copy()
        self.previous_effort = effort_final.copy()

        # ============================
        # STEP 12: Apply Sign Correction & Publish Effort
        # ============================
        # Apply sign correction to fix effort-to-velocity mismatch (Knee joint inverted)
        effort_corrected = effort_final * self.effort_sign_correction

        # DEBUG: Log final effort
        if self._debug_counter % 50 == 0:
            self.get_logger().info(f'[KNEE] P={P_term[1]:.3f}, I={I_term[1]:.3f}, D={D_term[1]:.3f}, Effort_final={effort_final[1]:.3f}, Effort_corrected={effort_corrected[1]:.3f}')

        msg = Float64MultiArray()
        msg.data = [float(e) for e in effort_corrected]
        self.effort_pub.publish(msg)

    def reset_controller(self):
        """Reset PID state"""
        self.integral_error = np.zeros(3)
        self.previous_error = np.zeros(3)
        self.filtered_derivative = np.zeros(3)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()