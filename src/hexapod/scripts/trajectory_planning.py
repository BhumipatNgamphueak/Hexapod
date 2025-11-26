#!/usr/bin/env python3
"""
Trajectory Generator Node - Cubic Hermite Interpolation (ONE PER LEG)
INPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete position)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - for timing)
OUTPUT: /hexapod/leg_X/end_effector_target (PointStamped - smooth interpolated position)
OUTPUT: /hexapod/leg_X/end_effector_velocity (Vector3Stamped - velocity)
Frequency: 100 Hz

FIXED ISSUES:
1. Frame consistency: Now uses 'base_link' throughout (matching gait_planner and IK)
2. Smooth trajectory updates: Doesn't clear buffer aggressively, allows trajectory to complete
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from collections import deque


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('interpolation_method', 'cubic')
        self.declare_parameter('trajectory_rate', 100.0)
        self.declare_parameter('swing_clearance', 0.03)
        self.declare_parameter('vmax', 1.0)
        self.declare_parameter('amax', 2.0)
        self.declare_parameter('buffer_threshold', 0.2)  # NEW: When to accept new setpoint (20% remaining)

        self.leg_id = self.get_parameter('leg_id').value
        trajectory_rate = self.get_parameter('trajectory_rate').value
        self.vmax = self.get_parameter('vmax').value
        self.amax = self.get_parameter('amax').value
        self.swing_clearance = self.get_parameter('swing_clearance').value
        self.buffer_threshold = self.get_parameter('buffer_threshold').value

        self.dt = 1.0 / trajectory_rate

        # State variables
        self.current_setpoint = None
        self.previous_setpoint = None
        self.pending_setpoint = None  # NEW: Queue next setpoint instead of overwriting
        self.trajectory_buffer = deque()
        self.current_time = 0.0
        self.trajectory_start_time = 0.0
        self.trajectory_total_points = 0  # NEW: Track total points for completion ratio
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])

        # INPUT: Subscribers
        self.setpoint_sub = self.create_subscription(
            PointStamped, 'end_effector_setpoint', self.setpoint_callback, 10)

        # OUTPUT: Publishers
        self.target_pub = self.create_publisher(
            PointStamped, 'end_effector_target', 10)
        self.velocity_pub = self.create_publisher(
            Vector3Stamped, 'end_effector_velocity', 10)

        # Timer - 100 Hz
        self.timer = self.create_timer(self.dt, self.generate_trajectory)

        self.get_logger().info(f'Trajectory Generator for leg {self.leg_id} initialized')
        self.get_logger().info(f'FIXED: Frame = base_link (consistent with gait_planner + IK)')
        self.get_logger().info(f'FIXED: Smooth trajectory updates (buffer_threshold = {self.buffer_threshold})')

    def setpoint_callback(self, msg):
        """
        INPUT: Receive discrete setpoint and generate trajectory

        FIXED LOGIC:
        - Only accept new setpoint if current trajectory is near completion (< buffer_threshold remaining)
        - Otherwise, queue it as pending_setpoint to avoid abrupt path changes
        """
        new_setpoint = np.array([msg.point.x, msg.point.y, msg.point.z])

        # Check if we should accept this setpoint now or queue it
        buffer_remaining = len(self.trajectory_buffer)
        buffer_ratio = buffer_remaining / max(self.trajectory_total_points, 1)

        if buffer_ratio < self.buffer_threshold or self.trajectory_total_points == 0:
            # Trajectory is near completion or empty -> accept new setpoint
            self._accept_new_setpoint(new_setpoint)
            self.get_logger().debug(
                f'Accepted new setpoint immediately (buffer {buffer_remaining}/{self.trajectory_total_points})'
            )
        else:
            # Trajectory still in progress -> queue as pending
            self.pending_setpoint = new_setpoint
            self.get_logger().debug(
                f'Queued pending setpoint (buffer {buffer_remaining}/{self.trajectory_total_points})'
            )

    def _accept_new_setpoint(self, new_setpoint):
        """Accept and process new setpoint"""
        # Store previous setpoint
        if self.current_setpoint is not None:
            self.previous_setpoint = self.current_setpoint.copy()
        else:
            self.previous_setpoint = self.current_position.copy()

        self.current_setpoint = new_setpoint

        # Generate new trajectory from current position to new setpoint
        self._generate_trajectory_buffer()

        # Clear pending since we just accepted it
        self.pending_setpoint = None

    def _generate_trajectory_buffer(self):
        """Generate cubic Hermite trajectory between waypoints"""
        if self.current_setpoint is None:
            return

        # Create waypoints: current -> target
        waypoints = np.array([self.current_position, self.current_setpoint])

        # Generate trajectory using cubic Hermite interpolation
        times, positions, velocities = self._cubic_hermite_trajectory(
            waypoints, self.vmax, self.amax, self.dt
        )

        # Clear and refill trajectory buffer
        self.trajectory_buffer.clear()
        for i in range(len(times)):
            self.trajectory_buffer.append({
                'position': positions[i],
                'velocity': velocities[i],
                'time': times[i]
            })

        self.trajectory_start_time = self.current_time
        self.trajectory_total_points = len(self.trajectory_buffer)  # Track total points

        self.get_logger().debug(
            f'Generated trajectory with {len(self.trajectory_buffer)} points'
        )

    def _cubic_hermite_trajectory(self, waypoints, vmax, amax, dt):
        """Generate cubic Hermite interpolated trajectory"""
        N = len(waypoints)
        if N < 2:
            return np.array([0.0]), waypoints, np.zeros_like(waypoints)

        # Calculate segment distances and durations
        segments = []
        for i in range(N - 1):
            p0 = waypoints[i]
            p1 = waypoints[i + 1]
            dist = np.linalg.norm(p1 - p0)

            # Plan duration using trapezoidal profile
            duration_info = self._plan_trapezoid_duration(dist, 0.0, 0.0, vmax, amax)
            duration = max(duration_info['T'], dt)

            segments.append({
                'p0': p0,
                'p1': p1,
                'duration': duration
            })

        # Estimate velocities at waypoints (zero for start/end)
        velocities = np.zeros_like(waypoints)
        # Keep zero velocities at endpoints for stop-at-waypoints behavior

        # Generate trajectory samples
        all_times = []
        all_positions = []
        all_velocities = []
        time_offset = 0.0

        for i, seg in enumerate(segments):
            p0 = seg['p0']
            p1 = seg['p1']
            v0 = velocities[i]
            v1 = velocities[i + 1]
            T = seg['duration']

            t_seg, pos_seg, vel_seg = self._hermite_segment(p0, p1, v0, v1, T, dt)

            if len(t_seg) > 0:
                all_times.append(time_offset + t_seg)
                all_positions.append(pos_seg)
                all_velocities.append(vel_seg)
                time_offset += T

        if len(all_times) == 0:
            return np.array([0.0]), waypoints[[0]], np.zeros((1, 3))

        times = np.concatenate(all_times)
        positions = np.vstack(all_positions)
        velocities_out = np.vstack(all_velocities)

        return times, positions, velocities_out

    def _plan_trapezoid_duration(self, s, u, v, vmax, amax):
        """Plan trapezoidal velocity profile duration"""
        if s <= 0:
            return {'t_acc': 0.0, 't_cruise': 0.0, 't_dec': 0.0, 'T': 0.0}

        # Acceleration and deceleration times
        t_acc = max(0.0, (vmax - u) / amax)
        t_dec = max(0.0, (vmax - v) / amax)
        s_acc = u * t_acc + 0.5 * amax * t_acc ** 2
        s_dec = vmax * t_dec - 0.5 * amax * t_dec ** 2

        if s_acc + s_dec <= s:
            # Can reach vmax
            s_cruise = s - s_acc - s_dec
            t_cruise = s_cruise / vmax if vmax > 0 else 0.0
            T = t_acc + t_cruise + t_dec
            return {'t_acc': t_acc, 't_cruise': t_cruise, 't_dec': t_dec, 'T': T}
        else:
            # Cannot reach vmax - triangular profile
            v_peak = np.sqrt((2 * s * amax + u**2 + v**2) / 2)
            t_acc = (v_peak - u) / amax
            t_dec = (v_peak - v) / amax
            T = t_acc + t_dec
            return {'t_acc': t_acc, 't_cruise': 0.0, 't_dec': t_dec, 'T': T}

    def _hermite_segment(self, p0, p1, v0, v1, T, dt):
        """Generate Hermite segment"""
        N = int(np.ceil(T / dt))
        if N == 0:
            return np.array([]), np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

        times = np.linspace(0, T, N)
        tau = times / T  # Normalized time [0, 1]
        tau2 = tau ** 2
        tau3 = tau ** 3

        # Hermite basis functions
        h00 = 2 * tau3 - 3 * tau2 + 1
        h10 = tau3 - 2 * tau2 + tau
        h01 = -2 * tau3 + 3 * tau2
        h11 = tau3 - tau2

        # Position interpolation
        positions = (h00.reshape(-1, 1) * p0 +
                    h10.reshape(-1, 1) * (T * v0) +
                    h01.reshape(-1, 1) * p1 +
                    h11.reshape(-1, 1) * (T * v1))

        # Velocity interpolation (derivatives of Hermite basis)
        dh00 = (6 * tau2 - 6 * tau) / T
        dh10 = (3 * tau2 - 4 * tau + 1)
        dh01 = (-6 * tau2 + 6 * tau) / T
        dh11 = (3 * tau2 - 2 * tau)

        velocities = (dh00.reshape(-1, 1) * p0 +
                     dh10.reshape(-1, 1) * (T * v0) +
                     dh01.reshape(-1, 1) * p1 +
                     dh11.reshape(-1, 1) * (T * v1))

        return times, positions, velocities

    def generate_trajectory(self):
        """
        OUTPUT: Generate smooth trajectory - 100 Hz

        FIXED: Checks for pending setpoint when buffer is near empty
        """
        self.current_time += self.dt

        # If we have trajectory points, use them
        if len(self.trajectory_buffer) > 0:
            point = self.trajectory_buffer.popleft()
            self.current_position = point['position']
            self.current_velocity = point['velocity']

            # Check if we should accept pending setpoint (buffer nearly empty)
            buffer_remaining = len(self.trajectory_buffer)
            buffer_ratio = buffer_remaining / max(self.trajectory_total_points, 1)

            if buffer_ratio < self.buffer_threshold and self.pending_setpoint is not None:
                self.get_logger().debug('Accepting pending setpoint (buffer nearly empty)')
                self._accept_new_setpoint(self.pending_setpoint)

        elif self.current_setpoint is not None:
            # Hold at last setpoint
            self.current_position = self.current_setpoint
            self.current_velocity = np.array([0.0, 0.0, 0.0])

            # Accept pending if available
            if self.pending_setpoint is not None:
                self.get_logger().debug('Accepting pending setpoint (buffer empty)')
                self._accept_new_setpoint(self.pending_setpoint)

        # FIXED: Publish with 'base_link' frame (consistent with gait_planner and IK)
        target_msg = PointStamped()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = 'base_link'  # FIXED: Was f'leg_{self.leg_id}_base'
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
