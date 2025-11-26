#!/usr/bin/env python3
"""
Trajectory Controller Bridge - Converts IK output to Multi-Point JointTrajectory messages
INPUT: /hexapod/leg_X/joint_position_target (Float64MultiArray - from IK)
OUTPUT: /joint_trajectory_controller_leg_X/joint_trajectory (JointTrajectory - to controller)
Frequency: Callback-based

IMPROVED: Multi-point trajectory generation for smoother motion
- Buffers recent targets
- Creates intermediate waypoints
- Produces smooth trajectories with proper time spacing
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import numpy as np
from collections import deque


class TrajectoryControllerBridge(Node):
    def __init__(self):
        super().__init__('trajectory_controller_bridge')

        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('trajectory_duration', 0.5)  # Total duration
        self.declare_parameter('num_waypoints', 3)  # Number of intermediate points
        self.declare_parameter('use_action_interface', False)

        leg_id = self.get_parameter('leg_id').value
        self.traj_duration = self.get_parameter('trajectory_duration').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.use_action = self.get_parameter('use_action_interface').value

        # Joint names for this leg
        self.joint_names = [
            f'hip_joint_{leg_id}',
            f'knee_joint_{leg_id}',
            f'ankle_joint_{leg_id}'
        ]

        # State tracking
        self.current_position = None  # Current joint positions
        self.target_position = None   # Latest target from IK
        self.position_received = False

        # INPUT: Subscribe to current joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/hexapod/leg_{leg_id}/joint_states',
            self.joint_state_callback,
            10
        )

        # INPUT: Subscribe to IK output
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            f'/hexapod/leg_{leg_id}/joint_position_target',
            self.target_callback,
            10
        )

        if self.use_action:
            # OUTPUT: Action client
            self.action_client = ActionClient(
                self,
                FollowJointTrajectory,
                f'/joint_trajectory_controller_leg_{leg_id}/follow_joint_trajectory'
            )
            self.get_logger().info(f'Waiting for action server...')
            self.action_client.wait_for_server()
        else:
            # OUTPUT: Topic publisher
            self.trajectory_pub = self.create_publisher(
                JointTrajectory,
                f'/joint_trajectory_controller_leg_{leg_id}/joint_trajectory',
                10
            )

        self.get_logger().info(f'Multi-Point Trajectory Bridge initialized for leg {leg_id}')
        self.get_logger().info(f'Waypoints: {self.num_waypoints}, Duration: {self.traj_duration}s')

    def joint_state_callback(self, msg):
        """Receive current joint positions"""
        if len(msg.position) >= 3:
            self.current_position = np.array(msg.position[:3])
            self.position_received = True

    def target_callback(self, msg):
        """
        Receive target from IK and create multi-point smooth trajectory

        LOGIC:
        1. Get current position (start point)
        2. Get target position (end point)
        3. Create N waypoints between start and end using linear interpolation
        4. Assign time stamps with equal spacing
        5. Optionally add velocity/acceleration profiles
        """
        if len(msg.data) < 3:
            self.get_logger().warn('Invalid joint target (less than 3 joints)')
            return

        # Store target
        self.target_position = np.array(msg.data[:3])

        # Wait for current position
        if not self.position_received or self.current_position is None:
            self.get_logger().warn('Current position not available yet, using target as single point')
            # Fallback: send single-point trajectory
            self.send_single_point_trajectory(self.target_position)
            return

        # Create multi-point trajectory
        trajectory = self.create_smooth_trajectory(
            self.current_position,
            self.target_position,
            self.num_waypoints
        )

        # Send trajectory
        if self.use_action:
            self.send_trajectory_action(trajectory)
        else:
            self.trajectory_pub.publish(trajectory)

    def create_smooth_trajectory(self, start_pos, end_pos, num_points):
        """
        Create smooth multi-point trajectory from start to end

        LOGIC:
        - Linear interpolation between start and end positions
        - Time spacing: Distribute points evenly across total duration
        - Each point has: position, velocity (optional), acceleration (optional), time

        Example with 3 waypoints:
          t=0.0s:   start position
          t=0.25s:  25% progress
          t=0.50s:  50% progress (midpoint)
          t=0.75s:  75% progress
          t=1.0s:   end position (target)
        """
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names

        # Total number of points = num_waypoints + start + end
        # But we skip the start (current position) since controller already knows it
        # So we send: intermediate points + end point
        total_points = num_points + 1  # waypoints + final target

        # Time increment between points
        dt = self.traj_duration / total_points

        # Create waypoints
        for i in range(1, total_points + 1):
            point = JointTrajectoryPoint()

            # Linear interpolation: pos = start + (end - start) * ratio
            ratio = i / total_points
            interpolated_pos = start_pos + (end_pos - start_pos) * ratio
            point.positions = [float(p) for p in interpolated_pos]

            # Optionally compute velocities for smoother motion
            # velocity ≈ (end - start) / duration
            if i < total_points:  # All points except last
                velocity = (end_pos - start_pos) / self.traj_duration
                point.velocities = [float(v) for v in velocity]
            else:  # Last point: velocity = 0 (stop)
                point.velocities = [0.0, 0.0, 0.0]

            # Time from start
            time_sec = dt * i
            point.time_from_start = Duration(
                sec=int(time_sec),
                nanosec=int((time_sec % 1) * 1e9)
            )

            trajectory.points.append(point)

        return trajectory

    def send_single_point_trajectory(self, target_pos):
        """Fallback: send single-point trajectory (original behavior)"""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in target_pos]
        point.time_from_start = Duration(
            sec=int(self.traj_duration),
            nanosec=int((self.traj_duration % 1) * 1e9)
        )

        trajectory.points = [point]

        if self.use_action:
            self.send_trajectory_action(trajectory)
        else:
            self.trajectory_pub.publish(trajectory)

    def send_trajectory_action(self, trajectory):
        """Send trajectory via action interface"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
