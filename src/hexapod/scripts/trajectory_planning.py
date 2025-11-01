#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
from pydrake.all import PiecewisePolynomial


class TrajectoryPlanningNode(Node):
    def __init__(self):
        super().__init__("trajectory_planning_node")
        
        # Subscriptions
        self.create_subscription(
            PoseStamped, "foot_setpoint/pose", self.foot_pose_setpoint_callback, 10
        )
        self.create_subscription(
            PoseStamped, "foot/pose", self.foot_pose_callback, 10
        )
        
        # Publisher
        self.trajectory_pub = self.create_publisher(JointState, "foot/trajectory", 10)

        # Current foot pose (from sensor feedback)
        self.foot_pose_position = np.array([0.0, 0.0, 0.0])
        self.foot_pose_orientation = np.array([0.0, 0.0, 0.0, 1.0])

        # Current commanded position/velocity (from trajectory)
        self.current_commanded_position = np.array([0.0, 0.0, 0.0])
        self.current_commanded_velocity = np.array([0.0, 0.0, 0.0])

        # Goal pose
        self.goal_position = np.array([0.0, 0.0, 0.0])
        self.goal_orientation = np.array([0.0, 0.0, 0.0, 1.0])

        # Trajectory parameters
        self.declare_parameter('sampling_frequency', 100.0)
        self.declare_parameter('trajectory_duration', 2.0)
        
        self.sampling_frequency = float(
            self.get_parameter('sampling_frequency').get_parameter_value().double_value
        )
        self.trajectory_duration = float(
            self.get_parameter('trajectory_duration').get_parameter_value().double_value
        )
        
        # Trajectory state
        self.trajectory = None
        self.trajectory_start_time = None
        self.is_executing_trajectory = False
        
        # Timer
        self.timer = self.create_timer(1.0 / self.sampling_frequency, self.timer_callback)
        
        self.get_logger().info("Trajectory Planning Node initialized")

    def cubic_trajectory(self, start_pos, goal_pos, start_vel, duration):
        """
        Create a cubic polynomial trajectory using Drake's PiecewisePolynomial
        
        Args:
            start_pos: Starting position (numpy array)
            goal_pos: Goal position (numpy array)
            start_vel: Starting velocity (numpy array)
            duration: Duration of the trajectory in seconds
        
        Returns:
            PiecewisePolynomial trajectory
        """
        # Time breakpoints
        times = [0.0, duration]
        
        # Position samples (columns are samples at different times)
        positions = np.column_stack([start_pos, goal_pos])
        
        # Create cubic trajectory with specified start velocity and zero end velocity
        trajectory = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(
            breaks=times,
            samples=positions,
            sample_dot_at_start=start_vel,     # Continue from current velocity
            sample_dot_at_end=np.zeros(3)      # Zero velocity at end
        )
        
        return trajectory

    def start_new_trajectory(self):
        """Start a new cubic trajectory from CURRENT COMMANDED position to goal"""
        # Key change: Use current commanded position and velocity, not sensor feedback
        self.trajectory = self.cubic_trajectory(
            self.current_commanded_position,  # Start from where we are in trajectory
            self.goal_position,
            self.current_commanded_velocity,  # Continue with current velocity
            self.trajectory_duration
        )
        self.trajectory_start_time = self.get_clock().now()
        self.is_executing_trajectory = True
        
        self.get_logger().info(
            f"Starting trajectory from {self.current_commanded_position} "
            f"(vel: {self.current_commanded_velocity}) to {self.goal_position}"
        )

    def foot_pose_setpoint_callback(self, msg: PoseStamped):
        """Callback when new goal pose is received"""
        self.goal_position[0] = msg.pose.position.x
        self.goal_position[1] = msg.pose.position.y
        self.goal_position[2] = msg.pose.position.z
        self.goal_orientation[0] = msg.pose.orientation.x
        self.goal_orientation[1] = msg.pose.orientation.y
        self.goal_orientation[2] = msg.pose.orientation.z
        self.goal_orientation[3] = msg.pose.orientation.w
        
        # If not executing, initialize current commanded position
        if not self.is_executing_trajectory:
            self.current_commanded_position = self.foot_pose_position.copy()
            self.current_commanded_velocity = np.zeros(3)
        
        # Start new trajectory (will smoothly continue from current state)
        self.start_new_trajectory()

    def foot_pose_callback(self, msg: PoseStamped):
        """Callback for current foot pose (sensor feedback)"""
        self.foot_pose_position[0] = msg.pose.position.x
        self.foot_pose_position[1] = msg.pose.position.y
        self.foot_pose_position[2] = msg.pose.position.z
        self.foot_pose_orientation[0] = msg.pose.orientation.x
        self.foot_pose_orientation[1] = msg.pose.orientation.y
        self.foot_pose_orientation[2] = msg.pose.orientation.z
        self.foot_pose_orientation[3] = msg.pose.orientation.w
        
        # Initialize commanded position if this is the first update
        if not self.is_executing_trajectory and np.allclose(self.current_commanded_position, 0.0):
            self.current_commanded_position = self.foot_pose_position.copy()

    def timer_callback(self):
        """Publish trajectory waypoint at each timer tick"""
        if not self.is_executing_trajectory or self.trajectory is None:
            return
        
        # Calculate elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.trajectory_start_time).nanoseconds / 1e9
        
        # Check if trajectory is complete
        if elapsed >= self.trajectory_duration:
            self.is_executing_trajectory = False
            self.get_logger().info("Trajectory completed")
            return
        
        # Evaluate trajectory at current time
        desired_position = self.trajectory.value(elapsed).flatten()
        desired_velocity = self.trajectory.derivative(1).value(elapsed).flatten()
        desired_acceleration = self.trajectory.derivative(2).value(elapsed).flatten()
        
        # UPDATE: Store current commanded state for smooth transitions
        self.current_commanded_position = desired_position.copy()
        self.current_commanded_velocity = desired_velocity.copy()
        
        # Create JointState message
        msg = JointState()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = "world"
        
        msg.name = ["x", "y", "z"]
        msg.position = desired_position.tolist()
        msg.velocity = desired_velocity.tolist()
        msg.effort = desired_acceleration.tolist()
        
        # Publish trajectory waypoint
        self.trajectory_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()