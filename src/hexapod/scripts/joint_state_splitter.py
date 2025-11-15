#!/usr/bin/env python3
"""
Joint State Splitter Node - IMPLEMENTED
Input: /joint_states (global joint states from Gazebo - 18 joints)
Output: /hexapod/leg_{1-6}/joint_states (filtered per leg - 3 joints each)
Frequency: Callback-based (no timer)

This node splits the global joint states from Gazebo into per-leg topics.
Each leg has 3 joints: hip, knee, ankle
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('joint_state_splitter')

        # Parameters: Define joint names for each leg
        self.leg_joints = {}
        for i in range(1, 7):
            param_name = f'leg_{i}_joints'
            default_value = [f'hip_joint_{i}', f'knee_joint_{i}', f'ankle_joint_{i}']
            self.declare_parameter(param_name, default_value)
            self.leg_joints[i] = self.get_parameter(param_name).value

        # QoS Profile to match joint_state_broadcaster
        # Publisher uses: TRANSIENT_LOCAL + RELIABLE
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # INPUT: Subscribe to global joint states with matching QoS
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, qos)

        # OUTPUT: Publishers (one per leg)
        self.leg_pubs = {}
        for i in range(1, 7):
            self.leg_pubs[i] = self.create_publisher(
                JointState, f'leg_{i}/joint_states', 10)

        # Debug counters
        self.input_count = 0
        self.output_counts = {i: 0 for i in range(1, 7)}
        self.first_message = True

        self.get_logger().info('Joint State Splitter initialized')
        for i in range(1, 7):
            self.get_logger().info(f'Leg {i} joints: {self.leg_joints[i]}')

    def joint_state_callback(self, msg):
        """
        INPUT: Full joint states from Gazebo (18 joints)
        OUTPUT: Split joint states to each leg (3 joints per leg)
        """
        self.input_count += 1

        # Debug first message
        if self.first_message:
            self.get_logger().info(f'[DEBUG] First message: {len(msg.name)} joints from Gazebo')
            self.get_logger().info(f'[DEBUG] Joint names: {msg.name[:6]}...')
            self.get_logger().info(f'[DEBUG] Has velocity: {len(msg.velocity) > 0}')
            self.first_message = False

        # Process each leg
        for leg_id in range(1, 7):
            leg_msg = JointState()
            leg_msg.header = msg.header

            # Get joint names for this leg
            target_joints = self.leg_joints[leg_id]

            # Filter joints by name (if names are provided)
            if len(msg.name) > 0:
                # Joint names are available - filter by name
                for joint_name in target_joints:
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        leg_msg.name.append(joint_name)

                        # Copy position if available
                        if idx < len(msg.position):
                            leg_msg.position.append(msg.position[idx])

                        # Copy velocity if available
                        if idx < len(msg.velocity):
                            leg_msg.velocity.append(msg.velocity[idx])

                        # Copy effort if available
                        if idx < len(msg.effort):
                            leg_msg.effort.append(msg.effort[idx])

            else:
                # No joint names - use positional indexing
                # Assume joints are ordered: leg1(3), leg2(3), leg3(3), leg4(3), leg5(3), leg6(3)
                start_idx = (leg_id - 1) * 3
                end_idx = start_idx + 3

                leg_msg.name = target_joints

                # Copy position
                if end_idx <= len(msg.position):
                    leg_msg.position = list(msg.position[start_idx:end_idx])
                else:
                    self.get_logger().warn(f'Insufficient position data for leg {leg_id}')
                    continue

                # Copy velocity if available
                if end_idx <= len(msg.velocity):
                    leg_msg.velocity = list(msg.velocity[start_idx:end_idx])

                # Copy effort if available
                if end_idx <= len(msg.effort):
                    leg_msg.effort = list(msg.effort[start_idx:end_idx])

            # Publish leg-specific joint states
            if len(leg_msg.position) > 0:
                self.leg_pubs[leg_id].publish(leg_msg)
                self.output_counts[leg_id] += 1

        # Periodic status report
        if self.input_count % 100 == 0:
            self.get_logger().info(f'[STATUS] Input: {self.input_count}, Leg1 output: {self.output_counts[1]}')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()