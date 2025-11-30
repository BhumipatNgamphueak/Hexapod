#!/usr/bin/env python3
"""
Joint State Splitter Node - FIXED for ROS 2 Humble
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from typing import Dict, List


class JointStateSplitter(Node):
    def __init__(self):
        super().__init__('joint_state_splitter')

        self.leg_joint_map: Dict[int, List[str]] = {}
        self._warned_missing: set = set()  # Track which legs have already warned

        for leg in range(1, 7):
            param_name = f'leg_{leg}_joints'
            default = [f'hip_joint_{leg}', f'knee_joint_{leg}', f'ankle_joint_{leg}']
            self.declare_parameter(param_name, default)
            joints = self.get_parameter(param_name).get_parameter_value().string_array_value
            self.leg_joint_map[leg] = list(joints)

            self.get_logger().info(f'Leg {leg} joints: {self.leg_joint_map[leg]}')

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.leg_pubs: Dict[int, rclpy.publishing.Publisher] = {}
        for leg in range(1, 7):
            topic = f'leg_{leg}/joint_states'
            self.leg_pubs[leg] = self.create_publisher(JointState, topic, 10)

        self.get_logger().info('JointStateSplitter initialized – waiting for /joint_states')

    def joint_state_callback(self, msg: JointState) -> None:
        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}

        for leg, wanted_names in self.leg_joint_map.items():
            missing = [n for n in wanted_names if n not in name_to_idx]
            if missing:
                warn_key = f'leg_{leg}_missing'
                if warn_key not in self._warned_missing:
                    self.get_logger().warn(
                        f'Leg {leg}: missing joints in /joint_states: {missing}'
                    )
                    self._warned_missing.add(warn_key)
                continue

            leg_msg = JointState()
            leg_msg.header = msg.header

            for joint_name in wanted_names:
                idx = name_to_idx[joint_name]
                leg_msg.name.append(joint_name)

                if msg.position:
                    leg_msg.position.append(msg.position[idx])
                if msg.velocity:
                    leg_msg.velocity.append(msg.velocity[idx])
                if msg.effort:
                    leg_msg.effort.append(msg.effort[idx])

            self.leg_pubs[leg].publish(leg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()