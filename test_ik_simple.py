#!/usr/bin/env python3
"""
Simple test to check if IK computation works
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray


class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')

        # Subscribe to IK output
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/hexapod/leg_1/joint_position_target',
            self.ik_result_callback,
            10
        )

        # Publish target
        self.pub = self.create_publisher(
            PointStamped,
            '/hexapod/leg_1/end_effector_target',
            10
        )

        # Send target after 1 second
        self.timer = self.create_timer(1.0, self.send_target)
        self.target_sent = False

        self.get_logger().info('IK Tester initialized - waiting to send target...')

    def send_target(self):
        if not self.target_sent:
            msg = PointStamped()
            msg.header.frame_id = 'base_link'
            msg.point.x = 0.15
            msg.point.y = -0.1
            msg.point.z = -0.05

            self.pub.publish(msg)
            self.get_logger().info(f'Target sent: ({msg.point.x}, {msg.point.y}, {msg.point.z})')
            self.target_sent = True

    def ik_result_callback(self, msg):
        self.get_logger().info(f'IK Result received: {msg.data}')
        self.get_logger().info('✅ IK node is working!')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = IKTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
