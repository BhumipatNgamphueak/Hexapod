#!/usr/bin/env python3
"""
Test if /joint_states is publishing continuously
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
import time


class JointStatesRateTest(Node):
    def __init__(self):
        super().__init__('joint_states_rate_test')

        # QoS matching joint_state_broadcaster
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.count = 0
        self.start_time = time.time()
        self.last_msg_time = None

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.callback, qos)

        # Timer to print stats
        self.timer = self.create_timer(1.0, self.print_stats)

        self.get_logger().info('Subscribed to /joint_states with TRANSIENT_LOCAL')
        self.get_logger().info('Counting messages for 10 seconds...')

    def callback(self, msg):
        self.count += 1
        current_time = time.time()

        if self.last_msg_time is not None:
            dt = current_time - self.last_msg_time
            freq = 1.0 / dt if dt > 0 else 0
            if self.count <= 5:  # Show first 5 messages
                self.get_logger().info(f'Message #{self.count}, dt={dt*1000:.1f}ms, freq={freq:.1f}Hz')

        self.last_msg_time = current_time

    def print_stats(self):
        elapsed = time.time() - self.start_time
        avg_rate = self.count / elapsed if elapsed > 0 else 0
        self.get_logger().info(f'[{elapsed:.1f}s] Total: {self.count} msgs, Avg rate: {avg_rate:.1f} Hz')

        if elapsed >= 10:
            self.get_logger().info('=' * 50)
            self.get_logger().info(f'FINAL RESULT: {self.count} messages in {elapsed:.1f}s')
            self.get_logger().info(f'Average rate: {avg_rate:.1f} Hz')
            if avg_rate > 50:
                self.get_logger().info('✅ Publishing continuously!')
            elif self.count > 1:
                self.get_logger().info(f'⚠️  Low rate ({avg_rate:.1f} Hz)')
            else:
                self.get_logger().info('❌ Only latched message (not continuous)')
            self.get_logger().info('=' * 50)
            raise SystemExit


def main():
    rclpy.init()
    node = JointStatesRateTest()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
