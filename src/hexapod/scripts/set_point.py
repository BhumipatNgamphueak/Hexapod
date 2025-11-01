#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DummyNode(Node):
    def __init__(self):
        super().__init__('set_point_node')
        self.set_point_pose_pub = self.create_publisher(PoseStamped, 'foot_setpoint/pose',10)
    

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
