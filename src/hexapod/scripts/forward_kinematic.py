#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import roboticstoolbox as rtb
import subprocess
from scipy.spatial.transform import Rotation


class ForwardKinematicNode(Node):
    def __init__(self):
        super().__init__("forward_kinematic_node")

def main(args=None):

    rclpy.init(args=args)
    node = ForwardKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
