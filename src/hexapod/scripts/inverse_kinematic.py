#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import subprocess
import os
import roboticstoolbox as rtb
from spatialmath import SE3


class InverseKinematicNode(Node):
    def __init__(self):
        super().__init__("inverse_kinematic_node")


def main(args=None):

    rclpy.init(args=args)
    node = InverseKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
