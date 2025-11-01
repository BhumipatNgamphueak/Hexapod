#!/usr/bin/python3

from hexapod.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DummyNode(Node):
    def __init__(self):
        super().__init__('state_controller_node')
        self.create_subscription(PoseStamped, 'top_right_leg/foot/pose' ,self.top_right_leg_footpose_callback,10)
        self.create_subscription(PoseStamped, 'top_left_leg/foot/pose' ,self.top_left_leg_footpose_callback,10)
        self.create_subscription(PoseStamped, 'middle_right_leg/foot/pose' ,self.middle_right_leg_footpose_callback,10)
        self.create_subscription(PoseStamped, 'middle_left_leg/foot/pose' ,self.middle_left_leg_footpose_callback,10)
        self.create_subscription(PoseStamped, 'bottom_right_leg/foot/pose' ,self.bottom_right_leg_footpose_callback,10)
        self.create_subscription(PoseStamped, 'bottom_left_leg/foot/pose' ,self.bottom_left_leg_footpose_callback,10)
        
    def top_right_leg_footpose_callback(self,msg):
        pass

    def top_left_leg_footpose_callback(self,msg):
        pass

    def middle_right_leg_footpose_callback(self,msg):
        pass

    def middle_left_leg_footpose_callback(self,msg):
        pass

    def bottom_right_leg_footpose_callback(self,msg):
        pass

    def bottom_left_leg_footpose_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
