#!/usr/bin/python3

#from controller.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class MIT_controller_Node(Node):
    def __init__(self):
        super().__init__('MIT_controller_node')
        self.namespace = self.get_namespace()
        self.create_subscription(JointState, f"{self.namespace}/trajectory" ,self.trajectory_callback,10)
        self.create_subscription(Float64, f"{self.namespace}/position" ,self.position_callback,10)

        self.base_cmd_pos_pub = self.create_publisher(Float64, f"{self.namespace}/cmd_pos" ,10)

    def trajectory_callback(self,msg):
        pass

    def position_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MIT_controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
