#!/usr/bin/env python3
"""
Set Point Generator Node - FIXED REFERENCE FRAME (ONE PER LEG)
INPUT: /hexapod/leg_X/phase_info (Float64MultiArray - [phase_type, progress, leg_phase])
INPUT: /hexapod/body_velocity (Twist - body velocity)
INPUT: /hexapod/gait_parameters (Float64MultiArray - [gait_type, step_height, step_length, cycle_time, duty_factor])
OUTPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete foot position in BASE_LINK)
Frequency: 50 Hz

CRITICAL FIX: 
- All positions now in base_link frame
- Stance positions defined relative to leg attachment points from URDF
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class SetPointGeneratorFixed(Node):
    def __init__(self):
        super().__init__('set_point_generator')
        
        # Parameters
        self.declare_parameter('leg_id', 1)
        self.declare_parameter('stance_offset_x', 0.15)  # Relative to attachment point
        self.declare_parameter('stance_offset_y', 0.0)
        self.declare_parameter('stance_offset_z', -0.05)
        self.declare_parameter('update_rate', 50.0)
        
        self.leg_id = self.get_parameter('leg_id').value
        self.stance_offset_x = self.get_parameter('stance_offset_x').value
        self.stance_offset_y = self.get_parameter('stance_offset_y').value
        self.stance_offset_z = self.get_parameter('stance_offset_z').value
        update_rate = self.get_parameter('update_rate').value
        
        # CRITICAL: Define leg attachment points from URDF (in base_link frame)
        self.leg_attachments = {
            1: np.array([-1.2616e-05, -0.095255, 0.0]),
            2: np.array([0.082487, -0.047638, 0.0]),
            3: np.array([0.082499, 0.047616, 0.0]),
            4: np.array([1.2616e-05, 0.095255, 0.0]),
            5: np.array([-0.082487, 0.047638, 0.0]),
            6: np.array([-0.082499, -0.047616, 0.0])
        }
        
        # Compute stance position in base_link frame
        self.stance_position_base = self._compute_stance_position_base_link()
        
        # State variables
        self.phase_type = 0  # 0: stance, 1: swing
        self.progress = 0.0
        self.leg_phase = 0.0
        self.body_velocity = Twist()
        self.step_height = 0.03
        self.step_length = 0.1
        
        # Current position (in base_link frame)
        self.current_position = self.stance_position_base.copy()
        
        # INPUT: Subscribers
        self.phase_sub = self.create_subscription(
            Float64MultiArray, f'/hexapod/leg_{self.leg_id}/phase_info', 
            self.phase_callback, 10)
        self.body_vel_sub = self.create_subscription(
            Twist, '/hexapod/body_velocity', self.body_velocity_callback, 10)
        self.gait_params_sub = self.create_subscription(
            Float64MultiArray, '/hexapod/gait_parameters', 
            self.gait_params_callback, 10)
        
        # OUTPUT: Publisher - setpoint for THIS leg (in base_link frame)
        self.setpoint_pub = self.create_publisher(
            PointStamped, f'/hexapod/leg_{self.leg_id}/end_effector_setpoint', 10)
        
        # Timer - 50 Hz
        self.timer = self.create_timer(1.0 / update_rate, self.generate_setpoint)
        
        self.get_logger().info(f'Set Point Generator (base_link frame) for leg {self.leg_id} initialized')
        self.get_logger().info(f'Attachment point: {self.leg_attachments[self.leg_id]}')
        self.get_logger().info(f'Stance position (base_link): {self.stance_position_base}')
    
    def _compute_stance_position_base_link(self):
        """
        Compute stance position in base_link frame
        Stance position = leg attachment point + offset
        """
        attachment = self.leg_attachments[self.leg_id]
        offset = np.array([
            self.stance_offset_x,
            self.stance_offset_y,
            self.stance_offset_z
        ])
        
        stance_pos = attachment + offset
        
        self.get_logger().info(
            f'Stance position computed: attachment={attachment} + offset={offset} = {stance_pos}'
        )
        
        return stance_pos
    
    def phase_callback(self, msg):
        """INPUT: Receive phase information [phase_type, progress, leg_phase]"""
        if len(msg.data) >= 3:
            self.phase_type = int(msg.data[0])
            self.progress = float(msg.data[1])
            self.leg_phase = float(msg.data[2])
    
    def body_velocity_callback(self, msg):
        """INPUT: Receive body velocity"""
        self.body_velocity = msg
    
    def gait_params_callback(self, msg):
        """INPUT: Receive gait parameters [gait_type, step_height, step_length, cycle_time, duty_factor]"""
        if len(msg.data) >= 3:
            self.step_height = float(msg.data[1])
            self.step_length = float(msg.data[2])
    
    def _generate_stance_position(self, progress):
        """
        Generate position during stance phase (foot on ground, moving backward relative to body)
        All positions in base_link frame
        """
        # Stance: foot moves from front to back (opposite to body forward motion)
        # Progress 0 -> 1: move from +step_length/2 to -step_length/2
        
        # Start at front position
        front_pos = self.stance_position_base.copy()
        front_pos[0] += self.step_length / 2.0
        
        # Move to back position
        back_pos = self.stance_position_base.copy()
        back_pos[0] -= self.step_length / 2.0
        
        # Interpolate based on progress
        position = front_pos + progress * (back_pos - front_pos)
        
        return position
    
    def _generate_swing_position(self, progress):
        """
        Generate position during swing phase (foot in air, moving forward)
        All positions in base_link frame
        """
        # Swing: foot lifts, moves forward, and lands
        # Progress 0 -> 1: move from back to front with parabolic arc
        
        # Start at back position (where stance ended)
        back_pos = self.stance_position_base.copy()
        back_pos[0] -= self.step_length / 2.0
        
        # End at front position (where stance will start)
        front_pos = self.stance_position_base.copy()
        front_pos[0] += self.step_length / 2.0
        
        # X: linear interpolation from back to front
        x = back_pos[0] + progress * (front_pos[0] - back_pos[0])
        
        # Y: no lateral movement (typically)
        y = self.stance_position_base[1]
        
        # Z: parabolic arc (peaks at progress = 0.5)
        z_base = self.stance_position_base[2]
        z = z_base + self.step_height * 4 * progress * (1 - progress)
        
        position = np.array([x, y, z])
        
        return position
    
    def generate_setpoint(self):
        """OUTPUT: Generate setpoint based on phase (in base_link frame) - 50 Hz"""
        # Generate position based on phase type
        if self.phase_type == 0:  # Stance
            position = self._generate_stance_position(self.progress)
        else:  # Swing
            position = self._generate_swing_position(self.progress)
        
        self.current_position = position
        
        # Publish setpoint (FIXED: base_link frame)
        setpoint_msg = PointStamped()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = 'base_link'  # CRITICAL FIX
        setpoint_msg.point.x = float(position[0])
        setpoint_msg.point.y = float(position[1])
        setpoint_msg.point.z = float(position[2])
        
        self.setpoint_pub.publish(setpoint_msg)
        
        self.get_logger().debug(
            f'Leg {self.leg_id}: phase={self.phase_type}, progress={self.progress:.3f}, '
            f'pos_base_link=[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SetPointGeneratorFixed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()