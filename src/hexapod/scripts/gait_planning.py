#!/usr/bin/env python3
"""
Gait Planner Node - Generates gait patterns for hexapod locomotion
INPUT: /cmd_vel (Twist - velocity commands)
OUTPUT: /hexapod/gait_parameters (Float64MultiArray - [gait_type, step_height, step_length, cycle_time, duty_factor])
OUTPUT: /hexapod/body_velocity (Twist - filtered velocity)
OUTPUT: /hexapod/leg_X/end_effector_setpoint (PointStamped - discrete waypoints for each leg)
Frequency: 10 Hz
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Float64MultiArray
import numpy as np


class GaitPlanner(Node):
    def __init__(self):
        super().__init__('gait_planner')

        # Parameters
        self.declare_parameter('default_gait_type', 0)  # 0: tripod, 1: wave, 2: ripple
        self.declare_parameter('max_linear_x', 0.3)
        self.declare_parameter('max_linear_y', 0.2)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('step_height', 0.03)
        self.declare_parameter('step_length_scale', 0.05)
        self.declare_parameter('min_cycle_time', 0.5)
        self.declare_parameter('max_cycle_time', 2.0)
        self.declare_parameter('num_legs', 6)
        self.declare_parameter('num_waypoints', 8)

        # Leg home positions (default stance) relative to base_link
        # Format: flat list [x1, y1, z1, x2, y2, z2, ...] (will be reshaped to 6x3)
        self.declare_parameter('leg_home_positions', [
            0.15, -0.15, -0.10,  # Leg 1 (Front Right)
            0.15,  0.00, -0.10,  # Leg 2 (Middle Right)
            0.15,  0.15, -0.10,  # Leg 3 (Rear Right)
           -0.15,  0.15, -0.10,  # Leg 4 (Rear Left)
           -0.15,  0.00, -0.10,  # Leg 5 (Middle Left)
           -0.15, -0.15, -0.10   # Leg 6 (Front Left)
        ])
        
        # Get parameters
        self.default_gait = self.get_parameter('default_gait_type').value
        self.max_linear_x = self.get_parameter('max_linear_x').value
        self.max_linear_y = self.get_parameter('max_linear_y').value
        self.max_angular_z = self.get_parameter('max_angular_z').value
        self.step_height = self.get_parameter('step_height').value
        self.step_length_scale = self.get_parameter('step_length_scale').value
        self.min_cycle_time = self.get_parameter('min_cycle_time').value
        self.max_cycle_time = self.get_parameter('max_cycle_time').value
        self.num_legs = self.get_parameter('num_legs').value
        self.num_waypoints = self.get_parameter('num_waypoints').value

        # Get leg home positions
        leg_home_raw = self.get_parameter('leg_home_positions').value
        self.leg_home_positions = np.array(leg_home_raw).reshape(6, 3)

        # State variables
        self.current_cmd_vel = Twist()
        self.filtered_velocity = Twist()
        self.current_gait_type = self.default_gait
        self.gait_phase = 0.0  # Phase tracker [0, 1]
        self.dt = 0.1  # 10 Hz
        self.leg_phases = np.zeros(6)  # Phase for each leg [0, 1]
        
        # Gait definitions (duty factor = fraction of cycle in stance)
        self.gait_configs = {
            0: {'name': 'tripod', 'duty_factor': 0.5, 'legs_per_group': 3},
            1: {'name': 'wave', 'duty_factor': 0.83, 'legs_per_group': 1},
            2: {'name': 'ripple', 'duty_factor': 0.67, 'legs_per_group': 2}
        }
        
        # INPUT: Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # OUTPUT: Publishers
        self.gait_params_pub = self.create_publisher(
            Float64MultiArray, '/hexapod/gait_parameters', 10)
        self.body_vel_pub = self.create_publisher(
            Twist, '/hexapod/body_velocity', 10)

        # End effector setpoint publishers for each leg
        self.setpoint_pubs = []
        for leg_id in range(1, 7):
            pub = self.create_publisher(
                PointStamped,
                f'/hexapod/leg_{leg_id}/end_effector_setpoint',
                10
            )
            self.setpoint_pubs.append(pub)

        # Timer - 10 Hz
        self.timer = self.create_timer(self.dt, self.publish_gait)

        self.get_logger().info(f'Gait Planner initialized with {self.num_legs} legs')
        self.get_logger().info(f'Publishing setpoints to /hexapod/leg_X/end_effector_setpoint')
    
    def cmd_vel_callback(self, msg):
        """INPUT: Receive velocity commands"""
        # Store and clip velocity to limits
        self.current_cmd_vel.linear.x = np.clip(
            msg.linear.x, -self.max_linear_x, self.max_linear_x)
        self.current_cmd_vel.linear.y = np.clip(
            msg.linear.y, -self.max_linear_y, self.max_linear_y)
        self.current_cmd_vel.angular.z = np.clip(
            msg.angular.z, -self.max_angular_z, self.max_angular_z)
    
    def _select_gait_type(self, velocity_magnitude):
        """Select gait type based on velocity"""
        # Simple gait selection logic
        if velocity_magnitude < 0.05:
            return 0  # Tripod for standing/slow
        elif velocity_magnitude < 0.15:
            return 2  # Ripple for medium speed
        else:
            return 0  # Tripod for fast
    
    def _filter_velocity(self, alpha=0.3):
        """Apply exponential smoothing to velocity commands"""
        self.filtered_velocity.linear.x = (alpha * self.current_cmd_vel.linear.x + 
                                          (1 - alpha) * self.filtered_velocity.linear.x)
        self.filtered_velocity.linear.y = (alpha * self.current_cmd_vel.linear.y + 
                                          (1 - alpha) * self.filtered_velocity.linear.y)
        self.filtered_velocity.angular.z = (alpha * self.current_cmd_vel.angular.z + 
                                           (1 - alpha) * self.filtered_velocity.angular.z)
    
    def _calculate_step_length(self, velocity, cycle_time):
        """Calculate step length based on velocity and cycle time"""
        vx = velocity.linear.x
        vy = velocity.linear.y
        
        # Step length proportional to velocity * time
        step_length_x = vx * cycle_time * self.step_length_scale
        step_length_y = vy * cycle_time * self.step_length_scale
        
        step_length = np.sqrt(step_length_x**2 + step_length_y**2)
        return max(step_length, 0.01)  # Minimum step length
    
    def _generate_gait_path(self, step_length, step_height, num_waypoints=8):
        """Generate a walking gait path (stance + swing) similar to cubic.py"""
        waypoints = []
        
        # Stance phase: straight line on ground (moving forward)
        x_start = -step_length / 2.0
        waypoints.append([x_start, 0.0, 0.0])
        waypoints.append([x_start + step_length, 0.0, 0.0])
        
        # Swing phase: arc in the air moving backward to starting position
        x_swing_start = x_start + step_length
        x_swing_end = x_start
        
        # Create arc waypoints using parabolic shape
        for i in range(1, num_waypoints):
            t = i / num_waypoints
            x = x_swing_start - t * step_length  # Move backward
            # Parabolic arc: peaks at middle
            z = step_height * 4 * t * (1 - t)  # peaks at t=0.5
            waypoints.append([x, 0.0, z])
        
        # End of swing phase back on ground (closes the loop)
        waypoints.append([x_swing_end, 0.0, 0.0])
        
        return np.array(waypoints)
    
    def _get_leg_phase_offset(self, leg_id, gait_type):
        """Calculate phase offset for each leg based on gait pattern"""
        # Leg arrangement (typical hexapod):
        # Front: L1, R1
        # Middle: L2, R2
        # Rear: L3, R3
        
        if gait_type == 0:  # Tripod gait
            # Group 1: L1, R2, L3 (phase 0)
            # Group 2: R1, L2, R3 (phase 0.5)
            if leg_id in [1, 4, 5]:  # L1, R2, L3
                return 0.0
            else:
                return 0.5
        
        elif gait_type == 1:  # Wave gait
            # Sequential: each leg offset by 1/6
            phase_offsets = [0.0, 0.5, 1/6, 4/6, 2/6, 5/6]
            return phase_offsets[leg_id - 1]
        
        elif gait_type == 2:  # Ripple gait
            # Three groups of two legs
            phase_offsets = [0.0, 2/3, 1/3, 0.0, 2/3, 1/3]
            return phase_offsets[leg_id - 1]
        
        return 0.0
    
    def _compute_foot_position(self, leg_id, phase, step_length, step_height, duty_factor, velocity):
        """
        Compute foot position for a single leg based on gait phase.

        Args:
            leg_id: Leg ID (0-5, internal indexing)
            phase: Current phase of this leg [0, 1]
            step_length: Step length in meters
            step_height: Step height in meters
            duty_factor: Fraction of cycle in stance phase
            velocity: Twist message with linear and angular velocities

        Returns:
            np.array([x, y, z]): Foot position relative to base_link
        """
        # Get home position for this leg
        home_pos = self.leg_home_positions[leg_id].copy()

        # Calculate step direction from velocity
        vx = velocity.linear.x
        vy = velocity.linear.y

        # Step direction (normalized)
        step_direction = np.array([vx, vy, 0.0])
        step_mag = np.linalg.norm(step_direction[:2])

        if step_mag > 1e-6:
            step_direction[:2] /= step_mag
        else:
            # No movement - stay at home position
            return home_pos

        # Determine if leg is in stance or swing phase
        if phase < duty_factor:
            # STANCE PHASE: Foot moves backward relative to body
            # Phase within stance: [0, 1]
            stance_phase = phase / duty_factor

            # Move from front to back of step
            offset = step_direction * step_length * (0.5 - stance_phase)
            foot_pos = home_pos + offset
            foot_pos[2] = home_pos[2]  # Ground contact

        else:
            # SWING PHASE: Foot lifts and moves forward
            # Phase within swing: [0, 1]
            swing_phase = (phase - duty_factor) / (1.0 - duty_factor)

            # Move from back to front of step
            offset = step_direction * step_length * (-0.5 + swing_phase)
            foot_pos = home_pos + offset

            # Parabolic arc for Z (lift foot)
            # Peaks at middle of swing (swing_phase = 0.5)
            z_lift = step_height * 4 * swing_phase * (1 - swing_phase)
            foot_pos[2] = home_pos[2] + z_lift

        return foot_pos

    def publish_gait(self):
        """OUTPUT: Publish gait parameters, body velocity, and foot setpoints - 10 Hz"""
        # Filter velocity
        self._filter_velocity()

        # Calculate velocity magnitude
        vel_mag = np.sqrt(self.filtered_velocity.linear.x**2 +
                         self.filtered_velocity.linear.y**2)

        # Select gait type
        self.current_gait_type = self._select_gait_type(vel_mag)
        gait_config = self.gait_configs[self.current_gait_type]

        # Calculate cycle time (slower for slower velocities)
        if vel_mag > 0.01:
            cycle_time = np.clip(1.0 / vel_mag, self.min_cycle_time, self.max_cycle_time)
        else:
            cycle_time = self.max_cycle_time

        # Calculate step parameters
        step_length = self._calculate_step_length(self.filtered_velocity, cycle_time)
        duty_factor = gait_config['duty_factor']

        # Update gait phase
        self.gait_phase += self.dt / cycle_time
        if self.gait_phase >= 1.0:
            self.gait_phase -= 1.0

        # Publish gait parameters
        gait_msg = Float64MultiArray()
        gait_msg.data = [
            float(self.current_gait_type),
            float(self.step_height),
            float(step_length),
            float(cycle_time),
            float(duty_factor)
        ]
        self.gait_params_pub.publish(gait_msg)

        # Publish filtered body velocity
        self.body_vel_pub.publish(self.filtered_velocity)

        # Compute and publish foot setpoints for each leg
        for leg_id in range(6):
            # Get phase offset for this leg based on gait pattern
            phase_offset = self._get_leg_phase_offset(leg_id + 1, self.current_gait_type)

            # Calculate current phase for this leg
            leg_phase = (self.gait_phase + phase_offset) % 1.0
            self.leg_phases[leg_id] = leg_phase

            # Compute foot position
            foot_pos = self._compute_foot_position(
                leg_id, leg_phase, step_length, self.step_height,
                duty_factor, self.filtered_velocity
            )

            # Publish setpoint
            setpoint_msg = PointStamped()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.header.frame_id = 'base_link'
            setpoint_msg.point.x = float(foot_pos[0])
            setpoint_msg.point.y = float(foot_pos[1])
            setpoint_msg.point.z = float(foot_pos[2])
            self.setpoint_pubs[leg_id].publish(setpoint_msg)

        self.get_logger().debug(
            f'Gait: {gait_config["name"]}, step_length: {step_length:.3f}, '
            f'cycle_time: {cycle_time:.3f}, duty_factor: {duty_factor:.3f}, '
            f'phase: {self.gait_phase:.3f}'
        )
    
def main(args=None):
    rclpy.init(args=args)
    node = GaitPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()