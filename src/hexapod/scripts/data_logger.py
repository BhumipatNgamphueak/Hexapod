#!/usr/bin/env python3
"""
Data Logger Node - CSV logging for hexapod analysis

Task 1: Log for each leg (6 legs):
  - joint_states (position, velocity): hip, knee, ankle
  - joint_targets (position, velocity): hip, knee, ankle
  - end_effector_position (x, y, z, vx, vy, vz)
  - end_effector_target (x, y, z)

Task 2: Log base_link:
  - position relative to reference (0, 0, 0.5)
  - velocity

Output: CSV files in ~/hexapod_logs/
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import csv
import os
from datetime import datetime


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # Parameters
        self.declare_parameter('num_legs', 6)
        self.declare_parameter('log_rate', 50.0)  # Hz
        self.declare_parameter('log_directory', os.path.expanduser('~/hexapod_logs'))
        self.declare_parameter('reference_position', [0.0, 0.0, 0.5])  # Reference frame
        self.declare_parameter('spawn_position', [0.0, 0.0, 0.1])  # Robot spawn position
        
        self.num_legs = self.get_parameter('num_legs').value
        log_rate = self.get_parameter('log_rate').value
        self.log_dir = self.get_parameter('log_directory').value
        self.ref_pos = np.array(self.get_parameter('reference_position').value)
        self.spawn_pos = np.array(self.get_parameter('spawn_position').value)
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Generate timestamp for this logging session
        self.session_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        # Data storage for each leg
        self.leg_data = {}
        for leg_id in range(1, self.num_legs + 1):
            self.leg_data[leg_id] = {
                'joint_positions': np.zeros(3),  # [hip, knee, ankle]
                'joint_velocities': np.zeros(3),
                'joint_targets': np.zeros(3),
                'joint_target_velocities': np.zeros(3),
                'ee_position': np.zeros(3),  # [x, y, z]
                'ee_velocity': np.zeros(3),
                'ee_target': np.zeros(3),
                'ee_target_velocity': np.zeros(3),  # NEW: target velocity
                'timestamp': 0.0
            }
        
        
        # Base link data (from Gazebo ground truth)
        self.base_position = np.zeros(3)
        self.base_velocity = np.zeros(3)
        self.base_angular_velocity = np.zeros(3)
        
        # Additional base metadata
        self.base_frame_id = ''
        self.base_child_frame_id = ''
        self.base_orientation = np.zeros(4)  # quaternion x,y,z,w

        self.base_data = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'angular_velocity': np.zeros(3),
            'orientation': np.zeros(4),
            'frame_id': '',
            'child_frame_id': '',
            'timestamp': 0.0
        }
        
        # ===== SUBSCRIBERS =====
        
        # Subscribe to joint states for each leg
        for leg_id in range(1, self.num_legs + 1):
            self.create_subscription(
                JointState,
                f'/hexapod/leg_{leg_id}/joint_states',
                lambda msg, lid=leg_id: self.joint_states_callback(msg, lid),
                10
            )
            
            # Subscribe to joint targets (from PID controller)
            self.create_subscription(
                Float64MultiArray,
                f'/hexapod/leg_{leg_id}/joint_targets',
                lambda msg, lid=leg_id: self.joint_targets_callback(msg, lid),
                10
            )
            
            # Subscribe to end effector position
            self.create_subscription(
                PointStamped,
                f'/hexapod/leg_{leg_id}/end_effector_position',
                lambda msg, lid=leg_id: self.ee_position_callback(msg, lid),
                10
            )
            
            # Subscribe to end effector target
            self.create_subscription(
                PointStamped,
                f'/hexapod/leg_{leg_id}/end_effector_target',
                lambda msg, lid=leg_id: self.ee_target_callback(msg, lid),
                10
            )
            
            # Subscribe to end effector target velocity
            self.create_subscription(
                Vector3Stamped,
                f'/hexapod/leg_{leg_id}/end_effector_velocity',
                lambda msg, lid=leg_id: self.ee_target_velocity_callback(msg, lid),
                10
            )
        
        # Subscribe to ground truth odometry (use /model/hexapod/odometry as provided by Gazebo P3D)
        # This topic provides pose.position.x/y and twist.linear.x/y/z as in the supplied image
        self.create_subscription(
            Odometry,
            '/model/hexapod/odometry',
            self.odom_callback,
            10
        )
        
        # ===== CSV FILES =====
        
        # Create CSV files for each leg
        self.leg_csv_files = {}
        self.leg_csv_writers = {}
        
        for leg_id in range(1, self.num_legs + 1):
            filename = os.path.join(
                self.log_dir, 
                f'leg_{leg_id}_{self.session_timestamp}.csv'
            )
            csvfile = open(filename, 'w', newline='')
            writer = csv.writer(csvfile)
            
            # Header
            writer.writerow([
                'timestamp',
                'joint_hip_pos', 'joint_knee_pos', 'joint_ankle_pos',
                'joint_hip_vel', 'joint_knee_vel', 'joint_ankle_vel',
                'joint_hip_target', 'joint_knee_target', 'joint_ankle_target',
                'joint_hip_target_vel', 'joint_knee_target_vel', 'joint_ankle_target_vel',
                'ee_pos_x', 'ee_pos_y', 'ee_pos_z',
                'ee_vel_x', 'ee_vel_y', 'ee_vel_z',
                'ee_target_x', 'ee_target_y', 'ee_target_z',
                'ee_target_vel_x', 'ee_target_vel_y', 'ee_target_vel_z'
            ])
            
            self.leg_csv_files[leg_id] = csvfile
            self.leg_csv_writers[leg_id] = writer
        
        # Create CSV file for base_link
        base_filename = os.path.join(
            self.log_dir,
            f'base_link_{self.session_timestamp}.csv'
        )
        self.base_csv_file = open(base_filename, 'w', newline='')
        self.base_csv_writer = csv.writer(self.base_csv_file)
        # CSV header now includes frame ids, position (x,y from odom), orientation quaternion,
        # linear velocity (x,y,z from odom.twist) and angular velocity
        self.base_csv_writer.writerow([
            'timestamp',
            'frame_id', 'child_frame_id',
            'pos_x', 'pos_y',
            'orient_x', 'orient_y', 'orient_z', 'orient_w',
            'vel_x', 'vel_y', 'vel_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'distance_from_ref'
        ])
        
        # Timer for logging
        self.timer = self.create_timer(1.0 / log_rate, self.log_data)
        
        self.get_logger().info(f'Data Logger initialized')
        self.get_logger().info(f'Log directory: {self.log_dir}')
        self.get_logger().info(f'Session: {self.session_timestamp}')
        self.get_logger().info(f'Reference position: {self.ref_pos}')
        self.get_logger().info(f'Spawn position: {self.spawn_pos}')
        self.get_logger().info(f'Logging {self.num_legs} legs at {log_rate} Hz')
    
    def joint_states_callback(self, msg, leg_id):
        """Receive joint states (positions and velocities)"""
        if len(msg.position) >= 3 and len(msg.velocity) >= 3:
            self.leg_data[leg_id]['joint_positions'] = np.array(msg.position[:3])
            self.leg_data[leg_id]['joint_velocities'] = np.array(msg.velocity[:3])
            self.leg_data[leg_id]['timestamp'] = self.get_clock().now().nanoseconds / 1e9
    
    def joint_targets_callback(self, msg, leg_id):
        """Receive joint targets from PID controller"""
        if len(msg.data) >= 3:
            self.leg_data[leg_id]['joint_targets'] = np.array(msg.data[:3])
        if len(msg.data) >= 6:
            self.leg_data[leg_id]['joint_target_velocities'] = np.array(msg.data[3:6])
    
    def ee_position_callback(self, msg, leg_id):
        """Receive end effector position"""
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate velocity from position change
        prev_pos = self.leg_data[leg_id]['ee_position']
        prev_time = self.leg_data[leg_id]['timestamp']
        
        if prev_time > 0:
            dt = current_time - prev_time
            if dt > 0:
                velocity = (current_pos - prev_pos) / dt
                self.leg_data[leg_id]['ee_velocity'] = velocity
        
        self.leg_data[leg_id]['ee_position'] = current_pos
        self.leg_data[leg_id]['timestamp'] = current_time
    
    def ee_target_callback(self, msg, leg_id):
        """Receive end effector target"""
        self.leg_data[leg_id]['ee_target'] = np.array([
            msg.point.x, msg.point.y, msg.point.z
        ])
    
    def ee_target_velocity_callback(self, msg, leg_id):
        """Receive end effector target velocity from trajectory planner"""
        self.leg_data[leg_id]['ee_target_velocity'] = np.array([
            msg.vector.x, msg.vector.y, msg.vector.z
        ])
    
    def odom_callback(self, msg):
        """Receive ground truth odometry from Gazebo P3D plugin"""
        # Extract position (in world frame)
        self.base_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # Extract linear velocity (in world frame)
        self.base_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # Extract angular velocity
        self.base_angular_velocity = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # Store orientation quaternion and frame ids
        q = msg.pose.pose.orientation
        self.base_orientation = np.array([q.x, q.y, q.z, q.w])
        self.base_frame_id = msg.header.frame_id if hasattr(msg, 'header') else ''
        # nav_msgs/Odometry has child_frame_id field
        self.base_child_frame_id = msg.child_frame_id if hasattr(msg, 'child_frame_id') else ''

        # Update base_data metadata as well
        self.base_data['orientation'] = self.base_orientation.copy()
        self.base_data['frame_id'] = self.base_frame_id
        self.base_data['child_frame_id'] = self.base_child_frame_id
    
    def log_data(self):
        """Log data to CSV files"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # ===== LOG LEG DATA =====
        for leg_id in range(1, self.num_legs + 1):
            data = self.leg_data[leg_id]
            
            row = [
                current_time,
                # Joint positions
                data['joint_positions'][0],
                data['joint_positions'][1],
                data['joint_positions'][2],
                # Joint velocities
                data['joint_velocities'][0],
                data['joint_velocities'][1],
                data['joint_velocities'][2],
                # Joint targets
                data['joint_targets'][0],
                data['joint_targets'][1],
                data['joint_targets'][2],
                # Joint target velocities
                data['joint_target_velocities'][0],
                data['joint_target_velocities'][1],
                data['joint_target_velocities'][2],
                # End effector position
                data['ee_position'][0],
                data['ee_position'][1],
                data['ee_position'][2],
                # End effector velocity
                data['ee_velocity'][0],
                data['ee_velocity'][1],
                data['ee_velocity'][2],
                # End effector target
                data['ee_target'][0],
                data['ee_target'][1],
                data['ee_target'][2],
                # End effector target velocity
                data['ee_target_velocity'][0],
                data['ee_target_velocity'][1],
                data['ee_target_velocity'][2]
            ]
            
            self.leg_csv_writers[leg_id].writerow(row)
        
        # ===== LOG BASE_LINK DATA =====
        # Position in world frame (use odometry x,y as requested)
        pos_world = self.base_position.copy()

        # Velocity from odometry (ground truth linear x,y,z)
        velocity = self.base_velocity.copy()

        # Angular velocity
        ang_velocity = self.base_angular_velocity.copy()

        # Distance from reference (still useful) computed from relative position
        position_rel = pos_world - self.ref_pos
        distance = np.linalg.norm(position_rel)

        # Write to CSV — only x,y position columns (z omitted as requested)
        row = [
            current_time,
            self.base_frame_id, self.base_child_frame_id,
            pos_world[0], pos_world[1],
            float(self.base_orientation[0]), float(self.base_orientation[1]), float(self.base_orientation[2]), float(self.base_orientation[3]),
            velocity[0], velocity[1], velocity[2],
            ang_velocity[0], ang_velocity[1], ang_velocity[2],
            distance
        ]
        self.base_csv_writer.writerow(row)
        
        self.base_data['position'] = position_rel
        self.base_data['velocity'] = velocity
        self.base_data['angular_velocity'] = ang_velocity
        self.base_data['timestamp'] = current_time
        
        # Flush files periodically
        if int(current_time * 10) % 10 == 0:  # Every 1 second
            for csvfile in self.leg_csv_files.values():
                csvfile.flush()
            self.base_csv_file.flush()
    
    def destroy_node(self):
        """Close CSV files on shutdown"""
        self.get_logger().info('Closing CSV files...')
        
        for csvfile in self.leg_csv_files.values():
            csvfile.close()
        
        self.base_csv_file.close()
        
        self.get_logger().info(f'Logs saved to: {self.log_dir}')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()