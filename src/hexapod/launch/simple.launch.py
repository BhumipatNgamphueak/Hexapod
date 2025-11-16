#!/usr/bin/env python3
"""
Hexapod Control Launch File - SKELETON VERSION
Launches all nodes for hexapod control system

Total nodes: 3 global + (7 × 6 legs) = 45 nodes

Structure:
- 3 Global nodes (gait planning, state machine, joint splitter)
- 7 Per-leg nodes × 6 legs = 42 nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    num_legs_arg = DeclareLaunchArgument(
        'num_legs',
        default_value='6',
        description='Number of legs to launch (1-6, use 1 for testing)'
    )
    
    nodes_to_launch = []
    
    # =================================================================
    # GLOBAL NODES (3)
    # =================================================================
    
    # 1. Gait Planner
    nodes_to_launch.append(
        Node(
            package='hexapod',
            executable='gait_planning.py',
            name='gait_planner',
            namespace='hexapod',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'default_gait_type': 0,
                'max_linear_x': 0.3,
                'max_linear_y': 0.2,
                'max_angular_z': 1.0,
                'step_height': 0.03,
                'step_length_scale': 0.05,
                'cycle_time': 1.0,
            }],
            remappings=[
                # Input
                ('cmd_vel', '/cmd_vel'),
                # Output
                ('gait_parameters', '/hexapod/gait_parameters'),
                ('body_velocity', '/hexapod/body_velocity'),
            ],
            output='screen'
        )
    )
    
    # 2. State Machine
    nodes_to_launch.append(
        Node(
            package='hexapod',
            executable='state_controller.py',
            name='state_machine',
            namespace='hexapod',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'control_rate': 50.0,
                # Phase offsets for tripod gait
                'leg_1_phase_offset': 0.0,
                'leg_2_phase_offset': 0.5,
                'leg_3_phase_offset': 0.0,
                'leg_4_phase_offset': 0.5,
                'leg_5_phase_offset': 0.0,
                'leg_6_phase_offset': 0.5,
            }],
            remappings=[
                # Input
                ('gait_parameters', '/hexapod/gait_parameters'),
                ('body_velocity', '/hexapod/body_velocity'),
                # Output: leg_{1-6}/phase_info (handled by node)
            ],
            output='screen'
        )
    )
    
    # 3. Joint State Splitter
    nodes_to_launch.append(
        Node(
            package='hexapod',
            executable='joint_state_splitter.py',
            name='joint_state_splitter',
            namespace='hexapod',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Joint names for each leg (adjust based on your URDF)
                'leg_1_joints': ['hip_joint_1', 'knee_joint_1', 'ankle_joint_1'],
                'leg_2_joints': ['hip_joint_2', 'knee_joint_2', 'ankle_joint_2'],
                'leg_3_joints': ['hip_joint_3', 'knee_joint_3', 'ankle_joint_3'],
                'leg_4_joints': ['hip_joint_4', 'knee_joint_4', 'ankle_joint_4'],
                'leg_5_joints': ['hip_joint_5', 'knee_joint_5', 'ankle_joint_5'],
                'leg_6_joints': ['hip_joint_6', 'knee_joint_6', 'ankle_joint_6'],
            }],
            remappings=[
                # Input
                ('joint_states', '/joint_states'),
                # Output: leg_{1-6}/joint_states (handled by node)
            ],
            output='screen'
        )
    )
    
    # =================================================================
    # PER-LEG NODES (7 nodes × 6 legs = 42 nodes)
    # For testing, use range(1, 2) to launch only leg 1
    # For full system, use range(1, 7) to launch all 6 legs
    # =================================================================
    
    # TODO: Change this to range(1, 7) for all legs
    for leg_id in range(1, 2):  # Currently only leg 1 for testing
        
        leg_ns = f'hexapod/leg_{leg_id}'
        
        # -----------------------------------------------------------
        # 4. Set Point Generator
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='set_point.py',
                name='set_point_generator',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'stance_x': 0.15,
                    'stance_y': 0.0,
                    'stance_z': -0.05,
                    'update_rate': 50.0,
                }],
                remappings=[
                    # Input
                    ('phase_info', f'/hexapod/leg_{leg_id}/phase_info'),
                    ('body_velocity', '/hexapod/body_velocity'),
                    # Output
                    ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 5. Trajectory Generator
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='trajectory_planning.py',
                name='trajectory_generator',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'interpolation_method': 'cubic',
                    'trajectory_rate': 100.0,
                    'swing_clearance': 0.03,
                }],
                remappings=[
                    # Input
                    ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                    ('phase_info', f'/hexapod/leg_{leg_id}/phase_info'),
                    # Output
                    ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                    ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 6. Inverse Position Kinematics
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='inverse_position_kinematic.py',
                name='inverse_kinematics',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'max_iterations': 100,
                    'tolerance': 0.001,
                    'use_analytical_ik': True,
                }],
                remappings=[
                    # Input
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                    # Output
                    ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 7. Inverse Velocity Kinematics
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='inverse_velocity_kinematic.py',
                name='inverse_velocity_kinematics',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'update_rate': 100.0,
                }],
                remappings=[
                    # Input
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                    # Output
                    ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 8. Position PID Controller
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='pid_position_controller.py',
                name='position_pid_controller',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'position_kp': [10.0, 10.0, 10.0],
                    'position_ki': [0.1, 0.1, 0.1],
                    'position_kd': [1.0, 1.0, 1.0],
                    'velocity_limit': 5.0,
                    'control_rate': 100.0,
                }],
                remappings=[
                    # Input
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target'),
                    # Output
                    ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 9. Velocity PID Controller
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='pid_velocity_controller.py',
                name='velocity_pid_controller',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'velocity_kp': [5.0, 5.0, 5.0],
                    'velocity_ki': [0.5, 0.5, 0.5],
                    'velocity_kd': [0.1, 0.1, 0.1],
                    'effort_limit': 10.0,
                    'control_rate': 100.0,
                }],
                remappings=[
                    # Input
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target'),
                    ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward'),
                    # Output (to Gazebo controller)
                    ('commands', f'/effort_controller_leg_{leg_id}/commands'),
                ],
                output='screen'
            )
        )
        
        # -----------------------------------------------------------
        # 10. Forward Kinematics (Optional - for monitoring)
        # -----------------------------------------------------------
        nodes_to_launch.append(
            Node(
                package='hexapod',
                executable='forward_position_kinematic.py',
                name='forward_kinematics',
                namespace=leg_ns,
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'update_rate': 100.0,
                }],
                remappings=[
                    # Input
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    # Output
                    ('end_effector_position', f'/hexapod/leg_{leg_id}/end_effector_position'),
                ],
                output='screen'
            )
        )
    
    # Return launch description
    return LaunchDescription([
        use_sim_time_arg,
        num_legs_arg,
        *nodes_to_launch
    ])