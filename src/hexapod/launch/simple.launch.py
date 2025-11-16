#!/usr/bin/env python3
"""
Hexapod Control Launch File - FIXED VERSION
Launches all nodes for hexapod control system with proper sequencing

FIXES:
1. Added TimerAction delays to ensure simulation is ready
2. Start joint_state_splitter first (critical dependency)
3. Staggered leg controller startup
4. Added proper parameter handling
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
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
    # CRITICAL: Joint State Splitter MUST start first (at 2 seconds)
    # This node is essential as it provides joint states to all other nodes
    # =================================================================
    
    joint_splitter = Node(
        package='hexapod',
        executable='joint_state_splitter.py',
        name='joint_state_splitter',
        namespace='hexapod',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Joint names for each leg (matching URDF)
            'leg_1_joints': ['hip_joint_1', 'knee_joint_1', 'ankle_joint_1'],
            'leg_2_joints': ['hip_joint_2', 'knee_joint_2', 'ankle_joint_2'],
            'leg_3_joints': ['hip_joint_3', 'knee_joint_3', 'ankle_joint_3'],
            'leg_4_joints': ['hip_joint_4', 'knee_joint_4', 'ankle_joint_4'],
            'leg_5_joints': ['hip_joint_5', 'knee_joint_5', 'ankle_joint_5'],
            'leg_6_joints': ['hip_joint_6', 'knee_joint_6', 'ankle_joint_6'],
        }],
        remappings=[
            ('joint_states', '/joint_states'),
        ],
    )
    
    # Start joint_state_splitter after 2 seconds (wait for simulation + controllers)
    nodes_to_launch.append(TimerAction(
        period=2.0,
        actions=[joint_splitter]
    ))
    
    # =================================================================
    # GLOBAL NODES (start at 3 seconds - after joint splitter)
    # =================================================================
    
    # 1. Gait Planner
    gait_planner = Node(
        package='hexapod',
        executable='gait_planning.py',
        name='gait_planner',
        namespace='hexapod',
        output='screen',
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
            ('cmd_vel', '/cmd_vel'),
            ('gait_parameters', '/hexapod/gait_parameters'),
            ('body_velocity', '/hexapod/body_velocity'),
        ],
    )
    
    # 2. State Machine
    state_machine = Node(
        package='hexapod',
        executable='state_controller.py',
        name='state_machine',
        namespace='hexapod',
        output='screen',
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
            ('gait_parameters', '/hexapod/gait_parameters'),
            ('body_velocity', '/hexapod/body_velocity'),
        ],
    )
    
    # Start global nodes at 3 seconds
    nodes_to_launch.append(TimerAction(
        period=3.0,
        actions=[gait_planner, state_machine]
    ))
    
    # =================================================================
    # PER-LEG NODES (7 nodes × 6 legs = 42 nodes)
    # Start each leg's nodes at staggered times to avoid overwhelming the system
    # =================================================================
    
    # Leg 1 starts at 4 seconds
    for leg_id in range(1, 7):
        
        leg_ns = f'hexapod/leg_{leg_id}'
        
        # Calculate delay: Leg 1 at 4s, Leg 2 at 4.5s, Leg 3 at 5s, etc.
        leg_start_delay = 4.0 + (leg_id - 1) * 0.5
        
        leg_nodes = []
        
        # -----------------------------------------------------------
        # Set Point Generator
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='set_point.py',
                name='set_point_generator',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'stance_x': 0.15,
                    'stance_y': 0.0,
                    'stance_z': -0.05,
                    'update_rate': 50.0,
                }],
                remappings=[
                    ('phase_info', f'/hexapod/leg_{leg_id}/phase_info'),
                    ('body_velocity', '/hexapod/body_velocity'),
                    ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Trajectory Generator
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='trajectory_planning.py',
                name='trajectory_generator',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'interpolation_method': 'cubic',
                    'trajectory_rate': 100.0,
                    'swing_clearance': 0.03,
                }],
                remappings=[
                    ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                    ('phase_info', f'/hexapod/leg_{leg_id}/phase_info'),
                    ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                    ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Inverse Position Kinematics
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='inverse_position_kinematic.py',
                name='inverse_kinematics',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'max_iterations': 100,
                    'tolerance': 0.001,
                    'use_analytical_ik': True,
                    'use_numerical_ik': True,  # Use numerical IK for accuracy
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                    ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Inverse Velocity Kinematics
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='inverse_velocity_kinematic.py',
                name='inverse_velocity_kinematics',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'update_rate': 100.0,
                    'damping_factor': 0.01,
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                    ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Position PID Controller
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='pid_position_controller.py',
                name='position_pid_controller',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'position_kp': [30.0, 30.0, 30.0],
                    'position_ki': [0.5, 0.5, 0.5],
                    'position_kd': [3.0, 3.0, 3.0],
                    'velocity_limit': 10.0,
                    'control_rate': 100.0,
                    'integral_limit': 2.0,
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target'),
                    ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Velocity PID Controller
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='pid_velocity_controller.py',
                name='velocity_pid_controller',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'velocity_kp': [15.0, 15.0, 15.0],
                    'velocity_ki': [1.0, 1.0, 1.0],
                    'velocity_kd': [1.0, 1.0, 1.0],
                    'effort_limit': 24.0,
                    'control_rate': 100.0,
                    'integral_limit': 1.0,
                    'use_feedforward': True,
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target'),
                    ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward'),
                    ('commands', f'/effort_controller_leg_{leg_id}/commands'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Forward Kinematics (Optional - for monitoring)
        # -----------------------------------------------------------
        leg_nodes.append(
            Node(
                package='hexapod',
                executable='forward_position_kinematic.py',
                name='forward_kinematics',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'update_rate': 100.0,
                    # Link parameters (from URDF)
                    'knee_joint_x': 0.078424,
                    'knee_joint_y': -0.0031746,
                    'knee_joint_z': 0.0010006,
                    'ankle_joint_x': -0.087752,
                    'ankle_joint_y': -0.081834,
                    'ankle_joint_z': 0.0,
                    'foot_pointer_joint_x': 0.18098,
                    'foot_pointer_joint_y': -0.022156,
                    'end_effector_joint_x': 0.0075528,
                    'end_effector_joint_y': -0.00094278,
                    'hip_xyz_x': -1.2616e-05,
                    'hip_xyz_y': -0.095255,
                    'hip_xyz_z': 0.0,
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_position', f'/hexapod/leg_{leg_id}/end_effector_position'),
                ],
            )
        )
        
        # Add this leg's nodes with delay
        nodes_to_launch.append(TimerAction(
            period=leg_start_delay,
            actions=leg_nodes
        ))
    
    # Return launch description
    return LaunchDescription([
        use_sim_time_arg,
        num_legs_arg,
        *nodes_to_launch
    ])