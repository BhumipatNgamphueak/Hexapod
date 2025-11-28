#!/usr/bin/env python3
"""
Hexapod Control Launch File - FIXED SYNCHRONIZED STARTUP

CRITICAL FIX: All leg controllers start at the SAME TIME (4.0s)
Previously: Staggered 4.0s, 4.5s, 5.0s... ❌
Now: All at 4.0s ✅

TIMING SEQUENCE:
  0.0s: Simulation starts
  2.0s: Joint state splitter (CRITICAL - must be first!)
  3.0s: Global nodes (gait planner, state machine)
  4.0s: ALL 6 legs simultaneously ✅
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
        description='Number of legs'
    )
    
    nodes_to_launch = []
    
    # =================================================================
    # CRITICAL: Joint State Splitter FIRST at 2s
    # =================================================================
    
    joint_splitter = Node(
        package='hexapod',
        executable='joint_state_splitter.py',
        name='joint_state_splitter',
        namespace='hexapod',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
    
    nodes_to_launch.append(TimerAction(
        period=2.0,
        actions=[joint_splitter]
    ))
    
    # =================================================================
    # GLOBAL NODES at 3s
    # =================================================================
    
    gait_planner = Node(
        package='hexapod',
        executable='gait_planning.py',
        name='gait_planner',
        namespace='hexapod',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'default_gait_type': 0,
            'max_linear_x': 2.0,
            'max_linear_y': 0.2,
            'max_angular_z': 1.0,
            'step_height': 0.03,
            'step_length_scale': 0.5,  # Increased from 0.05 to make feet step further
            'cycle_time': 20.0,  # Increased to slow down leg movement
            # Default cmd_vel values (robot will walk forward at startup)
            'default_linear_x': 0.1,  # 0.1 m/s forward
            'default_linear_y': 0.0,
            'default_angular_z': 0.0,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('gait_parameters', '/hexapod/gait_parameters'),
            ('body_velocity', '/hexapod/body_velocity'),
        ],
    )
    
    state_machine = Node(
        package='hexapod',
        executable='state_controller.py',
        name='state_machine',
        namespace='hexapod',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'control_rate': 50.0,
        }],
        remappings=[
            ('gait_parameters', '/hexapod/gait_parameters'),
            ('body_velocity', '/hexapod/body_velocity'),
        ],
    )
    
    nodes_to_launch.append(TimerAction(
        period=3.0,
        actions=[gait_planner, state_machine]
    ))
    
    # =================================================================
    # ALL LEGS START SIMULTANEOUSLY at 4.0s ✅
    # =================================================================
    
    # Define leg attachment points and stance offsets (from URDF)
    # Each leg's offset is oriented in its own direction based on hexagon geometry
    # Reduced to 0.18m radial distance for reachability
    leg_configs = {
        1: {'attach': [-1.2616e-05, -0.095255, 0.0], 'rpy': [0.0, 0.0, -1.5708], 'offset': [0.0, -0.18, -0.05]},    # Back (−90°)
        2: {'attach': [0.082487, -0.047638, 0.0], 'rpy': [0.0, 0.0, -0.5236], 'offset': [0.156, -0.09, -0.05]},     # Back-right (−30°)
        3: {'attach': [0.082499, 0.047616, 0.0], 'rpy': [0.0, 0.0, 0.5236], 'offset': [0.156, 0.09, -0.05]},        # Front-right (+30°)
        4: {'attach': [1.2616e-05, 0.095255, 0.0], 'rpy': [0.0, 0.0, 1.5708], 'offset': [0.0, 0.18, -0.05]},        # Front (+90°)
        5: {'attach': [-0.082487, 0.047638, 0.0], 'rpy': [0.0, 0.0, 2.618], 'offset': [-0.156, 0.09, -0.05]},       # Front-left (+150°)
        6: {'attach': [-0.082499, -0.047616, 0.0], 'rpy': [0.0, 0.0, -2.618], 'offset': [-0.156, -0.09, -0.05]},    # Back-left (−150°)
    }
    
    all_leg_nodes = []  # Collect ALL leg nodes here
    
    for leg_id in range(1, 7):
        
        leg_ns = f'hexapod/leg_{leg_id}'
        
        # Get leg-specific configuration
        attach = leg_configs[leg_id]['attach']
        offset = leg_configs[leg_id]['offset']
        rpy = leg_configs[leg_id]['rpy']
        
        # Calculate default position for this leg (attachment + offset)
        default_x = attach[0] + offset[0]
        default_y = attach[1] + offset[1]
        default_z = attach[2] + offset[2]
        
        # -----------------------------------------------------------
        # Set Point Generator
        # -----------------------------------------------------------
        all_leg_nodes.append(
            Node(
                package='hexapod',
                executable='set_point.py',
                name='set_point_generator',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'stance_offset_x': offset[0],  # Leg-specific stance offset
                    'stance_offset_y': offset[1],
                    'stance_offset_z': offset[2],
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
        all_leg_nodes.append(
            Node(
                package='hexapod',
                executable='trajectory_planning.py',
                name='trajectory_generator',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'trajectory_rate': 100.0,
                    'smoothing_alpha': 0.05,  # Reduced from 0.1 for smoother tracking
                    'default_x': default_x,  # Leg-specific default position
                    'default_y': default_y,
                    'default_z': default_z,
                }],
                remappings=[
                    ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                    ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                    ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                ],
            )
        )
        
        # -----------------------------------------------------------
        # Inverse Position Kinematics
        # -----------------------------------------------------------
        all_leg_nodes.append(
            Node(
                package='hexapod',
                executable='inverse_position_kinematic.py',
                name='inverse_kinematics',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'max_iterations': 15,
                    'tolerance': 0.001,
                    'use_numerical_ik': True,
                    'enforce_joint_limits': True,
                    'clamp_to_limits': False,
                    # Leg-specific hip attachment point
                    'hip_xyz_x': attach[0],
                    'hip_xyz_y': attach[1],
                    'hip_xyz_z': attach[2],
                    # Leg-specific hip orientation
                    'hip_rpy_roll': rpy[0],
                    'hip_rpy_pitch': rpy[1],
                    'hip_rpy_yaw': rpy[2],
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
        all_leg_nodes.append(
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
        all_leg_nodes.append(
            Node(
                package='hexapod',
                executable='pid_position_controller.py',
                name='position_pid_controller',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'position_kp': [12.5, 12.5, 10.0],
                    'position_ki': [0.5, 0.5, 1.0],
                    'position_kd': [0.5, 0.5, 0.75],
                    'velocity_limit': 10.0,
                    'control_rate': 100.0,
                    'integral_limit': 2.0,
                    'use_angle_wrapping': True,       # NEW: Angle wrap-around
                    'enforce_joint_limits': True,     # NEW: Joint limit enforcement
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
        all_leg_nodes.append(
            Node(
                package='hexapod',
                executable='pid_velocity_controller.py',
                name='velocity_pid_controller',
                namespace=leg_ns,
                output='screen',
                parameters=[{
                    'leg_id': leg_id,
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'velocity_kp': [5.0, 5.0, 5.0],
                    'velocity_ki': [0.5, 0.5, 0.5],
                    'velocity_kd': [0.0, 0.0, 0.0],
                    'effort_limit': 24.0,
                    'control_rate': 100.0,
                    'integral_limit': 1.0,
                    'use_feedforward': True,
                    'urdf_path': '/home/prime/kinematic_project_ws/src/hexapod_description/robot/visual/hexapod.urdf',
                    'use_gravity_compensation': True,
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
        # Forward Kinematics (Optional - monitoring)
        # -----------------------------------------------------------
        all_leg_nodes.append(
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
                    # Leg-specific hip attachment point
                    'hip_xyz_x': attach[0],
                    'hip_xyz_y': attach[1],
                    'hip_xyz_z': attach[2],
                    # Leg-specific hip orientation
                    'hip_rpy_roll': rpy[0],
                    'hip_rpy_pitch': rpy[1],
                    'hip_rpy_yaw': rpy[2],
                }],
                remappings=[
                    ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                    ('end_effector_position', f'/hexapod/leg_{leg_id}/end_effector_position'),
                ],
            )
        )
    
    # =================================================================
    # CRITICAL FIX: Launch ALL legs at the SAME TIME (4.0s) ✅
    # =================================================================
    nodes_to_launch.append(TimerAction(
        period=4.0,
        actions=all_leg_nodes  # All 42 leg nodes start together!
    ))
    
    # Return launch description
    return LaunchDescription([
        use_sim_time_arg,
        num_legs_arg,
        *nodes_to_launch
    ])