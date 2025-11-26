#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package shares
    desc_pkg = get_package_share_directory('hexapod_description')
    sim_pkg = get_package_share_directory('hexapod_simulation')

    # Get the parent directory (install/share) to make model:// URIs work
    install_share_dir = os.path.dirname(desc_pkg)
    
    # Set Gazebo resource paths - point to the share directory containing all packages
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )
    
    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=install_share_dir + ':' + 
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description (xacro)
    xacro_file = PathJoinSubstitution([
        FindPackageShare('hexapod_description'), 
        'robot',
        'visual', 
        'hexapod.xacro'
    ])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Gazebo simulation launch with default empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': '-r -v 1'}.items()
    )

    # Robot state publisher
    state_pub = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',
        parameters=[
            {'robot_description': robot_description_param},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create', 
        name='spawn_entity', 
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'hexapod',
            '-allow_renaming', 'true',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ]
    )

    # Controller manager spawners
    jsb_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn effort controllers for each leg
    effort_spawner_leg_1 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_1',
        arguments=['effort_controller_leg_1', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner_leg_2 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_2',
        arguments=['effort_controller_leg_2', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner_leg_3 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_3',
        arguments=['effort_controller_leg_3', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner_leg_4 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_4',
        arguments=['effort_controller_leg_4', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner_leg_5 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_5',
        arguments=['effort_controller_leg_5', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    effort_spawner_leg_6 = Node(
        package='controller_manager', 
        executable='spawner', 
        name='spawner_effort_controller_leg_6',
        arguments=['effort_controller_leg_6', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ROS <-> Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # RViz
    rviz_config = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ============================================================================
    # GAIT PLANNER - Generate gait patterns and foot trajectories
    # ============================================================================

    gait_planner = Node(
        package='hexapod',
        executable='gait_planning.py',
        name='gait_planner',
        namespace='hexapod',
        output='screen',
        parameters=[
            {'default_gait_type': 0},  # 0=tripod, 1=wave, 2=ripple
            {'max_linear_x': 0.3},
            {'max_linear_y': 0.2},
            {'max_angular_z': 1.0},
            {'step_height': 0.03},
            {'step_length_scale': 0.05},
            {'min_cycle_time': 0.5},
            {'max_cycle_time': 2.0},
            {'num_legs': 6},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # ============================================================================
    # JOINT STATE SPLITTER - Split /joint_states into per-leg topics
    # ============================================================================

    joint_state_splitter = Node(
        package='hexapod',
        executable='joint_state_splitter.py',
        name='joint_state_splitter',
        namespace='hexapod',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('joint_states', '/joint_states'),
            ('leg_1/joint_states', '/hexapod/leg_1/joint_states'),
            ('leg_2/joint_states', '/hexapod/leg_2/joint_states'),
            ('leg_3/joint_states', '/hexapod/leg_3/joint_states'),
            ('leg_4/joint_states', '/hexapod/leg_4/joint_states'),
            ('leg_5/joint_states', '/hexapod/leg_5/joint_states'),
            ('leg_6/joint_states', '/hexapod/leg_6/joint_states')
        ]
    )

    # ============================================================================
    # TRAJECTORY GENERATORS - Smooth trajectory generation for each leg
    # ============================================================================

    trajectory_generators = []

    for leg_id in range(1, 7):
        traj_gen = Node(
            package='hexapod',
            executable='trajectory_planning.py',
            name=f'trajectory_generator_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'interpolation_method': 'cubic'},
                {'trajectory_rate': 100.0},
                {'swing_clearance': 0.03},
                {'vmax': 1.0},
                {'amax': 2.0},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('end_effector_setpoint', f'/hexapod/leg_{leg_id}/end_effector_setpoint'),
                ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity')
            ]
        )
        trajectory_generators.append(traj_gen)

    # ============================================================================
    # KINEMATIC NODES - IK and IVK for all 6 legs
    # ============================================================================

    ik_nodes = []
    ivk_nodes = []

    for leg_id in range(1, 7):
        # Inverse Position Kinematics (IK)
        ik_node = Node(
            package='hexapod',
            executable='inverse_position_kinematic.py',
            name=f'ik_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'use_numerical_ik': True},  # Use high-accuracy numerical IK
                {'max_iterations': 15},
                {'tolerance': 0.001},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target')
            ]
        )

        # Inverse Velocity Kinematics (IVK) - for feedforward control
        ivk_node = Node(
            package='hexapod',
            executable='inverse_velocity_kinematic.py',
            name=f'ivk_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'update_rate': 100.0},
                {'damping_factor': 0.01},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                ('end_effector_velocity', f'/hexapod/leg_{leg_id}/end_effector_velocity'),
                ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward')
            ]
        )

        ik_nodes.append(ik_node)
        ivk_nodes.append(ivk_node)

    # ============================================================================
    # PID CONTROLLERS - Position and Velocity Control for all 6 legs
    # ============================================================================

    # PID Parameters
    position_kp = [10.0, 10.0, 10.0]
    position_ki = [0.1, 0.1, 0.1]
    position_kd = [1.0, 1.0, 1.0]

    # Velocity PID - ULTRA-LOW GAINS FOR STABILITY (FINAL ATTEMPT)
    # [Hip, Knee, Ankle]
    # System is HIGHLY unstable - need VERY low gains
    velocity_kp = [0.05, 0.05, 0.3]  # Reduced 5x from 0.25 → 0.05
    velocity_ki = [0.01, 0.01, 0.0]  # Reduced 10x from 0.1 → 0.01
    velocity_kd = [0.0, 0.01, 0.0]   # Reduced 5x from 0.05 → 0.01
    damping_coefficient = [0.0, 0.0, 0.0]  # Disable - using Kd instead
    effort_rate_limit = 1000.0  # VERY HIGH for testing (disable rate limiting)
    antiwindup_gain = 1.0

    # Create PID controller nodes for each leg
    pid_controllers = []

    for leg_id in range(1, 7):
        # Position PID Controller (Outer Loop)
        position_pid = Node(
            package='hexapod',
            executable='pid_position_controller.py',
            name=f'position_pid_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'position_kp': position_kp},
                {'position_ki': position_ki},
                {'position_kd': position_kd},
                {'velocity_limit': 5.0},
                {'control_rate': 100.0},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target'),
                ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target')
            ]
        )

        # Velocity PID Controller (Inner Loop)
        velocity_pid = Node(
            package='hexapod',
            executable='pid_velocity_controller.py',
            name=f'velocity_pid_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'velocity_kp': velocity_kp},
                {'velocity_ki': velocity_ki},
                {'velocity_kd': velocity_kd},
                {'damping_coefficient': damping_coefficient},
                {'effort_limit': 20.0},
                {'control_rate': 100.0},  # Using 100 Hz (testing if 500 Hz was too fast)
                {'effort_rate_limit': effort_rate_limit},
                {'antiwindup_gain': antiwindup_gain},
                {'derivative_filter_alpha': 0.1},
                {'gravity_filter_alpha': 0.05},
                {'use_feedforward': False},  # DISABLED for baseline testing
                {'use_gravity_compensation': False},  # DISABLED - causing position drift!
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('joint_states', f'/hexapod/leg_{leg_id}/joint_states'),
                ('joint_velocity_target', f'/hexapod/leg_{leg_id}/joint_velocity_target'),
                ('joint_velocity_feedforward', f'/hexapod/leg_{leg_id}/joint_velocity_feedforward'),
                ('commands', f'/effort_controller_leg_{leg_id}/commands')
            ]
        )

        pid_controllers.append(position_pid)
        pid_controllers.append(velocity_pid)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gz_model_path,
        ign_model_path,
        gz_sim,
        state_pub,
        spawn_entity,
        bridge,
        # After spawning the robot, start the joint_state_broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),
        # After joint_state_broadcaster is active, start effort controllers AND joint state splitter
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[
                    effort_spawner_leg_1,
                    effort_spawner_leg_2,
                    effort_spawner_leg_3,
                    effort_spawner_leg_4,
                    effort_spawner_leg_5,
                    effort_spawner_leg_6,
                    joint_state_splitter  # Start splitter with effort controllers
                ]
            )
        ),
        # After effort controllers are ready, start ONLY Velocity PID controllers
        # NOTE: For PID tuning, we start ONLY velocity PIDs.
        # Position PIDs, IK, trajectory generators, and gait planner are DISABLED.
        # To enable full system, uncomment the line below and comment out the velocity-only line.
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=effort_spawner_leg_6,
                on_exit=[pid_controllers[i] for i in range(1, len(pid_controllers), 2)]  # Only velocity PIDs (odd indices)
                # FULL SYSTEM (uncomment to enable):
                # on_exit=pid_controllers + ik_nodes + ivk_nodes + trajectory_generators + [gait_planner]
            )
        ),
        rviz
    ])