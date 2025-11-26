#!/usr/bin/env python3
"""
Hexapod Simulation with Joint Trajectory Controller
This launch file uses Joint Trajectory Controller instead of custom PID controllers.

ARCHITECTURE:
  Gait Planning → Trajectory Generation → IK → Bridge → Joint Trajectory Controller → Gazebo
       ↓                   ↓               ↓       ↓              ↓
  (Kinematic)        (Kinematic)     (Kinematic) (Bridge)    (Control)
"""
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

    # Set Gazebo resource paths - FIXED: Add explicit hexapod_description path
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=desc_pkg + ':' + install_share_dir + ':' +
              os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    ign_model_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=desc_pkg + ':' + install_share_dir + ':' +
              os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    )

    # Robot description (xacro) - Using trajectory controller version
    xacro_file = PathJoinSubstitution([
        FindPackageShare('hexapod_description'),
        'robot',
        'visual',
        'hexapod_trajectory.xacro'
    ])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])

    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # Gazebo simulation launch
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
            '-x', '0.0', '-y', '0.0', '-z', '0.2'  # Increased spawn height
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

    # Spawn Joint Trajectory Controllers for each leg
    trajectory_spawners = []
    for leg_id in range(1, 7):
        spawner = Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_trajectory_controller_leg_{leg_id}',
            arguments=[f'joint_trajectory_controller_leg_{leg_id}', '--controller-manager', '/controller_manager'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        trajectory_spawners.append(spawner)

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
    # KINEMATIC LAYER - IK, IVK, Gait Planning, Trajectory Generation
    # ============================================================================

    # Joint State Splitter - Split /joint_states into per-leg topics
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

    # Gait Planner - Generate gait patterns
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
            # Leg home positions as flat list (will be reshaped to 6x3 in node)
            {'leg_home_positions': [
                0.15, -0.15, -0.10,  # Leg 1 (Front Right)
                0.15,  0.00, -0.10,  # Leg 2 (Middle Right)
                0.15,  0.15, -0.10,  # Leg 3 (Rear Right)
               -0.15,  0.15, -0.10,  # Leg 4 (Rear Left)
               -0.15,  0.00, -0.10,  # Leg 5 (Middle Left)
               -0.15, -0.15, -0.10   # Leg 6 (Front Left)
            ]},
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # Trajectory Generators - Smooth trajectory generation for each leg
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

    # IK Nodes - Inverse Position Kinematics for all 6 legs
    ik_nodes = []
    for leg_id in range(1, 7):
        ik_node = Node(
            package='hexapod',
            executable='inverse_position_kinematic.py',
            name=f'ik_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'use_numerical_ik': True},
                {'max_iterations': 15},
                {'tolerance': 0.001},
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('end_effector_target', f'/hexapod/leg_{leg_id}/end_effector_target'),
                ('joint_position_target', f'/hexapod/leg_{leg_id}/joint_position_target')
            ]
        )
        ik_nodes.append(ik_node)

    # ============================================================================
    # BRIDGE LAYER - Convert IK output to Joint Trajectory messages
    # ============================================================================

    bridge_nodes = []
    for leg_id in range(1, 7):
        bridge_node = Node(
            package='hexapod',
            executable='trajectory_controller_bridge.py',
            name=f'trajectory_bridge_leg_{leg_id}',
            namespace=f'hexapod/leg_{leg_id}',
            output='screen',
            parameters=[
                {'leg_id': leg_id},
                {'trajectory_duration': 0.5},  # 500ms - เพิ่มจาก 100ms เพื่อให้ robot มีเวลาเคลื่อนที่
                {'use_action_interface': False},  # Use topic interface
                {'use_sim_time': use_sim_time}
            ]
        )
        bridge_nodes.append(bridge_node)

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
        # After joint_state_broadcaster, start trajectory controllers AND joint state splitter
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=trajectory_spawners + [joint_state_splitter]
            )
        ),
        # After trajectory controllers are ready, start Kinematic layer + Bridge
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=trajectory_spawners[-1],  # Wait for last spawner
                on_exit=ik_nodes + trajectory_generators + bridge_nodes + [gait_planner]
            )
        ),
        rviz
    ])
