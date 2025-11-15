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

    # Gazebo simulation launch with custom world
    world_file = PathJoinSubstitution([
        FindPackageShare('hexapod_simulation'),
        'worlds',
        'hexapod.sdf'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items()
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
        # After joint_state_broadcaster is active, start ALL effort controllers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[
                    effort_spawner_leg_1,
                    effort_spawner_leg_2,
                    effort_spawner_leg_3,
                    effort_spawner_leg_4,
                    effort_spawner_leg_5,
                    effort_spawner_leg_6
                ]
            )
        ),
        rviz
    ])