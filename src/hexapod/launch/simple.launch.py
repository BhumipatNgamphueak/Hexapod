from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()
    number_of_leg = 6
    leg_ns = ["top_left_leg","middle_left_leg","bottom_left_leg","top_right_leg","middle_right_leg","bottom_right_leg"]
    #leg_ns = ["top_left_leg"]
    #node_name = ["controller","forward_kinematic","gait_planning","inverse_kinematic","set_point","state_controller","trajectory_planning"]
    node_name = ["forward_kinematic","gait_planning","inverse_kinematic","set_point","trajectory_planning"]
    joint_name = ["base", "hip", "knee"]


    for namespace in leg_ns:
        for node in node_name:

            leg_node = Node(
                package = 'hexapod',
                namespace = namespace,
                executable = node + ".py",
                name = node
            )
            launch_description.add_action(leg_node)

    for namespace in leg_ns:
        for joint in joint_name:

            joint_controller_node = Node(
                    package = 'controller',
                    namespace = namespace + "/" +joint,
                    executable = "controller.py",
                    name = "controller"
                )
            launch_description.add_action(joint_controller_node)

    state_control = Node(
        package = 'hexapod',
        namespace = "",
        executable = "state_controller.py",
        name = "state_controller"
        )
    launch_description.add_action(state_control)
    
    return launch_description
