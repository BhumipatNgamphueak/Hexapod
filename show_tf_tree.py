#!/usr/bin/env python3
"""
TF Tree Visualization Helper
Prints the expected TF tree structure for the hexapod robot
"""

def print_tree():
    tree = """
    ╔════════════════════════════════════════════════════════════╗
    ║         HEXAPOD TF TREE STRUCTURE                          ║
    ║         (Gazebo + RViz Consistent Frame)                   ║
    ╚════════════════════════════════════════════════════════════╝

    world (Fixed Reference Frame)
      │
      └── base_link (Robot Body Center)
            │
            ├── hip_link_1 ──> knee_link_1 ──> ankle_link_1 ──> foot_pointer_1 ──> end_effector_1 ──> foot_contact_1
            │
            ├── hip_link_2 ──> knee_link_2 ──> ankle_link_2 ──> foot_pointer_2 ──> end_effector_2 ──> foot_contact_2
            │
            ├── hip_link_3 ──> knee_link_3 ──> ankle_link_3 ──> foot_pointer_3 ──> end_effector_3 ──> foot_contact_3
            │
            ├── hip_link_4 ──> knee_link_4 ──> ankle_link_4 ──> foot_pointer_4 ──> end_effector_4 ──> foot_contact_4
            │
            ├── hip_link_5 ──> knee_link_5 ──> ankle_link_5 ──> foot_pointer_5 ──> end_effector_5 ──> foot_contact_5
            │
            └── hip_link_6 ──> knee_link_6 ──> ankle_link_6 ──> foot_pointer_6 ──> end_effector_6 ──> foot_contact_6

    ╔════════════════════════════════════════════════════════════╗
    ║  COORDINATE FRAME CONVENTIONS (REP-103)                    ║
    ╚════════════════════════════════════════════════════════════╝

    world frame:
      • Origin: (0, 0, 0) - Ground plane center
      • X-axis: Forward (red)
      • Y-axis: Left (green)
      • Z-axis: Up (blue)
      • Convention: Right-handed

    base_link frame:
      • Origin: Center of hexapod body
      • Initial position: (0.0, 0.0, 0.1) in world frame
      • Orientation: Initially aligned with world
      • Moves with robot during simulation

    ╔════════════════════════════════════════════════════════════╗
    ║  FRAME PUBLISHERS                                          ║
    ╚════════════════════════════════════════════════════════════╝

    robot_state_publisher:
      • Publishes URDF structure (world -> base_link -> legs)
      • Frequency: 100 Hz
      • Source: hexapod.xacro

    Gazebo Pose Publisher:
      • Publishes dynamic base_link pose in world frame
      • Frequency: 100 Hz
      • Source: Physics simulation

    ╔════════════════════════════════════════════════════════════╗
    ║  VERIFICATION COMMANDS                                     ║
    ╚════════════════════════════════════════════════════════════╝

    # View TF tree (generates frames.pdf)
    $ ros2 run tf2_tools view_frames

    # Echo world -> base_link transform
    $ ros2 run tf2_ros tf2_echo world base_link

    # List all TF frames
    $ ros2 topic echo /tf_static
    $ ros2 topic echo /tf

    # Check TF topics
    $ ros2 topic list | grep tf

    ╔════════════════════════════════════════════════════════════╗
    ║  RVIZ CONFIGURATION                                        ║
    ╚════════════════════════════════════════════════════════════╝

    1. Open RViz: ros2 launch hexapod_simulation simulation-full.launch.py
    2. Set "Fixed Frame" to "world" in left panel
    3. Add displays:
       - RobotModel: Shows the hexapod structure
       - TF: Shows all coordinate frames
       - Axes: Shows XYZ axes at world origin
    4. Robot should appear 10cm above ground at origin

    """
    print(tree)

if __name__ == '__main__':
    print_tree()
