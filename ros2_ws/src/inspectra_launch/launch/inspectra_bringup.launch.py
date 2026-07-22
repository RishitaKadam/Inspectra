"""
Brings up the full Inspectra simulation stack in one command:
  - Panda MoveIt demo (robot_state_publisher, fake ros2_control, RViz, move_group)
  - inspectra_manipulation's motion_planner_node (delayed to let controllers start)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    panda_demo_launch = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "launch",
        "demo.launch.py",
    )

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panda_demo_launch)
    )

    motion_planner_node = Node(
        package="inspectra_manipulation",
        executable="motion_planner_node",
        name="motion_planner_node",
        output="screen",
        parameters=[{"startup_pose": ""}],
    )

    delayed_motion_planner_node = TimerAction(
        period=6.0,
        actions=[motion_planner_node],
    )

    return LaunchDescription([
        demo,
        delayed_motion_planner_node,
    ])
