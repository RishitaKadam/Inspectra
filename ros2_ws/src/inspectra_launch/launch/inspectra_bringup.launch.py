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

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        arguments=[
            "--x", "0.5", "--y", "0.0", "--z", "1.0",
            "--roll", "0", "--pitch", "1.5708", "--yaw", "0",
            "--frame-id", "panda_link0", "--child-frame-id", "camera_link",
        ],
    )

    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
            "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame",
        ],
    )

    return LaunchDescription([
        demo,
        camera_mount_tf,
        camera_optical_tf,
        delayed_motion_planner_node,
    ])
