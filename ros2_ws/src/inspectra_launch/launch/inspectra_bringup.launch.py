"""
Brings up the full Inspectra simulation stack in one command:
  - Panda MoveIt demo (robot_state_publisher, fake ros2_control, RViz, move_group)
  - inspectra_manipulation's motion_planner_node (delayed to let controllers start)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    startup_pose_arg = DeclareLaunchArgument(
        "startup_pose",
        default_value="",
        description="Named pose to execute after motion planner startup",
    )

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
        parameters=[{
            "startup_pose": LaunchConfiguration("startup_pose"),
            "use_sim_time": False,
        }],
    )

    delayed_motion_planner_node = TimerAction(
        period=6.0,
        actions=[motion_planner_node],
    )

    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_panda_link0_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world", "--child-frame-id", "panda_link0",
        ],
    )

    camera_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_mount_tf",
        arguments=[
            "--x", "0.78", "--y", "-0.15", "--z", "0.62",
            "--roll", "0", "--pitch", "1.20", "--yaw", "0",
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

    inspectra_launch_share = get_package_share_directory("inspectra_launch")
    rviz_config_path = os.path.join(inspectra_launch_share, "config", "inspectra.rviz")

    custom_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_inspectra",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": False}],
    )
    delayed_rviz = TimerAction(period=3.0, actions=[custom_rviz])

    sim_time_param = SetParameter(name="use_sim_time", value=False)

    return LaunchDescription([
        startup_pose_arg,
        sim_time_param,
        demo,
        world_tf,
        camera_mount_tf,
        camera_optical_tf,
        delayed_motion_planner_node,
        delayed_rviz,
    ])
