"""
Inspectra full demo: ONE command brings up everything except Gazebo itself
(a separate GUI process, unavoidably its own window).

Run this, then in a SEPARATE terminal:
    ros2 launch inspectra_description inspectra_gazebo.launch.py

This launches: RViz/MoveIt (fake-controller planning brain), camera TFs,
motion_planner_node, conveyor_motion_node (drives the Gazebo PCB boxes),
conveyor_publisher (cycles through real, DIFFERENT PCB photos, not one
static image), object_detector_node, pose_estimator_node.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    inspectra_launch_share = get_package_share_directory("inspectra_launch")
    bringup_launch = os.path.join(inspectra_launch_share, "launch", "inspectra_bringup.launch.py")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch)
    )

    conveyor_publisher_node = Node(
        package="inspectra_perception",
        executable="conveyor_publisher",
        name="conveyor_publisher",
        output="screen",
        parameters=[{"dwell_time_sec": 6.0}],
    )

    object_detector_node = Node(
        package="inspectra_perception",
        executable="object_detector_node",
        name="object_detector_node",
        output="screen",
    )

    pose_estimator_node = Node(
        package="inspectra_perception",
        executable="pose_estimator_node",
        name="pose_estimator_node",
        output="screen",
    )

    # Delay perception start until MoveIt/controllers are up (matches the
    # existing 6s delay pattern used for motion_planner_node in bringup)
    delayed_perception = TimerAction(
        period=8.0,
        actions=[
            conveyor_publisher_node,
            object_detector_node,
            pose_estimator_node,
        ],
    )

    return LaunchDescription([
        bringup,
        delayed_perception,
    ])
