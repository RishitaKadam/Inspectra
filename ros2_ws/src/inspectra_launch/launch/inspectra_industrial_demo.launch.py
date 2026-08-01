"""
Inspectra Industrial PCB Sorting Demo Launch File.
Launches the full ROS 2 industrial pipeline:
  - Upstream camera TF static transform publishers
  - RViz visualization with custom dark brown theme & MoveIt planning panels
  - MotionPlannerNode (MoveIt 2 state machine & 3D pick-and-place planner)
  - ConveyorMotionNode (continuous conveyor belt motion & box spawning)
  - ConveyorPublisher (real PCB photo defect stream)
  - ObjectDetectorNode (YOLOv8 defect classifier)
  - PoseEstimatorNode (Raycasting 3D pose estimator)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    inspectra_launch_share = get_package_share_directory("inspectra_launch")
    inspectra_description_share = get_package_share_directory("inspectra_description")
    bringup_launch = os.path.join(inspectra_launch_share, "launch", "inspectra_bringup.launch.py")
    gazebo_launch = os.path.join(inspectra_description_share, "launch", "inspectra_gazebo.launch.py")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments=[("startup_pose", "READY")],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch)
    )

    conveyor_motion_node = Node(
        package="inspectra_perception",
        executable="conveyor_motion_node",
        name="conveyor_motion_node",
        output="screen",
        parameters=[{"belt_speed": 0.10}],
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

    rqt_graph_node = Node(
        package="rqt_graph",
        executable="rqt_graph",
        name="rqt_graph",
        output="screen",
    )

    delayed_perception = TimerAction(
        period=7.0,
        actions=[
            conveyor_motion_node,
            conveyor_publisher_node,
            object_detector_node,
            pose_estimator_node,
        ],
    )

    delayed_graph = TimerAction(
        period=8.5,
        actions=[rqt_graph_node],
    )

    headless_env = SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="offscreen")

    return LaunchDescription([
        headless_env,
        gazebo,
        bringup,
        delayed_perception,
        delayed_graph,
    ])
