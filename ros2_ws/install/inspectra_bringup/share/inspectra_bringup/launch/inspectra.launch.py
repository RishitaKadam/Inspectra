from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    controller = Node(
        package="inspectra_bringup",
        executable="controller",
        name="inspectra_controller",
        output="screen",
    )

    return LaunchDescription([
        controller
    ])
