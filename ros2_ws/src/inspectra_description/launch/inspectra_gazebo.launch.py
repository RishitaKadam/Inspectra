"""
Spawns the Inspectra Panda (via inspectra_panda.urdf.xacro) into Gazebo
Harmonic, with ros2_control's controller_manager driving it through the
gz_ros2_control plugin. MoveIt/RViz are NOT included here (Feature 7b).
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("inspectra_description")
    xacro_path = os.path.join(pkg_share, "urdf", "inspectra_panda.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "inspectra_world.sdf")

    # Rename fer_* -> panda_* so link/joint names match the existing
    # moveit_resources_panda_moveit_config SRDF/kinematics/joint_limits
    # (same physical robot structure, renamed by Franka upstream).
    # Safe as plain text substitution: mesh paths use "/fer/" (no "fer_"
    # substring), so only entity names are affected.
    generate_script = os.path.join(pkg_share, "urdf", "generate_panda_urdf.sh")
    robot_description = ParameterValue(
        Command(generate_script),
        value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "inspectra_panda", "-z", "0.0"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
    )

    # Start controllers only after the robot is actually spawned
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, panda_arm_controller_spawner],
        )
    )

    franka_share = os.path.dirname(get_package_share_directory("franka_description"))
    inspectra_share = os.path.dirname(pkg_share)

    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=f"{franka_share}:{inspectra_share}"
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        delayed_controllers,
    ])
