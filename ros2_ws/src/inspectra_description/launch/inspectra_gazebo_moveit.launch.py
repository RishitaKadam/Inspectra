"""
Combined Gazebo + MoveIt launch (Feature 7b-ii): spawns the Inspectra
Panda in Gazebo (real physics), starts move_group using MoveItConfigsBuilder
(same proven builder used by demo.launch.py and MotionPlanner) so all the
internal namespacing (ompl.*, controller configs, etc.) is handled
correctly, then overrides robot_description with our own Gazebo-connected
URDF and the controller config with our real panda_arm_controller.
Executed trajectories go through Gazebo's real FollowJointTrajectory
action, not RViz's fake controller.
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_share_dir, file_relative_path):
    path = os.path.join(package_share_dir, file_relative_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    inspectra_share = get_package_share_directory("inspectra_description")

    # 1. Bring up Gazebo + spawned robot + real controllers (Feature 7a/7b-i)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspectra_share, "launch", "inspectra_gazebo.launch.py")
        )
    )

    # 2. Our own Gazebo-connected robot_description (fer_* renamed to panda_*)
    generate_script = os.path.join(inspectra_share, "urdf", "generate_panda_urdf.sh")
    our_robot_description = {
        "robot_description": ParameterValue(Command(generate_script), value_type=str)
    }

    # 3. Build the rest of the MoveIt config using the SAME proven builder
    #    MotionPlanner (planner.py) already uses successfully — handles all
    #    internal namespacing (ompl.*, adapters, etc.) correctly, unlike our
    #    earlier hand-rolled yaml loading attempt.
    moveit_config = (
        MoveItConfigsBuilder(
            "inspectra_panda",
            package_name="inspectra_moveit_config"
        ).to_moveit_configs()
    )
     

    config_dict = moveit_config.to_dict()
    # Override: use OUR Gazebo-connected URDF instead of moveit_resources'
    # own fake-controller URDF (safe: joint/link names match after rename)
    print(config_dict.keys()) 
    config_dict["robot_description"] = our_robot_description["robot_description"]

    # Override: point at Gazebo's REAL panda_arm_controller instead of
    # moveit_resources' bundled fake controller config
    moveit_controllers_yaml = load_yaml(
        inspectra_share, os.path.join("config", "inspectra_moveit_controllers.yaml")
    )
    config_dict.update(moveit_controllers_yaml)
    config_dict["use_sim_time"] = True

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
    )

    panda_moveit_share = get_package_share_directory("moveit_resources_panda_moveit_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(panda_moveit_share, "launch", "moveit.rviz")],
        parameters=[
            our_robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    delayed_rviz = TimerAction(period=5.0, actions=[rviz_node])

    return LaunchDescription([
        gazebo_launch,
        move_group_node,
        delayed_rviz,
    ])
