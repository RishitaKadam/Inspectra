"""
MotionPlanner: a thin, reusable wrapper around MoveItPy for the Panda arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

from inspectra_manipulation.utils import get_inspectra_logger


class MotionPlanner:
    def __init__(self, node_name: str = "inspectra_moveit_py", group_name: str = "panda_arm"):
        self._logger = get_inspectra_logger("planner")
        self._logger.info("Initializing MoveItPy instance...")

        config_yaml_path = os.path.join(
            get_package_share_directory("inspectra_manipulation"),
            "config",
            "motion_planning_python_api_tutorial.yaml",
        )

        if not os.path.isfile(config_yaml_path):
            raise FileNotFoundError(
                f"MoveItCpp config not found at {config_yaml_path}. "
                "Did colcon build install it under share/inspectra_manipulation/config/?"
            )

        moveit_config = (
            MoveItConfigsBuilder(
                robot_name="panda",
                package_name="moveit_resources_panda_moveit_config",
            )
            .moveit_cpp(file_path=config_yaml_path)
            .to_moveit_configs()
            .to_dict()
        )

        self._moveit = MoveItPy(
            node_name=node_name,
            config_dict=moveit_config,
        )

        self._arm = self._moveit.get_planning_component(group_name)
        self._group_name = group_name

        self._logger.info(f"MoveItPy instance created, group='{group_name}'")

    @property
    def moveit_py(self):
        """Expose the underlying MoveItPy instance."""
        return self._moveit

    def set_start_to_current(self) -> None:
        self._arm.set_start_state_to_current_state()

    def plan_to_named_config(self, config_name: str):
        self._arm.set_goal_state(configuration_name=config_name)
        return self._plan()

    def plan_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        frame_id: str = "panda_link0",
        pose_link: str = "panda_link8",
        qx: float = 0.0,
        qy: float = 0.0,
        qz: float = 0.0,
        qw: float = 1.0,
    ):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = frame_id

        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z

        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw

        self._arm.set_goal_state(
            pose_stamped_msg=pose_goal,
            pose_link=pose_link,
        )

        return self._plan()

    def plan_to_joint_values(self, joint_values: dict):
        from moveit.core.kinematic_constraints import construct_joint_constraint

        robot_model = self._moveit.get_robot_model()
        robot_state = RobotState(robot_model)
        robot_state.joint_positions = joint_values

        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=robot_model.get_joint_model_group(self._group_name),
        )

        self._arm.set_goal_state(
            motion_plan_constraints=[joint_constraint]
        )

        return self._plan()

    def _plan(self):
        self._logger.info("Planning trajectory...")

        plan_result = self._arm.plan()

        if not plan_result:
            self._logger.error("Planning failed")

        return plan_result

    def execute(self, plan_result) -> bool:
        if not plan_result:
            self._logger.error("No valid plan to execute")
            return False

        self._logger.info("Executing trajectory...")

        self._moveit.execute(
            plan_result.trajectory,
            controllers=[],
        )

        return True

    def plan_and_execute(
        self,
        config_name=None,
        joint_values=None,
        pose_xyz=None,
    ):
        self.set_start_to_current()

        if config_name is not None:
            result = self.plan_to_named_config(config_name)
        elif joint_values is not None:
            result = self.plan_to_joint_values(joint_values)
        elif pose_xyz is not None:
            result = self.plan_to_pose(*pose_xyz)
        else:
            raise ValueError(
                "Provide one of: config_name, joint_values or pose_xyz"
            )

        return self.execute(result)

    def shutdown(self):
        self._logger.info("Shutting down MoveItPy instance")
        self._moveit.shutdown()
