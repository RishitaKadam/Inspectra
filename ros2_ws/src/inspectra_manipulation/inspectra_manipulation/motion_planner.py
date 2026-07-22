"""
ROS2 node wrapping MotionPlanner so it can be launched via inspectra_launch
instead of run as a standalone script.

Commands a move by publishing a pose name (from pose_library) to the
'~/move_to_pose' topic, e.g.:
    ros2 topic pub --once /motion_planner_node/move_to_pose std_msgs/String "data: 'READY'"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from inspectra_manipulation.planner import MotionPlanner
from inspectra_manipulation import pose_library


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")

        self.declare_parameter("startup_pose", "")
        startup_pose = self.get_parameter("startup_pose").get_parameter_value().string_value

        self.get_logger().info("Starting MotionPlannerNode, constructing MotionPlanner...")
        self._planner = MotionPlanner(node_name="inspectra_moveit_py")

        self._sub = self.create_subscription(
            String, "~/move_to_pose", self._on_move_to_pose, 10
        )

        self.get_logger().info(
            "MotionPlannerNode ready. Publish a pose name (e.g. 'READY') to "
            "'~/move_to_pose' to command a move."
        )

        if startup_pose:
            self.get_logger().info(f"Executing startup_pose='{startup_pose}'")
            self._move_to_named_pose(startup_pose)

    def _on_move_to_pose(self, msg: String):
        self._move_to_named_pose(msg.data)

    def _move_to_named_pose(self, name: str):
        try:
            pose = pose_library.get_pose(name)
        except (KeyError, NotImplementedError) as e:
            self.get_logger().error(str(e))
            return

        config_name = pose["value"]
        self.get_logger().info(f"Moving to pose '{name}' -> config '{config_name}'")
        success = self._planner.plan_and_execute(config_name=config_name)
        if success:
            self.get_logger().info(f"Reached pose '{name}'")
        else:
            self.get_logger().error(f"Failed to reach pose '{name}'")

    def destroy_node(self):
        self._planner.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
