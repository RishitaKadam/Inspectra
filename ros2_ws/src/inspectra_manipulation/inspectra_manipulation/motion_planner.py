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
from inspectra_manipulation.scene_manager import SceneManager
from inspectra_manipulation.executor import TrajectoryExecutor
from geometry_msgs.msg import PoseStamped
from inspectra_manipulation import pose_library


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")

        self.declare_parameter("startup_pose", "")
        startup_pose = self.get_parameter("startup_pose").get_parameter_value().string_value

        self.get_logger().info("Starting MotionPlannerNode, constructing MotionPlanner...")
        self._planner = MotionPlanner(node_name="inspectra_moveit_py")

        self._scene = SceneManager(self._planner.moveit_py)
        self._scene.add_inspection_table()

        self._executor = TrajectoryExecutor(self._planner, max_retries=1)

        self._sub = self.create_subscription(
            String, "~/move_to_pose", self._on_move_to_pose, 10
        )

        self._seq_sub = self.create_subscription(
            String, "~/move_sequence", self._on_move_sequence, 10
        )

        self._latest_pick_pose = None
        self._pick_pose_sub = self.create_subscription(
            PoseStamped, "/pose_estimator_node/pick_pose", self._on_pick_pose, 10
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

    def _on_pick_pose(self, msg: PoseStamped):
        self._latest_pick_pose = msg

    def _move_to_named_pose(self, name: str):
        if name.upper() == "PICK":
            self._move_to_detected_pick_pose()
            return

        try:
            pose = pose_library.get_pose(name)
        except (KeyError, NotImplementedError) as e:
            self.get_logger().error(str(e))
            return

        config_name = pose["value"]
        self.get_logger().info(f"Moving to pose '{name}' -> config '{config_name}'")
        success = self._executor.execute_named_pose(config_name)
        if success:
            self.get_logger().info(f"Reached pose '{name}'")
        else:
            self.get_logger().error(f"Failed to reach pose '{name}'")

    def _on_move_sequence(self, msg: String):
        names = [n.strip() for n in msg.data.split(",") if n.strip()]
        if not names:
            self.get_logger().warning("Received empty move_sequence message, ignoring")
            return

        config_names = []
        for name in names:
            try:
                pose = pose_library.get_pose(name)
            except (KeyError, NotImplementedError) as e:
                self.get_logger().error(f"Aborting sequence: {e}")
                return
            config_names.append(pose["value"])

        self.get_logger().info(f"Starting sequence: {names}")
        results = self._executor.execute_sequence(config_names, stop_on_failure=True)
        self.get_logger().info(f"Sequence results: {results}")

    def _move_to_detected_pick_pose(self):
        if self._latest_pick_pose is None:
            self.get_logger().error(
                "No pick pose received yet from pose_estimator_node "
                "(is inspectra_perception running and detecting something?)"
            )
            return

        pose = self._latest_pick_pose.pose
        hover_z = pose.position.z + 0.15  # approach from above, don't collide with table
        self.get_logger().info(
            f"Moving to hover pose above detected object: "
            f"x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={hover_z:.3f}"
        )

        self.set_start_to_current_via_planner()
        result = self._planner.plan_to_pose(
            x=pose.position.x, y=pose.position.y, z=hover_z,
            frame_id="panda_link0",
        )
        success = self._planner.execute(result)
        if success:
            self.get_logger().info("Reached hover pose above detected object")
        else:
            self.get_logger().error("Failed to plan/execute to detected pick pose")

    def set_start_to_current_via_planner(self):
        self._planner.set_start_to_current()

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
