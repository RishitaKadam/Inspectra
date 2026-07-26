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
from vision_msgs.msg import Detection2DArray
from inspectra_manipulation import inspection_logic
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

        self._latest_detection = None  # (class_name, confidence)
        self._cycle_in_progress = False
        self._detections_sub = self.create_subscription(
            Detection2DArray, "/object_detector_node/detections", self._on_detections, 10
        )

        self._run_cycle_sub = self.create_subscription(
            String, "~/run_cycle", self._on_run_cycle, 10
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

    def _on_detections(self, msg: Detection2DArray):
        detections = [
            (d.results[0].hypothesis.class_id, d.results[0].hypothesis.score)
            for d in msg.detections
        ]
        self._latest_detection = detections[0] if detections else None

        if self._cycle_in_progress:
            return  # don't interrupt an active pick/sort cycle

        classification = inspection_logic.classify_pcb(detections)
        if classification == "GOOD":
            self.get_logger().info("GOOD PCB detected -> diverting to GOOD_BIN")
            self._cycle_in_progress = True
            self._run_inspection_cycle()
            self._cycle_in_progress = False
        else:
            self.get_logger().info(
                "BAD PCB detected -> no robot action, continues on belt to reject area"
            )

    def _on_run_cycle(self, msg: String):
        self._run_inspection_cycle()

    def _move_to_named_pose(self, name: str):
        if name.upper() == "PICK":
            self._move_to_detected_pick_pose()
            return

        try:
            pose = pose_library.get_pose(name)
        except (KeyError, NotImplementedError) as e:
            self.get_logger().error(str(e))
            return

        if pose["type"] == pose_library.PoseType.CARTESIAN:
            x, y, z = pose["value"]
            success = self._move_to_cartesian(x, y, z)
        else:
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
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        )
        success = self._planner.execute(result)
        if success:
            self.get_logger().info("Reached hover pose above detected object")
        else:
            self.get_logger().error("Failed to plan/execute to detected pick pose")

    def set_start_to_current_via_planner(self):
        self._planner.set_start_to_current()

    def _move_to_cartesian(self, x: float, y: float, z: float) -> bool:
        self.set_start_to_current_via_planner()
        # Point the flange straight down (180deg rotation about X from
        # identity) instead of the default identity orientation — this
        # cell only does top-down pick/inspect/bin moves, and identity
        # orientation is often physically unreachable at real (non-origin)
        # XY positions, causing GOAL_STATE_INVALID from OMPL's IK sampler.
        result = self._planner.plan_to_pose(
            x=x, y=y, z=z, frame_id="panda_link0",
            qx=1.0, qy=0.0, qz=0.0, qw=0.0,
        )
        return self._planner.execute(result)

    def _run_inspection_cycle(self):
        """Full pick -> inspect -> decide -> sort cycle.

        NOTE: 'pick' here means 'hover above the object' — there is no
        gripper in this MoveIt config, so no physical grasp occurs.
        The PASS/FAIL decision is a placeholder rule (inspection_logic.py),
        not real defect inspection.
        """
        self.get_logger().info("=== Starting inspection cycle ===")

        self.get_logger().info("Step 1/4: Moving to detected object (PICK)")
        if not self._move_to_detected_pick_pose_bool():
            self.get_logger().error("Cycle aborted: failed to reach PICK pose")
            return

        self.get_logger().info("Step 2/4: Moving to INSPECTION pose")
        inspection_pose = pose_library.get_pose("INSPECTION")
        ix, iy, iz = inspection_pose["value"]
        if not self._move_to_cartesian(ix, iy, iz):
            self.get_logger().error("Cycle aborted: failed to reach INSPECTION pose")
            return

        self.get_logger().info("Step 3/4: GOOD PCB confirmed, routing to GOOD_BIN")

        bin_name = "PASS_BIN"
        self.get_logger().info(f"Step 4/4: Moving to {bin_name}")
        bin_pose = pose_library.get_pose(bin_name)
        bx, by, bz = bin_pose["value"]
        if not self._move_to_cartesian(bx, by, bz):
            self.get_logger().error(f"Cycle aborted: failed to reach {bin_name}")
            return

        self.get_logger().info(f"=== Cycle complete: PCB sorted to {bin_name} ===")

    def _move_to_detected_pick_pose_bool(self) -> bool:
        """Same as _move_to_detected_pick_pose but returns success/failure
        instead of only logging, so _run_inspection_cycle can chain on it."""
        if self._latest_pick_pose is None:
            self.get_logger().error("No pick pose received yet from pose_estimator_node")
            return False
        pose = self._latest_pick_pose.pose
        hover_z = pose.position.z + 0.15
        return self._move_to_cartesian(pose.position.x, pose.position.y, hover_z)

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
