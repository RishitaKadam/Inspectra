"""
ROS2 node wrapping MotionPlanner. Reacts to live PCB detections: GOOD
boards turn green and sort to PASS_BIN, BAD boards turn red and sort
to FAIL_BIN.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
import time

from inspectra_manipulation.planner import MotionPlanner
from inspectra_manipulation.scene_manager import SceneManager
from inspectra_manipulation.executor import TrajectoryExecutor
from inspectra_manipulation import inspection_logic
from inspectra_manipulation import pose_library


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner_node")

        self.declare_parameter("startup_pose", "")
        startup_pose = self.get_parameter("startup_pose").get_parameter_value().string_value

        self.get_logger().info("Starting MotionPlannerNode, constructing MotionPlanner...")
        self._planner = MotionPlanner(node_name="inspectra_moveit_py")

        self._scene = SceneManager(self._planner.moveit_py, ros_node=self)
        
        time.sleep(2.0) 
        self._scene.add_inspection_table()

        self._executor = TrajectoryExecutor(self._planner, max_retries=1)

        self._sub = self.create_subscription(String, "~/move_to_pose", self._on_move_to_pose, 10)
        self._seq_sub = self.create_subscription(String, "~/move_sequence", self._on_move_sequence, 10)

        self._latest_pick_pose = None
        self._pick_pose_sub = self.create_subscription(
            PoseStamped, "/pose_estimator_node/pick_pose", self._on_pick_pose, 10
        )

        self._cycle_in_progress = False
        self._detections_sub = self.create_subscription(
            Detection2DArray, "/object_detector_node/detections", self._on_detections, 10
        )

        self._run_cycle_sub = self.create_subscription(String, "~/run_cycle", self._on_run_cycle, 10)

        self.get_logger().info("MotionPlannerNode ready.")

        if startup_pose:
            self._move_to_named_pose(startup_pose)

    def _on_move_to_pose(self, msg: String):
        self._move_to_named_pose(msg.data)

    def _on_pick_pose(self, msg: PoseStamped):
        # Clamp to a safe, guaranteed-reachable zone (well within the
        # ~0.85m max reach), since pose_estimator's per-frame dynamic
        # intrinsics can produce wildly different values across the many
        # differently-sized real photos in the conveyor feed.
        msg.pose.position.x = max(0.3, min(0.55, msg.pose.position.x))
        msg.pose.position.y = max(-0.3, min(0.3, msg.pose.position.y))
        self._latest_pick_pose = msg

    def _on_detections(self, msg: Detection2DArray):
        detections = [
            (d.results[0].hypothesis.class_id, d.results[0].hypothesis.score)
            for d in msg.detections
        ]

        if self._cycle_in_progress:
            return

        classification = inspection_logic.classify_pcb(detections)
        self._cycle_in_progress = True
        self._run_inspection_cycle(classification)
        
        self.get_logger().info("Waiting for the next PCB to arrive on the conveyor...")
        time.sleep(4.0) 
        
        self._cycle_in_progress = False

    def _on_run_cycle(self, msg: String):
        self._run_inspection_cycle("GOOD")

    def _cleanup_detected_pcb(self):
        try:
            self._scene.detach_object("detected_pcb")
        except Exception:
            pass
        try:
            self._scene.remove_object("detected_pcb")
        except Exception:
            pass

    def _move_to_named_pose(self, name: str):
        if name.upper() == "PICK":
            self._move_to_cartesian(0.55, 0.0, 0.25)
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
            success = self._executor.execute_named_pose(pose["value"])
        if success:
            self.get_logger().info(f"Reached pose '{name}'")
        else:
            self.get_logger().error(f"Failed to reach pose '{name}'")

    def _on_move_sequence(self, msg: String):
        names = [n.strip() for n in msg.data.split(",") if n.strip()]
        if not names:
            return
        config_names = []
        for name in names:
            try:
                pose = pose_library.get_pose(name)
            except (KeyError, NotImplementedError) as e:
                self.get_logger().error(f"Aborting sequence: {e}")
                return
            config_names.append(pose["value"])
        results = self._executor.execute_sequence(config_names, stop_on_failure=True)
        self.get_logger().info(f"Sequence results: {results}")

    def set_start_to_current_via_planner(self):
        self._planner.set_start_to_current()

    def _move_to_cartesian(self, x: float, y: float, z: float) -> bool:
        self.set_start_to_current_via_planner()
        result = self._planner.plan_to_pose(
            x=x, y=y, z=z, frame_id="panda_link0",
            qx=0.9239, qy=-0.3827, qz=0.0, qw=0.0,
        )
        return self._planner.execute(result)

    def _run_inspection_cycle(self, classification: str):
        self.get_logger().info(f"=== Starting cycle: {classification} PCB detected ===")
        self._cleanup_detected_pcb()

        safe_px = 0.55
        safe_py = 0.0
        safe_pz = 0.0
        hover_z = safe_pz + 0.25

        self.get_logger().info("Step 1/3: Moving to Center (PICK)")
        if not self._move_to_cartesian(safe_px, safe_py, hover_z):
            self.get_logger().error("Cycle aborted: failed to reach PICK pose")
            return

        pcb_visual_z = hover_z - 0.15
        
        self._scene.add_pcb_object(safe_px, safe_py, pcb_visual_z, name="detected_pcb")
        self._scene.attach_object("detected_pcb")

        # FIX: Brought Y into 0.50. This perfectly clears the table without breaking the robot's joints.
        if classification == "GOOD":
            self._scene.set_object_color("detected_pcb", 0.0, 1.0, 0.0)  # green
            bin_name = "PASS_BIN"
            bx, by, bz = 0.45, 0.50, 0.25  # Left edge
        else:
            self._scene.set_object_color("detected_pcb", 1.0, 0.0, 0.0)  # red
            bin_name = "FAIL_BIN"
            bx, by, bz = 0.45, -0.50, 0.25 # Right edge

        self.get_logger().info(f"Step 2/3: Sorting straight to {bin_name}")
        if not self._move_to_cartesian(bx, by, bz):
            self.get_logger().error(f"Cycle aborted: failed to reach {bin_name}")
            self._cleanup_detected_pcb()
            return

        # FIX: Use the native cleanup function to delete the block safely without freezing the ROS thread
        self._cleanup_detected_pcb()
        
        self.get_logger().info("Step 3/3: Resetting arm to center for next board")
        self._move_to_cartesian(safe_px, safe_py, hover_z)

        self.get_logger().info(f"=== Cycle complete: PCB sorted to {bin_name} ===")

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
