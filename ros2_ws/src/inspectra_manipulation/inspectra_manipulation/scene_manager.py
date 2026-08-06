"""
SceneManager: adds/removes/colors collision objects in the MoveIt planning scene.
"""

from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene, ObjectColor
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA

from inspectra_manipulation.utils import get_inspectra_logger


class SceneManager:
    def __init__(self, moveit_py_instance, ros_node=None):
        """
        Args:
            moveit_py_instance: planner.moveit_py
            ros_node: the owning rclpy Node (pass motion_planner_node's
                'self'), used only to create a real publisher for object
                colors. Optional -- color calls are silently skipped if
                not provided.
        """
        self._logger = get_inspectra_logger("scene_manager")
        self._moveit = moveit_py_instance
        self._psm = self._moveit.get_planning_scene_monitor()
        self._color_pub = None
        if ros_node is not None:
            self._color_pub = ros_node.create_publisher(PlanningScene, "/planning_scene", 10)

    def add_box(self, name: str, dimensions, position, frame_id: str = "panda_link0",
                orientation=(0.0, 0.0, 0.0, 1.0)):
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(dimensions)

        box_pose = Pose()
        box_pose.position.x, box_pose.position.y, box_pose.position.z = position
        (box_pose.orientation.x, box_pose.orientation.y,
         box_pose.orientation.z, box_pose.orientation.w) = orientation

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        with self._psm.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        self._logger.info(f"Added collision object '{name}' at {position}")

    def remove_object(self, name: str):
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE

        with self._psm.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        self._logger.info(f"Removed collision object '{name}'")

    def clear_all(self):
        with self._psm.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()
        self._logger.info("Cleared all collision objects")

    def attach_object(self, name: str, link_name: str = "panda_link8"):
        attached = AttachedCollisionObject()
        attached.object.id = name
        attached.link_name = link_name
        attached.object.operation = attached.object.ADD

        with self._psm.read_write() as scene:
            scene.process_attached_collision_object(attached)
            scene.current_state.update()

        self._logger.info(f"Attached '{name}' to '{link_name}'")

    def detach_object(self, name: str, link_name: str = "panda_link8"):
        detached = AttachedCollisionObject()
        detached.object.id = name
        detached.link_name = link_name
        detached.object.operation = detached.object.REMOVE

        with self._psm.read_write() as scene:
            scene.process_attached_collision_object(detached)
            scene.current_state.update()

        self._logger.info(f"Detached '{name}' from '{link_name}'")

    def set_object_color(self, name: str, r: float, g: float, b: float, a: float = 1.0):
        """Set a collision object's RViz display color."""
        if self._color_pub is None:
            self._logger.warning(f"No color publisher configured; skipping color for '{name}'")
            return
        msg = PlanningScene()
        msg.is_diff = True
        oc = ObjectColor()
        oc.id = name
        oc.color = ColorRGBA(r=r, g=g, b=b, a=a)
        msg.object_colors = [oc]
        self._color_pub.publish(msg)
        self._logger.info(f"Set color of '{name}' to ({r},{g},{b})")

    def add_pcb_object(self, x: float, y: float, z: float = 0.0, name: str = "detected_pcb"):
        self.add_box(
            name=name,
            dimensions=(0.08, 0.08, 0.01),
            position=(x, y, z + 0.005),
        )

    def add_bins(self):
        """Add visible PASS/FAIL bin objects, positioned forward (same
        reach depth as the table/pick area) so the robot's sorting motion
        actually ends at the bin instead of stopping short of it."""
        # Bins moved outside the table footprint (table depth is y=[-0.6,0.6],
        # so y=+-0.7 sits just past the edge). Box height 0.2, centered at
        # z=0.10, so its TOP surface is at z=0.20 -- matching exactly where
        # the arm's flange (and attached object) actually ends up when
        # sorting, per pose_library.PASS_BIN/FAIL_BIN's z=0.20 value.
        self.add_box(name="pass_bin", dimensions=(0.15, 0.15, 0.2), position=(0.4, 0.3, 0.10))
        self.set_object_color("pass_bin", 0.0, 0.6, 0.0)

        self.add_box(name="fail_bin", dimensions=(0.15, 0.15, 0.2), position=(0.4, -0.3, 0.10))
        self.set_object_color("fail_bin", 0.6, 0.0, 0.0)

    def add_inspection_table(self):
        """Table (tabletop + 4 legs), colored brown. Top surface stays at
        z=0 to match pose_estimator's table_plane_z assumption."""
        table_x, table_y = 0.5, 0.0
        width, depth, thickness = 0.6, 1.2, 0.05

        self.add_box(
            name="inspection_table_top",
            dimensions=(width, depth, thickness),
            position=(table_x, table_y, -thickness / 2.0),
        )
        self.set_object_color("inspection_table_top", 0.35, 0.20, 0.10)

        leg_height = 0.4
        leg_size = 0.05
        leg_z = -thickness - (leg_height / 2.0)
        x_offset = (width / 2.0) - (leg_size / 2.0)
        y_offset = (depth / 2.0) - (leg_size / 2.0)

        for leg_name, (lx, ly) in {
            "inspection_table_leg_1": (table_x + x_offset, table_y + y_offset),
            "inspection_table_leg_2": (table_x - x_offset, table_y + y_offset),
            "inspection_table_leg_3": (table_x + x_offset, table_y - y_offset),
            "inspection_table_leg_4": (table_x - x_offset, table_y - y_offset),
        }.items():
            self.add_box(
                name=leg_name,
                dimensions=(leg_size, leg_size, leg_height),
                position=(lx, ly, leg_z),
            )
            self.set_object_color(leg_name, 0.15, 0.09, 0.05)
