"""
SceneManager: adds/removes collision objects in the MoveIt planning scene.

Based directly on the official MoveIt2 planning-scene tutorial
(moveit/moveit2_tutorials: motion_planning_python_api_planning_scene.py),
using planning_scene_monitor.read_write() + apply_collision_object().

Responsibilities (SRP):
  - Own collision object add/remove logic
Does NOT:
  - Know about motion planning goals (that's planner.py)
  - Know about named poses (that's pose_library.py)
"""

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from inspectra_manipulation.utils import get_inspectra_logger


class SceneManager:
    def __init__(self, moveit_py_instance):
        """
        Args:
            moveit_py_instance: the MoveItPy instance owned by a MotionPlanner
                (pass planner.moveit_py, not a new MoveItPy()).
        """
        self._logger = get_inspectra_logger("scene_manager")
        self._moveit = moveit_py_instance
        self._psm = self._moveit.get_planning_scene_monitor()

    def add_box(self, name: str, dimensions, position, frame_id: str = "panda_link0",
                orientation=(0.0, 0.0, 0.0, 1.0)):
        """Add a box collision object.

        Args:
            name: unique collision object id
            dimensions: (size_x, size_y, size_z) in meters
            position: (x, y, z) in meters, relative to frame_id
            frame_id: reference frame, defaults to the robot base
            orientation: (x, y, z, w) quaternion, defaults to identity
        """
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
            scene.current_state.update()  # required to refresh collision state

        self._logger.info(f"Added collision object '{name}' at {position}")

    def remove_object(self, name: str):
        """Remove a single collision object by id."""
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE

        with self._psm.read_write() as scene:
            scene.apply_collision_object(collision_object)
            scene.current_state.update()

        self._logger.info(f"Removed collision object '{name}'")

    def clear_all(self):
        """Remove every collision object from the scene."""
        with self._psm.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()
        self._logger.info("Cleared all collision objects")

    # ------------------------------------------------------------------
    # Inspectra-specific convenience objects
    # ------------------------------------------------------------------
    def add_inspection_table(self):
        """Add the Inspectra inspection table in front of the robot base."""
        self.add_box(
            name="inspection_table",
            dimensions=(0.6, 1.2, 0.05),   # x, y, z (meters)
            position=(0.5, 0.0, -0.025),   # top surface sits at z=0
        )
