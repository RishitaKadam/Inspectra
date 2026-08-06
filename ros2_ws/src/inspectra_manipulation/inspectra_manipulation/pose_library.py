"""
Named pose definitions for the Inspectra Panda arm.

Two kinds of poses are supported:
  1. MoveIt "predefined configurations" (SRDF group states), referenced
     by name only, e.g. "ready", "extended" — these come from the
     panda_arm.srdf shipped with moveit_resources_panda_moveit_config.
  2. Explicit joint-value dictionaries, for poses we define ourselves
     (used once inspectra_description ships its own SRDF).

PRE_PICK / PICK / INSPECTION / PASS_BIN / FAIL_BIN are intentionally
left unresolved until inspectra_perception + inspectra_description
exist, so a bad call fails loudly instead of moving to a wrong pose.
"""

from enum import Enum


class PoseType(Enum):
    NAMED_CONFIG = "named_config"
    JOINT_VALUES = "joint_values"
    CARTESIAN = "cartesian"
    UNRESOLVED = "unresolved"


# Poses available out of the box from moveit_resources_panda_moveit_config's SRDF
HOME = {"type": PoseType.NAMED_CONFIG, "value": "ready"}
READY = {"type": PoseType.NAMED_CONFIG, "value": "ready"}
EXTENDED = {"type": PoseType.NAMED_CONFIG, "value": "extended"}

# PICK is resolved dynamically at runtime from live perception
# (pose_estimator_node), handled as a special case in motion_planner_node
# rather than a static entry here.
PICK = {"type": PoseType.UNRESOLVED, "value": None}

# Fixed Cartesian waypoints (x, y, z in panda_link0), same mechanism as
# PICK's hover pose (MotionPlanner.plan_to_pose). Values chosen to be
# reachable/collision-free relative to the inspection_table at (0.5, 0, 0);
# not derived from any real bin/fixture geometry yet.
PRE_PICK = {"type": PoseType.CARTESIAN, "value": (0.4, -0.3, 0.25)}
INSPECTION = {"type": PoseType.CARTESIAN, "value": (0.4, -0.3, 0.35)}
# Bin coordinates match scene_manager.add_bins()'s visible box positions
# exactly (x=0.6 forward, matching the pick/inspection reach depth), plus
# a small z offset above the bin's top surface for a clean drop-in motion.
PASS_BIN = {"type": PoseType.CARTESIAN, "value": (0.3, 0.5, 0.25)}
FAIL_BIN = {"type": PoseType.CARTESIAN, "value": (0.3, -0.5, 0.25)}

POSE_LIBRARY = {
    "HOME": HOME,
    "READY": READY,
    "EXTENDED": EXTENDED,
    "PRE_PICK": PRE_PICK,
    "PICK": PICK,
    "INSPECTION": INSPECTION,
    "PASS_BIN": PASS_BIN,
    "FAIL_BIN": FAIL_BIN,
}


def get_pose(name: str) -> dict:
    """Look up a named pose. Raises KeyError for unknown names and
    NotImplementedError for poses not yet wired up (scene-dependent)."""
    if name not in POSE_LIBRARY:
        raise KeyError(f"Unknown pose '{name}'. Available: {list(POSE_LIBRARY)}")

    pose = POSE_LIBRARY[name]
    if pose["type"] == PoseType.UNRESOLVED:
        raise NotImplementedError(
            f"Pose '{name}' requires inspectra_perception / scene_manager, "
            "which don't exist yet. Use HOME, READY, or EXTENDED for now."
        )
    return pose
