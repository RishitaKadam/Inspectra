"""
Common helper utilities for the inspectra_manipulation package.

Provides a consistent logger setup used across planner.py, executor.py,
scene_manager.py and motion_planner.py so log output is uniform and
easy to filter (e.g. `ros2 launch ... | grep inspectra`).
"""

from rclpy.logging import get_logger, LoggingSeverity


def get_inspectra_logger(name: str = "inspectra_manipulation"):
    """Return a configured rclpy logger.

    Args:
        name: Logical component name, e.g. 'planner', 'executor'.
              Will be prefixed so logs are easy to grep.

    Returns:
        rclpy.impl.rcutils_logger.RcutilsLogger
    """
    logger = get_logger(f"inspectra.{name}")
    logger.set_level(LoggingSeverity.INFO)
    return logger


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a value into [low, high]. Small shared helper for future
    joint-limit / velocity-scaling checks."""
    return max(low, min(value, high))
