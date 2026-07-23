"""
TrajectoryExecutor: retry logic and multi-pose sequencing on top of
MotionPlanner. Does not talk to MoveItPy directly — everything goes
through the MotionPlanner instance it's given, so planning/execution
logic itself stays in planner.py (single responsibility).
"""

from inspectra_manipulation.utils import get_inspectra_logger


class TrajectoryExecutor:
    def __init__(self, motion_planner, max_retries: int = 1):
        """
        Args:
            motion_planner: an initialized MotionPlanner instance.
            max_retries: number of EXTRA attempts after the first failure
                (max_retries=1 means try once, retry once more on failure = 2 attempts total).
        """
        self._planner = motion_planner
        self._max_retries = max_retries
        self._logger = get_inspectra_logger("executor")

    def execute_named_pose(self, config_name: str) -> bool:
        """Plan+execute a single named config, retrying on failure."""
        total_attempts = self._max_retries + 1

        for attempt in range(1, total_attempts + 1):
            self._logger.info(
                f"Attempt {attempt}/{total_attempts}: executing pose '{config_name}'"
            )
            success = self._planner.plan_and_execute(config_name=config_name)
            if success:
                if attempt > 1:
                    self._logger.info(f"Pose '{config_name}' succeeded on retry {attempt}")
                return True
            self._logger.warning(f"Attempt {attempt} failed for pose '{config_name}'")

        self._logger.error(
            f"Pose '{config_name}' failed after {total_attempts} attempt(s)"
        )
        return False

    def execute_sequence(self, config_names: list, stop_on_failure: bool = True) -> dict:
        """Execute a list of named configs in order.

        Args:
            config_names: e.g. ["ready", "extended", "ready"]
            stop_on_failure: if True, abort the remaining sequence the
                first time a pose fails; if False, attempt all poses
                regardless and report per-pose results.

        Returns:
            dict mapping each pose name -> bool success. If stopped early,
            remaining poses are absent from the dict (not marked False),
            so callers can distinguish "failed" from "never attempted".
        """
        results = {}
        for name in config_names:
            success = self.execute_named_pose(name)
            results[name] = success
            if not success and stop_on_failure:
                self._logger.error(
                    f"Sequence aborted at '{name}' "
                    f"({len(results)}/{len(config_names)} poses attempted)"
                )
                break

        succeeded = sum(1 for v in results.values() if v)
        self._logger.info(
            f"Sequence complete: {succeeded}/{len(results)} attempted poses succeeded"
        )
        return results
