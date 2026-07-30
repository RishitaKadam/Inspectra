"""
ConveyorMotionNode: continuously moves the 3 pcb_1/pcb_2/pcb_3 Gazebo
models along the visual conveyor belt (y: -1.0 -> 0.0, looping),
using ros_gz_interfaces/srv/SetEntityPose.

NOTE (honest limitation): this drives VISUAL motion only. The robot's
actual GOOD/BAD decisions come from the separate, proven image-based
detection pipeline (object_detector_node fed real PCB photos), not from
Gazebo's rendered camera view of these boxes -- the trained model was
never designed to detect defects on a synthetic 3D render. This node
exists purely to make the simulation look like a real industrial
conveyor while the actual detection logic runs independently.
"""

import math
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class ConveyorMotionNode(Node):
    def __init__(self):
        super().__init__("conveyor_motion_node")

        self.declare_parameter("world_name", "inspectra_world")
        self.declare_parameter("belt_speed", 0.05)  # m/s
        self.declare_parameter("belt_start_y", -1.0)
        self.declare_parameter("belt_end_y", 0.0)

        world_name = self.get_parameter("world_name").get_parameter_value().string_value
        self._speed = self.get_parameter("belt_speed").get_parameter_value().double_value
        self._start_y = self.get_parameter("belt_start_y").get_parameter_value().double_value
        self._end_y = self.get_parameter("belt_end_y").get_parameter_value().double_value

        self._client = self.create_client(SetEntityPose, f"/world/{world_name}/set_pose")
        self.get_logger().info(f"Waiting for /world/{world_name}/set_pose service...")
        self._client.wait_for_service()
        self.get_logger().info("Service available, starting conveyor motion.")

        # Stagger 3 PCBs evenly along the belt, matching their SDF spawn spacing
        span = self._end_y - self._start_y
        self._positions = {
            "pcb_1": self._start_y + span * 0.1,
            "pcb_2": self._start_y + span * 0.4,
            "pcb_3": self._start_y + span * 0.7,
        }

        self._timer = self.create_timer(0.1, self._tick)  # 10Hz update

    def _tick(self):
        dt = 0.1
        for name in self._positions:
            self._positions[name] += self._speed * dt
            if self._positions[name] > self._end_y:
                self._positions[name] = self._start_y  # loop back to start

            self._send_pose(name, 0.5, self._positions[name], 0.012)

    def _send_pose(self, name: str, x: float, y: float, z: float):
        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = name
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.w = 1.0
        self._client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
