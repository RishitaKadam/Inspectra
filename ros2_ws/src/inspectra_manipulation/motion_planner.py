
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class MotionPlanner(Node):

    def __init__(self):
        super().__init__("motion_planner")
        self.get_logger().info("Inspectra Motion Planner Started")


def main(args=None):
    rclpy.init(args=args)

    node = MotionPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
