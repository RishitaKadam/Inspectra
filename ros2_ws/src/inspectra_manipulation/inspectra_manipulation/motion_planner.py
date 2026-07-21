import rclpy
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient


class MotionPlanner(Node):

    def __init__(self):
        super().__init__('inspectra_motion_planner')

        self.client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info(
            "Inspectra Motion Planner connected"
        )


def main(args=None):
    rclpy.init(args=args)

    node = MotionPlanner()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
