import rclpy
from rclpy.node import Node


class InspectraController(Node):

    def __init__(self):
        super().__init__("inspectra_controller")

        self.get_logger().info("=" * 40)
        self.get_logger().info("Inspectra Controller Started")
        self.get_logger().info("System Initializing...")
        self.get_logger().info("=" * 40)


def main(args=None):
    rclpy.init(args=args)

    node = InspectraController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
