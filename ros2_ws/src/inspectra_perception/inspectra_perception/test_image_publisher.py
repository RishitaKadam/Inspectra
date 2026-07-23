"""
TestImagePublisher: publishes a static test image on a loop to simulate
a camera feed, until a real Gazebo/hardware camera is wired up.
"""

import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class TestImagePublisher(Node):
    def __init__(self):
        super().__init__("test_image_publisher")

        self.declare_parameter("image_path", "")
        self.declare_parameter("publish_rate_hz", 2.0)

        image_path = self.get_parameter("image_path").get_parameter_value().string_value
        if not image_path:
            image_path = os.path.join(
                get_package_share_directory("inspectra_perception"),
                "media",
                "test_scene.jpg",
            )

        if not os.path.isfile(image_path):
            raise FileNotFoundError(
                f"Test image not found at {image_path}. "
                "Add a .jpg to inspectra_perception/media/ named test_scene.jpg, "
                "or pass image_path as a parameter."
            )

        self._cv_image = cv2.imread(image_path)
        if self._cv_image is None:
            raise RuntimeError(f"OpenCV failed to read image at {image_path} (corrupt file?)")

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, "/camera/image_raw", 10)

        rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_frame)

        self.get_logger().info(f"Publishing '{image_path}' to /camera/image_raw at {rate_hz} Hz")

    def _publish_frame(self):
        msg = self._bridge.cv2_to_imgmsg(self._cv_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
