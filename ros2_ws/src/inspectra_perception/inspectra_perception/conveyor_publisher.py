"""
ConveyorPublisher: cycles through all images in media/conveyor_feed/,
publishing each on /camera/image_raw for a configurable dwell time,
simulating PCBs arriving one after another on a conveyor belt. Stands
in for a real/Gazebo camera until physical conveyor integration exists.
"""

import os
import glob
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


class ConveyorPublisher(Node):
    def __init__(self):
        super().__init__("conveyor_publisher")

        self.declare_parameter("dwell_time_sec", 4.0)
        self.declare_parameter("loop", True)

        dwell_time = self.get_parameter("dwell_time_sec").get_parameter_value().double_value
        self._loop = self.get_parameter("loop").get_parameter_value().bool_value

        media_dir = os.path.join(
            get_package_share_directory("inspectra_perception"),
            "media", "conveyor_feed",
        )
        self._image_paths = sorted(glob.glob(os.path.join(media_dir, "*.jpg")))
        if not self._image_paths:
            raise FileNotFoundError(f"No images found in {media_dir}")

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self._index = 0

        self.get_logger().info(
            f"ConveyorPublisher ready: {len(self._image_paths)} images, "
            f"{dwell_time}s dwell time, loop={self._loop}"
        )

        self._timer = self.create_timer(dwell_time, self._publish_next)
        self._publish_next()  # publish first image immediately, don't wait for first timer tick

    def _publish_next(self):
        if self._index >= len(self._image_paths):
            if self._loop:
                self._index = 0
            else:
                self.get_logger().info("Conveyor feed complete (loop=False), stopping.")
                self._timer.cancel()
                return

        path = self._image_paths[self._index]
        cv_image = cv2.imread(path)
        if cv_image is None:
            self.get_logger().error(f"Failed to read {path}, skipping")
            self._index += 1
            return

        msg = self._bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self._pub.publish(msg)

        self.get_logger().info(
            f"[{self._index + 1}/{len(self._image_paths)}] Now on belt: {os.path.basename(path)}"
        )
        self._index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
