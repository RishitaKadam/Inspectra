"""
ObjectDetectorNode: subscribes to a camera image topic, runs YOLOv8
(via the official ultralytics Python API) on each frame, and publishes
detections as vision_msgs/Detection2DArray plus a debug annotated image.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge

from ultralytics import YOLO


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.5)
        self.declare_parameter("input_image_topic", "/camera/image_raw")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self._conf_threshold = (
            self.get_parameter("confidence_threshold").get_parameter_value().double_value
        )
        input_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value

        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(model_path)
        self.get_logger().info("Model loaded successfully")

        self._bridge = CvBridge()

        self._image_sub = self.create_subscription(
            Image, input_topic, self._on_image, 10
        )
        self._detections_pub = self.create_publisher(
            Detection2DArray, "~/detections", 10
        )
        self._debug_image_pub = self.create_publisher(
            Image, "~/detection_image", 10
        )

        self.get_logger().info(
            f"ObjectDetectorNode ready. Listening on '{input_topic}', "
            f"publishing to '~/detections' and '~/detection_image'."
        )

    def _on_image(self, msg: Image):
        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self._model(cv_image, conf=self._conf_threshold, verbose=False)
        result = results[0]

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        for box in result.boxes:
            xyxy = box.xyxy[0].tolist()
            x1, y1, x2, y2 = xyxy
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            width, height = x2 - x1, y2 - y1

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = self._model.names[class_id]

            detection = Detection2D()
            detection.bbox.center.position.x = cx
            detection.bbox.center.position.y = cy
            detection.bbox.size_x = width
            detection.bbox.size_y = height

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        self._detections_pub.publish(detection_array)

        annotated_frame = result.plot()
        debug_msg = self._bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        debug_msg.header = msg.header
        self._debug_image_pub.publish(debug_msg)

        if len(detection_array.detections) > 0:
            names = [d.results[0].hypothesis.class_id for d in detection_array.detections]
            self.get_logger().info(f"Detected {len(names)} object(s): {names}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
