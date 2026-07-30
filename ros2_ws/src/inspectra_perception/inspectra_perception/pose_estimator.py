"""
PoseEstimatorNode: converts 2D image coordinates into a 3D pick pose by
casting a ray through the camera model and intersecting it with the
inspection table plane.

GOOD boards have no detections, so we publish the board centre every
camera frame. Defect detections are only used for logging.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException

from inspectra_manipulation.utils import get_inspectra_logger


def _rotate_vector_by_quaternion(v, q):
    qx, qy, qz, qw = q
    vx, vy, vz = v

    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx

    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx

    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


class PoseEstimatorNode(Node):

    def __init__(self):
        super().__init__("pose_estimator_node")

        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "panda_link0")
        self.declare_parameter("table_plane_z", 0.0)
        self.declare_parameter("assumed_hfov_deg", 60.0)

        self._camera_frame = self.get_parameter(
            "camera_frame").get_parameter_value().string_value

        self._target_frame = self.get_parameter(
            "target_frame").get_parameter_value().string_value

        self._table_z = self.get_parameter(
            "table_plane_z").get_parameter_value().double_value

        self._hfov_deg = self.get_parameter(
            "assumed_hfov_deg").get_parameter_value().double_value

        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None

        self._logger = get_inspectra_logger("pose_estimator")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer,
            self
        )

        self._pick_pose_pub = self.create_publisher(
            PoseStamped,
            "~/pick_pose",
            10,
        )

        self._image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self._on_image,
            10,
        )

        self._detections_sub = self.create_subscription(
            Detection2DArray,
            "/object_detector_node/detections",
            self._on_detections,
            10,
        )

        self.get_logger().info(
            f"PoseEstimatorNode ready. "
            f"camera_frame='{self._camera_frame}', "
            f"target_frame='{self._target_frame}', "
            f"table_plane_z={self._table_z}"
        )

    def _on_image(self, msg: Image):

        width = msg.width
        height = msg.height

        self._cx = width / 2.0
        self._cy = height / 2.0

        hfov = math.radians(self._hfov_deg)

        self._fx = (width / 2.0) / math.tan(hfov / 2.0)
        self._fy = self._fx

        self._publish_pose_for_pixel(
            self._cx,
            self._cy,
            "board center",
        )

    def _on_detections(self, msg: Detection2DArray):

        if not msg.detections:
            return

        best = max(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score,
        )

        cls = best.results[0].hypothesis.class_id
        score = best.results[0].hypothesis.score

        self.get_logger().info(
            f"Detected '{cls}' (conf={score:.2f})"
        )

    def _pixel_to_ray(self, u, v):

        x = (u - self._cx) / self._fx
        y = (v - self._cy) / self._fy
        z = 1.0

        norm = math.sqrt(x * x + y * y + z * z)

        return (
            x / norm,
            y / norm,
            z / norm,
        )

    def _publish_pose_for_pixel(self, u, v, label=""):

        if self._fx is None:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                self._camera_frame,
                Time(),
            )

        except TransformException as e:
            self.get_logger().warning(
                f"TF lookup failed: {e}"
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        camera_origin = (
            t.x,
            t.y,
            t.z,
        )

        camera_quat = (
            q.x,
            q.y,
            q.z,
            q.w,
        )

        ray = self._pixel_to_ray(u, v)

        ray_world = _rotate_vector_by_quaternion(
            ray,
            camera_quat,
        )

        if abs(ray_world[2]) < 1e-6:
            return

        s = (self._table_z - camera_origin[2]) / ray_world[2]

        if s <= 0:
            return

        px = camera_origin[0] + s * ray_world[0]
        py = camera_origin[1] + s * ray_world[1]
        pz = self._table_z

        pose = PoseStamped()

        pose.header.frame_id = self._target_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz

        pose.pose.orientation.w = 1.0

        self._pick_pose_pub.publish(pose)

        self.get_logger().info(
            f"Estimated pose ({label}) "
            f"x={px:.3f} "
            f"y={py:.3f} "
            f"z={pz:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = PoseEstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
