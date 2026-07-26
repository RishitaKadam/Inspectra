"""
PoseEstimatorNode: converts 2D YOLOv8 detections into 3D poses in
panda_link0, by casting a ray through the camera's pinhole model and
intersecting it with the known inspection table plane (z=0 in
panda_link0 — see scene_manager.add_inspection_table()).

LIMITATION (by design, not a bug): this assumes detected objects are
flat/thin and resting on the table surface. It cannot recover true
height/depth without a depth camera or multi-view geometry. Fine for
Inspectra's flat-part inspection use case; would need revisiting for
tall objects.

Camera intrinsics below are PLACEHOLDER values (a common Gazebo default
camera profile), since no real/simulated camera exists yet. Replace via
parameters once a real camera is calibrated.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

import tf2_ros
from tf2_ros import TransformException

from inspectra_manipulation.utils import get_inspectra_logger


def _rotate_vector_by_quaternion(v, q):
    """Rotate 3D vector v=(x,y,z) by quaternion q=(x,y,z,w). Pure math,
    no extra tf_transformations/scipy dependency."""
    qx, qy, qz, qw = q
    vx, vy, vz = v

    # v' = q * v * q_conjugate, expanded directly (standard formula)
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx

    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx

    rx = vx + 2.0 * (qw * uvx + uuvx)
    ry = vy + 2.0 * (qw * uvy + uuvy)
    rz = vz + 2.0 * (qw * uvz + uuvz)
    return (rx, ry, rz)


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__("pose_estimator_node")

        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "panda_link0")
        self.declare_parameter("table_plane_z", 0.0)
        self.declare_parameter("assumed_hfov_deg", 60.0)
        # fx/fy/cx/cy are now derived per-frame from the actual image
        # dimensions (see _on_image) instead of fixed here, since the
        # conveyor feed mixes very different image resolutions (real
        # PCB defect photos vs separately-sourced GOOD photos).

        self._camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self._target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self._table_z = self.get_parameter("table_plane_z").get_parameter_value().double_value
        self._hfov_deg = self.get_parameter("assumed_hfov_deg").get_parameter_value().double_value
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
        self._image_sub = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, 10
        )

        self._logger = get_inspectra_logger("pose_estimator")

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._detections_sub = self.create_subscription(
            Detection2DArray, "/object_detector_node/detections", self._on_detections, 10
        )

    def _on_image(self, msg: Image):
        """Derive camera intrinsics from the actual image size, since the
        conveyor feed mixes different resolutions. Assumes a horizontal
        FOV of assumed_hfov_deg (rough placeholder, not a real calibration)."""
        import math
        width, height = msg.width, msg.height
        self._cx = width / 2.0
        self._cy = height / 2.0
        hfov_rad = math.radians(self._hfov_deg)
        self._fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        self._fy = self._fx  # assume square pixels
        self._pick_pose_pub = self.create_publisher(PoseStamped, "~/pick_pose", 10)

        self.get_logger().info(
            f"PoseEstimatorNode ready. camera_frame='{self._camera_frame}', "
            f"target_frame='{self._target_frame}', table_plane_z={self._table_z}"
        )

    def _pixel_to_ray(self, u: float, v: float):
        """Pinhole camera model: pixel -> normalized ray direction in the
        camera's own optical frame (Z forward)."""
        x = (u - self._cx) / self._fx
        y = (v - self._cy) / self._fy
        z = 1.0
        norm = math.sqrt(x * x + y * y + z * z)
        return (x / norm, y / norm, z / norm)

    def _on_detections(self, msg: Detection2DArray):
        if not msg.detections:
            return
        if self._fx is None:
            self.get_logger().warning("No image received yet, cannot compute intrinsics")
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, self._camera_frame, Time()
            )
        except TransformException as e:
            self.get_logger().warning(f"TF lookup failed ({e}); skipping this frame")
            return

        t = transform.transform.translation
        q = transform.transform.rotation
        camera_origin = (t.x, t.y, t.z)
        camera_quat = (q.x, q.y, q.z, q.w)

        # Pick the highest-confidence detection as the pick target
        best_detection = max(
            msg.detections, key=lambda d: d.results[0].hypothesis.score
        )
        u = best_detection.bbox.center.position.x
        v = best_detection.bbox.center.position.y
        class_name = best_detection.results[0].hypothesis.class_id
        confidence = best_detection.results[0].hypothesis.score

        ray_camera = self._pixel_to_ray(u, v)
        ray_world = _rotate_vector_by_quaternion(ray_camera, camera_quat)

        if abs(ray_world[2]) < 1e-6:
            self.get_logger().warning("Ray is parallel to table plane; cannot intersect")
            return

        t_param = (self._table_z - camera_origin[2]) / ray_world[2]
        if t_param <= 0:
            self.get_logger().warning(
                f"Ray-plane intersection behind camera (t={t_param:.3f}); skipping"
            )
            return

        px = camera_origin[0] + t_param * ray_world[0]
        py = camera_origin[1] + t_param * ray_world[1]
        pz = self._table_z

        pose = PoseStamped()
        pose.header.frame_id = self._target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.w = 1.0  # identity; grasp orientation is future work

        self._pick_pose_pub.publish(pose)
        self.get_logger().info(
            f"Estimated pose for '{class_name}' (conf={confidence:.2f}): "
            f"x={px:.3f}, y={py:.3f}, z={pz:.3f} in {self._target_frame}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
