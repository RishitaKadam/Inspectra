import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from datetime import datetime
import json
import os
import time

class IotTelemetryNode(Node):
    def __init__(self):
        super().__init__('iot_telemetry_node')
        self.pass_count = 0
        self.fail_count = 0
        self.last_counted_time = 0.0
        self.cooldown_seconds = 3.5  # Time window per PCB inspection

        # Listen to YOLO detections
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/object_detector_node/detections',
            self.detection_callback,
            10
        )

        # Listen to Defect Targeter / Pick Pose
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/pose_estimator_node/pick_pose',
            self.pose_callback,
            10
        )

        # Publisher to notify the conveyor / motion planner that a PCB passed inspection
        self.pub_pass = self.create_publisher(String, '/inspectra/pass_signal', 10)

        self.data_file = os.path.expanduser('~/inspectra/ros2_ws/telemetry_data.json')
        self.history = []
        self.get_logger().info('Updated Telemetry Engine Active! Empty detections now count as PASS.')

    def pose_callback(self, msg):
        """If a pick pose is generated, it means a defect requires robotic action -> FAIL"""
        current_time = time.time()
        if current_time - self.last_counted_time >= self.cooldown_seconds:
            self.fail_count += 1
            self.last_counted_time = current_time
            self.record_event("FAIL", "Defect Targeted by Robot")

    def detection_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_counted_time < self.cooldown_seconds:
            return

        # KEY FIX: If detections array is EMPTY, it means 0 defects were found -> GOOD PCB / PASS!
        if len(msg.detections) == 0:
            self.pass_count += 1
            self.last_counted_time = current_time
            
            # Send pass signal to conveyor/motion system
            pass_msg = String()
            pass_msg.data = "PASS"
            self.pub_pass.publish(pass_msg)
            
            self.record_event("PASS", "Clean PCB (0 Defects Found)")
            return

        # If bounding boxes ARE present, check if they are defects
        has_defect = False
        for det in msg.detections:
            for result in det.results:
                class_id = str(result.hypothesis.class_id).lower()
                if 'good' not in class_id and 'pass' not in class_id:
                    has_defect = True

        if has_defect:
            self.fail_count += 1
            self.last_counted_time = current_time
            self.record_event("FAIL", "Defect Detected by YOLO")

    def record_event(self, status, detail):
        total = self.pass_count + self.fail_count
        yield_rate = (self.pass_count / total) * 100.0 if total > 0 else 0.0
        timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        event = {
            "time": datetime.now().strftime("%H:%M:%S"),
            "status": status,
            "detail": detail,
            "yield_after": round(yield_rate, 1)
        }
        self.history.insert(0, event)
        if len(self.history) > 15:
            self.history.pop()

        payload = {
            "total_inspected": total,
            "pass_count": self.pass_count,
            "fail_count": self.fail_count,
            "yield_rate": round(yield_rate, 2),
            "last_updated": timestamp_str,
            "system_status": "ONLINE / INSPECTING",
            "history": self.history
        }

        with open(self.data_file, 'w') as f:
            json.dump(payload, f, indent=2)

        self.get_logger().info(f'[{status}] Total: {total} | Pass: {self.pass_count} | Fail: {self.fail_count} | Yield: {yield_rate:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = IotTelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
