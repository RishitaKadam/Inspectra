import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class DefectTargeter(Node):
    def __init__(self):
        super().__init__('defect_targeter_node')
        
        # Subscribe to your existing 3D pick pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/pose_estimator_node/pick_pose',
            self.pose_callback,
            10
        )
        
        # Publisher for the final 2mm hover target
        self.target_pub = self.create_publisher(PoseStamped, '/inspectra/hover_target', 10)
        
        # The exact 2mm hover offset
        self.hover_offset = 0.002  
        
        self.get_logger().info('Defect Targeter Active! Intercepting poses to add 2mm hover...')

    def pose_callback(self, msg):
        hover_pose = PoseStamped()
        hover_pose.header = msg.header
        
        # Keep the exact same orientation and X/Y coordinates
        hover_pose.pose.orientation = msg.pose.orientation
        hover_pose.pose.position.x = msg.pose.position.x
        hover_pose.pose.position.y = msg.pose.position.y
        
        # Inject the 2mm hover precisely above the defect
        hover_pose.pose.position.z = msg.pose.position.z + self.hover_offset
        
        # Send the modified target out
        self.target_pub.publish(hover_pose)
        
        self.get_logger().info(f'Hover target sent! Z shifted from {msg.pose.position.z:.3f} to {hover_pose.pose.position.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = DefectTargeter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
