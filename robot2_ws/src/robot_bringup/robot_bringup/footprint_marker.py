import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import math

class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        self.publisher = self.create_publisher(Marker, 'footprint_marker', 10)
        self.sub = self.create_subscription(Odometry, '/robot2/odom', self.odom_callback, 10)
        self.current_pose = None
        self.timer = self.create_timer(0.2, self.publish_marker)  # 5 Hz

    def odom_callback(self, msg: Odometry):
        """Store the latest odometry pose."""
        self.current_pose = msg.pose.pose

    def publish_marker(self):
        if self.current_pose is None:
            return

        # --- Footprint (rectangle) ---
        footprint = Marker()
        # footprint.header.frame_id = "odom"
        footprint.header.frame_id = "base_link"
        
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.ns = "robot_footprint"
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD

        footprint.scale.x = 0.3
        footprint.scale.y = 0.2
        footprint.scale.z = 0.01
        footprint.pose = self.current_pose

        footprint.color.r = 0.0
        footprint.color.g = 1.0
        footprint.color.b = 0.0
        footprint.color.a = 0.5

        # --- Direction Arrow ---
        arrow = Marker()
        # arrow.header.frame_id = "odom"
        arrow.header.frame_id = "base_link"
        
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "robot_direction"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        arrow.scale.x = 0.4   # length
        arrow.scale.y = 0.05  # shaft diameter
        arrow.scale.z = 0.05  # head diameter

        arrow.pose = self.current_pose

        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        self.publisher.publish(footprint)
        self.publisher.publish(arrow)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
