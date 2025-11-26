#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class FootprintVisualizer(Node):
    def __init__(self):
        super().__init__('footprint_visualizer')

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10
        )

        # Publisher for RViz markers
        self.marker_pub = self.create_publisher(Marker, 'footprint_marker', 10)

        # Robot footprint radius
        self.footprint_radius = 0.15

        self.get_logger().info("✅ Footprint visualizer started. Subscribed to /robot1/odom")

    # ---------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        # Extract pose from odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # Publish both markers
        self.publish_footprint(msg.header, x, y)
        self.publish_arrow(msg.header, x, y, q)

    # ---------------------------------------------------------
    def publish_footprint(self, header, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = header.stamp  # use odometry timestamp

        marker.ns = "footprint"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0  # cylinder doesn't need orientation

        marker.scale.x = self.footprint_radius * 2.0
        marker.scale.y = self.footprint_radius * 2.0
        marker.scale.z = 0.01

        marker.color.a = 0.6
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0

        self.marker_pub.publish(marker)

    # ---------------------------------------------------------
    def publish_arrow(self, header, x, y, orientation):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = header.stamp  # use odometry timestamp

        marker.ns = "orientation_arrow"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.05  # slightly above ground

        # Directly use odometry orientation quaternion
        marker.pose.orientation = orientation

        marker.scale.x = 0.3  # arrow length
        marker.scale.y = 0.05 # arrow width
        marker.scale.z = 0.05 # arrow height

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = FootprintVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
