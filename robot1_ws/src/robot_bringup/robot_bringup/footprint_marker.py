#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from transforms3d.euler import euler2quat
from transforms3d.quaternions import qmult


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__('footprint_publisher')
        self.publisher = self.create_publisher(Marker, 'footprint_marker', 10)
        self.sub = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
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
        footprint.header.frame_id = "base_link"  # Match odom frame of Odometry
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.ns = "robot_footprint"
        footprint.id = 0
        footprint.type = Marker.CUBE
        footprint.action = Marker.ADD

        footprint.scale.x = 0.3
        footprint.scale.y = 0.2
        footprint.scale.z = 0.01

        footprint.pose.position = self.current_pose.position

        # --- Direction Arrow ---
        arrow = Marker()
        arrow.header.frame_id = "base_link"
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = "robot_direction"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        arrow.scale.x = 0.4
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05

        arrow.pose.position = self.current_pose.position

        # --- Apply +90° rotation about Z-axis ---
        # Current quaternion in (x, y, z, w)
        q = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w,
        ]

        # transforms3d expects (w, x, y, z)
        q_current = [q[3], q[0], q[1], q[2]]

        # +90° rotation around Z (in radians)
        q_rot = euler2quat(0, 0, 1.5708)  # (w, x, y, z)

        # Combine rotations
        q_new = qmult(q_current, q_rot)  # (w, x, y, z)

        # Reorder back to ROS format (x, y, z, w)
        footprint.pose.orientation.x = q_new[1]
        footprint.pose.orientation.y = q_new[2]
        footprint.pose.orientation.z = q_new[3]
        footprint.pose.orientation.w = q_new[0]

        arrow.pose.orientation.x = q_new[1]
        arrow.pose.orientation.y = q_new[2]
        arrow.pose.orientation.z = q_new[3]
        arrow.pose.orientation.w = q_new[0]

        # --- Colors ---
        footprint.color.r = 0.0
        footprint.color.g = 1.0
        footprint.color.b = 0.0
        footprint.color.a = 0.5

        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        # Publish both
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
