#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from transforms3d.euler import euler2mat

class RotateScanNode(Node):
    def __init__(self):
        super().__init__('rotate_scan_node')

        # QoS for reliable LIDAR scan communication
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the input scan topic
        self.sub = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            qos
        )

        # Publisher for rotated scan
        self.pub = self.create_publisher(
            LaserScan,
            '/robot1/scan_rotated',
            qos
        )

        # Define rotation in radians: roll=1.57, pitch=0, yaw=3.14
        self.rotation_matrix = euler2mat(1.57, 0.0, 3.14)  # RPY

        self.get_logger().info("RotateScanNode initialized. Applying rotation (1.57, 0, 3.14) to /robot1/scan")

    def scan_callback(self, msg: LaserScan):
        num_readings = len(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_readings)
        ranges = np.array(msg.ranges)

        # Remove invalid ranges (Inf or NaN)
        valid = np.isfinite(ranges)
        angles = angles[valid]
        ranges = ranges[valid]

        # Convert polar to Cartesian
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        # Stack points
        points = np.vstack((xs, ys, zs))

        # Apply rotation
        rotated_points = self.rotation_matrix.dot(points)

        # Convert back to polar
        new_ranges = np.linalg.norm(rotated_points[:2, :], axis=0)
        new_angles = np.arctan2(rotated_points[1, :], rotated_points[0, :])

        # Recreate LaserScan message
        new_msg = LaserScan()
        new_msg.header = msg.header
        new_msg.header.frame_id = "laser_frame"
        new_msg.angle_min = np.min(new_angles)
        new_msg.angle_max = np.max(new_angles)
        new_msg.angle_increment = (new_msg.angle_max - new_msg.angle_min) / len(new_angles)
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = new_ranges.tolist()
        new_msg.intensities = msg.intensities  # optional reuse

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotateScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
