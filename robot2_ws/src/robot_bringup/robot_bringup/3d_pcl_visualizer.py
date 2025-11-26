#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Int16, String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Lidar3DHardware(Node):
    def __init__(self):
        super().__init__('lidar_3d_hardware')

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/robot2/lidar_points', 10)

        # QoS for lidar scan
        qos_scan = QoSProfile(depth=10)
        qos_scan.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.scan_callback,
            qos_scan
        )
        self.angle_sub = self.create_subscription(Int16, '/robot2/lidar_angle', self.angle_callback, 10)
        self.speed_sub = self.create_subscription(String, '/robot2/lidar_speed', self.speed_callback, 10)

        # Current lidar state
        self.pitch_angle = 0.0   # radians
        self.lidar_speed = 80.0   # optional

        # Accumulated points
        self.point_layers = []
        self.last_layer_angle = 0.0
        self.layer_increment = np.deg2rad(1.0)  # add layer every 1 degree

        # ---- Manual rotation offsets ----
        # Calibrate these experimentally
        self.manual_x_deg = 5.0  # rotate around X-axis
        self.manual_y_deg = -25.0  # rotate around Y-axis
        self.manual_x = np.deg2rad(self.manual_x_deg)
        self.manual_y = np.deg2rad(self.manual_y_deg)

        self.get_logger().info("3D Hardware Lidar Node Started (Manual X/Y Alignment Active)")

    def angle_callback(self, msg: Int16):
        """Update current lidar pitch angle (Int16 -> radians)."""
        self.pitch_angle = np.deg2rad(float(msg.data))

    def speed_callback(self, msg: String):
        """Update lidar speed from string (optional)."""
        try:
            self.lidar_speed = float(msg.data)
        except ValueError:
            self.get_logger().warn(f"Invalid lidar_speed: {msg.data}")

    def scan_callback(self, msg: LaserScan):
        """Convert 2D scan into 3D points using current pitch + manual X/Y rotations."""
        if abs(self.pitch_angle - self.last_layer_angle) < self.layer_increment:
            return  # skip if angle change is too small

        new_points = []
        angle = msg.angle_min

        # Effective pitch angle (from lidar tilt)
        total_pitch = self.pitch_angle

        cp = np.cos(total_pitch)
        sp = np.sin(total_pitch)

        cx = np.cos(self.manual_x)
        sx = np.sin(self.manual_x)
        cy = np.cos(self.manual_y)
        sy = np.sin(self.manual_y)

        for r in msg.ranges:
            if np.isfinite(r):
                # Original 2D scan
                x0 = r * np.cos(angle)
                y0 = r * np.sin(angle)
                z0 = 0.0

                # Rotate by lidar pitch (Y-axis)
                x1 = cp * x0 + sp * z0
                y1 = y0
                z1 = -sp * x0 + cp * z0

                # Apply manual X rotation
                x2 = x1
                y2 = cx * y1 - sx * z1
                z2 = sx * y1 + cx * z1

                # Apply manual Y rotation
                x3 = cy * x2 + sy * z2
                y3 = y2
                z3 = -sy * x2 + cy * z2

                new_points.append([x3, y3, z3])

            angle += msg.angle_increment

        if new_points:
            self.point_layers.extend(new_points)
            self.last_layer_angle = self.pitch_angle

        header = msg.header
        header.frame_id = "base_link"
        pc_msg = pc2.create_cloud_xyz32(header, self.point_layers)
        self.pc_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DHardware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
