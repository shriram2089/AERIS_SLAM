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

        # Publisher for 3D point cloud
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
        self.pitch_angle = 0.0  # radians
        self.yaw_angle = 0.0    # radians (spinning lidar)
        self.lidar_speed = 80.0

        # Accumulated points
        self.point_layers = []
        self.last_layer_angle = 0.0
        self.layer_increment = np.deg2rad(1.0)  # add layer every 1 degree

        self.get_logger().info("3D Hardware Lidar Node Started")

    def angle_callback(self, msg: Int16):
        """Update current lidar pitch angle (tilt)."""
        self.pitch_angle = np.deg2rad(float(msg.data))

    def speed_callback(self, msg: String):
        """Update lidar speed from string (optional)."""
        try:
            self.lidar_speed = float(msg.data)
        except ValueError:
            self.get_logger().warn(f"Invalid lidar_speed: {msg.data}")

    def scan_callback(self, msg: LaserScan):
        """Convert 2D scan into 3D points and rotate to ground frame."""
        if abs(self.pitch_angle - self.last_layer_angle) < self.layer_increment:
            return  # skip if angle change is too small

        new_points = []
        angle = msg.angle_min

        # Rotation matrices
        cp = np.cos(self.pitch_angle)
        sp = np.sin(self.pitch_angle)
        cy = np.cos(self.yaw_angle)
        sy = np.sin(self.yaw_angle)

        # Rotation around Y (pitch) then Z (yaw)
        R = np.array([
            [cp * cy, -sy, sp * cy],
            [cp * sy, cy, sp * sy],
            [-sp, 0, cp]
        ])

        for r in msg.ranges:
            if np.isfinite(r):
                x0 = r * np.cos(angle)
                y0 = r * np.sin(angle)
                z0 = 0.0

                point_lidar = np.array([x0, y0, z0])
                point_world = R @ point_lidar

                new_points.append(point_world.tolist())

            angle += msg.angle_increment

        if new_points:
            self.point_layers.extend(new_points)
            self.last_layer_angle = self.pitch_angle
            # Update yaw based on lidar rotation speed and time
            self.yaw_angle += np.deg2rad(self.lidar_speed) * (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            self.yaw_angle %= 2 * np.pi

        # Publish accumulated 3D points
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
