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

        self.get_logger().info("3D Hardware Lidar Node Started")

    def angle_callback(self, msg: Int16):
        """Update current lidar pitch angle (Int16 -> radians)."""
        # Ensure angle is always positive and within 0 to 2*pi for predictable wrap-around detection
        angle_rad = np.deg2rad(float(msg.data)) % (2 * np.pi) 
        self.pitch_angle = angle_rad

    def speed_callback(self, msg: String):
        """Update lidar speed from string (optional)."""
        try:
            self.lidar_speed = float(msg.data)
        except ValueError:
            self.get_logger().warn(f"Invalid lidar_speed: {msg.data}")

    def scan_callback(self, msg: LaserScan):
        """Convert 2D scan into 3D points using current pitch and manage rotation."""
        current_pitch_deg = np.rad2deg(self.pitch_angle)
        last_pitch_deg = np.rad2deg(self.last_layer_angle)
        
        # --- MODIFICATION: Check for Full Rotation (e.g., passing the 360-degree boundary) ---
        # If the angle wraps from near 360 degrees back to near 0 degrees, the scan is complete.
        if last_pitch_deg > 300.0 and current_pitch_deg < 60.0 and self.point_layers:
             # 1. Publish the full cloud before clearing
             header = msg.header
             header.frame_id = "base_link"
             pc_msg = pc2.create_cloud_xyz32(header, self.point_layers)
             self.pc_pub.publish(pc_msg)
             self.get_logger().info(f"Published full cloud with {len(self.point_layers)} points.")

             # 2. CLEAR the accumulated points for the next rotation
             self.point_layers = []
             self.last_layer_angle = self.pitch_angle 
             
             # Crucial: return to skip adding points to the new layer until the angle changes again
             return 

        # --- EXISTING: Skip if angle change is too small ---
        if abs(self.pitch_angle - self.last_layer_angle) < self.layer_increment:
            return  # skip if angle change is too small

        # --- EXISTING: Point Generation ---
        new_points = []
        angle = msg.angle_min
        c = np.cos(self.pitch_angle)
        s = np.sin(self.pitch_angle)

        for r in msg.ranges:
            if np.isfinite(r):
                x0 = r * np.cos(angle)
                y0 = r * np.sin(angle)
                # z0 is implicitly 0.0, as the 2D scan is on the XY plane

                # Rotation about the Y-axis (Pitch angle)
                x = c * x0 
                y = y0
                z = -s * x0 

                new_points.append([x, y, z])
            angle += msg.angle_increment

        if new_points:
            self.point_layers.extend(new_points)
            self.last_layer_angle = self.pitch_angle

        # NOTE: Publishing the partial cloud is removed here to reduce network load.
        # The full cloud is published only when the rotation completes (see logic above).


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DHardware()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()