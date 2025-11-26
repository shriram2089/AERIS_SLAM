#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Int16, String
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Lidar3DHardware(Node):
    """
    Combines 2D LaserScan messages with an associated pitch angle (Int16) 
    to construct a 3D PointCloud2 message by rotating each scan line.
    """
    def __init__(self):
        super().__init__('lidar_3d_hardware')

        # --- Configuration ---
        # IMPORTANT: Change this value ('X', 'Y', or 'Z') based on which axis 
        # your physical 2D Lidar is rotating around to sweep the 3D space.
        self.rotation_axis = 'Y' 
        # Define the minimum angular difference to accept a new layer (e.g., 1 degree)
        self.layer_increment = np.deg2rad(1.0) 
        # ---------------------

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/robot2/lidar_points', 10)

        # QoS for lidar scan
        qos_scan = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

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
        self.pitch_angle_rad = 0.0   # Current sweep angle in radians
        self.lidar_speed = 80.0   # optional

        # Accumulated points for 3D cloud
        self.point_layers = []
        self.last_layer_angle = 0.0

        self.get_logger().info(f"3D Hardware Lidar Node Started. Rotation Axis: {self.rotation_axis}")

    def angle_callback(self, msg: Int16):
        """Update current lidar sweep angle (Int16 -> radians)."""
        # Convert integer degrees to radians
        self.pitch_angle_rad = np.deg2rad(float(msg.data))

    def speed_callback(self, msg: String):
        """Update lidar speed from string (optional)."""
        try:
            self.lidar_speed = float(msg.data)
        except ValueError:
            self.get_logger().warn(f"Invalid lidar_speed value received: {msg.data}")

    def apply_rotation(self, x0, y0, z0, theta, axis):
        """Applies the correct rotation matrix based on the specified axis."""
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        
        # Identity for all rotations where z0 = 0
        if axis == 'Y':
            # Y-axis Rotation (Pitch): x = x0*cos(t), y = y0, z = -x0*sin(t)
            x = x0 * cos_t + z0 * sin_t
            y = y0
            z = -x0 * sin_t + z0 * cos_t
            return x, y, z
        
        elif axis == 'X':
            # X-axis Rotation (Roll): x = x0, y = y0*cos(t) - z0*sin(t), z = y0*sin(t) + z0*cos(t)
            x = x0
            y = y0 * cos_t - z0 * sin_t
            z = y0 * sin_t + z0 * cos_t
            return x, y, z

        elif axis == 'Z':
            # Z-axis Rotation (Yaw): x = x0*cos(t) - y0*sin(t), y = x0*sin(t) + y0*cos(t), z = z0
            x = x0 * cos_t - y0 * sin_t
            y = x0 * sin_t + y0 * cos_t
            z = z0
            return x, y, z
        
        else:
            self.get_logger().error(f"Invalid rotation axis: {axis}. Defaulting to no rotation.")
            return x0, y0, z0

    def scan_callback(self, msg: LaserScan):
        """
        Convert 2D scan into 3D points using current sweep angle and rotation axis.
        """
        # 1. Filter angle change
        if abs(self.pitch_angle_rad - self.last_layer_angle) < self.layer_increment:
            return  # Skip if angle change is too small to add a new layer

        new_points = []
        angle = msg.angle_min
        
        theta = self.pitch_angle_rad

        # 2. Convert each 2D range point (r, alpha) to 3D point (x, y, z)
        for r in msg.ranges:
            if np.isfinite(r) and r > msg.range_min and r < msg.range_max:
                
                # Step 1: Convert 2D polar to Cartesian coordinates (in the Lidar's 2D plane)
                # The 2D scan usually lies on the X-Y plane of its own frame, with Z=0.
                x0 = r * np.cos(angle)
                y0 = r * np.sin(angle)
                z0 = 0.0 # Initial Z is zero

                # Step 2: Apply rotation based on the configured axis
                x, y, z = self.apply_rotation(x0, y0, z0, theta, self.rotation_axis)

                new_points.append([x, y, z])
            
            angle += msg.angle_increment

        # 3. Update point cloud and publish
        if new_points:
            self.point_layers.extend(new_points)
            self.last_layer_angle = self.pitch_angle_rad
            
            # Create PointCloud2 message
            header = msg.header
            # Use "base_link" as the frame, assuming the Lidar's rotation origin is fixed relative to the base
            header.frame_id = "base_link" 
            
            # Use 'xyz' fields for the simplest point cloud
            pc_msg = pc2.create_cloud_xyz32(header, self.point_layers)
            
            # Important: Set the timestamp correctly
            pc_msg.header.stamp = self.get_clock().now().to_msg()
            
            self.pc_pub.publish(pc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Lidar3DHardware()
    try:
        # Uncomment the line below to see debug messages about layers being added
        # node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
