#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time
from auto_grip_node import AutoGripNode  # your gripping node

# Rotation helper
def rotate_points(points, angle_deg):
    """Rotate Nx3 array around Z axis by angle_deg."""
    angle = np.deg2rad(angle_deg)
    R = np.array([[np.cos(angle), -np.sin(angle), 0],
                  [np.sin(angle),  np.cos(angle), 0],
                  [0, 0, 1]])
    return points @ R.T

class BoxApproachFusion(Node):
    def __init__(self):
        super().__init__('box_approach_fusion')

        # Subscribers
        self.obj_sub = self.create_subscription(String, '/detected_objects', self.obj_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/robot2/lidar_points', self.pc_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Grip sequence
        self.grip_node = AutoGripNode()

        # Parameters
        self.target_object = "box"  # your YOLO reports box as book
        self.safe_distance = 0.8    # meters
        self.approach_speed = 0.1
        self.lidar_rotation_deg = 45.0  # compensate LiDAR rotation

        # State
        self.state = "idle"
        self.latest_pc = None
        self.latest_obj_detected = False

        self.get_logger().info("Box Approach Fusion Node Ready 🚀")

    def obj_callback(self, msg: String):
        detected_objects = [x.strip() for x in msg.data.split(",")]
        if self.target_object in detected_objects:
            self.latest_obj_detected = True
            if self.state == "idle" and self.latest_pc is not None:
                self.get_logger().info(f"Target '{self.target_object}' detected. Starting fusion approach...")
                self.state = "approaching"
                self.approach_object()

    def pc_callback(self, msg: PointCloud2):
        self.latest_pc = msg

    def approach_object(self):
        if self.latest_pc is None:
            self.get_logger().warn("No LiDAR points yet. Waiting...")
            return

        # Convert PointCloud2 to numpy array
        points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(self.latest_pc, field_names=("x","y","z"), skip_nans=True)])
        if points.size == 0:
            self.get_logger().warn("PointCloud is empty.")
            return

        # Rotate LiDAR points back to align with robot frame
        points = rotate_points(points, -self.lidar_rotation_deg)

        # Filter points in front of robot (simple approach)
        forward_points = points[(points[:,0] > 0) & (np.abs(points[:,1]) < 0.3)]
        if forward_points.size == 0:
            self.get_logger().warn("No points in front after filtering.")
            return

        # Compute nearest point in front
        distance = np.min(forward_points[:,0])
        self.get_logger().info(f"Nearest point distance: {distance:.2f} m")

        # Move forward until safe distance
        move_cmd = Twist()
        while distance > self.safe_distance:
            move_cmd.linear.x = self.approach_speed
            self.cmd_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            # Update distance
            points = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(self.latest_pc, field_names=("x","y","z"), skip_nans=True)])
            points = rotate_points(points, -self.lidar_rotation_deg)
            forward_points = points[(points[:,0] > 0) & (np.abs(points[:,1]) < 0.3)]
            if forward_points.size == 0:
                break
            distance = np.min(forward_points[:,0])

        # Stop
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)
        self.get_logger().info("Reached safe distance, running gripper sequence...")

        # Trigger gripping sequence
        self.grip_node.run_sequence()
        self.state = "idle"
        self.latest_obj_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = BoxApproachFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down BoxApproachFusionNode...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
