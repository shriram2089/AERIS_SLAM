#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from auto_grip_node import AutoGripNode  # your gripping node

class AutoScanPick(Node):
    def __init__(self):
        super().__init__('auto_scan_pick_node')

        # Subscribers
        self.object_sub = self.create_subscription(String, '/detected_objects', self.obj_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Gripping sequence node
        self.grip_node = AutoGripNode()

        # Parameters
        self.target_object = "box"  # YOLO may detect as book
        self.safe_distance = 1.0    # meters (future: use sensor)
        self.approach_speed = 0.1
        self.scan_speed = 0.05      # slow scanning forward
        self.drop_distance = 1.0    # move forward after grip to drop
        self.state = "scanning"

        # Timer for scanning movement
        self.scan_timer = self.create_timer(0.1, self.scan_move)

        self.get_logger().info("AutoScanPick node started — scanning for boxes continuously 🚀")

    def scan_move(self):
        """Move forward slowly while scanning surroundings."""
        if self.state != "scanning":
            return
        move_cmd = Twist()
        move_cmd.linear.x = self.scan_speed
        self.cmd_pub.publish(move_cmd)

    def obj_callback(self, msg):
        detected_objects = [x.strip() for x in msg.data.split(",")]
        if self.target_object in detected_objects and self.state == "scanning":
            self.get_logger().info(f"Target '{self.target_object}' detected! Approaching...")
            self.state = "approaching"
            self.stop_robot()
            self.approach_and_grip()

    def stop_robot(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)

    def approach_and_grip(self):
        """Move forward, pick, move forward, drop, then resume scanning."""
        # Step 1: Approach target
        move_cmd = Twist()
        move_cmd.linear.x = self.approach_speed
        duration = 2.0  # seconds to approximate safe distance
        end_time = self.get_clock().now().nanoseconds / 1e9 + duration
        self.get_logger().info("Approaching box...")
        while self.get_clock().now().nanoseconds / 1e9 < end_time:
            self.cmd_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info("Reached box. Running grip sequence...")
        self.grip_node.run_sequence()

        # Step 2: Move forward to drop
        move_cmd.linear.x = self.approach_speed
        duration = self.drop_distance / self.approach_speed
        end_time = self.get_clock().now().nanoseconds / 1e9 + duration
        self.get_logger().info("Moving forward to drop object...")
        while self.get_clock().now().nanoseconds / 1e9 < end_time:
            self.cmd_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()
        self.get_logger().info("Object dropped. Resuming scan...")
        self.state = "scanning"

def main(args=None):
    rclpy.init(args=args)
    node = AutoScanPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down AutoScanPick node...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
