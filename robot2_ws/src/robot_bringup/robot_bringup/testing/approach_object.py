#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from auto_grip_node import AutoGripNode  # import your gripping node class

class BoxApproach(Node):
    def __init__(self):
        super().__init__('box_approach_node')

        # Subscribers
        self.object_sub = self.create_subscription(String, '/detected_objects', self.obj_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Grip sequence node
        self.grip_node = AutoGripNode()  # instantiate once

        # Parameters
        self.target_object = "box"  # YOLO detects box as book
        self.safe_distance = 1.2     # meters (for future real sensor)
        self.approach_speed = 0.1
        self.state = "idle"

        self.get_logger().info("Box approach node ready 🚀")

    def obj_callback(self, msg):
        detected_objects = [x.strip() for x in msg.data.split(",")]
        if self.target_object in detected_objects and self.state == "idle":
            self.get_logger().info(f"Target '{self.target_object}' detected. Starting approach...")
            self.state = "approaching"
            self.approach_object()

    def approach_object(self):
        # Fixed time forward motion (replace later with sensor feedback)
        move_cmd = Twist()
        move_cmd.linear.x = self.approach_speed
        duration = 3.0  # seconds to reach approximate safe distance

        end_time = self.get_clock().now().nanoseconds / 1e9 + duration
        self.get_logger().info("Moving forward to object...")

        while self.get_clock().now().nanoseconds / 1e9 < end_time:
            self.cmd_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)
        self.get_logger().info("Reached safe distance, running gripper sequence...")

        # Trigger gripping sequence from AutoGripNode
        self.grip_node.run_sequence()
        self.state = "idle"

def main(args=None):
    rclpy.init(args=args)
    node = BoxApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down BoxApproachNode...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
