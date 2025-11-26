#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class AutoGripNode(Node):
    def __init__(self):
        super().__init__('auto_grip_node')
        self.pub = self.create_publisher(String, '/robot2/gripper_cmd', 10)
        self.get_logger().info("AutoGripNode started — performing grip sequence...")
        self.timer = self.create_timer(2.0, self.run_sequence)  # start after 2s
        self.has_run = False

    def send_cmd(self, cmd, repeats=10, delay=0.05):
        """Send a single command multiple times to move the gripper fully."""
        msg = String()
        msg.data = cmd
        for _ in range(repeats):
            self.pub.publish(msg)
            self.get_logger().debug(f"Sent gripper cmd: {cmd}")
            time.sleep(delay)

    def run_sequence(self):
        if self.has_run:
            return
        self.has_run = True

        # Sequence: open → down → close → up
        self.get_logger().info("Opening gripper...")
        self.send_cmd('o', repeats=20, delay=0.05)

        self.get_logger().info("Moving down...")
        self.send_cmd('d', repeats=20, delay=0.05)

        self.get_logger().info("Closing gripper...")
        self.send_cmd('c', repeats=20, delay=0.05)

        self.get_logger().info("Moving up...")
        self.send_cmd('u', repeats=15, delay=0.05)

        self.get_logger().info("Grip sequence complete ✅")

def main(args=None):
    rclpy.init(args=args)
    node = AutoGripNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down AutoGripNode...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
