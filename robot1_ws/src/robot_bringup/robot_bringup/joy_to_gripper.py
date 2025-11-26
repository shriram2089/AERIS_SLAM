#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoyToGripper(Node):
    def __init__(self):
        super().__init__('joy_to_gripper')

        self.publisher_ = self.create_publisher(String, '/robot1/gripper_cmd', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Button mappings (adjust as needed)
        self.button_open = 0    # A
        self.button_close = 4   # Y
        self.button_up = 1      # B
        self.button_down = 3    # X
        self.button_lidar_left = 2  # LB
        self.button_lidar_right = 5 # RB

        # Current command state
        self.active_cmd = None

        # Create a timer to publish continuously when button held
        self.timer = self.create_timer(0.1, self.publish_active_cmd)  # 10 Hz

    def joy_callback(self, msg: Joy):
        """
        Update active command based on currently pressed button(s).
        """
        if msg.buttons[self.button_open]:
            self.active_cmd = 'o'
        elif msg.buttons[self.button_close]:
            self.active_cmd = 'c'
        elif msg.buttons[self.button_up]:
            self.active_cmd = 'u'
        elif msg.buttons[self.button_down]:
            self.active_cmd = 'd'
        elif msg.buttons[self.button_lidar_left]:
            self.active_cmd = 'l'
        elif msg.buttons[self.button_lidar_right]:
            self.active_cmd = 'r'
        else:
            self.active_cmd = None  # nothing pressed

    def publish_active_cmd(self):
        """
        Continuously publish the active command if a button is held down.
        """
        if self.active_cmd is not None:
            msg = String()
            msg.data = self.active_cmd
            self.publisher_.publish(msg)
            self.get_logger().info(f"Held -> publishing gripper cmd: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyToGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
