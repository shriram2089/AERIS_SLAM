#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16
import serial
import time
import threading

DEFAULT_LIDAR_SPEED = 150  # ms per step, change this as needed

class GripperLidarController(Node):
    def __init__(self):
        super().__init__('gripper_lidar_controller')
        
        
        # try:
        #     self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        # except:
        #     self.ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
            
        self.ser = serial.Serial('/dev/arduino', 115200, timeout=1)
        
        # self.ser = None

        # try:
        #     try:
        #         self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        #         print("Connected to /dev/ttyACM1")
        #     except serial.SerialException:
        #         self.ser = serial.Serial('/dev/ttyACM2', 115200, timeout=1)
        #         print("Connected to /dev/ttyACM2")

        #     time.sleep(2)

        # except serial.SerialException:
        #     print("❌ Could not connect to /dev/ttyACM1 or /dev/ttyACM2")
        #     self.ser = None

        self.get_logger().info("Connected to Arduino via Serial.")

        # Set default LiDAR speed on startup
        self.ser.write(f"s{DEFAULT_LIDAR_SPEED}\n".encode())
        self.get_logger().info(f"LiDAR default speed set to: {DEFAULT_LIDAR_SPEED} ms per step")

        # Subscribers
        self.subscription = self.create_subscription(String, '/robot1/gripper_cmd', self.cmd_callback, 10)
        self.speed_subscription = self.create_subscription(String, '/robot1/lidar_speed', self.speed_callback, 10)

        # Publisher for LiDAR angle
        self.lidar_pub = self.create_publisher(Int16, '/robot1/lidar_angle', 10)

        # Start a background thread to read LiDAR angle from Arduino
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    def cmd_callback(self, msg):
        cmd = msg.data.lower().strip()
        valid_cmds = ['o', 'c', 'u', 'd', 'l', 'r']
        if cmd in valid_cmds:
            self.ser.write(cmd.encode())
            self.get_logger().info(f"Sent command: {cmd}")
        else:
            self.get_logger().warn(f"Ignored invalid command: {cmd}")

    def speed_callback(self, msg):
        try:
            speed = int(msg.data)
            speed = max(10, min(500, speed))
            self.ser.write(f"s{speed}\n".encode())
            self.get_logger().info(f"LiDAR speed set to: {speed} ms per step")
        except ValueError:
            self.get_logger().warn(f"Invalid LiDAR speed: {msg.data}")

    def read_serial_loop(self):
        """
        Continuously read serial lines from Arduino and publish LiDAR angle.
        Arduino should send lines like: "Lidar:42"
        """
        while True:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("Lidar:"):
                    angle = int(line.split(":")[1])
                    msg = Int16()
                    msg.data = angle
                    self.lidar_pub.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperLidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
