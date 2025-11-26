import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile
import serial
import math
import time


class DiffDriveSerialOdomNode(Node):
    def __init__(self):
        super().__init__('diff_drive_serial_odom_node')
        
        self.current_linear = 0.0
        self.current_angular = 0.0


        # Serial connection to ESP32
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connected for diff drive control + odom")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10))

        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        self.mode_sent = False
        self.cmd_sent = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.0  # seconds

        # Publish static transforms (base_link -> laser_frame)
        self.publish_static_transforms()

        # Timer for updating odometry & timeout
        self.create_timer(0.05, self.update_odometry)  # 20Hz

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        tf1 = TransformStamped()
        tf1.header.stamp = now
        tf1.header.frame_id = 'base_footprint'
        tf1.child_frame_id = 'base_link'
        tf1.transform.rotation.w = 1.0

        tf2 = TransformStamped()
        tf2.header.stamp = now
        tf2.header.frame_id = 'base_link'
        tf2.child_frame_id = 'laser_frame'
        tf2.transform.translation.z = 0.2
        tf2.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([tf1, tf2])

    def cmd_vel_callback(self, msg):
        if not self.mode_sent:
            try:
                self.ser.write(b"MODE:AUTO\n")
                self.ser.flush()
                self.get_logger().info("Sent: MODE:AUTO")
            except Exception as e:
                self.get_logger().warn(f"Serial write error (mode): {e}")
            self.mode_sent = True

        self.current_linear = msg.linear.x
        self.current_angular = msg.angular.z

        command = f"V:{self.current_linear:.2f},W:{self.current_angular:.2f}\n"
        try:
            self.ser.write(command.encode())
            self.ser.flush()
            self.cmd_sent = True
            self.last_cmd_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f"Serial write error (cmd_vel): {e}")

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Stop if no command for timeout duration
        if self.cmd_sent and (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            self.current_linear = 0.0
            self.current_angular = 0.0
            self.cmd_sent = False
            try:
                self.ser.write(b"V:0.00,W:0.00\n")
                self.ser.flush()
            except Exception as e:
                self.get_logger().warn(f"Serial stop error: {e}")

        # --- FAKE ODOMETRY INTEGRATION ---
        # just integrates velocities over time for relative pose
        delta_x = self.current_linear * math.cos(self.th) * dt
        delta_y = self.current_linear * math.sin(self.th) * dt
        delta_th = self.current_angular * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Normalize angle
        if self.th > math.pi:
            self.th -= 2 * math.pi
        elif self.th < -math.pi:
            self.th += 2 * math.pi

        # --- Publish odometry ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.th / 2.0)

        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.angular.z = self.current_angular
        self.odom_pub.publish(odom)

        # --- Publish TF ---
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation.z = math.sin(self.th / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
