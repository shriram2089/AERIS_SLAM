#feedback node

#!/usr/bin/env python3

import math
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
# from tf_transformations import quaternion_from_euler
from transforms3d.euler import euler2quat as quaternion_from_euler
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
from rclpy.constants import S_TO_NS


class FakeOdometryPublisher(Node):
    def __init__(self):
        super().__init__('fake_odometry_publisher')

        # Parameters
        self.declare_parameter('wheel_radius', 0.0366)
        self.declare_parameter('wheel_separation', 0.195)

        self.wheel_radius_ = self.get_parameter('wheel_radius').value
        self.wheel_separation_ = self.get_parameter('wheel_separation').value

        # Odometry state
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Publishers
        self.odom_pub_ = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # Fake linear and angular velocity
        self.linear_velocity_ = 0.1    # m/s
        self.angular_velocity_ = 0.2   # rad/s

        # One-time info flag
        self.first_log_done = False

        # Timer for periodic publishing
        self.timer_ = self.create_timer(0.05, self.publish_fake_odometry)  # 20 Hz

        self.get_logger().info("Fake odometry publisher started")

    def publish_fake_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time_).nanoseconds / S_TO_NS
        self.prev_time_ = current_time

        # Integrate motion
        self.theta_ += self.angular_velocity_ * dt
        self.x_ += self.linear_velocity_ * math.cos(self.theta_) * dt
        self.y_ += self.linear_velocity_ * math.sin(self.theta_) * dt

        # Create quaternion
        q = quaternion_from_euler(0.0, 0.0, self.theta_)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        odom_msg.twist.twist.linear.x = self.linear_velocity_
        odom_msg.twist.twist.angular.z = self.angular_velocity_

        self.odom_pub_.publish(odom_msg)

        # Publish TF transform (odom → base_footprint)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x_
        tf_msg.transform.translation.y = self.y_
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster_.sendTransform(tf_msg)

        # Log only once
        if not self.first_log_done:
            self.get_logger().info("Publishing fake odometry and TF from odom → base_footprint")
            self.first_log_done = True


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






#joysitck node


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time

class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('joystick_motor_controller')
        self.serial_conn = None

        try:
            self.serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Serial connected to /dev/ttyACM0")
            time.sleep(2)  # Allow Arduino to reset
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info("Joystick subscriber started")

    def joy_callback(self, msg):
        if self.serial_conn is None:
            self.get_logger().warn("Serial not connected. Ignoring joystick input.")
            return

        try:
            linear_axis = msg.axes[1]   # Forward/backward (up/down)
            angular_axis = msg.axes[2]  # Left/right turning (twist)

            linear_rpm = int(linear_axis * 210)
            angular_rpm = int(angular_axis * 210)

            # Calculate wheel RPMs
            # Assuming motors 1 & 3 (Left), 2 & 4 (Right)
            # Left wheels need reverse RPMs due to flipped orientation
            left_rpm = -(linear_rpm - angular_rpm)
            right_rpm =  (linear_rpm + angular_rpm)

            self.get_logger().info(
                f"Linear: {linear_rpm}, Angular: {angular_rpm}, "
                f"Left RPM: {left_rpm}, Right RPM: {right_rpm}"
            )

            # Map motor IDs accordingly
            self.send_motor_command(1, left_rpm)   # Rear Left
            self.send_motor_command(4, right_rpm)  # Rear Right
            self.send_motor_command(2, left_rpm)   # Front Left
            self.send_motor_command(3, right_rpm)  # Front Right

        except Exception as e:
            self.get_logger().error(f"Error in joy_callback: {e}")

    def send_motor_command(self, motor_id, rpm):
        try:
            cmd = f"{motor_id}:{rpm}\n"
            self.serial_conn.write(cmd.encode('utf-8'))
            self.get_logger().info(f"Sent to motor {motor_id}: RPM {rpm}")
        except Exception as e:
            self.get_logger().error(f"Failed to send to motor {motor_id}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMotorController()

    if node.serial_conn is None:
        node.get_logger().error("Exiting due to serial connection failure.")
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down joystick motor controller")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()