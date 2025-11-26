#!/usr/bin/env python3
import math
import serial
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2quat as quaternion_from_euler
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.constants import S_TO_NS


class KalmanFilter:
    def __init__(self, process_var=1e-3, meas_var=1e-2, init_val=0.0):
        self.x = init_val
        self.P = 1.0
        self.Q = process_var
        self.R = meas_var

    def update(self, z):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P *= (1 - K)
        return self.x


class OdomMotorNode(Node):
    def __init__(self):
        super().__init__('odom_motor_node')

        # Parameters
        self.declare_parameter("wheel_radius", 0.0366)
        self.declare_parameter("wheel_separation", 0.195)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("namespace", "robot2")

        self.wheel_radius_ = self.get_parameter("wheel_radius").value
        self.wheel_separation_ = self.get_parameter("wheel_separation").value
        self.odom_frame_ = self.get_parameter("odom_frame").value
        self.base_frame_ = self.get_parameter("base_frame").value
        self.dt = 1.0 / self.get_parameter("update_rate").value
        self.namespace_ = self.get_parameter("namespace").value

        # Serial setup
        serial_port = self.get_parameter("serial_port").value
        baudrate = self.get_parameter("baudrate").value
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=0.1)
            self.get_logger().info(f"Serial connected to {serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # Pose
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Kalman filters
        self.kf_left = KalmanFilter(process_var=1e-3, meas_var=1e-1)
        self.kf_right = KalmanFilter(process_var=1e-3, meas_var=1e-1)

        # Publishers
        self.odom_pub_ = self.create_publisher(Odometry, f"/{self.namespace_}/odom", 10)
        self.imu_pub = self.create_publisher(Imu, f"/{self.namespace_}/imu", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Subscriber
        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            f"/{self.namespace_}/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        # Timers
        self.create_timer(self.dt, self.timer_callback)
        self.create_timer(1.0, self.publish_map_to_odom)  # map->odom
        self.publish_static_transforms()

        # Command state
        self.cmd_linear_ = 0.0
        self.cmd_angular_ = 0.0
        self.cmd_sent = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.0

        self.prev_time = self.get_clock().now()

        self.get_logger().info("OdomMotorNode initialized and running")

    # ===== CMD_VEL CALLBACK =====
    def cmd_vel_callback(self, msg):
        self.cmd_linear_ = msg.linear.x
        self.cmd_angular_ = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        self.cmd_sent = True

        SPEED_SCALE = 10
        v_left = self.cmd_linear_ - self.cmd_angular_ * self.wheel_separation_ / 2.0
        v_right = self.cmd_linear_ + self.cmd_angular_ * self.wheel_separation_ / 2.0

        rpm_left = int((v_left / (2 * math.pi * self.wheel_radius_)) * 60) * SPEED_SCALE
        rpm_right = int((v_right / (2 * math.pi * self.wheel_radius_)) * 60) * SPEED_SCALE

        if self.ser is not None:
            self.send_motor_command(1, rpm_left)   # Rear Left
            self.send_motor_command(2, -rpm_left)  # Front Left
            self.send_motor_command(3, -rpm_right)  # Front Right
            self.send_motor_command(4, rpm_right)   # Rear Right

    def send_motor_command(self, motor_id, rpm):
        try:
            cmd = f"{motor_id}:{rpm}\n"
            self.ser.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send to motor {motor_id}: {e}")

    # ===== ODOM UPDATE =====
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / S_TO_NS
        if dt <= 0.0:
            return
        self.prev_time = now

        # Stop motors if cmd_vel timed out
        if self.cmd_sent and (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            if self.ser is not None:
                try:
                    self.ser.write(b"V:0.00,W:0.00\n")
                    self.cmd_sent = False
                    self.get_logger().info("Stopped motors due to cmd_vel timeout")
                except Exception as e:
                    self.get_logger().warn(f"Serial stop error: {e}")

        # Fallback odom calculation from cmd_vel (replace with actual feedback if available)
        v = self.cmd_linear_
        omega = self.cmd_angular_

        self.theta_ += omega * dt
        self.x_ += v * math.cos(self.theta_) * dt
        self.y_ += v * math.sin(self.theta_) * dt

        q = quaternion_from_euler(0.0, 0.0, self.theta_)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame_
        odom_msg.child_frame_id = self.base_frame_
        odom_msg.pose.pose.position.x = self.x_
        odom_msg.pose.pose.position.y = self.y_
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega
        self.odom_pub_.publish(odom_msg)

        # Broadcast base_footprint TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = self.odom_frame_
        tf_msg.child_frame_id = self.base_frame_
        tf_msg.transform.translation.x = self.x_
        tf_msg.transform.translation.y = self.y_
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    # ===== STATIC TRANSFORMS =====
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

        tf3 = TransformStamped()
        tf3.header.stamp = now
        tf3.header.frame_id = 'base_link'
        tf3.child_frame_id = 'imu_link'
        tf3.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([tf1, tf2, tf3])

    # ===== DYNAMIC MAP->ODOM =====
    def publish_map_to_odom(self):
        tf4 = TransformStamped()
        tf4.header.stamp = self.get_clock().now().to_msg()
        tf4.header.frame_id = 'map'
        tf4.child_frame_id = self.odom_frame_
        tf4.transform.translation.x = 0.0
        tf4.transform.translation.y = 0.0
        tf4.transform.translation.z = 0.0
        tf4.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf4)


def main(args=None):
    rclpy.init(args=args)
    node = OdomMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OdomMotorNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
