#!/usr/bin/env python3
import math
import time
import serial
import smbus
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat


# -------------------- Simple IMU Reader --------------------
class IMUReader:
    def __init__(self, bus_num=1, address=0x68, calib_time=2.0):
        self.bus = smbus.SMBus(bus_num)
        self.address = address

        # Wake up MPU6050
        self.bus.write_byte_data(address, 0x6B, 0)
        self.bus.write_byte_data(address, 0x1B, 0)  # gyro 250 dps
        self.bus.write_byte_data(address, 0x1C, 0)  # accel 2g
        print("✅ MPU6050 connected. Calibrating gyro...")

        # Gyro bias calibration
        start = time.time()
        samples = []
        while time.time() - start < calib_time:
            data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            gz = self.bytes_to_int16(data[4], data[5]) / 131.0
            samples.append(gz)
        self.gyro_bias_z = sum(samples) / len(samples)
        print(f"🛰️ Gyro Z bias: {self.gyro_bias_z:.3f} deg/s -> Calibration done!")

    def bytes_to_int16(self, high, low):
        val = (high << 8) | low
        return val - 65536 if val > 32767 else val

    def read_gyro_z(self):
        data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
        gz = self.bytes_to_int16(data[4], data[5]) / 131.0
        return gz - self.gyro_bias_z  # deg/s


# -------------------- Kalman Filter --------------------
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


# -------------------- Fused Odom Node --------------------
class FusedOdomNode(Node):
    def __init__(self):
        super().__init__("fused_odom_node")

        # ---- Parameters ----
        self.declare_parameter("wheel_radius", 0.0366)
        self.declare_parameter("wheel_separation", 0.195)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("update_rate", 20.0)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.dt = 1.0 / self.get_parameter("update_rate").value

        # ---- Serial ----
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baudrate").value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"✅ Serial connected: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            self.ser = None

        # ---- IMU ----
        try:
            self.imu = IMUReader(bus_num=1, address=0x68)
        except Exception as e:
            self.get_logger().error(f"❌ IMU init failed: {e}")
            self.imu = None

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

        # ---- Commanded velocities ----
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # ---- Kalman filters ----
        self.kf_left = KalmanFilter()
        self.kf_right = KalmanFilter()

        # ---- ROS Interfaces ----
        self.create_subscription(Twist, "/robot1/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/robot1/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/robot1/imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_time = self.get_clock().now()
        self.create_timer(self.dt, self.timer_callback)

    # -------------------- CMD Callback --------------------
    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

        # Compute wheel linear velocities
        v_left = self.cmd_linear - self.cmd_angular * self.wheel_separation / 2.0
        v_right = self.cmd_linear + self.cmd_angular * self.wheel_separation / 2.0

        # Convert to RPM
        rpm_left = int((v_left / (2 * math.pi * self.wheel_radius)) * 600)
        rpm_right = int((v_right / (2 * math.pi * self.wheel_radius)) * 600)

        # Send commands to motors
        self.send_motor(1, -rpm_left)
        self.send_motor(2, -rpm_left)
        self.send_motor(3, rpm_right)
        self.send_motor(4, rpm_right)

    def send_motor(self, motor_id, rpm):
        if not self.ser:
            return
        try:
            self.ser.write(f"{motor_id}:{rpm}\n".encode())
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to send motor {motor_id}: {e}")

    # -------------------- Read Wheel Feedback --------------------
    def read_ddsm_feedback(self):
        rpm_left, rpm_right = None, None
        if not self.ser:
            return None, None

        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode().strip()
                print("📨 Raw feedback:", line)

                if not line:
                    continue
                parts = line.split()
                motor_id = int(parts[1])
                rpm_val = float(parts[4]) / 10.0
                if motor_id == 1:
                    rpm_left = rpm_val
                elif motor_id == 2:
                    rpm_right = rpm_val
            except Exception:
                continue
        return rpm_left, rpm_right

    # -------------------- Timer Callback --------------------
    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.prev_time = now

        rpm_left, rpm_right = self.read_ddsm_feedback()
        if rpm_left is None or rpm_right is None:
            return

        rpm_left = self.kf_left.update(rpm_left)
        rpm_right = self.kf_right.update(rpm_right)

        # Convert RPM -> rad/s
        w_left = 2 * math.pi * rpm_left / 60.0
        w_right = 2 * math.pi * rpm_right / 60.0

        # Linear velocities
        v_left = w_left * self.wheel_radius
        v_right = w_right * self.wheel_radius

        # Odometry velocities
        v = (v_left + v_right) / 2.0
        omega_enc = (v_right - v_left) / self.wheel_separation

        # IMU yaw rate (fusion)
        omega_imu = 0.0
        if self.imu:
            gz_deg = self.imu.read_gyro_z()
            omega_imu = math.radians(gz_deg)
        omega_fused = 0.7 * omega_imu + 0.3 * omega_enc  # weighted fusion

        # Update pose
        self.theta += omega_fused * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # ---- Publish Odometry ----
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = euler2quat(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]
        self.odom_pub.publish(odom)

        # ---- Publish TF ----
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[1]
        tf.transform.rotation.y = q[2]
        tf.transform.rotation.z = q[3]
        tf.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(tf)

        # ---- Debug ----
        print(f"📡 GyroZ: {omega_imu:.3f} rad/s | Enc ω: {omega_enc:.3f} | Fused ω: {omega_fused:.3f}")
        print(f"📍 Pose: x={self.x:.3f}, y={self.y:.3f}, θ={math.degrees(self.theta):.2f}°\n")


# -------------------- MAIN --------------------
def main(args=None):
    rclpy.init(args=args)
    node = FusedOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node stopped by user")
    finally:
        if node.ser:
            for i in range(1, 5):
                node.ser.write(f"{i}:0\n".encode())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
