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

# -------------------- IMU Reader --------------------
class IMUReader:
    def __init__(self, bus_num=1, address=0x68, calib_time=2.0):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        # Wake up
        self.bus.write_byte_data(address, 0x6B, 0)
        self.bus.write_byte_data(address, 0x1B, 0)  # gyro 250 deg/s
        self.bus.write_byte_data(address, 0x1C, 0)  # accel 2g
        print("✅ MPU6050 connected. Calibrating gyro...")

        # ----- calibrate gyro Z -----
        start = time.time()
        samples = []
        while time.time() - start < calib_time:
            data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            gz = self.bytes_to_int16(data[4], data[5]) / 131.0
            samples.append(gz)
        self.gyro_bias_z = sum(samples) / len(samples)
        print(f"🛰️ Gyro Z bias: {self.gyro_bias_z:.3f} deg/s")
        print("✅ Calibration done!")

    def bytes_to_int16(self, high, low):
        val = (high << 8) | low
        return val - 65536 if val > 32767 else val

    def read_gyro_z(self):
        data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
        gz = self.bytes_to_int16(data[4], data[5]) / 131.0
        return gz - self.gyro_bias_z  # deg/s

# -------------------- Node --------------------
class OdomMotorNode(Node):
    def __init__(self):
        super().__init__('simple_odom_node')

        # Params
        self.declare_parameter("wheel_radius", 0.0366)
        self.declare_parameter("wheel_separation", 0.195)
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("update_rate", 20.0)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.dt = 1.0 / self.get_parameter("update_rate").value

        # Serial
        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baudrate").value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            print(f"✅ Serial connected: {port}")
        except:
            print(f"❌ Failed to connect to serial {port}")
            self.ser = None

        # IMU
        try:
            self.imu = IMUReader(bus_num=1, address=0x68)
        except Exception as e:
            print(f"❌ IMU init failed: {e}")
            self.imu = None

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # CMD_VEL
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # Publishers/subscribers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Timer
        self.create_timer(self.dt, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

        # Compute wheel RPMs
        v_left = self.cmd_linear - self.cmd_angular * self.wheel_separation / 2.0
        v_right = self.cmd_linear + self.cmd_angular * self.wheel_separation / 2.0
        rpm_left = int((v_left / (2*math.pi*self.wheel_radius)) * 60)
        rpm_right = int((v_right / (2*math.pi*self.wheel_radius)) * 60)

        if self.ser:
            self.ser.write(f"1:{rpm_left}\n".encode())
            self.ser.write(f"2:{rpm_left}\n".encode())
            self.ser.write(f"3:{rpm_right}\n".encode())
            self.ser.write(f"4:{rpm_right}\n".encode())

    def timer_callback(self):
        # Read gyro
        gz = self.imu.read_gyro_z() if self.imu else 0.0
        gz_rad = math.radians(gz)

        # Integrate heading
        self.theta += gz_rad * self.dt

        # Integrate forward using cmd_vel
        self.x += self.cmd_linear * math.cos(self.theta) * self.dt
        self.y += self.cmd_linear * math.sin(self.theta) * self.dt

        # Print debug
        print(f"🛰️ Gyro Z: {gz:.2f} deg/s -> {gz_rad:.3f} rad/s")
        print(f"📍 Pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {math.degrees(self.theta):.2f} deg")

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = euler2quat(0,0,self.theta)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]
        self.odom_pub.publish(odom)

        # Publish TF
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
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

def main(args=None):
    rclpy.init(args=args)
    node = OdomMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        if node.ser:
            for i in range(1,5):
                node.ser.write(f"{i}:0\n".encode())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
