import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile

import serial
import re
import math
import time
import numpy as np
import smbus
from ahrs.filters import Madgwick


def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quaternion_to_euler(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def parse_line(line):
    match = re.match(r"x:\s*(-?\d+\.\d+)\s*m,\s*y:\s*(-?\d+\.\d+)\s*m", line)
    return (float(match.group(1)), float(match.group(2))) if match else None


class IMUReader:
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.address, 0x1C, 0x00)
        self.bus.write_byte_data(self.address, 0x1B, 0x00)

    def read_raw_data(self):
        accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
        ax = self.bytes_to_int16(accel_data[0], accel_data[1]) / 16384.0
        ay = self.bytes_to_int16(accel_data[2], accel_data[3]) / 16384.0
        az = self.bytes_to_int16(accel_data[4], accel_data[5]) / 16384.0
        gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
        gx = self.bytes_to_int16(gyro_data[0], gyro_data[1]) / 131.0
        gy = self.bytes_to_int16(gyro_data[2], gyro_data[3]) / 131.0
        gz = self.bytes_to_int16(gyro_data[4], gyro_data[5]) / 131.0
        return {'accel': (ax, ay, az), 'gyro': (gx, gy, gz)}

    def bytes_to_int16(self, high, low):
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.get_logger().info("Serial connected")
        
        self.ser.write(b"RESET_ODOM\n")
        self.get_logger().info("Sent RESET_ODOM to ESP32")

        self.imu_reader = IMUReader()
        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10))
        self.imu_pub = self.create_publisher(Imu, 'imu/data', QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.create_subscription(Twist, '/robot1/cmd_vel', self.cmd_vel_callback, QoSProfile(depth=10))

        self.publish_transforms()
        self.create_timer(1.0, self.publish_map_to_odom)
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.madgwick = Madgwick(frequency=50.0)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        self.calibrate_imu()

        self.last_time = self.get_clock().now()

        # ==== NEW: origin and previous values for relative odom ====
        self.origin_x = None
        self.origin_y = None
        self.origin_theta = None
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.current_theta = 0.0

        self.mode_sent = False
        self.cmd_sent = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.0

    def calibrate_imu(self, samples=500):
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        for _ in range(samples):
            data = self.imu_reader.read_raw_data()
            for i in range(3):
                gyro_sum[i] += data['gyro'][i]
                accel_sum[i] += data['accel'][i]
            time.sleep(0.005)
        self.gyro_bias = [x / samples for x in gyro_sum]
        self.accel_bias = [x / samples for x in accel_sum]
        self.accel_bias[2] -= 1.0
        self.get_logger().info("IMU calibration done")

    def cmd_vel_callback(self, msg):
        if not self.mode_sent:
            self.ser.write(b"MODE:AUTO\n")
            self.mode_sent = True
            self.get_logger().info("Sent MODE:AUTO to ESP32")

        linear = msg.linear.x
        angular = msg.angular.z
        command = f"V:{linear:.2f},W:{angular:.2f}\n"
        try:
            self.ser.write(command.encode())
            self.cmd_sent = True
            self.last_cmd_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if self.cmd_sent and (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            try:
                self.ser.write(b"V:0.00,W:0.00\n")
                self.cmd_sent = False
                self.get_logger().info("Sent stop due to cmd_vel timeout")
            except Exception as e:
                self.get_logger().warn(f"Serial stop error: {e}")

        try:
            line = self.ser.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received line: {line}")

            pos = parse_line(line)
            if not pos:
                return
            x_abs, y_abs = pos
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        # IMU
        raw = self.imu_reader.read_raw_data()
        acc = np.array([raw['accel'][i] - self.accel_bias[i] for i in range(3)])
        gyr = np.array([math.radians(raw['gyro'][i] - self.gyro_bias[i]) for i in range(3)])
        self.quaternion = self.madgwick.updateIMU(self.quaternion, gyr, acc)
        qw, qx, qy, qz = self.quaternion
        roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)
        self.current_theta = -yaw

        # ==== NEW: initialize origin on first reading ====
        if self.origin_x is None:
            self.origin_x = x_abs
            self.origin_y = y_abs
            self.origin_theta = self.current_theta
            self.prev_x = 0.0
            self.prev_y = 0.0
            self.prev_theta = 0.0

        # Relative position
        x = x_abs - self.origin_x
        y = y_abs - self.origin_y
        theta = self.current_theta - self.origin_theta

        # Wrap theta to [-pi, pi]
        if theta > math.pi: theta -= 2*math.pi
        if theta < -math.pi: theta += 2*math.pi

        # Velocities
        dx = x - self.prev_x
        dy = y - self.prev_y
        dtheta = theta - self.prev_theta
        if dtheta > math.pi: dtheta -= 2*math.pi
        if dtheta < -math.pi: dtheta += 2*math.pi

        vx = dx / dt if dt > 0 else 0.0
        vy = dy / dt if dt > 0 else 0.0
        vth = dtheta / dt if dt > 0 else 0.0

        self.prev_x = x
        self.prev_y = y
        self.prev_theta = theta

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        # Publish IMU
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu_link'
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw
        self.imu_pub.publish(imu)

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)
        
        
    # ===== STATIC TRANSFORMS =====
    def publish_transforms(self):
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

        self.static_tf_broadcaster.sendTransform([ tf1, tf2, tf3])

    # ===== DYNAMIC TRANSFORM: map -> odom =====
    def publish_map_to_odom(self):
        tf4 = TransformStamped()
        tf4.header.stamp = self.get_clock().now().to_msg()
        tf4.header.frame_id = 'map'
        tf4.child_frame_id = 'odom'
        tf4.transform.translation.x = 0.0
        tf4.transform.translation.y = 0.0
        tf4.transform.translation.z = 0.0
        tf4.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf4)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
