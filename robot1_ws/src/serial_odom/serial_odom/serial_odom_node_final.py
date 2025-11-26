#!/usr/bin/env python3
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
from transforms3d.euler import euler2quat


# -------------------------
# Utilities
# -------------------------
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


# New parse_line expects Arduino to print:
# ds: <value> m, dtheta: <value> rad
def parse_line(line):
    match = re.match(r"ds:\s*(-?\d+(?:\.\d+)?)\s*m,\s*dtheta:\s*(-?\d+(?:\.\d+)?)\s*rad", line)
    return (float(match.group(1)), float(match.group(2))) if match else None


# -------------------------
# IMU Reader (unchanged)
# -------------------------
class IMUReader:
    def __init__(self, bus_num=1, address=0x68):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        # wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        time.sleep(0.1)
        # accel/gyro config default
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


# -------------------------
# Main Node
# -------------------------
class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')

        # Serial to Arduino (adjust port if necessary)
        #self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.ser = serial.Serial('/dev/esp', 115200, timeout=1)
        
        time.sleep(1.0)
        # reset odom on esp
        try:
            self.ser.write(b"RESET:ODOM\n")
        except Exception:
            pass
        self.get_logger().info("Serial connected (reset odom sent)")

        # IMU reader + Madgwick
        self.imu_reader = IMUReader()
        self.madgwick = Madgwick(frequency=50.0)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]
        self.calibrate_imu()

        # Publishers and TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', QoSProfile(depth=10))
        self.imu_pub = self.create_publisher(Imu, 'imu/data', QoSProfile(depth=10))
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # cmd_vel subscription -> forwarded to Arduino (unchanged)
        self.create_subscription(Twist, '/robot1/cmd_vel', self.cmd_vel_callback, QoSProfile(depth=10))

        # static TFs
        self.publish_transforms()
        self.create_timer(1.0, self.publish_map_to_odom)

        # timer for reading serial + IMU and publishing odom
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        # odometry state (now integrated in ROS side from ds/dtheta)
        self.x = 0.0
        self.y = 0.0
        self.theta_enc = 0.0    # integrated from encoder dtheta (for fusion)
        self.theta_imu = 0.0    # from Madgwick
        self.theta_fused = 0.0

        # For velocity calculations
        self.last_time = self.get_clock().now()
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        # complementary filter alpha (tune as needed: higher => trust encoders more)
        self.alpha = 0.98

        # serial input buffer debug
        self.get_logger().info("SerialOdomNode initialised")

        # command send flags (unchanged behaviour)
        self.mode_sent = False
        self.cmd_sent = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.0

    def calibrate_imu(self, samples=300):
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        for _ in range(samples):
            data = self.imu_reader.read_raw_data()
            for i in range(3):
                gyro_sum[i] += data['gyro'][i]
                accel_sum[i] += data['accel'][i]
            time.sleep(0.002)
        self.gyro_bias = [x / samples for x in gyro_sum]
        self.accel_bias = [x / samples for x in accel_sum]
        # assume gravity on z
        self.accel_bias[2] -= 1.0
        self.get_logger().info("IMU calibration done")

    def cmd_vel_callback(self, msg):
        linear = -msg.linear.x
        angular = msg.angular.z

        # ensure Arduino is in AUTO once we send first command
        if not self.mode_sent:
            try:
                self.ser.write(b"MODE:AUTO\n")
                self.mode_sent = True
                self.get_logger().info("Sent MODE:AUTO to ESP32")
            except Exception as e:
                self.get_logger().warn(f"Failed to send MODE:AUTO: {e}")

        # forward command to Arduino
        command = f"V:{linear:.2f},W:{angular:.2f}\n"
        try:
            self.ser.write(command.encode())
            self.cmd_sent = True
            self.last_cmd_time = self.get_clock().now()
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def timer_callback(self):
        
        ds_scale_yaw      = 0.3     # lower if translation too large
        ds_scale_trans = 0.06
        gyro_scale    = 1.0     # lower if rotation too large
        alpha_yaw     = 0.9     # 0.9–0.95 usually stable



        # timing
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9 if self.last_time is not None else 0.02
        self.last_time = now

        # stop command timeout (unchanged)
        if self.cmd_sent and (now - self.last_cmd_time).nanoseconds / 1e9 > self.cmd_timeout:
            try:
                self.ser.write(b"V:0.00,W:0.00\n")
                self.cmd_sent = False
                self.get_logger().info("Sent stop due to cmd_vel timeout")
            except Exception as e:
                self.get_logger().warn(f"Serial stop error: {e}")

        # ---------------------------
        # Read incremental motion from Arduino (ds, dtheta)
        # ---------------------------
        try:
            line = self.ser.readline().decode('utf-8').strip()
            motion = parse_line(line)
            if motion is None:
                # nothing to read or format mismatch; continue but still update IMU
                motion = None
            else:
                ds, dtheta = motion
                ds *= ds_scale_trans
                dtheta *= -ds_scale_yaw  # if Arduino sends dtheta too large
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            motion = None

        # ---------------------------
        # Read IMU and update Madgwick
        # ---------------------------
        try:
            raw = self.imu_reader.read_raw_data()
            acc = np.array([raw['accel'][i] - self.accel_bias[i] for i in range(3)])
            # gyr = np.array([math.radians(raw['gyro'][i] - self.gyro_bias[i]) for i in range(3)])
            
            
            scale_factor = 3.0  # experimentally determined (you rotated 90° but got ~1°)
        
            gyr = np.array([
                math.radians(raw['gyro'][0] - self.gyro_bias[0]) * scale_factor,
                math.radians(raw['gyro'][1] - self.gyro_bias[1]) * scale_factor,
                -math.radians(raw['gyro'][2] - self.gyro_bias[2]) * scale_factor
            ])
            
        
            # optionally apply small scale factor tune (avoid huge factors)
            # self.quaternion = self.madgwick.updateIMU(self.quaternion, gyr, acc)
            self.quaternion = self.madgwick.updateIMU(self.quaternion, gyr, acc)
            qw, qx, qy, qz = self.quaternion
            _, _, yaw = quaternion_to_euler(qw, qx, qy, qz)
            self.theta_imu = yaw
        except Exception as e:
            # IMU read failed — keep last IMU yaw
            self.get_logger().warn(f"IMU read/update error: {e}")
            yaw = self.theta_imu

        # ---------------------------
        # Integrate encoder increments (if any) and fuse with IMU
        # ---------------------------
        if motion is not None:
            # encoder-based heading integration
            self.theta_enc += dtheta

            # complementary fusion of heading
            # fused = alpha * enc + (1-alpha) * imu
            # self.theta_fused = (self.alpha * self.theta_enc) + ((1.0 - self.alpha) * self.theta_imu)
            self.theta_fused = self.theta_imu
            

            # integrate position using fused heading
            self.x -= ds * math.cos(self.theta_fused)
            self.y -= ds * math.sin(self.theta_fused)
        else:
            # no encoder increment; still blend theta_enc gently to follow imu to avoid divergence
            # small correction step to prevent runaway if encoders stop sending
            # self.theta_fused = (self.alpha * self.theta_enc) + ((1.0 - self.alpha) * self.theta_imu)
            self.theta_fused = self.theta_imu
            
            # do not change x,y if no ds

        # normalize fused theta
        if self.theta_fused > math.pi:
            self.theta_fused -= 2 * math.pi
        elif self.theta_fused < -math.pi:
            self.theta_fused += 2 * math.pi

        # estimate velocities
        vx = (self.x - self.prev_x) / dt if dt > 0 else 0.0
        vy = (self.y - self.prev_y) / dt if dt > 0 else 0.0
        vth = (self.theta_fused - self.prev_theta) / dt if dt > 0 else 0.0

        # save prevs
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta_fused

        # ---------------------------
        # Publish Odometry
        # ---------------------------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx_, qy_, qz_, qw_ = euler_to_quaternion(0.0, 0.0,self.theta_fused)
        odom.pose.pose.orientation.x = qx_
        odom.pose.pose.orientation.y = qy_
        odom.pose.pose.orientation.z = qz_
        odom.pose.pose.orientation.w = qw_

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        # publish IMU message using Madgwick quaternion
        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.imu_pub.publish(imu_msg)

        # broadcast TF odom -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = qx_
        tf_msg.transform.rotation.y = qy_
        tf_msg.transform.rotation.z = qz_
        tf_msg.transform.rotation.w = qw_
        
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

   
        yaw = math.pi / 2   # Example: facing left

        # ---- Compute quaternion ----
        qw, qx, qy, qz = euler2quat(0.0, 0.0, yaw)  # roll=0, pitch=0, yaw chosen above

        # ---- Assign to TF ----
        tf2.transform.rotation.w = qw
        tf2.transform.rotation.x = qx
        tf2.transform.rotation.y = qy
        tf2.transform.rotation.z = qz

        tf3 = TransformStamped()
        tf3.header.stamp = now
        tf3.header.frame_id = 'base_link'
        tf3.child_frame_id = 'imu_link'
        tf3.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([tf1, tf2, tf3])

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
        # self.tf_broadcaster.sendTransform(tf4)


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



