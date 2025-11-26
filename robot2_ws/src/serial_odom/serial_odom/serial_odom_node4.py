import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu, MagneticField
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile
import serial
import re
import math
import time
import numpy as np

# IMU library imports - supporting multiple IMU types
try:
    import smbus
    IMU_AVAILABLE = True
    print("SMBus available - IMU support enabled")
except ImportError as e:
    print(f"SMBus not found: {e}")
    IMU_AVAILABLE = False

# AHRS library import
try:
    from ahrs.filters import Madgwick
    AHRS_AVAILABLE = True
    print("AHRS library available - Madgwick filter enabled")
except ImportError as e:
    print(f"AHRS library not found: {e}")
    print("Install with: pip install AHRS")
    AHRS_AVAILABLE = False

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
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

    return (qx, qy, qz, qw)

def quaternion_to_euler(qw, qx, qy, qz):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def parse_line(line):
    """Parse position data from serial line"""
    match = re.match(r"x:\s*(-?\d+\.\d+)\s*m,\s*y:\s*(-?\d+\.\d+)\s*m", line)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    return None

class IMUReader:
    """Generic IMU reader that can handle different IMU types with magnetometer support"""
    
    def __init__(self, bus_num=1, address=0x68, mag_address=0x0C):
        self.bus = smbus.SMBus(bus_num)
        self.address = address
        self.mag_address = mag_address  # For MPU-9250/9255 magnetometer
        self.imu_type = None
        self.initialized = False
        self.has_magnetometer = False
        
        # Try to detect IMU type
        try:
            who_am_i = self.bus.read_byte_data(address, 0x75)
            print(f"WHO_AM_I: 0x{who_am_i:02X}")
            
            if who_am_i == 0x71:
                self.imu_type = "MPU-9250"
                self.has_magnetometer = True
            elif who_am_i == 0x70:
                self.imu_type = "MPU-9255"
                self.has_magnetometer = True
            elif who_am_i == 0x68:
                self.imu_type = "MPU-6050"
                self.has_magnetometer = False
            else:
                self.imu_type = f"Unknown_0x{who_am_i:02X}"
                self.has_magnetometer = False
            
            print(f"Detected IMU: {self.imu_type}")
            self.initialize_imu()
            
        except Exception as e:
            print(f"IMU detection failed: {e}")
            raise
    
    def initialize_imu(self):
        """Initialize the IMU with basic settings"""
        try:
            # Wake up the IMU (clear sleep bit)
            self.bus.write_byte_data(self.address, 0x6B, 0x00)
            time.sleep(0.1)
            
            # Set accelerometer range to ±2g
            self.bus.write_byte_data(self.address, 0x1C, 0x00)
            
            # Set gyroscope range to ±250°/s
            self.bus.write_byte_data(self.address, 0x1B, 0x00)
            
            # Set sample rate divider for higher rate (1kHz / (1 + 7) = 125Hz)
            self.bus.write_byte_data(self.address, 0x19, 0x07)
            
            # Configure digital low pass filter
            self.bus.write_byte_data(self.address, 0x1A, 0x03)
            
            # Initialize magnetometer if available
            if self.has_magnetometer:
                self.initialize_magnetometer()
            
            self.initialized = True
            print(f"IMU {self.imu_type} initialized successfully")
            
        except Exception as e:
            print(f"IMU initialization failed: {e}")
            raise
    
    def initialize_magnetometer(self):
        """Initialize magnetometer for MPU-9250/9255"""
        try:
            # Enable I2C bypass mode to access magnetometer
            self.bus.write_byte_data(self.address, 0x37, 0x02)
            time.sleep(0.01)
            
            # Set magnetometer to continuous measurement mode
            self.bus.write_byte_data(self.mag_address, 0x0A, 0x16)  # 16-bit, 100Hz
            time.sleep(0.01)
            
            print("Magnetometer initialized")
            
        except Exception as e:
            print(f"Magnetometer initialization failed: {e}")
            self.has_magnetometer = False
    
    def read_raw_data(self):
        """Read raw accelerometer, gyroscope, and magnetometer data"""
        if not self.initialized:
            return None
            
        try:
            # Read accelerometer data (registers 0x3B to 0x40)
            accel_data = self.bus.read_i2c_block_data(self.address, 0x3B, 6)
            ax = self.bytes_to_int16(accel_data[0], accel_data[1]) / 16384.0  # ±2g range
            ay = self.bytes_to_int16(accel_data[2], accel_data[3]) / 16384.0
            az = self.bytes_to_int16(accel_data[4], accel_data[5]) / 16384.0
            
            # Read gyroscope data (registers 0x43 to 0x48)
            gyro_data = self.bus.read_i2c_block_data(self.address, 0x43, 6)
            gx = self.bytes_to_int16(gyro_data[0], gyro_data[1]) / 131.0  # ±250°/s range
            gy = self.bytes_to_int16(gyro_data[2], gyro_data[3]) / 131.0
            gz = self.bytes_to_int16(gyro_data[4], gyro_data[5]) / 131.0
            
            # Read magnetometer data if available
            mx, my, mz = 0.0, 0.0, 0.0
            if self.has_magnetometer:
                try:
                    # Check if magnetometer data is ready
                    status = self.bus.read_byte_data(self.mag_address, 0x02)
                    if status & 0x01:  # Data ready
                        mag_data = self.bus.read_i2c_block_data(self.mag_address, 0x03, 6)
                        mx = self.bytes_to_int16(mag_data[1], mag_data[0]) * 0.15  # uT
                        my = self.bytes_to_int16(mag_data[3], mag_data[2]) * 0.15
                        mz = self.bytes_to_int16(mag_data[5], mag_data[4]) * 0.15
                        
                        # Read ST2 register to complete the measurement
                        self.bus.read_byte_data(self.mag_address, 0x09)
                        
                except Exception as e:
                    # If magnetometer read fails, continue with zeros
                    mx, my, mz = 0.0, 0.0, 0.0
            
            return {
                'accel': (ax, ay, az),
                'gyro': (gx, gy, gz),
                'mag': (mx, my, mz),
                'temp': self.read_temperature()
            }
            
        except Exception as e:
            print(f"IMU read error: {e}")
            return None
    
    def read_temperature(self):
        """Read temperature from IMU"""
        try:
            temp_data = self.bus.read_i2c_block_data(self.address, 0x41, 2)
            temp_raw = self.bytes_to_int16(temp_data[0], temp_data[1])
            # Convert to Celsius
            temp_c = (temp_raw / 340.0) + 36.53
            return temp_c
        except:
            return 0.0
    
    def bytes_to_int16(self, high_byte, low_byte):
        """Convert two bytes to signed 16-bit integer"""
        value = (high_byte << 8) | low_byte
        if value > 32767:
            value -= 65536
        return value

class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_node')

        # Check if AHRS library is available
        if not AHRS_AVAILABLE:
            self.get_logger().error("AHRS library not available! Install with: pip install AHRS")
            raise SystemExit

        # Serial port setup
        port = '/dev/ttyACM0'
        baud = 115200

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port: {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            raise SystemExit

        # IMU setup
        if IMU_AVAILABLE:
            try:
                self.imu_reader = IMUReader(bus_num=1, address=0x68)
                self.imu_available = True
                
                # Initialize Madgwick filter
                self.madgwick = Madgwick(frequency=50.0, beta=0.1)  # 50Hz update rate, beta for convergence
                
                # Initial quaternion (identity)
                self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
                
                # Calibration parameters
                self.gyro_bias = [0.0, 0.0, 0.0]
                self.accel_bias = [0.0, 0.0, 0.0]
                self.mag_bias = [0.0, 0.0, 0.0]
                
                # Calibration flags
                self.calibrated = False
                
                self.get_logger().info(f"IMU initialized: {self.imu_reader.imu_type}")
                self.get_logger().info(f"Magnetometer available: {self.imu_reader.has_magnetometer}")
                
            except Exception as e:
                self.get_logger().error(f"Failed to initialize IMU: {e}")
                self.imu_available = False
        else:
            self.imu_available = False
            self.get_logger().warn("IMU library not available")

        # ROS2 publishers and subscribers
        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Twist,
            '/robot2/cmd_vel',
            self.cmd_vel_callback,
            qos)

        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        # State variables
        self.last_time = self.get_clock().now()
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0
        self.current_theta = 0.0  # This will come from Madgwick filter

        self.mode_sent = False
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 1.01
        self.cmd_sent = False
        
        # Setup static transforms
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

    def read_imu_data(self):
        """Read and process IMU data using Madgwick filter"""
        if not self.imu_available:
            return None, None, None, None

        try:
            data = self.imu_reader.read_raw_data()
            if data is None:
                return None, None, None, None
            
            accel = data['accel']
            gyro = data['gyro']
            mag = data['mag']
            
            # Apply calibration
            accel_cal = [a - b for a, b in zip(accel, self.accel_bias)]
            gyro_cal = [math.radians(g - b) for g, b in zip(gyro, self.gyro_bias)]  # Convert to rad/s
            mag_cal = [m - b for m, b in zip(mag, self.mag_bias)]
            
            # Convert to numpy arrays for Madgwick filter
            acc = np.array(accel_cal)
            gyr = np.array(gyro_cal)
            
            # Use magnetometer if available and has valid data
            if self.imu_reader.has_magnetometer and any(abs(m) > 0.1 for m in mag_cal):
                magn = np.array(mag_cal)
                # Update Madgwick filter with all sensors
                self.quaternion = self.madgwick.updateMARG(self.quaternion, gyr, acc, magn)
            else:
                # Update Madgwick filter with only gyro and accelerometer
                self.quaternion = self.madgwick.updateIMU(self.quaternion, gyr, acc)
            
            # Convert quaternion to Euler angles
            # Madgwick quaternion format: [w, x, y, z]
            qw, qx, qy, qz = self.quaternion
            roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)
            
            return roll, pitch, yaw, data
            
        except Exception as e:
            self.get_logger().warn(f"IMU read error: {e}")
            return None, None, None, None

    def publish_imu_data(self, roll, pitch, yaw, raw_data, timestamp):
        """Publish IMU data"""
        if not self.imu_available or roll is None or raw_data is None:
            return
            
        try:
            accel = raw_data['accel']
            gyro = raw_data['gyro']
            mag = raw_data['mag']
            
            # Publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp.to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Orientation (quaternion from Madgwick filter)
            qw, qx, qy, qz = self.quaternion
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            
            # Angular velocity (rad/s) - apply bias correction
            gx, gy, gz = [math.radians(g - b) for g, b in zip(gyro, self.gyro_bias)]
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
            
            # Linear acceleration (m/s²) - apply bias correction
            ax, ay, az = [a - b for a, b in zip(accel, self.accel_bias)]
            imu_msg.linear_acceleration.x = ax * 9.81
            imu_msg.linear_acceleration.y = ay * 9.81
            imu_msg.linear_acceleration.z = az * 9.81
            
            # Covariance matrices (Madgwick provides better accuracy)
            imu_msg.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
            imu_msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            
            self.imu_pub.publish(imu_msg)
            
            # Publish magnetometer data if available
            if self.imu_reader.has_magnetometer:
                mag_msg = MagneticField()
                mag_msg.header.stamp = timestamp.to_msg()
                mag_msg.header.frame_id = "imu_link"
                
                mx, my, mz = [m - b for m, b in zip(mag, self.mag_bias)]
                mag_msg.magnetic_field.x = mx * 1e-6  # Convert uT to T
                mag_msg.magnetic_field.y = my * 1e-6
                mag_msg.magnetic_field.z = mz * 1e-6
                
                # Magnetometer covariance
                mag_msg.magnetic_field_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
                
                self.mag_pub.publish(mag_msg)
            
        except Exception as e:
            self.get_logger().warn(f"IMU publish error: {e}")

    def cmd_vel_callback(self, msg):
        if not self.mode_sent:
            self.ser.write(b"MODE:AUTO\n")
            self.get_logger().info("Sent MODE:AUTO to ESP32")
            self.mode_sent = True

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
        # Handle timeout stop
        now = self.get_clock().now()
        dt_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if self.cmd_sent and dt_cmd > self.cmd_timeout:
            try:
                self.ser.write(b"V:0.00,W:0.00\n")
                self.cmd_sent = False
                self.get_logger().info("No /cmd_vel - Sent stop command to ESP32")
            except Exception as e:
                self.get_logger().warn(f"Serial stop write error: {e}")

        # Read IMU data and update orientation using Madgwick filter
        roll, pitch, yaw, raw_data = self.read_imu_data()
        
        # Update current theta from Madgwick filter yaw
        if yaw is not None:
            self.current_theta = yaw
        
        # Publish IMU data
        self.publish_imu_data(roll, pitch, yaw, raw_data, now)

        # Read position from serial (wheel encoders)
        try:
            line = self.ser.readline().decode('utf-8').strip()
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        result = parse_line(line)
        if not result:
            return

        # Get position from wheel encoders
        x, y = result
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Calculate linear velocities from wheel encoder positions
        dx = x - self.prev_x
        dy = y - self.prev_y
        
        # Calculate angular velocity from Madgwick filter yaw change
        dtheta = self.current_theta - self.prev_theta

        # Handle angle wrap-around
        if dtheta > math.pi:
            dtheta -= 2 * math.pi
        elif dtheta < -math.pi:
            dtheta += 2 * math.pi

        # Calculate velocities
        vx = dx / dt if dt > 0 else 0.0
        vy = dy / dt if dt > 0 else 0.0
        vth = dtheta / dt if dt > 0 else 0.0

        # Update previous values
        self.prev_x = x
        self.prev_y = y
        self.prev_theta = self.current_theta

        # Get quaternion from Madgwick filter
        qw, qx, qy, qz = self.quaternion

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position from wheel encoders
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        
        # Orientation from Madgwick filter
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocities: linear from encoders, angular from Madgwick filter
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Improved covariance matrices (Madgwick provides better orientation accuracy)
        odom.pose.covariance = [0.05, 0, 0, 0, 0, 0,
                               0, 0.05, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.001]  # Much better yaw covariance with Madgwick
        
        odom.twist.covariance = [0.05, 0, 0, 0, 0, 0,
                                0, 0.05, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.001]

        self.odom_pub.publish(odom)

        # Publish transform: odom -> base_footprint
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_footprint"

        # Position from wheel encoders
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        
        # Orientation from Madgwick filter
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_static_transforms(self):
        """Publish all static transforms at once"""
        current_time = self.get_clock().now().to_msg()
        transforms = []

        # Static transform: base_footprint -> base_link
        tf_base_footprint_to_base_link = TransformStamped()
        tf_base_footprint_to_base_link.header.stamp = current_time
        tf_base_footprint_to_base_link.header.frame_id = 'base_footprint'
        tf_base_footprint_to_base_link.child_frame_id = 'base_link'
        tf_base_footprint_to_base_link.transform.translation.x = 0.0
        tf_base_footprint_to_base_link.transform.translation.y = 0.0
        tf_base_footprint_to_base_link.transform.translation.z = 0.0
        tf_base_footprint_to_base_link.transform.rotation.x = 0.0
        tf_base_footprint_to_base_link.transform.rotation.y = 0.0
        tf_base_footprint_to_base_link.transform.rotation.z = 0.0
        tf_base_footprint_to_base_link.transform.rotation.w = 1.0
        transforms.append(tf_base_footprint_to_base_link)

        # Static transform: base_link -> laser_frame
        tf_base_link_to_laser = TransformStamped()
        tf_base_link_to_laser.header.stamp = current_time
        tf_base_link_to_laser.header.frame_id = 'base_link'
        tf_base_link_to_laser.child_frame_id = 'laser_frame'
        tf_base_link_to_laser.transform.translation.x = 0.0
        tf_base_link_to_laser.transform.translation.y = 0.0
        tf_base_link_to_laser.transform.translation.z = 0.2
        tf_base_link_to_laser.transform.rotation.x = 0.0
        tf_base_link_to_laser.transform.rotation.y = 0.0
        tf_base_link_to_laser.transform.rotation.z = 0.0
        tf_base_link_to_laser.transform.rotation.w = 1.0
        transforms.append(tf_base_link_to_laser)

        # Static transform: base_link -> imu_link (if IMU is available)
        if self.imu_available:
            tf_base_link_to_imu = TransformStamped()
            tf_base_link_to_imu.header.stamp = current_time
            tf_base_link_to_imu.header.frame_id = 'base_link'
            tf_base_link_to_imu.child_frame_id = 'imu_link'
            tf_base_link_to_imu.transform.translation.x = 0.0
            tf_base_link_to_imu.transform.translation.y = 0.0
            tf_base_link_to_imu.transform.translation.z = 0.1
            tf_base_link_to_imu.transform.rotation.x = 0.0
            tf_base_link_to_imu.transform.rotation.y = 0.0
            tf_base_link_to_imu.transform.rotation.z = 0.0
            tf_base_link_to_imu.transform.rotation.w = 1.0
            transforms.append(tf_base_link_to_imu)

        # Send all static transforms
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published static transforms")

    def calibrate_imu(self, samples=200):
        """Calibrate IMU sensors (call this method to calibrate sensor biases)"""
        if not self.imu_available:
            self.get_logger().warn("IMU not available for calibration")
            return
            
        self.get_logger().info(f"Calibrating IMU with {samples} samples...")
        self.get_logger().info("Keep the robot stationary during calibration...")
        
        gyro_sum = [0.0, 0.0, 0.0]
        accel_sum = [0.0, 0.0, 0.0]
        mag_sum = [0.0, 0.0, 0.0]
        valid_samples = 0
        
        for i in range(samples):
            try:
                data = self.imu_reader.read_raw_data()
                if data is not None:
                    gyro = data['gyro']
                    accel = data['accel']
                    mag = data['mag']
                    
                    for j in range(3):
                        gyro_sum[j] += gyro[j]
                        accel_sum[j] += accel[j]
                        mag_sum[j] += mag[j]
                    
                    valid_samples += 1
                    
                    if i % 200 == 0:
                        self.get_logger().info(f"Calibration progress: {i}/{samples}")
                        
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().warn(f"Calibration sample {i} failed: {e}")
                
        if valid_samples > 0:
            self.gyro_bias = [s / valid_samples for s in gyro_sum]
            self.accel_bias = [s / valid_samples for s in accel_sum]
            self.mag_bias = [s / valid_samples for s in mag_sum]
            
            # For accelerometer, we expect ~1g on Z-axis when stationary, so adjust Z bias
            self.accel_bias[2] -= 1.0  # Remove 1g from Z-axis bias
            
            self.calibrated = True
            
            self.get_logger().info(f"Calibration complete with {valid_samples} samples")
            self.get_logger().info(f"Gyroscope bias: {[f'{x:.4f}' for x in self.gyro_bias]} deg/s")
            self.get_logger().info(f"Accelerometer bias: {[f'{x:.4f}' for x in self.accel_bias]} g")
            if self.imu_reader.has_magnetometer:
                self.get_logger().info(f"Magnetometer bias: {[f'{x:.4f}' for x in self.mag_bias]} uT")
        else:
            self.get_logger().error("Calibration failed - no valid samples")

    def reset_madgwick_filter(self):
        """Reset the Madgwick filter to initial state"""
        if self.imu_available and AHRS_AVAILABLE:
            self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Reset to identity quaternion
            self.madgwick = Madgwick(frequency=50.0, beta=0.1)  # Reinitialize filter
            self.get_logger().info("Madgwick filter reset")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialOdomNode()
        
        # Uncomment the next line to calibrate IMU on startup
        node.calibrate_imu()
        
        # Uncomment to reset Madgwick filter (useful if orientation seems wrong)
        node.reset_madgwick_filter()
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node startup failed: {e}")
    finally:
        try:
            if 'node' in locals() and hasattr(node, 'ser') and node.ser.is_open:
                node.ser.close()
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()