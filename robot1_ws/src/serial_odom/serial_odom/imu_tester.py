import rclpy
from rclpy.node import Node
import smbus
import time
import math

class IMUReader:
    def __init__(self, bus_num=1, address=0x68):   #0x68
        self.bus = smbus.SMBus(bus_num)
        self.address = address

        # Wake up MPU6050
        self.bus.write_byte_data(self.address, 0x6B, 0x00)  # Power management: wake up
        time.sleep(0.1)

        # Set sample rate = 1 kHz / (1 + 7) = 125 Hz
        self.bus.write_byte_data(self.address, 0x19, 7)

        # Set DLPF to 42Hz for accel & gyro
        self.bus.write_byte_data(self.address, 0x1A, 0x03)

        # Gyro full scale ±250°/s
        self.bus.write_byte_data(self.address, 0x1B, 0x00)

        # Accel full scale ±2g
        self.bus.write_byte_data(self.address, 0x1C, 0x00)

        time.sleep(0.1)


    def bytes_to_int16(self, high, low):
        value = (high << 8) | low
        return value - 65536 if value > 32767 else value

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

class IMUDebugNode(Node):
    def __init__(self):
        super().__init__('imu_debug_node')
        self.imu = IMUReader()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        data = self.imu.read_raw_data()
        ax, ay, az = data['accel']
        gx, gy, gz = data['gyro']
        self.get_logger().info(
            f"Accel[g]: ax={ax:.3f}, ay={ay:.3f}, az={az:.3f} | "
            f"Gyro[deg/s]: gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = IMUDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
