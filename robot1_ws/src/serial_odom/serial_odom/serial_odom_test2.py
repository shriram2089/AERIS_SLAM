#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

class FixedOdomNode(Node):
    def __init__(self):
        super().__init__("fixed_odom_node")

        # Parameters
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
            self.get_logger().info(f"✅ Serial connected: {port}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial connection failed: {e}")
            self.ser = None

        # Pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Commanded velocities
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # Kalman filters
        self.kf_left = KalmanFilter()
        self.kf_right = KalmanFilter()

        # Subscriptions
        self.create_subscription(Twist, "/robot1/cmd_vel", self.cmd_vel_callback, 10)

        # Timer
        self.prev_time = self.get_clock().now()
        self.create_timer(self.dt, self.update_odom)

    def cmd_vel_callback(self, msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

        if self.ser:
            # Compute wheel linear velocities
            if abs(self.cmd_angular) > 1e-3 and abs(self.cmd_linear) < 1e-3:
                # Rotation only → invert left wheel
                v_left = -self.cmd_angular * self.wheel_separation / 2.0
                v_right = self.cmd_angular * self.wheel_separation / 2.0
            elif abs(self.cmd_linear) > 1e-3 and abs(self.cmd_angular) < 1e-3:
                # Linear only → both positive
                v_left = self.cmd_linear
                v_right = self.cmd_linear
            else:
                # Combined motion → leave as-is (or apply custom rule if needed)
                v_left = self.cmd_linear - self.cmd_angular * self.wheel_separation / 2.0
                v_right = self.cmd_linear + self.cmd_angular * self.wheel_separation / 2.0

            # Convert to RPM
            rpm_left = int((v_left / (2 * math.pi * self.wheel_radius)) * 60)  # use 60 for 1/s
            rpm_right = int((v_right / (2 * math.pi * self.wheel_radius)) * 60)

            # Send motor commands
            # Left motors physically inverted → send negative RPM
            self.send_motor_command(1, -rpm_left)   # Rear Left
            self.send_motor_command(2, -rpm_left)   # Front Left
            self.send_motor_command(3, rpm_right)   # Front Right
            self.send_motor_command(4, rpm_right)   # Rear Right

    def send_motor_command(self, motor_id, rpm):
        if self.ser is None:
            return
        try:
            cmd = f"{motor_id}:{rpm}\n"
            self.ser.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send to motor {motor_id}: {e}")

    def read_ddsm_feedback(self):
        rpm_left = None
        rpm_right = None
        if self.ser is None:
            return None, None
        while self.ser.in_waiting:
            try:
                line = self.ser.readline().decode("utf-8").strip()
                if not line:
                    continue
                parts = line.split()
                motor_id = int(parts[1])
                rpm_val = float(parts[4]) / 10.0
                if motor_id == 1:
                    rpm_left = rpm_val
                elif motor_id == 2:
                    rpm_right = rpm_val
            except Exception as e:
                self.get_logger().warn(f"Failed to parse line: {line} ({e})")
        return rpm_left, rpm_right

    def update_odom(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.prev_time = now

        rpm_left, rpm_right = self.read_ddsm_feedback()
        if rpm_left is None or rpm_right is None:
            return

        # Kalman filter
        rpm_left = self.kf_left.update(rpm_left)
        rpm_right = self.kf_right.update(rpm_right)

        # Convert RPM -> rad/s
        w_left = 2 * math.pi * rpm_left / 60.0
        w_right = 2 * math.pi * rpm_right / 60.0

        # Decide wheel directions based on commanded motion
        if abs(self.cmd_angular) > 1e-3 and abs(self.cmd_linear) < 1e-3:
            # Rotation-only
            v_left = -w_left * self.wheel_radius
            v_right = w_right * self.wheel_radius
        elif abs(self.cmd_linear) > 1e-3 and abs(self.cmd_angular) < 1e-3:
            # Linear-only
            v_left = w_left * self.wheel_radius
            v_right = w_right * self.wheel_radius
        else:
            # Combined motion
            v_left = w_left * self.wheel_radius
            v_right = w_right * self.wheel_radius

        # Robot velocities
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        # Update pose
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Debug
        print(f"📡 CMD -> linear: {self.cmd_linear:.3f} m/s, angular: {self.cmd_angular:.3f} rad/s")
        print(f"🛞 RPM -> left: {rpm_left:.2f}, right: {rpm_right:.2f}")
        print(f"🚀 Vel -> v_left: {v_left:.3f}, v_right: {v_right:.3f}, v: {v:.3f}, omega: {omega:.3f}")
        print(f"📍 Pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {math.degrees(self.theta):.2f} deg\n")

def main(args=None):
    rclpy.init(args=args)
    node = FixedOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Test stopped")
    finally:
        if node.ser:
            for i in range(1,5):
                node.ser.write(f"{i}:0\n".encode())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
