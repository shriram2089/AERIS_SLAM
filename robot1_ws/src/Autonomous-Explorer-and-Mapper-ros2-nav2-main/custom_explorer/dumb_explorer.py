import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ✅ Same as debugger
ROTATION_OFFSET_DEG = 90
ROTATION_OFFSET_RAD = math.radians(ROTATION_OFFSET_DEG)


class MapBasedExplorer(Node):
    def __init__(self):
        super().__init__('map_based_explorer')
        self.get_logger().info("🗺️ Map + LIDAR Explorer Node Started")

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/robot2/map', self.map_callback, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.laser_callback, qos_profile=sensor_qos)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Data holders
        self.map_data = None
        self.scan_data = None

        # Robot state
        self.state = "FORWARD"
        self.turn_duration = 0
        self.turn_timer = 0
        self.turn_direction = 1.0  # +1 left, -1 right

    def map_callback(self, msg):
        self.map_data = msg

    def laser_callback(self, msg):
        # ✅ Convert to numpy + filter invalid values
        ranges = np.array([r if r > 0.01 else np.inf for r in msg.ranges])
        # ✅ Apply rotation correction
        shift = int(ROTATION_OFFSET_RAD / msg.angle_increment)
        ranges = np.roll(ranges, -shift)
        self.scan_data = (ranges, msg)

    def control_loop(self):
        if self.scan_data is None or self.map_data is None:
            return

        ranges, scan_msg = self.scan_data
        n = len(ranges)

        twist = Twist()

        # === OBSTACLE DETECTION (after correction) ===
        front = np.min(np.concatenate([ranges[0:10], ranges[-10:]]))
        left  = np.min(ranges[n//4 - 5 : n//4 + 5])
        right = np.min(ranges[-n//4 - 5 : -n//4 + 5])
        back  = np.min(ranges[n//2 - 5 : n//2 + 5])

        obstacle_threshold = 0.6  # meters

        # === FRONTIER CHECK ===
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        unexplored_count = np.sum(map_array == -1)

        if unexplored_count < 10:
            self.get_logger().info("✅ Exploration Complete! Stopping robot.")
            self.cmd_pub.publish(Twist())
            return

        if self.state == "FORWARD":
            if front < obstacle_threshold:
                # Switch to turn mode
                self.state = "TURN"
                self.turn_duration = random.randint(15, 30)
                self.turn_timer = 0

                # Bias turning towards regions with more unknown space
                left_half = map_array[:, :map_array.shape[1]//2]
                right_half = map_array[:, map_array.shape[1]//2:]
                left_unknown = np.sum(left_half == -1)
                right_unknown = np.sum(right_half == -1)

                if left_unknown > right_unknown:
                    self.turn_direction = 1.0
                elif right_unknown > left_unknown:
                    self.turn_direction = -1.0
                else:
                    # fallback: use LIDAR – turn to side with more free space
                    self.turn_direction = 1.0 if left > right else -1.0

                self.get_logger().info(
                    f"🚧 Obstacle Ahead ({front:.2f} m)! Turning {'LEFT' if self.turn_direction > 0 else 'RIGHT'}")

            else:
                twist.linear.x = 0.2  # forward motion

        elif self.state == "TURN":
            if self.turn_timer < self.turn_duration:
                twist.angular.z = 1.0 * self.turn_direction
                self.turn_timer += 1
            else:
                self.state = "FORWARD"
                self.get_logger().info("↪️ Turn complete, moving forward again...")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MapBasedExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
