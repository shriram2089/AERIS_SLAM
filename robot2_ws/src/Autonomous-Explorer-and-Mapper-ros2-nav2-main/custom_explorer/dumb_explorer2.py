import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ROTATION_OFFSET_DEG = 90
ROTATION_OFFSET_RAD = math.radians(ROTATION_OFFSET_DEG)


class MapBasedExplorer(Node):
    def __init__(self):
        super().__init__('map_based_explorer')
        self.get_logger().info("🗺️ Smarter Map + LIDAR Explorer Started")

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/robot2/map', self.map_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/robot2/odom', self.odom_callback, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.laser_sub = self.create_subscription(
            LaserScan, '/robot2/scan', self.laser_callback, qos_profile=sensor_qos)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # State
        self.state = "FORWARD"
        self.turn_end_time = 0.0
        self.backoff_end_time = 0.0
        self.turn_direction = 1.0

        # Data
        self.map_data = None
        self.scan_data = None
        self.last_unexplored = None

        # Odometry tracking
        self.last_odom = None
        self.last_progress_time = time.time()

    # === Callbacks ===
    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.last_odom = msg.pose.pose.position

    def laser_callback(self, msg):
        ranges = np.array([r if r > 0.01 else np.inf for r in msg.ranges])
        shift = int(ROTATION_OFFSET_RAD / msg.angle_increment)
        ranges = np.roll(ranges, -shift)
        self.scan_data = (ranges, msg)

    # === Main Control ===
    def control_loop(self):
        if self.scan_data is None or self.map_data is None or self.last_odom is None:
            return

        twist = Twist()
        ranges, scan_msg = self.scan_data
        n = len(ranges)

        # === Obstacle Detection ===
        front = np.min(np.concatenate([ranges[0:10], ranges[-10:]]))
        left = np.min(ranges[n//4 - 5 : n//4 + 5])
        right = np.min(ranges[-n//4 - 5 : -n//4 + 5])
        obstacle_threshold = 0.5

        # === Map Progress ===
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        unexplored_count = np.sum(map_array == -1)

        if self.last_unexplored is None:
            self.last_unexplored = unexplored_count

        # Periodic progress logging
        if time.time() - self.last_progress_time > 5:
            explored_pct = 100 * (1 - unexplored_count / max(1, self.last_unexplored))
            self.get_logger().info(f"🗺️ Map progress: {explored_pct:.2f}%")
            self.last_progress_time = time.time()

        # === Stuck Detection ===
        if unexplored_count < self.last_unexplored - 10:
            # progress made → reset timer
            self.last_progress_time = time.time()
            self.last_unexplored = unexplored_count
        else:
            # No progress for 3+ sec → consider stuck
            if time.time() - self.last_progress_time > 3.0 and self.state == "FORWARD":
                self.get_logger().warn("⚠️ No progress detected → Escape maneuver!")
                self.start_backoff()

        # === State Machine ===
        now = time.time()

        if unexplored_count < 10:
            self.get_logger().info("✅ Exploration complete. Stopping.")
            self.cmd_pub.publish(Twist())
            return

        if self.state == "FORWARD":
            if front < obstacle_threshold:
                self.get_logger().info(f"🚧 Obstacle ahead ({front:.2f} m)! Backing up.")
                self.start_backoff()
            else:
                twist.linear.x = 0.2

        elif self.state == "BACKOFF":
            if now < self.backoff_end_time:
                twist.linear.x = -0.15
            else:
                self.start_turn(left, right)

        elif self.state == "TURN":
            if now < self.turn_end_time:
                twist.angular.z = self.turn_direction * 1.0
            else:
                self.state = "FORWARD"
                self.get_logger().info("✅ Turn complete. Moving forward.")

        self.cmd_pub.publish(twist)

    # === Helper Functions ===
    def start_backoff(self):
        self.state = "BACKOFF"
        duration = random.uniform(1.0, 2.0)
        self.backoff_end_time = time.time() + duration
        self.get_logger().info(f"🔙 Backing up for {duration:.1f} sec")

    def start_turn(self, left, right):
        self.state = "TURN"
        turn_angle_sec = random.uniform(1.5, 2.5)
        self.turn_end_time = time.time() + turn_angle_sec

        if left > right:
            self.turn_direction = 1.0
        elif right > left:
            self.turn_direction = -1.0
        else:
            self.turn_direction = random.choice([1.0, -1.0])

        self.get_logger().info(
            f"↪️ Turning {'LEFT' if self.turn_direction > 0 else 'RIGHT'} for {turn_angle_sec:.1f} sec"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapBasedExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
