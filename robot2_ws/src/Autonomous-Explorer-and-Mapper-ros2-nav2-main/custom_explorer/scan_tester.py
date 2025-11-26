import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ✅ CONFIG: Adjust these depending on how your LIDAR is mounted
ROTATION_OFFSET_DEG = 90  # 0, 90, 180, -90 etc.
FLIP_LEFT_RIGHT = True    # Set True if L/R are swapped but F/B are correct

ROTATION_OFFSET_RAD = math.radians(ROTATION_OFFSET_DEG)

class LidarFullDebugger(Node):
    def __init__(self):
        super().__init__('lidar_full_debugger')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot2/scan',
            self.scan_callback,
            qos_profile=sensor_qos
        )

        self.last_print_time = 0.0
        self.print_interval = 1.0  # Print every 1 second

        self.printed_info = False
        self.get_logger().info("🔧 LIDAR Full Debugger Node Started")

    def scan_callback(self, msg):
        now = time.time()
        if not self.printed_info:
            self.get_logger().info(
                f"angle_min={msg.angle_min:.2f}, angle_max={msg.angle_max:.2f}, "
                f"angle_increment={msg.angle_increment:.4f}, total_points={len(msg.ranges)}"
            )
            self.printed_info = True

        # Convert ranges -> numpy for easy manipulation
        ranges = np.array([r if r > 0.01 else np.inf for r in msg.ranges])

        # ✅ Apply rotation correction
        shift = int(ROTATION_OFFSET_RAD / msg.angle_increment)
        ranges = np.roll(ranges, -shift)

        # ✅ Optionally flip L/R (mirror horizontally)
        if FLIP_LEFT_RIGHT:
            ranges = np.flip(ranges)

        # Find closest obstacle
        min_idx = np.argmin(ranges)
        min_angle = msg.angle_min + min_idx * msg.angle_increment - ROTATION_OFFSET_RAD
        min_dist = ranges[min_idx]

        if now - self.last_print_time >= self.print_interval:
            self.last_print_time = now
            self.get_logger().info(
                f"Closest obstacle: idx={min_idx}, angle={min_angle:.2f} rad, dist={min_dist:.2f} m"
            )

            # Compute sectors
            n = len(ranges)
            front = np.min(np.concatenate([ranges[0:10], ranges[-10:]]))
            left  = np.min(ranges[n//4 - 5 : n//4 + 5])
            right = np.min(ranges[-n//4 - 5 : -n//4 + 5])
            back  = np.min(ranges[n//2 - 5 : n//2 + 5])

            self.get_logger().info(
                f"📊 Front: {front:.2f} | Left: {left:.2f} | Right: {right:.2f} | Back: {back:.2f}"
            )

            # Obstacle warnings
            if front < 0.5:
                self.get_logger().info(f"🚧 Obstacle FRONT ({front:.2f} m)")
            if left < 0.5:
                self.get_logger().info(f"🚧 Obstacle LEFT ({left:.2f} m)")
            if right < 0.5:
                self.get_logger().info(f"🚧 Obstacle RIGHT ({right:.2f} m)")
            if back < 0.5:
                self.get_logger().info(f"🚧 Obstacle BACK ({back:.2f} m)")

def main(args=None):
    rclpy.init(args=args)
    node = LidarFullDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    