import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math
import collections
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ---------------- CONFIG ----------------
ROTATION_OFFSET_DEG = 90
FLIP_LEFT_RIGHT = False

OBSTACLE_THRESHOLD = 0.6     # meters considered "blocked"
FORWARD_SPEED = 0.18         # m/s
BACKUP_SPEED = -0.12         # m/s
TURN_SPEED = 0.6             # rad/s for small corrective turns
LARGE_TURN_SPEED = 1.0       # rad/s for recovery

SMALL_TURN_TICKS = (6, 14)   # ticks of timer (0.1s each) for small turns
LARGE_TURN_TICKS = (20, 40)  # ticks for recovery big turn
BACKUP_TICKS = 8             # ticks to back up during recovery (0.8s)

STUCK_HISTORY = 10           # how many front samples we keep
STUCK_DISTANCE_DELTA = 0.10  # required improvement (m) to consider progress
FORWARD_TRIAL_TICKS = 12     # after clearing, attempt forward for this many ticks before evaluating stuck
RECOVERY_COOLDOWN = 2.0      # seconds between recovery attempts

PRINT_INTERVAL = 1.0         # seconds
# ----------------------------------------

ROTATION_OFFSET_RAD = math.radians(ROTATION_OFFSET_DEG)


class DumbExplorerAdvancedFixed(Node):
    def __init__(self):
        super().__init__('dumb_explorer_adv_fixed')
        self.get_logger().info("🗺️ Advanced Dumb Explorer (fixed) Started")

        # publishers / subscribers
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/robot2/map', self.map_callback, 10)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.laser_sub = self.create_subscription(LaserScan, '/robot2/scan',
                                                  self.laser_callback, qos_profile=sensor_qos)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # state/data
        self.map_data = None
        self.scan_data = None         # tuple (ranges_np, scan_msg)
        self.state = "FORWARD"        # FORWARD / TURN / BACKUP
        self.turn_ticks_remaining = 0
        self.current_turn_speed = TURN_SPEED
        self.turn_direction = 1       # +1 left / -1 right

        self.front_history = collections.deque(maxlen=STUCK_HISTORY)
        self.last_forward_attempt_front = None
        self.forward_trial_ticks_remaining = 0
        self.last_recovery_time = 0.0
        self.recovery_attempts = 0

        self.last_print_time = 0.0

    def map_callback(self, msg):
        self.map_data = msg

    def laser_callback(self, msg):
        # sanitize ranges
        ranges = np.array([r if (r is not None and r > 0.01) else np.inf for r in msg.ranges])
        # rotation correction
        shift = int(ROTATION_OFFSET_RAD / msg.angle_increment)
        ranges = np.roll(ranges, -shift)
        if FLIP_LEFT_RIGHT:
            ranges = np.flip(ranges)
        self.scan_data = (ranges, msg)

    def find_frontier_count(self, map_msg):
        arr = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        h, w = arr.shape
        count = 0
        for i in range(1, h-1, 4):
            for j in range(1, w-1, 4):
                if arr[i, j] == -1:
                    n = (arr[i+1, j] == 0) or (arr[i-1, j] == 0) or (arr[i, j+1] == 0) or (arr[i, j-1] == 0)
                    if n:
                        count += 1
        return count

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def set_turn(self, direction, ticks, angular_speed=TURN_SPEED):
        self.state = "TURN"
        self.turn_direction = 1 if direction >= 0 else -1
        self.turn_ticks_remaining = ticks
        self.current_turn_speed = angular_speed
        self.get_logger().info(f"Starting turn: {'LEFT' if self.turn_direction>0 else 'RIGHT'} for {ticks} ticks (speed {angular_speed:.2f})")

    def start_recovery(self):
        now = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]*1e-9
        if now - self.last_recovery_time < RECOVERY_COOLDOWN:
            # cooldown - skip additional recovery
            self.get_logger().info("Recovery cooldown active, skipping duplicate recovery")
            return
        self.last_recovery_time = now
        self.recovery_attempts += 1
        self.state = "BACKUP"
        self.turn_ticks_remaining = BACKUP_TICKS
        self.current_turn_speed = 0.0
        self.get_logger().warn(">>> RECOVERY: backing up")

    def control_loop(self):
        # ensure we have both data
        if self.scan_data is None or self.map_data is None:
            return

        ranges, scan_msg = self.scan_data
        n = len(ranges)

        # define sectors robustly (using percentages)
        span = max(3, int(n * 0.10))
        center = n // 2
        front_slice = np.concatenate([ranges[center - span:center + span]])
        left_slice = ranges[int(n * 0.25) - span: int(n * 0.25) + span]
        right_slice = ranges[int(n * 0.75) - span: int(n * 0.75) + span]
        back_slice = ranges[int(n * 0.5) - span: int(n * 0.5) + span]

        front_min = np.nanmin(front_slice)
        left_min = np.nanmin(left_slice)
        right_min = np.nanmin(right_slice)
        back_min = np.nanmin(back_slice)

        # maintain history to detect stuck
        self.front_history.append(front_min)

        # occasional status print
        now_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1]*1e-9
        if now_time - self.last_print_time > PRINT_INTERVAL:
            self.last_print_time = now_time
            frontier_count = self.find_frontier_count(self.map_data)
            self.get_logger().info(f"Status | State={self.state} | Front={front_min:.2f} L={left_min:.2f} R={right_min:.2f} | frontiers~{frontier_count}")

        twist = Twist()

        # Behavior state machine
        if self.state == "FORWARD":
            if front_min > OBSTACLE_THRESHOLD:
                # go forward normally
                twist.linear.x = FORWARD_SPEED
                twist.angular.z = 0.0
                # start a forward trial window if not already
                if self.last_forward_attempt_front is None:
                    self.last_forward_attempt_front = front_min
                    self.forward_trial_ticks_remaining = FORWARD_TRIAL_TICKS
            else:
                # front blocked: choose small corrective turn
                direction = 1 if left_min > right_min else -1
                ticks = random.randint(*SMALL_TURN_TICKS)
                self.set_turn(direction, ticks, angular_speed=TURN_SPEED)
                self.last_forward_attempt_front = None
                self.forward_trial_ticks_remaining = 0

        elif self.state == "TURN":
            if self.turn_ticks_remaining > 0:
                twist.angular.z = self.current_turn_speed * self.turn_direction
                twist.linear.x = 0.0
                self.turn_ticks_remaining -= 1

                # If front cleared during turn, switch to forward and start trial
                if front_min > OBSTACLE_THRESHOLD + 0.03:
                    self.get_logger().info("Small turn cleared path -> moving forward (start trial)")
                    self.state = "FORWARD"
                    self.turn_ticks_remaining = 0
                    self.last_forward_attempt_front = front_min
                    self.forward_trial_ticks_remaining = FORWARD_TRIAL_TICKS
                    twist = Twist()
                    twist.linear.x = FORWARD_SPEED
            else:
                # finished nominal turn without clearing -> attempt forward for a trial
                self.state = "FORWARD"
                self.last_forward_attempt_front = front_min
                self.forward_trial_ticks_remaining = FORWARD_TRIAL_TICKS
                twist.linear.x = FORWARD_SPEED
                self.get_logger().info("Turn ended, trying forward (trial)")

        elif self.state == "BACKUP":
            if self.turn_ticks_remaining > 0:
                twist.linear.x = BACKUP_SPEED
                twist.angular.z = 0.0
                self.turn_ticks_remaining -= 1
            else:
                # after backup, start a larger recovery turn
                direction = random.choice([-1, 1])
                ticks = random.randint(*LARGE_TURN_TICKS)
                self.set_turn(direction, ticks, angular_speed=LARGE_TURN_SPEED)
                self.get_logger().warn("Recovery: starting large turn after backup")

        # If we are in a forward trial, decrement and only evaluate stuck when trial finishes
        if self.state == "FORWARD" and self.forward_trial_ticks_remaining > 0:
            self.forward_trial_ticks_remaining -= 1
            # only evaluate stuck when trial ended
            if self.forward_trial_ticks_remaining == 0 and self.last_forward_attempt_front is not None:
                # compare recent best front value to the value when trial started
                recent_best = max(self.front_history) if len(self.front_history) > 0 else self.last_forward_attempt_front
                # success if recent_best increased by STUCK_DISTANCE_DELTA
                if recent_best >= (self.last_forward_attempt_front + STUCK_DISTANCE_DELTA):
                    self.get_logger().info("Forward trial succeeded (progress detected)")
                    # reset forward attempt memory so future checks start fresh
                    self.last_forward_attempt_front = None
                else:
                    self.get_logger().warn("Forward trial failed (no progress). Initiating recovery.")
                    self.start_recovery()
                    self.last_forward_attempt_front = None

        # publish twist
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DumbExplorerAdvancedFixed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
