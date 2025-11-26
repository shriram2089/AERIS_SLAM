#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math


class LidarSideDetectorTF(Node):
    def __init__(self):
        super().__init__('lidar_side_detector_tf')

        # --- QoS (Best Effort same as LiDAR) ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --- TF setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            qos_profile
        )

        self.get_logger().info("🟢 TF-aware Lidar side detector started (Best Effort QoS)")

    def scan_callback(self, msg: LaserScan):
        # ---- Lookup TF transform ----
        try:
            # Correct direction: get transform laser_frame -> base_link
            trans = self.tf_buffer.lookup_transform(
                msg.header.frame_id, 'base_link', rclpy.time.Time())
        except Exception as e:
            now = self.get_clock().now().nanoseconds / 1e9
            if not hasattr(self, "_last_tf_warn") or now - self._last_tf_warn > 5.0:
                self.get_logger().warn(f"No transform yet: {e}")
                self._last_tf_warn = now
            return

        # ---- Clean invalid range data ----
        ranges = np.array(msg.ranges, dtype=float)
        ranges[np.isnan(ranges)] = msg.range_max
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[ranges <= 0.0] = msg.range_max

        n = len(ranges)
        if n == 0:
            return

        # ---- Compute angles for each reading ----
        angles = msg.angle_min + np.arange(n) * msg.angle_increment

        # ---- Extract yaw from TF quaternion (laser -> base) ----
        q = trans.transform.rotation
        yaw_offset = self.quaternion_to_yaw(q)

        # ---- Transform scan into robot (base_link) frame ----
        angles_robot = angles - yaw_offset + math.pi  # subtract (since lookup was laser->base)
        

        # ---- Define angular sectors ----
        front_idx = np.where((np.abs(angles_robot) < math.pi / 4))[0]
        right_idx = np.where((angles_robot < -math.pi / 4) & (angles_robot > -3 * math.pi / 4))[0]
        back_idx = np.where((np.abs(np.abs(angles_robot) - math.pi) < math.pi / 4))[0]
        left_idx = np.where((angles_robot > math.pi / 4) & (angles_robot < 3 * math.pi / 4))[0]

        # ---- Safe minimum helper ----
        def safe_min(idxs):
            return np.min(ranges[idxs]) if len(idxs) > 0 else msg.range_max

        front_min, right_min, back_min, left_min = map(
            safe_min, [front_idx, right_idx, back_idx, left_idx]
        )

        sides = {
            'Front': front_min,
            'Right': right_min,
            'Back': back_min,
            'Left': left_min,
        }

        # ---- Find closest side ----
        closest_side = min(sides, key=sides.get)
        closest_dist = sides[closest_side]

        if closest_dist < msg.range_max - 0.05:
            self.get_logger().info(f"📡 Closest obstacle: {closest_side} ({closest_dist:.2f} m)")
        else:
            self.get_logger().info("🌌 No obstacle nearby")

    # ---- Utility: quaternion to yaw ----
    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSideDetectorTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
