#!/usr/-bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from sensor_msgs.msg import LaserScan
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

import transforms3d.quaternions as tq
import tf2_ros
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# -------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------
def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle (radians) to Quaternion using transforms3d."""
    q = tq.axangle2quat([0, 0, 1], yaw)
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])


def safe_transform_pose(pose_stamped: PoseStamped, transform: TransformStamped) -> PoseStamped:
    """Safely apply a TF transform to a PoseStamped (robust version)."""
    t = transform.transform.translation
    q = transform.transform.rotation
    trans = np.array([t.x, t.y, t.z])
    quat = np.array([q.x, q.y, q.z, q.w])
    rot = tq.quat2mat([quat[3], quat[0], quat[1], quat[2]])  # w, x, y, z -> 3x3

    p = pose_stamped.pose.position
    pos = np.array([p.x, p.y, p.z])
    pos_new = rot.dot(pos) + trans

    out = PoseStamped()
    out.header.frame_id = transform.header.frame_id
    out.header.stamp = pose_stamped.header.stamp
    out.pose.position.x = float(pos_new[0])
    out.pose.position.y = float(pos_new[1])
    out.pose.position.z = float(pos_new[2])
    out.pose.orientation = pose_stamped.pose.orientation
    return out


# -------------------------------------------------------------
# Main Node
# -------------------------------------------------------------
class ObjectToNavGoal(Node):
    def __init__(self):
        super().__init__('object_to_nav_goal')

        # Parameters
        self.declare_parameter('yolo_topic', '/robot1/yolo_detections')
        self.declare_parameter('tf_base_frame', 'base_link')
        self.declare_parameter('tf_map_frame', 'map') # <-- Fixed from 'odom'
        self.declare_parameter('target_class', 'bottle')
        self.declare_parameter('stop_distance', 0.5)
        self.declare_parameter('fallback_distance', 1.0)
        self.declare_parameter('nav_action_name', '/robot1/navigate_to_pose')
        # FIX: Add threshold to prevent re-sending goals when already arrived
        self.declare_parameter('goal_resend_threshold', 0.25) # meters

        # Load parameters
        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.tf_base_frame = self.get_parameter('tf_base_frame').value
        self.tf_map_frame = self.get_parameter('tf_map_frame').value
        self.target_class = self.get_parameter('target_class').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.fallback_distance = self.get_parameter('fallback_distance').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        # FIX: Load the new parameter
        self.goal_resend_threshold = self.get_parameter('goal_resend_threshold').value

        # Subscriptions
        self.detection_sub = self.create_subscription(
            Detection2DArray, self.yolo_topic, self.detections_cb, 10
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/robot1/map', self.map_cb, 10)

        # Publisher for status
        self.status_pub = self.create_publisher(String, '/robot1/object_nav_status', 10)

        # TF + Nav2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._action_client = ActionClient(self, NavigateToPose, self.nav_action_name)

        # Data
        self.map_data = None

        self.get_logger().info("✅ ObjectToNavGoal node (map-based) started")

    # ---------------------------------------------------------
    def map_cb(self, msg: OccupancyGrid):
        self.map_data = msg

    # ---------------------------------------------------------
    def detections_cb(self, detections):
        if not detections.detections:
            return

        # Find target object in YOLO detections
        found_target = any(
            getattr(getattr(hypo, "hypothesis", hypo), "class_id", None) == self.target_class
            for det in detections.detections for hypo in det.results
        )

        if not found_target:
            return

        self.get_logger().info(f"Detected {self.target_class}, searching obstacle ahead...")
        goal = self.compute_goal_from_map()

        if goal:
            # --- START FIX: Check if we are already at the goal ---
            try:
                # Get robot's current pose to check if we're already at the goal
                robot_tf = self.tf_buffer.lookup_transform(
                    self.tf_map_frame, self.tf_base_frame,
                    rclpy.time.Time(), Duration(seconds=0.5)
                )
                rx = robot_tf.transform.translation.x
                ry = robot_tf.transform.translation.y
                gx = goal.pose.position.x
                gy = goal.pose.position.y

                distance_to_goal = math.dist((rx, ry), (gx, gy))

                if distance_to_goal < self.goal_resend_threshold:
                    self.get_logger().info(f"Already at goal (dist: {distance_to_goal:.2f}m < {self.goal_resend_threshold}m). Not resending.")
                    return  # Exit callback, don't send the goal
                
            except Exception as e:
                self.get_logger().warn(f"TF lookup for goal check failed: {e}. Sending goal anyway.")
            # --- END FIX ---

            self.send_nav_goal(goal)
        else:
            self.get_logger().warn("❌ Could not compute goal.")

    # ---------------------------------------------------------
    def compute_goal_from_map(self):
        """Find the closest obstacle ahead of the robot in map coordinates."""
        if self.map_data is None:
            self.get_logger().warn("⚠️ No map received yet. Using fallback.")
            return self.make_fallback_goal(self.fallback_distance)

        # Get robot pose (map → base_link)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_map_frame, self.tf_base_frame,
                rclpy.time.Time(), Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

        # Extract pose and heading
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        yaw_vec = np.array([math.cos(yaw), math.sin(yaw)])

        # Map metadata
        res = self.map_data.info.resolution
        width = self.map_data.info.width
        height = self.map_data.info.height
        origin = self.map_data.info.origin.position
        data = np.array(self.map_data.data).reshape((height, width))

        def world_to_map(x, y):
            mx = int((x - origin.x) / res)
            my = int((y - origin.y) / res)
            return mx, my

        def map_to_world(mx, my):
            x = origin.x + (mx + 0.5) * res
            y = origin.y + (my + 0.5) * res
            return x, y

        mx0, my0 = world_to_map(rx, ry)

        # Step forward along robot yaw until hitting obstacle
        steps = int(3.0 / res)
        for i in range(1, steps):
            wx = rx + yaw_vec[0] * (i * res)
            wy = ry + yaw_vec[1] * (i * res)
            mx, my = world_to_map(wx, wy)
            if 0 <= mx < width and 0 <= my < height:
                if data[my, mx] > 50:  # occupied
                    gx, gy = map_to_world(mx, my)
                    self.get_logger().info(f"🌍 Obstacle detected at map coords: x={gx:.2f}, y={gy:.2f}")
                    backoff = self.stop_distance
                    goal_x = gx - yaw_vec[0] * backoff
                    goal_y = gy - yaw_vec[1] * backoff
                    self.get_logger().info(f"🟢 Goal set 0.5m before obstacle: x={goal_x:.2f}, y={goal_y:.2f}")

                    goal = PoseStamped()
                    goal.header.frame_id = self.tf_map_frame
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position = Point(x=goal_x, y=goal_y, z=0.0)
                    goal.pose.orientation = yaw_to_quaternion(yaw)
                    return goal

        self.get_logger().warn("No obstacle ahead found within 3m — using fallback goal.")
        return self.make_fallback_goal(self.fallback_distance)

    # ---------------------------------------------------------
    def make_fallback_goal(self, dist):
        """Fallback goal directly ahead of robot."""
        pose_base = PoseStamped()
        pose_base.header.frame_id = self.tf_base_frame
        pose_base.header.stamp = self.get_clock().now().to_msg()
        pose_base.pose.position = Point(x=dist, y=0.0, z=0.0)
        pose_base.pose.orientation = yaw_to_quaternion(0.0)
        self.get_logger().warn(f"⚠️ Fallback goal {dist:.2f} m ahead.")

        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_map_frame, self.tf_base_frame, rclpy.time.Time(), Duration(seconds=1.0)
            )
            return safe_transform_pose(pose_base, tf)
        except Exception as e:
            self.get_logger().error(f"TF lookup failed in fallback: {e}")
            return None

    # ---------------------------------------------------------
    def send_nav_goal(self, pose_stamped: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        gx = pose_stamped.pose.position.x
        gy = pose_stamped.pose.position.y
        self.get_logger().info(f"🚀 Sending Nav2 goal to (x={gx:.2f}, y={gy:.2f})")

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # ---------------------------------------------------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    # ---------------------------------------------------------
    def goal_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().warn("Goal aborted.")
        else:
            self.get_logger().info("✅ Goal reached.")


# -------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObjectToNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()