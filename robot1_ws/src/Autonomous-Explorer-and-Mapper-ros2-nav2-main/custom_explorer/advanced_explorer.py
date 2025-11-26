import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
import numpy as np
import math

class AdvancedExplorerNode(Node):
    def __init__(self):
        super().__init__('advanced_explorer')

        self.get_logger().info("Advanced Explorer Node started")

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/robot2/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot2/pose',  # Or pose estimate topic
            self.pose_callback,
            10)

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_handle = None

        # User RViz goal subscriber
        self.manual_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',  # Topic RViz 2D Nav Goal publishes to by default, adjust if needed
            self.manual_goal_callback,
            10)

        # Internal state
        self.current_map = None
        self.robot_pose = None  # PoseStamped or (x, y, yaw)
        self.visited_frontiers = set()

        # Timer for periodic exploration (runs every 8 sec)
        self.explore_timer = self.create_timer(8.0, self.explore_callback)

    def map_callback(self, msg):
        self.current_map = msg
        # self.get_logger().debug("Map updated")

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose
        # self.get_logger().debug(f"Robot pose updated: {self.robot_pose.position.x}, {self.robot_pose.position.y}")

    def manual_goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Manual navigation goal received from RViz: {msg.pose.position.x}, {msg.pose.position.y}")
        # Cancel any current autonomous goal when manual command received
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.get_logger().info("Cancelled current autonomous goal to process manual goal")

        self.send_nav_goal(msg)

    def send_nav_goal(self, pose_stamped: PoseStamped):
        if not self.nav_client.server_is_ready():
            self.get_logger().warning("Navigation action server not ready")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Navigation goal rejected")
            self.goal_handle = None
            return

        self.get_logger().info("Navigation goal accepted")
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = None
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Navigation failed with exception: {e}")
            self.goal_handle = None
            return

        if result:
            self.get_logger().info(f"Navigation completed with success status: {result.success}")
        else:
            self.get_logger().info("Navigation finished with no result received")

        self.goal_handle = None  # Reset goal handle

    def explore_callback(self):
        if self.goal_handle:  # Already navigating somewhere
            self.get_logger().info("Currently navigating, skipping exploration")
            return

        if self.current_map is None or self.robot_pose is None:
            self.get_logger().info("Waiting for map and robot pose before exploring")
            return

        map_array = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("No frontiers found, exploration complete.")
            return

        robot_cell = self.world_to_map(self.robot_pose.position.x,
                                      self.robot_pose.position.y,
                                      self.current_map.info)
        if robot_cell is None:
            self.get_logger().warning("Robot position outside map bounds")
            return

        chosen_frontier = self.choose_frontier(frontiers, robot_cell)
        if chosen_frontier is None:
            self.get_logger().info("No valid frontier to explore")
            return

        goal_pose = self.map_to_world(chosen_frontier[1],
                                      chosen_frontier[0],
                                      self.current_map.info)

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = self.current_map.header.frame_id
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = goal_pose[0]
        goal_msg.pose.position.y = goal_pose[1]
        goal_msg.pose.orientation.w = 1.0

        self.get_logger().info(f"Auto navigating to frontier at map cell {chosen_frontier} (world {goal_pose})")
        self.send_nav_goal(goal_msg)

    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # free space
                    neighbors = map_array[r-1:r+2, c-1:c+2]
                    if -1 in neighbors:
                        frontiers.append((r, c))
        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers, robot_cell):
        min_dist = float('inf')
        chosen = None
        for f in frontiers:
            if f in self.visited_frontiers:
                continue
            dist = math.hypot(robot_cell[0] - f[0], robot_cell[1] - f[1])
            if dist < min_dist:
                min_dist = dist
                chosen = f
        if chosen:
            self.visited_frontiers.add(chosen)
            self.get_logger().info(f"Chosen frontier at cell {chosen}")
        return chosen

    def world_to_map(self, x, y, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)
        if 0 <= mx < info.width and 0 <= my < info.height:
            return (my, mx)
        else:
            return None

    def map_to_world(self, mx, my, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        x = mx * res + origin_x + res / 2.0
        y = my * res + origin_y + res / 2.0
        return (x, y)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
