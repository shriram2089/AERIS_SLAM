import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from scipy.ndimage import label


class AdvancedExplorerNode(Node):
    def __init__(self):
        super().__init__('advanced_explorer')

        # Get namespace
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = "robot"
        self.get_logger().info(f"Advanced Explorer Started in namespace: {self.namespace}")

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, f'/{self.namespace}/map', self.map_callback, 10)

        # Action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')

        # Visited frontiers & shared frontiers
        self.visited_frontiers = set()
        self.shared_frontiers = set()  # to coordinate with other robots

        # Map and position
        self.map_data = None
        self.robot_position = (0, 0)  # update from localization

        # Adaptive timer
        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.explore)

    def map_callback(self, msg):
        self.map_data = msg

    def navigate_to(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def find_frontiers(self, map_array):
        """
        Frontier detection with clustering.
        """
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        # Cluster nearby frontiers
        frontier_map = np.zeros_like(map_array)
        for r, c in frontiers:
            frontier_map[r, c] = 1

        labeled_array, num_features = label(frontier_map)
        clustered_frontiers = []
        for i in range(1, num_features + 1):
            indices = np.argwhere(labeled_array == i)
            # pick centroid as representative frontier
            centroid = tuple(indices.mean(axis=0).astype(int))
            clustered_frontiers.append(centroid)

        self.get_logger().info(f"Found {len(frontiers)} raw frontiers, {len(clustered_frontiers)} clustered")
        return clustered_frontiers

    def choose_frontier(self, frontiers):
        """
        Weighted frontier selection: distance + unexplored neighbors.
        """
        robot_row, robot_col = self.robot_position
        best_score = float('inf')
        chosen_frontier = None

        for r, c in frontiers:
            if (r, c) in self.visited_frontiers or (r, c) in self.shared_frontiers:
                continue
            distance = np.linalg.norm([robot_row - r, robot_col - c])
            # Count unknown neighbors as exploration potential
            unexplored_neighbors = self.count_unexplored_neighbors(r, c)
            score = distance / (1 + unexplored_neighbors)  # prioritize high potential
            if score < best_score:
                best_score = score
                chosen_frontier = (r, c)

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            self.shared_frontiers.add(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier to explore")

        return chosen_frontier

    def count_unexplored_neighbors(self, r, c):
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        neighbors = map_array[max(r-1,0):r+2, max(c-1,0):c+2].flatten()
        return np.sum(neighbors == -1)

    def explore(self):
        if self.map_data is None:
            self.get_logger().warning("No map data yet")
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("Exploration complete!")
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            return

        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        self.navigate_to(goal_x, goal_y)

        # Adaptive timer: explore faster in open areas
        self.timer_period = max(2.0, min(8.0, 10.0 / (len(frontiers) + 1)))
        self.timer.timer_period_sec = self.timer_period


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
