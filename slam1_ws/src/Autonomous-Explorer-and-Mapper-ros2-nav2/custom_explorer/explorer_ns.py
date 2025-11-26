import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')

        # Namespace
        self.namespace = self.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = "robot"
        self.get_logger().info(f"Explorer Node Started in namespace: {self.namespace}")

        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid, f'/{self.namespace}/map', self.map_callback, 10)

        # Nav2 action client
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')

        # Robot state
        self.map_data = None
        self.robot_position = (0, 0)  # Can be updated from localization
        self.visited_frontiers = set()
        self.is_navigating = False

        self.get_logger().info("Explorer Node initialized. Waiting for map...")

    # Map callback: start exploration when first map is received
    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")
        if not self.is_navigating:
            self.explore()

    # Main exploration function
    def explore(self):
        if self.is_navigating:
            return
        if self.map_data is None:
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("No frontiers left. Exploration complete!")
            return

        chosen_frontier = self.choose_frontier(frontiers)
        if not chosen_frontier:
            self.get_logger().info("No unvisited frontiers available.")
            return

        # Convert to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        self.navigate_to(goal_x, goal_y)

    # Detect frontiers in the occupancy grid
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))
        return frontiers

    # Choose the closest unvisited frontier
    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen = None
        for f in frontiers:
            if f in self.visited_frontiers:
                continue
            dist = np.sqrt((robot_row - f[0])**2 + (robot_col - f[1])**2)
            if dist < min_distance:
                min_distance = dist
                chosen = f
        if chosen:
            self.visited_frontiers.add(chosen)
        return chosen

    # Send navigation goal asynchronously
    def navigate_to(self, x, y):
        if self.is_navigating:
            return

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Nav2 server not available")
            return

        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"[{self.namespace}] Sending goal: x={x}, y={y}")

    # Handle goal acceptance
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.is_navigating = False
            return

        self.get_logger().info("Goal accepted")
        self.is_navigating = True  # Start navigating
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    # Handle navigation completion
    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.is_navigating = False
            # Pick next frontier immediately
            self.explore()


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(explorer_node)

    try:
        explorer_node.get_logger().info("Starting exploration...")
        executor.spin()
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

