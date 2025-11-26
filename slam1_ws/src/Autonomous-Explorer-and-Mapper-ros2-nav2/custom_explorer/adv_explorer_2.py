import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D
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

        # Visited & shared frontiers
        self.visited_frontiers = set()
        self.shared_frontiers = set()  # frontiers claimed by all robots

        # Publishers & subscribers for multi-robot coordination
        self.frontier_pub = self.create_publisher(Int32MultiArray, '/shared_frontiers', 10)
        self.frontier_sub = self.create_subscription(
            Int32MultiArray, '/shared_frontiers', self.shared_frontier_callback, 10)

        self.position_pub = self.create_publisher(Pose2D, '/robot_positions', 10)
        self.position_sub = self.create_subscription(
            Pose2D, '/robot_positions', self.robot_positions_callback, 10)

        # RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # Map and position
        self.map_data = None
        self.robot_position = (0, 0)
        self.other_robots_positions = {}  # {namespace: (x, y)}

        # Adaptive timer
        self.timer_period = 5.0
        self.timer = self.create_timer(self.timer_period, self.explore)
        self.position_timer = self.create_timer(0.5, self.publish_position)  # frequent updates

    # ----------------------- ROS Callbacks -----------------------
    def map_callback(self, msg):
        self.map_data = msg

    def shared_frontier_callback(self, msg):
        data = np.array(msg.data).reshape(-1, 2)
        for r, c in data:
            self.shared_frontiers.add((r, c))

    def robot_positions_callback(self, msg):
        if msg is None or not hasattr(msg, 'x') or not hasattr(msg, 'y'):
            return
        # store/update other robot positions
        # use namespace to avoid self
        robot_ns = f"robot_{int(msg.theta)}"  # using theta as a placeholder ID
        if robot_ns != self.namespace:
            self.other_robots_positions[robot_ns] = (msg.x, msg.y)

    def publish_position(self):
        msg = Pose2D()
        msg.x = float(self.robot_position[0])
        msg.y = float(self.robot_position[1])
        # use namespace index as ID in theta (just for uniqueness)
        msg.theta = float(int(self.namespace.split('robot')[-1] if 'robot' in self.namespace else 0))
        self.position_pub.publish(msg)


    # ----------------------- Navigation -----------------------
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

    # ----------------------- Frontier Detection -----------------------
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape

        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        # Cluster frontiers
        frontier_map = np.zeros_like(map_array)
        for r, c in frontiers:
            frontier_map[r, c] = 1

        labeled_array, num_features = label(frontier_map)
        clustered_frontiers = []
        for i in range(1, num_features + 1):
            indices = np.argwhere(labeled_array == i)
            centroid = tuple(indices.mean(axis=0).astype(int))
            clustered_frontiers.append(centroid)

        self.get_logger().info(f"Found {len(frontiers)} raw frontiers, {len(clustered_frontiers)} clustered")
        return clustered_frontiers

    # ----------------------- Frontier Selection -----------------------
    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        best_score = float('inf')
        chosen_frontier = None

        for r, c in frontiers:
            if (r, c) in self.visited_frontiers or (r, c) in self.shared_frontiers:
                continue

            # Avoid frontiers too close to other robots
            too_close = False
            for x, y in self.other_robots_positions.values():
                robot_r = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
                robot_c = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                if np.linalg.norm([robot_r - r, robot_c - c]) < 3:  # safety distance
                    too_close = True
                    break
            if too_close:
                continue

            distance = np.linalg.norm([robot_row - r, robot_col - c])
            unexplored_neighbors = self.count_unexplored_neighbors(r, c)
            score = distance / (1 + unexplored_neighbors)
            if score < best_score:
                best_score = score
                chosen_frontier = (r, c)

        if chosen_frontier:
            self.claim_frontier_cluster(chosen_frontier)
            self.visualize_frontier(chosen_frontier)
            self.get_logger().info(f"Chosen frontier: {chosen_frontier}")
        else:
            self.get_logger().warning("No valid frontier to explore")

        return chosen_frontier

    def count_unexplored_neighbors(self, r, c):
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        neighbors = map_array[max(r-1,0):r+2, max(c-1,0):c+2].flatten()
        return np.sum(neighbors == -1)

    # ----------------------- Multi-Robot Coordination -----------------------
    def claim_frontier_cluster(self, frontier, cluster_radius=3):
        r0, c0 = frontier
        cluster_cells = [(r, c) for r in range(r0 - cluster_radius, r0 + cluster_radius + 1)
                         for c in range(c0 - cluster_radius, c0 + cluster_radius + 1)
                         if r >= 0 and c >= 0]

        for cell in cluster_cells:
            self.visited_frontiers.add(cell)
            self.shared_frontiers.add(cell)

        msg = Int32MultiArray()
        msg.data = [int(val) for cell in cluster_cells for val in cell]
        self.frontier_pub.publish(msg)

    # ----------------------- RViz Visualization -----------------------
    def visualize_frontier(self, frontier, cluster_radius=3):
        r0, c0 = frontier
        marker_array = MarkerArray()
        marker_id = 0
        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        for r in range(r0 - cluster_radius, r0 + cluster_radius + 1):
            for c in range(c0 - cluster_radius, c0 + cluster_radius + 1):
                if r < 0 or c < 0:
                    continue
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"{self.namespace}_frontiers"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.scale.x = resolution
                marker.scale.y = resolution
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.6
                marker.pose.position.x = c * resolution + origin_x + resolution / 2.0
                marker.pose.position.y = r * resolution + origin_y + resolution / 2.0
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 1.0
                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    # ----------------------- Exploration -----------------------
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
