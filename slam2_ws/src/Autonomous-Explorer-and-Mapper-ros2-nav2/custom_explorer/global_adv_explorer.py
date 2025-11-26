import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, Pose2D, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Costmap2D
import numpy as np
from scipy.ndimage import label
import time

class GlobalExplorerNode(Node):
    def __init__(self, robot_names):
        super().__init__('global_explorer')

        self.robot_names = robot_names
        self.robot_positions = {name: (0.0, 0.0) for name in robot_names}
        self.robot_goals = {name: None for name in robot_names}
        self.visited_global = set()  # claimed frontiers globally
        self.last_goal_time = {name: 0.0 for name in robot_names}
        self.goal_timeout = 20.0  # seconds before considering stuck

        self.global_map = None
        self.local_costmaps = {name: None for name in robot_names}
        self.global_costmaps = {name: None for name in robot_names}
        self.cmd_vels = {name: (0.0, 0.0) for name in robot_names}

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        for name in robot_names:
            self.create_subscription(Pose2D, f'/{name}/odom_pose', self.create_odom_callback(name), 10)
            self.create_subscription(Twist, f'/{name}/cmd_vel_nav', self.create_cmdvel_callback(name), 10)
            self.create_subscription(OccupancyGrid, f'/{name}/local_costmap/costmap_raw', self.create_local_costmap_callback(name), 10)
            self.create_subscription(OccupancyGrid, f'/{name}/global_costmap/costmap_raw', self.create_global_costmap_callback(name), 10)

        # Navigation action clients
        self.nav_clients = {name: ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
                            for name in robot_names}

        # RViz markers
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # Exploration timer
        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.explore)

    # ----------------------- Callbacks -----------------------
    def map_callback(self, msg):
        self.global_map = msg

    def create_odom_callback(self, robot_name):
        def callback(msg):
            self.robot_positions[robot_name] = (float(msg.x), float(msg.y))
        return callback

    def create_cmdvel_callback(self, robot_name):
        def callback(msg):
            self.cmd_vels[robot_name] = (msg.linear.x, msg.angular.z)
        return callback

    def create_local_costmap_callback(self, robot_name):
        def callback(msg):
            self.local_costmaps[robot_name] = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        return callback

    def create_global_costmap_callback(self, robot_name):
        def callback(msg):
            self.global_costmaps[robot_name] = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        return callback

    # ----------------------- Exploration -----------------------
    def explore(self):
        if self.global_map is None:
            self.get_logger().info("Waiting for global map...")
            return

        map_array = np.array(self.global_map.data).reshape(
            (self.global_map.info.height, self.global_map.info.width))
        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("Exploration complete!")
            return

        current_time = time.time()
        # Assign frontiers to robots
        for robot in self.robot_names:
            # Check if robot is stuck or ready for new goal
            last_time = self.last_goal_time[robot]
            if self.robot_goals[robot] is not None:
                vx, _ = self.cmd_vels[robot]
                if vx < 0.05 and (current_time - last_time) > self.goal_timeout:
                    self.get_logger().info(f"{robot} seems stuck, reassigning goal")
                    self.robot_goals[robot] = None
                else:
                    continue  # still navigating
            frontier = self.choose_frontier(frontiers, robot, map_array)
            if frontier:
                self.robot_goals[robot] = frontier
                self.last_goal_time[robot] = current_time
                self.claim_frontier_cluster(frontier)
                self.visualize_frontier(frontier, robot)
                goal_x = frontier[1] * self.global_map.info.resolution + self.global_map.info.origin.position.x
                goal_y = frontier[0] * self.global_map.info.resolution + self.global_map.info.origin.position.y
                if self.is_safe_goal(robot, frontier):
                    self.send_goal(robot, goal_x, goal_y)
                else:
                    self.get_logger().info(f"Skipping unsafe goal for {robot}")

    # ----------------------- Frontier Detection -----------------------
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if map_array[r,c] == 0 and -1 in map_array[r-1:r+2, c-1:c+2]:
                    frontiers.append((r,c))
        # remove already visited
        frontiers = [f for f in frontiers if f not in self.visited_global]

        # cluster frontiers
        frontier_map = np.zeros_like(map_array)
        for r,c in frontiers:
            frontier_map[r,c] = 1
        labeled_array, num_features = label(frontier_map)
        clustered = []
        for i in range(1, num_features+1):
            indices = np.argwhere(labeled_array==i)
            centroid = tuple(indices.mean(axis=0).astype(int))
            clustered.append(centroid)
        return clustered

    # ----------------------- Frontier Assignment -----------------------
    def choose_frontier(self, frontiers, robot, map_array, safety_radius=5):
        rx, ry = self.robot_positions[robot]
        best_score = float('inf')
        chosen = None
        for r,c in frontiers:
            too_close = False
            for other, (ox, oy) in self.robot_positions.items():
                if other==robot:
                    continue
                d = np.linalg.norm([r - int((oy - self.global_map.info.origin.position.y)/self.global_map.info.resolution),
                                    c - int((ox - self.global_map.info.origin.position.x)/self.global_map.info.resolution)])
                if d < safety_radius:
                    too_close = True
                    break
            if too_close:
                continue
            dist = np.linalg.norm([r - int((ry - self.global_map.info.origin.position.y)/self.global_map.info.resolution),
                                   c - int((rx - self.global_map.info.origin.position.x)/self.global_map.info.resolution)])
            if dist < best_score:
                best_score = dist
                chosen = (r,c)
        return chosen

    # ----------------------- Safety Check -----------------------
    def is_safe_goal(self, robot, frontier):
        # Check against robot's local costmap to avoid walls
        local_map = self.local_costmaps[robot]
        if local_map is None:
            return True
        r,c = frontier
        if r>=local_map.shape[0] or c>=local_map.shape[1]:
            return False
        # any high cost near frontier? (>50)
        neighborhood = local_map[max(0,r-1):r+2, max(0,c-1):c+2]
        if np.any(neighborhood > 50):
            return False
        return True

    # ----------------------- Claim Frontier -----------------------
    def claim_frontier_cluster(self, frontier, cluster_radius=3):
        r0,c0 = frontier
        cluster = [(r,c) for r in range(r0-cluster_radius, r0+cluster_radius+1)
                   for c in range(c0-cluster_radius, c0+cluster_radius+1)
                   if r>=0 and c>=0]
        for cell in cluster:
            self.visited_global.add(cell)

    # ----------------------- Navigation -----------------------
    def send_goal(self, robot, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        client = self.nav_clients[robot]
        client.wait_for_server()
        send_goal_future = client.send_goal_async(NavigateToPose.Goal(pose=goal_msg))
        send_goal_future.add_done_callback(self.create_nav_response_callback(robot))

    def create_nav_response_callback(self, robot):
        def callback(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warning(f"Goal rejected for {robot}")
                self.robot_goals[robot] = None
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda f: self.nav_complete(robot, f))
        return callback

    def nav_complete(self, robot, future):
        self.robot_goals[robot] = None

    # ----------------------- RViz Visualization -----------------------
    def visualize_frontier(self, frontier, robot, cluster_radius=3):
        r0,c0 = frontier
        marker_array = MarkerArray()
        marker_id = 0
        resolution = self.global_map.info.resolution
        origin_x = self.global_map.info.origin.position.x
        origin_y = self.global_map.info.origin.position.y
        for r in range(r0-cluster_radius, r0+cluster_radius+1):
            for c in range(c0-cluster_radius, c0+cluster_radius+1):
                if r<0 or c<0:
                    continue
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"{robot}_frontiers"
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
                marker.pose.position.x = c*resolution + origin_x + resolution/2.0
                marker.pose.position.y = r*resolution + origin_y + resolution/2.0
                marker.pose.position.z = 0.05
                marker.pose.orientation.w = 1.0
                marker_array.markers.append(marker)
                marker_id += 1
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    robot_names = ['robot1','robot2']
    node = GlobalExplorerNode(robot_names)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()
