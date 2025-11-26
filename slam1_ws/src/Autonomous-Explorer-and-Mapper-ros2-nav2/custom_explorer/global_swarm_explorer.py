import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.action import ActionClient
import numpy as np
import json
import time
from collections import defaultdict

class SwarmGlobalExplorer(Node):
    def __init__(self, robot_names):
        super().__init__('swarm_global_explorer')

        self.robot_names = robot_names
        self.get_logger().info(f"Swarm Explorer for robots: {self.robot_names}")

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_subs = {}
        self.local_costmap_subs = {}
        self.global_costmap_subs = {}
        for robot in robot_names:
            self.odom_subs[robot] = self.create_subscription(
                Odometry, f'/{robot}/odom', lambda msg, r=robot: self.odom_callback(msg, r), 10)
            self.local_costmap_subs[robot] = self.create_subscription(
                OccupancyGrid, f'/{robot}/local_costmap/costmap', 
                lambda msg, r=robot: self.local_costmap_callback(msg, r), 10)
            self.global_costmap_subs[robot] = self.create_subscription(
                OccupancyGrid, f'/{robot}/global_costmap/costmap', 
                lambda msg, r=robot: self.global_costmap_callback(msg, r), 10)

        # Swarm communication
        self.frontier_pub = self.create_publisher(String, '/swarm/frontiers', 10)
        self.help_pub = self.create_publisher(String, '/swarm/help', 10)
        self.help_sub = self.create_subscription(String, '/swarm/help', self.help_callback, 10)

        # Action clients
        self.nav_clients = {robot: ActionClient(self, NavigateToPose, f'/{robot}/navigate_to_pose')
                            for robot in robot_names}

        # Data structures
        self.map_data = None
        self.reservation_map = None
        self.visited_map = None
        self.local_costmaps = {}
        self.global_costmaps = {}
        self.robot_positions = {}
        self.current_goals = {r: None for r in robot_names}
        self.goal_start_time = {r: None for r in robot_names}
        self.goal_timeout = 15.0  # seconds to wait before switching goal if stuck
        self.buffer_radius = 5  # cells around claimed frontiers
        self.claimed_frontiers = defaultdict(list)
        self.help_requests = []

        # Timers
        self.create_timer(3.0, self.assign_goals)
        self.create_timer(2.0, self.publish_claims)

    # ===== Map Callbacks =====
    def map_callback(self, msg):
        self.map_data = msg
        h, w = msg.info.height, msg.info.width
        if self.reservation_map is None:
            self.reservation_map = np.zeros((h, w), dtype=np.uint8)
            self.visited_map = np.zeros((h, w), dtype=np.uint8)

    def local_costmap_callback(self, msg, robot):
        self.local_costmaps[robot] = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def global_costmap_callback(self, msg, robot):
        self.global_costmaps[robot] = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    def odom_callback(self, msg, robot):
        self.robot_positions[robot] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ===== Frontier Detection =====
    def find_frontiers(self):
        if self.map_data is None:
            return []
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        frontiers = []
        h, w = map_array.shape
        for r in range(1, h-1):
            for c in range(1, w-1):
                if map_array[r, c] == 0 and self.visited_map[r, c] == 0 and self.reservation_map[r, c] == 0:
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))
        return frontiers

    def cluster_frontiers(self, frontiers):
        clusters = []
        remaining = set(frontiers)
        while remaining:
            f = remaining.pop()
            cluster = [f]
            to_check = [f]
            while to_check:
                cell = to_check.pop()
                neighbors = [(cell[0]+dr, cell[1]+dc)
                             for dr in range(-self.buffer_radius, self.buffer_radius+1)
                             for dc in range(-self.buffer_radius, self.buffer_radius+1)]
                neighbors = [n for n in neighbors if n in remaining]
                for n in neighbors:
                    cluster.append(n)
                    to_check.append(n)
                    remaining.remove(n)
            clusters.append(cluster)
        return clusters

    # ===== Goal Assignment =====
    def assign_goals(self):
        if self.map_data is None or not self.robot_positions:
            return
        frontiers = self.find_frontiers()
        clusters = self.cluster_frontiers(frontiers)

        for robot in self.robot_names:
            # Skip if goal is still valid
            current_goal = self.current_goals[robot]
            if current_goal:
                elapsed = time.time() - (self.goal_start_time[robot] or 0)
                if elapsed < self.goal_timeout:
                    continue
                else:
                    self.get_logger().info(f"{robot} goal timed out, reassigning")
                    self.current_goals[robot] = None

            rx, ry = self.robot_positions.get(robot, (0,0))
            best_cluster = None
            min_score = float('inf')

            for cluster in clusters:
                cr = int(np.mean([c[0] for c in cluster]))
                cc = int(np.mean([c[1] for c in cluster]))
                # Avoid clusters too close to other robots
                too_close = False
                for other_robot, goal in self.current_goals.items():
                    if goal is None or other_robot == robot:
                        continue
                    gr, gc = self.world_to_map(goal)
                    if np.linalg.norm([cr-gr, cc-gc]) <= self.buffer_radius*2:
                        too_close = True
                        break
                if too_close:
                    continue
                dist = np.linalg.norm([cr - self.world_to_map((rx, ry))[0], cc - self.world_to_map((rx, ry))[1]])
                if dist < min_score:
                    min_score = dist
                    best_cluster = cluster

            if best_cluster:
                cr = int(np.mean([c[0] for c in best_cluster]))
                cc = int(np.mean([c[1] for c in best_cluster]))
                gx, gy = self.map_to_world((cr, cc))
                self.current_goals[robot] = (gx, gy)
                self.goal_start_time[robot] = time.time()
                # Reserve cluster cells
                for cell in best_cluster:
                    r0, c0 = cell
                    for dr in range(-self.buffer_radius, self.buffer_radius+1):
                        for dc in range(-self.buffer_radius, self.buffer_radius+1):
                            rr, cc2 = r0+dr, c0+dc
                            if 0 <= rr < self.reservation_map.shape[0] and 0 <= cc2 < self.reservation_map.shape[1]:
                                self.reservation_map[rr, cc2] = 1
                self.navigate_robot(robot, gx, gy)

    # ===== Navigation =====
    def navigate_robot(self, robot, x, y):
        client = self.nav_clients[robot]
        client.wait_for_server()

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        send_goal_future = client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(lambda f, r=robot: self.goal_response_callback(f, r))

    def goal_response_callback(self, future, robot):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f"{robot} goal rejected")
            self.current_goals[robot] = None
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, r=robot: self.goal_done_callback(f, r))

    def goal_done_callback(self, future, robot):
        try:
            _ = future.result().result
            self.get_logger().info(f"{robot} reached goal")
        except Exception as e:
            self.get_logger().error(f"{robot} navigation failed: {e}")
        self.current_goals[robot] = None

    # ===== Help / Call =====
    def send_help_request(self, robot, reason="manual"):
        if robot not in self.robot_positions:
            return
        msg = {"robot": robot, "position": self.robot_positions[robot], "reason": reason}
        smsg = String()
        smsg.data = json.dumps(msg)
        self.help_pub.publish(smsg)

    def help_callback(self, msg):
        data = json.loads(msg.data)
        robot_pos = data.get("position")
        if robot_pos is None:
            return
        # Respond if nearby
        for r, pos in self.robot_positions.items():
            if r != data["robot"]:
                dist = np.linalg.norm([pos[0]-robot_pos[0], pos[1]-robot_pos[1]])
                if dist <= 3.0:
                    self.get_logger().info(f"{r} responding to help request from {data['robot']}")
                    self.navigate_robot(r, robot_pos[0], robot_pos[1])

    # ===== Map Conversions =====
    def world_to_map(self, pos):
        if not self.map_data:
            return (0,0)
        mx = int((pos[0]-self.map_data.info.origin.position.x)/self.map_data.info.resolution)
        my = int((pos[1]-self.map_data.info.origin.position.y)/self.map_data.info.resolution)
        return (my, mx)

    def map_to_world(self, cell):
        if not self.map_data:
            return (0.0,0.0)
        x = cell[1]*self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = cell[0]*self.map_data.info.resolution + self.map_data.info.origin.position.y
        return (x,y)

    # ===== Publish Frontier Claims =====
    def publish_claims(self):
        msg = {"claims": self.current_goals}
        smsg = String()
        smsg.data = json.dumps(msg)
        self.frontier_pub.publish(smsg)


def main(args=None):
    rclpy.init(args=args)
    robot_names = ["robot1", "robot2"]  # Add more as needed
    node = SwarmGlobalExplorer(robot_names)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down swarm explorer")
    finally:
        node.destroy_node()
        rclpy.shutdown()
