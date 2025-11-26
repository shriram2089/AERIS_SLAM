import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import numpy as np
import json

class SwarmExplorerNode(Node):
    def __init__(self):
        super().__init__('swarm_explorer')

        # Robot namespace
        self.namespace = self.get_namespace().strip('/') or "robot"
        self.get_logger().info(f"[{self.namespace}] Swarm Explorer started")

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, "/swarm/status", self.swarm_status_callback, 10)
        self.create_subscription(String, "/swarm/frontiers", self.swarm_frontier_callback, 10)
        self.create_subscription(String, "/swarm/help", self.swarm_help_callback, 10)

        # Publishers
        self.swarm_pub = self.create_publisher(String, "/swarm/status", 10)
        self.frontier_pub = self.create_publisher(String, "/swarm/frontiers", 10)
        self.help_pub = self.create_publisher(String, "/swarm/help", 10)

        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')

        # Internal data
        self.map_data = None
        self.visited_frontiers = set()
        self.swarm_frontiers = {}
        self.swarm_status = {}
        self.robot_position = (0, 0)
        self.current_goal = None

        # Timers
        self.create_timer(5.0, self.explore)
        self.create_timer(2.0, self.broadcast_status)

    # ===== Map =====
    def map_callback(self, msg):
        self.map_data = msg

    # ===== Swarm Communication =====
    def broadcast_status(self):
        status = {
            "robot": self.namespace,
            "position": self.robot_position,
            "goal": self.current_goal
        }
        msg = String()
        msg.data = json.dumps(status)
        self.swarm_pub.publish(msg)

    def swarm_status_callback(self, msg):
        data = json.loads(msg.data)
        if data["robot"] != self.namespace:
            self.swarm_status[data["robot"]] = data

    def swarm_frontier_callback(self, msg):
        data = json.loads(msg.data)
        if data["robot"] != self.namespace:
            self.swarm_frontiers[data["robot"]] = data["frontiers"]

    def swarm_help_callback(self, msg):
        data = json.loads(msg.data)
        # If help request is nearby, respond (move close)
        if data["robot"] != self.namespace:
            help_pos = data.get("position")
            if help_pos:
                distance = np.linalg.norm([self.robot_position[0]-help_pos[0],
                                           self.robot_position[1]-help_pos[1]])
                if distance < 3.0:  # within 3 meters
                    self.get_logger().info(f"Responding to help request from {data['robot']}")
                    self.navigate_to(help_pos[0], help_pos[1])

    # ===== Frontier Detection =====
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

    # ===== Frontier Selection with Buffer =====
    def choose_frontier(self, frontiers):
        if not frontiers:
            return None
        robot_row, robot_col = self.world_to_map(self.robot_position)
        min_dist = float('inf')
        chosen = None
        buffer_cells = 3  # avoid frontiers too close to others

        for f in frontiers:
            if f in self.visited_frontiers:
                continue
            too_close = False
            for swarm_f in self.swarm_frontiers.values():
                for sf in swarm_f:
                    dist = np.linalg.norm([f[0]-sf[0], f[1]-sf[1]])
                    if dist <= buffer_cells:
                        too_close = True
                        break
                if too_close:
                    break
            if too_close:
                continue
            dist = np.linalg.norm([robot_row-f[0], robot_col-f[1]])
            if dist < min_dist:
                min_dist = dist
                chosen = f

        if chosen:
            self.visited_frontiers.add(chosen)
            # Broadcast claim
            claim_msg = {"robot": self.namespace, "frontiers": [chosen]}
            swarm_msg = String()
            swarm_msg.data = json.dumps(claim_msg)
            self.frontier_pub.publish(swarm_msg)
        return chosen

    # ===== Map <-> World Conversion =====
    def world_to_map(self, pos):
        if not self.map_data:
            return (0, 0)
        mx = int((pos[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        my = int((pos[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        return (my, mx)

    def map_to_world(self, cell):
        if not self.map_data:
            return (0.0, 0.0)
        x = cell[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        y = cell[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
        return (x, y)

    # ===== Exploration Loop =====
    def explore(self):
        if self.map_data is None:
            return
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))
        frontiers = self.find_frontiers(map_array)
        chosen = self.choose_frontier(frontiers)
        if chosen:
            goal_x, goal_y = self.map_to_world(chosen)
            self.navigate_to(goal_x, goal_y)

    # ===== Navigation =====
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
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation completed: {result}")
            self.current_goal = None
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
            self.current_goal = None

def main(args=None):
    rclpy.init(args=args)
    node = SwarmExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down swarm explorer")
    finally:
        node.destroy_node()
        rclpy.shutdown()
