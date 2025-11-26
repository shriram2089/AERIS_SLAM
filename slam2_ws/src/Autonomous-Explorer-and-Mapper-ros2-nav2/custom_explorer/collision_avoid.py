
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

        # Get robot namespace
        self.namespace = self.get_namespace().strip('/') or "robot"
        self.get_logger().info(f"[{self.namespace}] Swarm Explorer started")

        # Subscribe to the merged global map from multirobot_map_merge
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Publishers for swarm communication
        self.swarm_pub = self.create_publisher(String, "/swarm/status", 10)
        self.frontier_pub = self.create_publisher(String, "/swarm/frontiers", 10)

        # Subscribers to swarm communication
        self.create_subscription(String, "/swarm/status", self.swarm_status_callback, 10)
        self.create_subscription(String, "/swarm/frontiers", self.swarm_frontier_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, f'/{self.namespace}/navigate_to_pose')

        # Data
        self.map_data = None
        self.frontiers = []
        self.visited_frontiers = set()
        self.swarm_frontiers = {}
        self.swarm_status = {}
        self.robot_position = (0, 0)  # TODO: update with localization

        # Timers
        self.create_timer(5.0, self.explore)
        self.create_timer(2.0, self.broadcast_status)

    # ===== Map Handling =====
    def map_callback(self, msg):
        self.map_data = msg

    # ===== Swarm Communication =====
    def broadcast_status(self):
        status = {
            "robot": self.namespace,
            "position": self.robot_position,
            "goal": list(self.visited_frontiers)[-1] if self.visited_frontiers else None
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

    # ===== Frontier Detection =====
    def find_frontiers(self, map_array):
        frontiers = []
        rows, cols = map_array.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))
        return frontiers

    # ===== Frontier Selection (Greedy + Claiming) =====
    def choose_frontier(self, frontiers):
        robot_row, robot_col = self.robot_position
        min_dist = float('inf')
        chosen = None

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue
            if any(frontier in f for f in self.swarm_frontiers.values()):
                continue  # already claimed

            dist = np.linalg.norm([robot_row - frontier[0], robot_col - frontier[1]])
            if dist < min_dist:
                min_dist = dist
                chosen = frontier

        if chosen:
            self.visited_frontiers.add(chosen)
            # Broadcast claim
            claim_msg = {"robot": self.namespace, "frontiers": [chosen]}
            swarm_msg = String()
            swarm_msg.data = json.dumps(claim_msg)
            self.frontier_pub.publish(swarm_msg)

        return chosen

    # ===== Exploration Loop =====
    def explore(self):
        if self.map_data is None:
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        self.frontiers = self.find_frontiers(map_array)
        if not self.frontiers:
            self.get_logger().info("No frontiers left.")
            return

        chosen = self.choose_frontier(self.frontiers)
        if not chosen:
            return

        # Convert to world coords
        goal_x = chosen[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y
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
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")


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

