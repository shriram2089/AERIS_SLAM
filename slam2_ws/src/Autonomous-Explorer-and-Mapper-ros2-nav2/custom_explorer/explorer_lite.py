

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import numpy as np

class SimpleExplorer(Node):
    def __init__(self):
        super().__init__('simple_explorer')

        # State
        self.map_data = None
        self.visited = set()
        self.current_goal_handle = None
        self.robot_position = (0, 0)  # TODO: subscribe to /amcl_pose for real position

        # Subscribers
        self.create_subscription(OccupancyGrid, '/robot2/map', self.map_callback, 10)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')
        self.get_logger().info("Simple Explorer started. Waiting for map...")

    def map_callback(self, msg):
        self.map_data = msg
        if self.current_goal_handle is None:
            self.explore()

    def explore(self):
        if self.map_data is None or self.current_goal_handle is not None:
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        frontiers = self.find_frontiers(map_array)
        if not frontiers:
            self.get_logger().info("Exploration complete!")
            return

        frontier = self.choose_frontier(frontiers)
        if frontier is None:
            self.get_logger().info("No valid frontier found")
            return

        # Convert to world coordinates
        goal_x = frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        # Skip if too close
        if np.hypot(goal_x - self.robot_position[0], goal_y - self.robot_position[1]) < 0.1:
            self.visited.add(frontier)
            self.explore()
            return

        self.send_goal(goal_x, goal_y)

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

    def choose_frontier(self, frontiers):
        min_dist = float('inf')
        chosen = None
        for f in frontiers:
            if f in self.visited:
                continue
            dist = np.sqrt((f[0]-self.robot_position[0])**2 + (f[1]-self.robot_position[1])**2)
            if dist < min_dist:
                min_dist = dist
                chosen = f
        if chosen:
            self.visited.add(chosen)
        return chosen

    def send_goal(self, x, y):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = goal_msg

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available")
            return

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

        self.get_logger().info(f"Sending goal: x={x:.2f}, y={y:.2f}")

    def goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.current_goal_handle = None
            self.explore()
            return

        self.get_logger().info("Goal accepted")
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(self.goal_complete)

    def goal_complete(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Goal completed: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")
        finally:
            self.current_goal_handle = None
            self.explore()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Exploration stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()