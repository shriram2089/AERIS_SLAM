import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import time

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # Track map readiness
        self.map_ready = False
        self.create_subscription(
            OccupancyGrid,
            '/robot2/map',
            self.map_callback,
            10
        )

        # Action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')

        # Wait until action server and map are ready
        self.get_logger().info("Waiting for /robot2/navigate_to_pose action server and first map message...")
        while not self.map_ready or not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Still waiting for action server or map...")
            time.sleep(0.5)

        self.get_logger().info("Map received and action server ready! Starting exploration...")

        # Start exploration
        self.start_exploration()

    def map_callback(self, msg):
        if not self.map_ready:
            self.get_logger().info("First map message received!")
            self.map_ready = True

    def start_exploration(self):
        # Example: navigate to a goal (replace with frontier logic)
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = 0.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"Sending goal: x={goal_msg.pose.position.x}, y={goal_msg.pose.position.y}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return
        self.get_logger().info("Goal accepted, moving...")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
