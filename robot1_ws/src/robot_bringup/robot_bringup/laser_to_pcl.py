#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import math
import random  # for small Z variation

class LaserToPointCloud(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')

        # QoS profile for YDLidar (usually best_effort)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            LaserScan,
            '/robot1/scan',  # YDLidar scan topic
            self.scan_callback,
            qos
        )
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(PointCloud2, '/robot1/point_cloud', pub_qos)
        self.get_logger().info("Laser to 2D PointCloud node started.")

    def scan_callback(self, scan_msg: LaserScan):
        points = []

        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # keep 2D
                points.append([x, y, z])
            angle += scan_msg.angle_increment

        header = scan_msg.header
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


##PSEUDO 3D CONVERTER CODE

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy

# from sensor_msgs.msg import LaserScan, PointCloud2
# from sensor_msgs_py import point_cloud2
# import math
# import random  # for small Z variation

# class LaserToPointCloud(Node):
#     def __init__(self):
#         super().__init__('laser_to_pointcloud')

#         # QoS profile for YDLidar (usually best_effort)
#         qos = QoSProfile(depth=10)
#         qos.reliability = ReliabilityPolicy.BEST_EFFORT

#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/robot1/scan',  # YDLidar scan topic
#             self.scan_callback,
#             qos
#         )
#         pub_qos = QoSProfile(depth=10)
#         pub_qos.reliability = ReliabilityPolicy.RELIABLE
#         self.publisher = self.create_publisher(PointCloud2, '/robot1/point_cloud', pub_qos)
#         self.get_logger().info("Laser to pseudo-3D PointCloud node started.")

#     def scan_callback(self, scan_msg: LaserScan):
#         points = []

#         angle = scan_msg.angle_min
#         for r in scan_msg.ranges:
#             if scan_msg.range_min < r < scan_msg.range_max:
#                 x = r * math.cos(angle)
#                 y = r * math.sin(angle)
#                 # Add tiny Z jitter to make it pseudo-3D
#                 z = random.uniform(-0.05,0.05)  
#                 points.append([x, y, z])
#             angle += scan_msg.angle_increment

#         header = scan_msg.header
#         cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
#         self.publisher.publish(cloud_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LaserToPointCloud()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
