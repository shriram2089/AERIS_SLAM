#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import std_msgs.msg


class MRGRelay(Node):
    def __init__(self):
        super().__init__('mrg_relay')

        # QoS profile for PointCloud2
        pointcloud_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.sub_cloud = self.create_subscription(
            PointCloud2,
            '/robot2/point_cloud',
            self.cloud_callback,
            pointcloud_qos
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/robot2/odom',
            self.odom_callback,
            10
        )

        self.sub_imu = self.create_subscription(
            Imu,
            '/robot2/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.pub_cloud = self.create_publisher(PointCloud2, '/bestla/velodyne_points', pointcloud_qos)
        self.pub_odom = self.create_publisher(Odometry, '/bestla/odom_ground_truth', 10)
        self.pub_imu = self.create_publisher(Imu, '/bestla/imu/data', 10)

        self.get_logger().info("MRG Relay Node Started with Intensity!")

    def cloud_callback(self, msg: PointCloud2):
        # Take x, y, z exactly as they are
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Add a constant intensity (e.g. 100.0) for every point
        # points_with_intensity = [(x, y, z, 100.0) for x, y, z in points]

        points_with_intensity = []
        for x, y, z in points:
            dist = (x**2 + y**2 + z**2) ** 0.5
            intensity = max(0.0, 255.0 - dist * 10.0)  # tweak scaling factor as needed
            points_with_intensity.append((x, y, z, intensity))
            
        # Define fields (x, y, z, intensity)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        header = std_msgs.msg.Header()
        header.stamp = msg.header.stamp
        header.frame_id = 'bestla/velodyne'

        # Create new point cloud
        new_cloud = pc2.create_cloud(header, fields, points_with_intensity)

        # Ensure dimensions are set correctly
        new_cloud.height = 1
        new_cloud.width = len(points_with_intensity)
        new_cloud.is_dense = False
        new_cloud.point_step = 16
        new_cloud.row_step = new_cloud.point_step * new_cloud.width

        self.pub_cloud.publish(new_cloud)

    def odom_callback(self, msg: Odometry):
        msg.header.frame_id = 'bestla/map'
        msg.child_frame_id = 'bestla/base_link'
        self.pub_odom.publish(msg)

    def imu_callback(self, msg: Imu):
        msg.header.frame_id = 'bestla/base_link'
        self.pub_imu.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MRGRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
