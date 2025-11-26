#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN


class PCLObjectDetector(Node):
    def __init__(self):
        super().__init__('pcl_object_detector')

        # Subscribers and publishers
        self.create_subscription(PointCloud2, '/robot1/lidar_points', self.pointcloud_callback, 10)
        self.centroid_pub = self.create_publisher(PointStamped, '/detected_object_centroid', 10)
        self.marker_pub = self.create_publisher(Marker, '/detected_object_marker', 10)

        # Parameters
        self.eps = 0.05  # clustering distance (m)
        self.min_samples = 30  # min points to form a cluster

        self.get_logger().info("PCL Object Detector Node Started.")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array (force float32)
        points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(points_list) == 0:
            return

        points = np.array(points_list, dtype=np.float32)
        if points.ndim == 1:  # single point
            points = points.reshape(1, -1)

        # Crop workspace to reduce noise
        mask = (points[:, 0] > 0.1) & (points[:, 0] < 2.0) & \
               (points[:, 1] > -1.0) & (points[:, 1] < 1.0) & \
               (points[:, 2] > -0.1) & (points[:, 2] < 1.0)
        points = points[mask]

        if len(points) < 100:  # not enough points to cluster
            return

        # Cluster the point cloud using DBSCAN
        clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points)
        labels = clustering.labels_
        unique_labels = np.unique(labels)

        # Remove noise label (-1)
        unique_labels = unique_labels[unique_labels != -1]

        if len(unique_labels) == 0:  # no clusters found
            return

        # Compute centroids for all clusters
        centroids = []
        for label in unique_labels:
            cluster_points = points[labels == label]
            centroid = np.mean(cluster_points, axis=0)
            centroids.append(centroid)

        centroids = np.array(centroids)

        # Pick the cluster closest to robot (smallest X)
        chosen_idx = np.argmin(centroids[:, 0])
        target_centroid = centroids[chosen_idx]

        # Publish centroid
        centroid_msg = PointStamped()
        centroid_msg.header = msg.header
        centroid_msg.point.x, centroid_msg.point.y, centroid_msg.point.z = target_centroid
        self.centroid_pub.publish(centroid_msg)

        # Publish a visualization marker in RViz
        marker = Marker()
        marker.header.frame_id = msg.header.frame_id
        marker.header.stamp = msg.header.stamp
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(target_centroid[0])
        marker.pose.position.y = float(target_centroid[1])
        marker.pose.position.z = float(target_centroid[2])
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

        self.get_logger().info(f"Detected object centroid: {target_centroid}")


def main(args=None):
    rclpy.init(args=args)
    node = PCLObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
