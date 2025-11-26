#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN # Used for basic geometric clustering

# --- CONSTANTS for Filtering and Clustering ---
# Bounding Box Filter (Focus on the area in front of the robot)
X_MIN_RANGE = 0.3    # Minimum distance in front (m)
X_MAX_RANGE = 2.0    # Maximum distance (m)
Y_MAX_DEVIATION = 0.5 # Max lateral distance (m) (e.g., 0.5m left/right)
Z_MIN_HEIGHT = 0.1   # Minimum height (m) (to ignore ground)

# DBSCAN Clustering Parameters (Tune these based on your point cloud density)
DBSCAN_EPS = 0.05    # The maximum distance between samples for one to be considered as in the neighborhood of the other (m)
DBSCAN_MIN_SAMPLES = 15 # The number of samples (or total weight) in a neighborhood for a point to be considered as a core point

class BottlePCLLocator(Node):
    def __init__(self):
        super().__init__('bottle_pcl_locator')

        # Subscriber to the 3D Point Cloud
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/robot2/lidar_points', # Topic from your 3D node
            self.pcl_callback,
            1
        )
        
        self.get_logger().info("PCL Bottle Locator Node Started.")

    def pcl_callback(self, msg: PointCloud2):
        # Convert PointCloud2 to numpy array
        points_xyz = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if points_xyz.size == 0:
            return

        # 1. Spatial Filtering (Voxel/Range Filter)
        # Keep points within the defined bounding box (e.g., in front of the robot, off the ground)
        filtered_indices = np.where(
            (points_xyz[:, 0] > X_MIN_RANGE) &  # Forward X > min
            (points_xyz[:, 0] < X_MAX_RANGE) &  # Forward X < max
            (np.abs(points_xyz[:, 1]) < Y_MAX_DEVIATION) & # Lateral Y < max deviation
            (points_xyz[:, 2] > Z_MIN_HEIGHT)    # Z > min height (off the ground)
        )
        points_filtered = points_xyz[filtered_indices]

        if len(points_filtered) < DBSCAN_MIN_SAMPLES:
            # self.get_logger().info("Not enough points after filtering.")
            return

        # 2. Clustering (DBSCAN to separate objects)
        try:
            db = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points_filtered)
            labels = db.labels_
            num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        except ValueError:
            # Handle case where points_filtered might be ill-conditioned
            return

        if num_clusters == 0:
            # self.get_logger().info("No significant clusters found.")
            return

        # 3. Object Identification (Find the best candidate cluster)
        
        best_cluster = None
        min_distance = float('inf')
        
        for i in range(num_clusters):
            cluster_indices = np.where(labels == i)
            cluster_points = points_filtered[cluster_indices]
            
            # Use distance of the cluster's centroid as the metric
            centroid = np.mean(cluster_points, axis=0)
            
            # Distance from the robot (origin)
            distance = np.linalg.norm(centroid) 

            # Assumption: The bottle is the closest cluster that is not too large/small 
            # (No size check here for simplicity, relying on closest)
            if distance < min_distance:
                min_distance = distance
                best_cluster = cluster_points
        
        # 4. Final Centroid and Distance Calculation
        if best_cluster is not None:
            final_centroid = np.mean(best_cluster, axis=0)
            
            # Distance is the Euclidean distance from (0, 0, 0)
            final_distance = np.linalg.norm(final_centroid) 
            
            self.get_logger().info(
                f"🎯 **Closest Object Detected** (Bottle Candidate):"
                f" Centroid (X,Y,Z): ({final_centroid[0]:.2f}, {final_centroid[1]:.2f}, {final_centroid[2]:.2f}) m |"
                f" **Distance: {final_distance:.3f} m**"
            )


def main(args=None):
    # Ensure scipy/sklearn is installed: pip install numpy scipy scikit-learn
    rclpy.init(args=args)
    node = BottlePCLLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()