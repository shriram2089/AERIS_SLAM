import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class LiDARFusion(Node):
    def __init__(self):
        super().__init__('lidar_fusion_detector')

        # Subscribe to YOLO bounding boxes topic (custom String: "label,x1,y1,x2,y2")
        self.box_sub = self.create_subscription(String, 'camera/object_boxes', self.box_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/robot2/lidar_points', self.pc_callback, 10)

        # Publisher for distances
        self.dist_pub = self.create_publisher(String, 'object_distance', 10)

        self.latest_boxes = []  # [(label, x1, y1, x2, y2), ...]
        self.latest_pc = None
        self.prev_distances = {}

        # Camera FOV (degrees)
        self.h_fov = 60.0
        self.v_fov = 45.0

        self.get_logger().info("✅ LiDAR fusion node running")

    def box_callback(self, msg):
        # Parse all boxes in message: "label1,x1,y1,x2,y2;label2,x1,y1,x2,y2;..."
        self.latest_boxes = []
        for obj_str in msg.data.split(';'):
            if not obj_str:
                continue
            parts = obj_str.split(',')
            if len(parts) != 5:
                continue
            label, x1, y1, x2, y2 = parts
            self.latest_boxes.append((label, int(x1), int(y1), int(x2), int(y2)))
        self.compute_distances()

    def pc_callback(self, msg):
        self.latest_pc = msg
        self.compute_distances()

    def compute_distances(self):
        if self.latest_pc is None or not self.latest_boxes:
            return

        # Convert PointCloud2 to Nx3 numpy array
        points_list = list(pc2.read_points(self.latest_pc, field_names=("x","y","z"), skip_nans=True))
        if not points_list:
            return
        # ✅ Extract x, y, z from structured array
        pc_points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
        if pc_points.ndim == 1:
            pc_points = pc_points.reshape(1,3)

        distances = {}

        for label, x1, y1, x2, y2 in self.latest_boxes:
            # Compute angles for all points
            angles_h = np.degrees(np.arctan2(pc_points[:,1], pc_points[:,0])) + self.h_fov/2
            angles_v = np.degrees(np.arctan2(pc_points[:,2], pc_points[:,0])) + self.v_fov/2

            # Map bounding box pixels to angle ranges
            w, h = 320, 240  # Assuming YOLO image size
            box_h_min = x1 / w * self.h_fov
            box_h_max = x2 / w * self.h_fov
            box_v_min = y1 / h * self.v_fov
            box_v_max = y2 / h * self.v_fov

            # Filter LiDAR points inside bounding box
            pc_filtered = pc_points[
                (pc_points[:,0] > 0) &
                (angles_h >= box_h_min) & (angles_h <= box_h_max) &
                (angles_v >= box_v_min) & (angles_v <= box_v_max)
            ]

            if pc_filtered.shape[0] > 0:
                dist = float(np.mean(np.linalg.norm(pc_filtered, axis=1)))
                # Temporal smoothing
                if label in self.prev_distances:
                    dist = 0.5*dist + 0.5*self.prev_distances[label]
                distances[label] = dist

        self.prev_distances = distances.copy()

        if distances:
            dist_msg = String()
            dist_msg.data = ";".join([f"{k}:{v:.2f}m" for k,v in distances.items()])
            self.dist_pub.publish(dist_msg)
            print("Distances:", dist_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = LiDARFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
