import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import cv2

class LiDARFusion(Node):
    def __init__(self):
        super().__init__('lidar_fusion_detector')

        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(Image, 'camera/objects_detected', self.image_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/robot2/lidar_points', self.pc_callback, 10)

        # Publisher
        self.dist_pub = self.create_publisher(String, 'object_distance', 10)

        # Latest data
        self.latest_frame = None
        self.latest_pc = None

        # Previous distances for smoothing
        self.prev_distances = {}

        # Camera FOV (assume known)
        self.h_fov = 60.0
        self.v_fov = 45.0

        self.get_logger().info("✅ LiDAR fusion node running")

    def image_callback(self, msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.compute_distances()

    def pc_callback(self, msg):
        self.latest_pc = msg
        self.compute_distances()

    def compute_distances(self):
        if self.latest_frame is None or self.latest_pc is None:
            return

        frame = self.latest_frame
        h, w, _ = frame.shape

        # Convert PointCloud2 to Nx3 numpy array
        points_list = list(pc2.read_points(self.latest_pc, field_names=("x", "y", "z"), skip_nans=True))
        if len(points_list) == 0:
            return

        pc_points = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
        if pc_points.ndim == 1:
            pc_points = pc_points.reshape(1, 3)

        # Mask green rectangles (YOLO annotated)
        mask = cv2.inRange(frame, (0, 250, 0), (0, 255, 0))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        distances = {}

        for cnt in contours:
            x, y, bw, bh = cv2.boundingRect(cnt)
            cx = x + bw/2
            cy = y + bh/2

            # Compute angles of LiDAR points
            angles_h = np.degrees(np.arctan2(pc_points[:,1], pc_points[:,0])) + self.h_fov/2
            angles_v = np.degrees(np.arctan2(pc_points[:,2], pc_points[:,0])) + self.v_fov/2

            # Bounding box angle ranges
            box_h_min = x / w * self.h_fov
            box_h_max = (x + bw) / w * self.h_fov
            box_v_min = y / h * self.v_fov
            box_v_max = (y + bh) / h * self.v_fov

            # Filter LiDAR points inside bounding box
            pc_filtered = pc_points[
                (pc_points[:,0] > 0) &
                (angles_h >= box_h_min) & (angles_h <= box_h_max) &
                (angles_v >= box_v_min) & (angles_v <= box_v_max)
            ]

            if pc_filtered.shape[0] > 0:
                dist = float(np.mean(np.linalg.norm(pc_filtered, axis=1)))
                # Assign a class label based on centroid proximity (optional: you can refine)
                label = f"object_{x}_{y}"

                # Temporal smoothing
                if label in self.prev_distances:
                    dist = 0.5 * dist + 0.5 * self.prev_distances[label]

                distances[label] = dist

        self.prev_distances = distances.copy()

        if distances:
            dist_msg = String()
            dist_msg.data = ", ".join([f"{k}:{v:.2f}m" for k,v in distances.items()])
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
