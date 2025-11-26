import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import Buffer, TransformListener
import struct

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.sub_lidar = self.create_subscription(PointCloud2, '/robot2/lidar_points', self.lidar_callback, 10)
        self.sub_yolo  = self.create_subscription(Detection2DArray, '/robot2/yolo_detections', self.yolo_callback, 10)
        self.sub_caminfo = self.create_subscription(CameraInfo, '/camera_info', self.caminfo_callback, 10)

        self.fused_pub = self.create_publisher(MarkerArray, '/fused_visualization', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lidar_points = None
        self.camera_info = None
        self.detections = None

    def caminfo_callback(self, msg):
        self.camera_info = msg

    def lidar_callback(self, msg):
        # Convert PointCloud2 to Nx3 NumPy array manually
        self.lidar_points = self.pointcloud2_to_array(msg)
        if self.lidar_points.size == 0:
            self.lidar_points = None
        self.try_fuse()

    def yolo_callback(self, msg):
        self.detections = msg
        self.try_fuse()

    def try_fuse(self):
        if self.lidar_points is None or self.detections is None or self.camera_info is None:
            return

        fused_markers = MarkerArray()
        points_3d = self.lidar_points

        u, v, depth = self.project_to_image(points_3d, self.camera_info)

        for det in self.detections.detections:
            bbox = det.bbox
            class_id = det.results[0].id if det.results else "unknown"

            cx = bbox.center.x
            cy = bbox.center.y
            w = bbox.size_x / 2
            h = bbox.size_y / 2

            mask = (u > cx - w) & (u < cx + w) & (v > cy - h) & (v < cy + h)
            if np.sum(mask) < 5:
                continue

            obj_points = points_3d[mask]
            centroid = np.mean(obj_points, axis=0)

            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = float(centroid[2])
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8
            marker.id = len(fused_markers.markers)
            fused_markers.markers.append(marker)

        self.fused_pub.publish(fused_markers)

    def project_to_image(self, points, cam_info):
        if points.size == 0:
            return np.array([]), np.array([]), np.array([])

        fx = cam_info.k[0]
        fy = cam_info.k[4]
        cx = cam_info.k[2]
        cy = cam_info.k[5]

        X, Y, Z = points[:,0], points[:,1], points[:,2]
        valid = Z > 0
        u = np.zeros_like(Z)
        v = np.zeros_like(Z)
        u[valid] = fx * X[valid] / Z[valid] + cx
        v[valid] = fy * Y[valid] / Z[valid] + cy
        return u, v, Z

    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 msg to Nx3 NumPy array (manual, avoids pc2 module)"""
        dtype_list = []
        for f in cloud_msg.fields:
            if f.datatype == 7:  # FLOAT32
                dtype_list.append((f.name, np.float32))
            elif f.datatype == 2:  # UINT8
                dtype_list.append((f.name, np.uint8))
            else:
                self.get_logger().warn(f"Unsupported PointField type: {f.datatype}")
                return np.array([])

        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.dtype(dtype_list))
        points = np.zeros((cloud_msg.width * cloud_msg.height, 3), dtype=np.float32)
        for i, name in enumerate(['x','y','z']):
            if name in cloud_arr.dtype.names:
                points[:,i] = cloud_arr[name]
        return points

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()