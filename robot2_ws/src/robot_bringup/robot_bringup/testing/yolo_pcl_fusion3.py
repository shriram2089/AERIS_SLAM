#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import cv2

class LidarFusionDebug(Node):
    def __init__(self):
        super().__init__('lidar_fusion_detector')
        self.get_logger().info("✅ LiDAR fusion debug node running")

        # Subscribers
        self.create_subscription(Image, '/camera/image_raw', self.img_callback, 10)
        self.create_subscription(PointCloud2, '/robot2/lidar_points', self.pc_callback, 10)
        self.create_subscription(String, '/camera/object_boxes', self.box_callback, 10)

        self.bridge = CvBridge()
        self.latest_img = None
        self.latest_boxes = []
        self.latest_pc = None

    # ---- Callbacks ----
    def img_callback(self, msg):
        print("📸 Got image")
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print("❌ Image conversion failed:", e)
        self.compute_and_visualize()

    def box_callback(self, msg):
        print("📦 Got boxes:", msg.data)
        try:
            self.latest_boxes = []
            for line in msg.data.strip().split("\n"):
                parts = line.split(',')
                if len(parts) == 5:
                    label, x1, y1, x2, y2 = parts
                    self.latest_boxes.append((label, int(x1), int(y1), int(x2), int(y2)))
        except Exception as e:
            print("❌ Box parsing failed:", e)
        self.compute_and_visualize()

    def pc_callback(self, msg):
        print("💠 Got point cloud")
        try:
            self.latest_pc = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            print("❌ PCL read failed:", e)
        self.compute_and_visualize()

    # ---- Processing and Visualization ----
    def compute_and_visualize(self):
        if self.latest_img is None or self.latest_pc is None or len(self.latest_boxes) == 0:
            return  # wait until we have all 3 inputs

        img = self.latest_img.copy()
        pc_points = np.array(self.latest_pc)
        print(f"🧮 LiDAR points: {len(pc_points)} | Boxes: {len(self.latest_boxes)} | Image: {img.shape}")

        # Simple fake projection for debug (scale & offset for visualization)
        if len(pc_points) > 0:
            scale = 50  # visual scaling
            for (x, y, z) in pc_points[::200]:  # sample points for speed
                u = int(x * scale + img.shape[1] // 2)
                v = int(-y * scale + img.shape[0] // 2)
                if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                    cv2.circle(img, (u, v), 1, (0, 255, 0), -1)

        # Draw bounding boxes
        for label, x1, y1, x2, y2 in self.latest_boxes:
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Save debug image instead of imshow (headless friendly)
        save_path = "/home/pi/fusion_debug.jpg"
        cv2.imwrite(save_path, img)
        print(f"✅ Saved fusion debug frame to {save_path}")

# ---- Main ----
def main(args=None):
    rclpy.init(args=args)
    node = LidarFusionDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
