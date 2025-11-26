import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.detect_pub = self.create_publisher(Image, 'camera/image_detected', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz = lighter

        # Lightweight setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Use the nano model (lighter than yolov8n)
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        self.get_logger().info("YOLO light node started on Raspberry Pi")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No frame captured!")
            return

        # YOLO inference (small image size, CPU only)
        results = self.model.predict(
            frame,
            imgsz=320,       # smaller = faster
            device='cpu',
            verbose=False,
            conf=0.4         # confidence threshold
        )[0]

        # Draw boxes (very lightweight)
        for box in results.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = map(int, box[:4])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

        # Publish both images
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.detect_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
