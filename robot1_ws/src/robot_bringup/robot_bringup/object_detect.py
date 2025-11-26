import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/objects_detected', 10)
        self.names_pub = self.create_publisher(String, 'detected_objects', 10)

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Load YOLOv8 nano model (lightweight)
        self.model = YOLO("yolov8n.pt")  # Download once and keep locally

        # Timer ~5–10 Hz
        self.timer = self.create_timer(0.2, self.detect_objects)
        self.get_logger().info("✅ YOLOv8n object detector running (Raspberry Pi 5, smooth CPU)")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # YOLOv8 inference (optimized for speed)
        results = self.model.predict(
            frame,
            imgsz=320,        # smaller input for speed
            device='cpu',     # CPU only
            conf=0.4,
            verbose=False
        )[0]

        detected_objects = []
        annotated_frame = frame.copy()

        # Draw bounding boxes and filter objects
        if results.boxes:
            for box, cls in zip(results.boxes.xyxy, results.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                label = self.model.names[int(cls)]
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                detected_objects.append(label)

        # Publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

        # Publish object names
        if detected_objects:
            msg = String()
            msg.data = ", ".join(detected_objects)
            self.names_pub.publish(msg)
            print("Detected:", ", ".join(detected_objects))

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
