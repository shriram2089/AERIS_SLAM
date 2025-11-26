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

        # Camera setup (keep small but better quality)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Load YOLOv8 nano model
        self.model = YOLO("yolov8n.pt")

        # Common indoor / robotics classes
        self.allowed_classes = {
            "chair", "person", "bottle", "cup", "book", "box", "laptop",
            "keyboard", "cell phone", "tv", "remote"
        }

        # ~6–8 Hz (every 0.15 s)
        self.timer = self.create_timer(0.15, self.detect_objects)
        self.get_logger().info("✅ YOLOv8n object detector running on Raspberry Pi 5 (balanced mode)")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Downscale for speed but keep color info
        small = cv2.resize(frame, (320, 240))

        # YOLO inference (CPU optimized)
        results = self.model.predict(
            small,
            imgsz=320,
            device='cpu',
            conf=0.4,
            verbose=False
        )[0]

        detected_objects = []
        annotated_frame = small.copy()

        if results.boxes:
            for box, cls in zip(results.boxes.xyxy, results.boxes.cls):
                label = self.model.names[int(cls)]
                # if label not in self.allowed_classes:
                #     continue

                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                detected_objects.append(label)

        # Publish annotated frame and names
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))
        if detected_objects:
            msg = String()
            msg.data = ", ".join(sorted(set(detected_objects)))
            self.names_pub.publish(msg)
            print("Detected:", msg.data)

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
