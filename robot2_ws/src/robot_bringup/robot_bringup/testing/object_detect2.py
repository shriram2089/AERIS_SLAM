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
        self.box_pub = self.create_publisher(String, 'camera/object_boxes', 10)  # NEW

        # Camera setup
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

        self.timer = self.create_timer(0.15, self.detect_objects)
        self.get_logger().info("✅ YOLOv8n object detector running on Raspberry Pi 5 (balanced mode)")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        small = cv2.resize(frame, (320, 240))
        results = self.model.predict(small, imgsz=320, device='cpu', conf=0.4, verbose=False)[0]

        detected_objects = []
        annotated_frame = small.copy()
        boxes_str_list = []

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
                boxes_str_list.append(f"{label},{x1},{y1},{x2},{y2}")  # NEW

        # Publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

        # Publish detected object labels
        if detected_objects:
            names_msg = String()
            names_msg.data = ", ".join(sorted(set(detected_objects)))
            self.box_pub.publish(String(data=";".join(boxes_str_list)))  # NEW structured boxes
            print("Detected:", names_msg.data)
            print("Boxes:", ";".join(boxes_str_list))

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
