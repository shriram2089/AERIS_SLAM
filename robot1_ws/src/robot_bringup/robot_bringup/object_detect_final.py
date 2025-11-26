import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, '/robot1/camera/objects_detected', 10)
        self.names_pub = self.create_publisher(String, '/robot1/detected_objects', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '/robot1/yolo_detections', 10)

        # Camera setup
        #self.cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture("/dev/camera0", cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")

        # Allowed classes
        self.allowed_classes = {
            "chair", "person", "bottle", "cup", "book", "box", "laptop",
            "keyboard", "cell phone", "tv", "remote"
        }

        # Timer
        self.timer = self.create_timer(0.15, self.detect_objects)
        self.get_logger().info("✅ YOLOv8n object detector running with Detection2DArray output (Jazzy-safe)")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Resize for faster inference
        small = cv2.resize(frame, (320, 240))

        # YOLO inference
        results = self.model.predict(
            small,
            imgsz=320,
            device='cpu',
            conf=0.4,
            verbose=False
        )[0]

        detected_objects = []
        annotated_frame = small.copy()
        detection_array_msg = Detection2DArray()
        detection_array_msg.header.stamp = self.get_clock().now().to_msg()
        detection_array_msg.header.frame_id = "camera_frame"

        if results.boxes:
            for box, cls, conf in zip(results.boxes.xyxy, results.boxes.cls, results.boxes.conf):
                label = self.model.names[int(cls)]
                if label not in self.allowed_classes:
                    continue

                x1, y1, x2, y2 = map(float, box)

                # Draw bounding box on frame
                cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (int(x1), int(y1) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                detected_objects.append(label)

                # Create Detection2D message
                detection_msg = Detection2D()
                detection_msg.header = detection_array_msg.header

                # ✅ Bounding box center (Jazzy-safe)
                detection_msg.bbox.center.position.x = float((x1 + x2) / 2.0)
                detection_msg.bbox.center.position.y = float((y1 + y2) / 2.0)
                detection_msg.bbox.center.theta = 0.0
                detection_msg.bbox.size_x = float(x2 - x1)
                detection_msg.bbox.size_y = float(y2 - y1)

                # ✅ Object hypothesis (Jazzy-safe)
                hypothesis = ObjectHypothesisWithPose()

                # Jazzy uses 'hypothesis.class_id' and 'hypothesis.score'
                if hasattr(hypothesis, 'hypothesis'):
                    if hasattr(hypothesis.hypothesis, 'class_id'):
                        hypothesis.hypothesis.class_id = label
                    if hasattr(hypothesis.hypothesis, 'score'):
                        hypothesis.hypothesis.score = float(conf)
                else:
                    # Fallback for other distros
                    if hasattr(hypothesis, 'class_id'):
                        hypothesis.class_id = label
                    if hasattr(hypothesis, 'score'):
                        hypothesis.score = float(conf)

                detection_msg.results.append(hypothesis)
                detection_array_msg.detections.append(detection_msg)

        # Publish annotated image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8'))

        # Publish detected object names
        if detected_objects:
            msg = String()
            msg.data = ", ".join(sorted(set(detected_objects)))
            self.names_pub.publish(msg)
            self.get_logger().info(f"Detected: {msg.data}")

        # Publish Detection2DArray
        if detection_array_msg.detections:
            self.detections_pub.publish(detection_array_msg)

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
