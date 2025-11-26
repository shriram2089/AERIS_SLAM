import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/objects_detected', 10)
        self.names_pub = self.create_publisher(String, 'detected_objects', 10)

        # Camera setup (small resolution for speed)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Model files (download manually once)
        model_dir = "/home/pi/models"
        os.makedirs(model_dir, exist_ok=True)
        self.onnx_path = os.path.join(model_dir, "yolov4-tiny.onnx")
        self.names_path = os.path.join(model_dir, "coco.names")

        if not os.path.exists(self.onnx_path) or not os.path.exists(self.names_path):
            self.get_logger().error("ONNX model or coco.names missing. Download them manually!")
            raise FileNotFoundError("yolov4-tiny.onnx or coco.names missing in /home/pi/models")

        # Load class names
        with open(self.names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        # Keep only graspable objects + humans
        self.graspable_classes = [
            'person', 'bottle', 'cup', 'cell phone', 'book', 'chair',
            'knife', 'fork', 'spoon', 'bowl', 'apple', 'orange'
        ]

        # Load ONNX model
        self.net = cv2.dnn.readNetFromONNX(self.onnx_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        # Timer ~15–20 FPS
        self.timer = self.create_timer(0.05, self.detect_objects)
        self.get_logger().info("✅ YOLOv4-Tiny ONNX detector running (Raspberry Pi 5, smooth)")

    def detect_objects(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320, 320), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward()  # shape [1, N, 6]

        boxes, confidences, class_ids = [], [], []

        for detection in outputs[0]:
            # Safe scalar conversion
            confidence = float(detection[4].item() if hasattr(detection[4], 'item') else detection[4])
            class_id = int(detection[5].item() if hasattr(detection[5], 'item') else detection[5])

            if confidence > 0.4 and class_id < len(self.classes) and self.classes[class_id] in self.graspable_classes:
                cx = int(detection[0] * w)
                cy = int(detection[1] * h)
                bw = int(detection[2] * w)
                bh = int(detection[3] * h)
                x = int(cx - bw / 2)
                y = int(cy - bh / 2)
                boxes.append([x, y, bw, bh])
                confidences.append(confidence)
                class_ids.append(class_id)

        # Non-Max Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)

        detected_objects = []
        for i in indices:
            i = i[0] if isinstance(i, (list, np.ndarray)) else i
            x, y, bw, bh = boxes[i]
            label = self.classes[class_ids[i]]
            cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            detected_objects.append(label)

        # Publish image
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # Publish names
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
