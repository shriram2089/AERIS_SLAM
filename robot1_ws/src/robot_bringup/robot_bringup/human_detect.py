import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import urllib.request

class HumanDetector(Node):
    def __init__(self):
        super().__init__('human_detector')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, 'camera/human_detected', 10)
        self.count_pub = self.create_publisher(Int32, 'human_count', 10)

        # --- camera setup ---
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # --- model files ---
        model_dir = "/home/pi/models"
        os.makedirs(model_dir, exist_ok=True)
        self.cfg_path = os.path.join(model_dir, "yolov4-tiny.cfg")
        self.weights_path = os.path.join(model_dir, "yolov4-tiny.weights")
        self.names_path = os.path.join(model_dir, "coco.names")

        # --- download if missing ---
        if not os.path.exists(self.cfg_path):
            urllib.request.urlretrieve(
                "https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg/yolov4-tiny.cfg",
                self.cfg_path)
        if not os.path.exists(self.weights_path):
            urllib.request.urlretrieve(
                "https://pjreddie.com/media/files/yolov4-tiny.weights",
                self.weights_path)
        if not os.path.exists(self.names_path):
            urllib.request.urlretrieve(
                "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names",
                self.names_path)

        # --- load model ---
        self.net = cv2.dnn.readNetFromDarknet(self.cfg_path, self.weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        with open(self.names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # --- timer ---
        self.timer = self.create_timer(0.2, self.detect_humans)
        self.get_logger().info("✅ YOLOv4-Tiny human detector running on Raspberry Pi 5")

    def detect_humans(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320, 320), swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        h, w = frame.shape[:2]
        boxes, confidences = [], []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if self.classes[class_id] == "person" and confidence > 0.4:
                    center_x, center_y, bw, bh = (
                        int(detection[0] * w),
                        int(detection[1] * h),
                        int(detection[2] * w),
                        int(detection[3] * h),
                    )
                    x, y = int(center_x - bw / 2), int(center_y - bh / 2)
                    boxes.append([x, y, bw, bh])
                    confidences.append(float(confidence))

        # Non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.4, 0.3)

        human_count = len(indices)
        for i in indices:
            x, y, bw, bh = boxes[i]
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(frame, "Human", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_pub.publish(img_msg)

        msg = Int32()
        msg.data = human_count
        self.count_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
