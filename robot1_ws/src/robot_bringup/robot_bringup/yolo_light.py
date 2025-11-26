import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.detect_pub = self.create_publisher(Image, 'camera/image_detected', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        # ---- YOLOv4-Tiny setup ----
        # self.net = cv2.dnn.readNet('yolov4-tiny.weights', 'yolov4-tiny.cfg')
        # self.net = cv2.dnn.readNet('yolov3-tiny.weights', 'yolov3-tiny.cfg')
        self.net = cv2.dnn.readNetFromONNX('yolov3-tiny.onnx')

        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        with open('coco.names', 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.output_layers = self.net.getUnconnectedOutLayersNames()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # ---- Prepare image ----
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (320, 320), swapRB=True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # ---- Process detections ----
        h, w = frame.shape[:2]
        boxes, confidences, class_ids = [], [], []

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x, center_y, bw, bh = (detection[0:4] * np.array([w, h, w, h])).astype(int)
                    x = int(center_x - bw / 2)
                    y = int(center_y - bh / 2)
                    boxes.append([x, y, int(bw), int(bh)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        detected_frame = frame.copy()
        for i in indices:
            i = i[0]
            x, y, bw, bh = boxes[i]
            label = self.classes[class_ids[i]]
            conf = confidences[i]
            cv2.rectangle(detected_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.putText(detected_frame, f"{label} {conf:.2f}", (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        # ---- Publish images ----
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
        self.detect_pub.publish(self.bridge.cv2_to_imgmsg(detected_frame, 'bgr8'))

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

if __name__ == '__main__':
    main()
