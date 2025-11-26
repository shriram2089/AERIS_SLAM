#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class WhiteBoxDetector(Node):
    def __init__(self):
        super().__init__('white_box_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/box_detector/image', 10)
        self.get_logger().info("📦 White Box Detector started")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # detect white using low saturation and high brightness
        lower = np.array([0, 0, 180])
        upper = np.array([180, 60, 255])
        mask = cv2.inRange(hsv, lower, upper)

        # cleanup
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected = False

        for c in contours:
            area = cv2.contourArea(c)
            if area > 1000:  # ignore small specks
                approx = cv2.approxPolyDP(c, 0.04 * cv2.arcLength(c, True), True)
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = w / float(h)
                    if 0.7 < aspect_ratio < 1.3:
                        cv2.drawContours(img, [approx], -1, (0, 255, 0), 2)
                        cv2.putText(img, "White Box", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        detected = True

        if detected:
            self.get_logger().info("✅ White box detected!")

        # publish image
        self.pub.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = WhiteBoxDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
