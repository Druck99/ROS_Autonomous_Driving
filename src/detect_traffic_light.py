#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

class TrafficLightDetector:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber(
            "/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw",
            Image, self.image_callback)
        self.red_range = ((180, 0, 0), (255, 80, 80))
        self.green_range = ((0, 180, 0), (80, 255, 80))
        self.yellow_range = ((180, 180, 0), (255, 255, 80))
        # 为了鼠标回调能用，保存当前图片
        self.current_img = None
        cv2.namedWindow("Traffic Light Detection")
        cv2.setMouseCallback("Traffic Light Detection", self.mouse_callback)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        self.current_img = img  # 方便鼠标回调用

        # 打印当前帧所有唯一颜色
        print("All colors:", np.unique(img.reshape(-1, 3), axis=0))

        # 检测红色
        lower, upper = np.array(self.red_range[0]), np.array(self.red_range[1])
        mask_red = cv2.inRange(img, lower, upper)
        num_red = np.sum(mask_red > 0)
        # 检测绿色
        lower, upper = np.array(self.green_range[0]), np.array(self.green_range[1])
        mask_green = cv2.inRange(img, lower, upper)
        num_green = np.sum(mask_green > 0)
        # 检测黄色
        lower, upper = np.array(self.yellow_range[0]), np.array(self.yellow_range[1])
        mask_yellow = cv2.inRange(img, lower, upper)
        num_yellow = np.sum(mask_yellow > 0)

        print(f"[INFO] Red: {num_red}, Green: {num_green}, Yellow: {num_yellow}")

        # 可视化（方便调试）
        vis = img.copy()
        vis[mask_red > 0] = [255, 0, 0]
        vis[mask_green > 0] = [0, 255, 0]
        vis[mask_yellow > 0] = [255, 255, 0]
        cv2.imshow("Traffic Light Detection", vis)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_img is not None:
            print(f"Pixel at ({x}, {y}): {self.current_img[y, x]}")  # 注意顺序

if __name__ == "__main__":
    rospy.init_node("traffic_light_detector")
    detector = TrafficLightDetector()
    rospy.spin()
