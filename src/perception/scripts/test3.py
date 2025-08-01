#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from message_filters import ApproximateTimeSynchronizer, Subscriber

class TrafficLightColorDetector:
    def __init__(self):
        rospy.init_node("traffic_light_color_detector")
        self.bridge = CvBridge()

        rgb_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", Image)
        sem_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", Image)
        depth_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", Image)

        self.sync = ApproximateTimeSynchronizer([rgb_sub, sem_sub, depth_sub], queue_size=10, slop=0.2)
        self.sync.registerCallback(self.callback)

        self.pub = rospy.Publisher("/traffic_light_status", Int32, queue_size=1, latch=True)
        self.vis_pub = rospy.Publisher("/traffic_light_roi_vis", Image, queue_size=1)

    def callback(self, rgb_msg, sem_msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        sem_img = self.bridge.imgmsg_to_cv2(sem_msg, "rgb8")
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except:
            depth_img = None
        rgb_vis = rgb_img.copy()

        sem_h, sem_w = sem_img.shape[:2]
        rgb_h, rgb_w = rgb_img.shape[:2]
        scale_x = rgb_w / sem_w
        scale_y = rgb_h / sem_h

        yellow_lower = np.array([180, 180, 0], dtype=np.uint8)
        yellow_upper = np.array([255, 255, 80], dtype=np.uint8)
        mask = cv2.inRange(sem_img, yellow_lower, yellow_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        status = -1
        any_green = False
        any_red = False

        roi_offset_x = 5  # 这里设置偏移量（向右平移像素数）

        for idx, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 5 or h < 5:
                continue
            rx1 = int(round(x * scale_x)) + roi_offset_x
            ry1 = int(round(y * scale_y))
            rx2 = int(round((x + w) * scale_x)) + roi_offset_x
            ry2 = int(round((y + h) * scale_y))

            # 防止越界
            rx1 = max(0, min(rx1, rgb_w - 1))
            rx2 = max(0, min(rx2, rgb_w - 1))
            ry1 = max(0, min(ry1, rgb_h - 1))
            ry2 = max(0, min(ry2, rgb_h - 1))
            if rx2 <= rx1 or ry2 <= ry1:
                continue

            roi = rgb_img[ry1:ry2, rx1:rx2]
            if roi.size == 0:
                continue

            # ------ 颜色判别用通道最大值法 ------
            THRESH = 120  # 通道亮度阈值，可以根据需要调整
            is_red = (roi[:,:,0] > THRESH) & (roi[:,:,0] > roi[:,:,1]) & (roi[:,:,0] > roi[:,:,2])
            is_green = (roi[:,:,1] > THRESH) & (roi[:,:,1] > roi[:,:,0]) & (roi[:,:,1] > roi[:,:,2])
            n_red = np.sum(is_red)
            n_green = np.sum(is_green)
            # ----------------------------------

            # rospy.loginfo(f"ROI #{idx}: n_red={n_red}, n_green={n_green}")

            if n_green > 0:
                any_green = True
                color = (0, 255, 0)  # 绿框
            elif n_red > 0:
                any_red = True
                color = (0, 0, 255)  # 红框
            else:
                color = (255, 255, 0)  # 黄框

            # 框出所有检测到的红绿灯
            cv2.rectangle(rgb_vis, (rx1, ry1), (rx2, ry2), color, 2)
            # 可选：在框旁写文字
            if n_green > 0:
                cv2.putText(rgb_vis, "Green", (rx1, ry1 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            elif n_red > 0:
                cv2.putText(rgb_vis, "Red", (rx1, ry1 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        if any_green:
            status = 1
        elif any_red:
            status = 0
        else:
            status = -1

        self.pub.publish(Int32(status))
        # rospy.loginfo(f"[Traffic Light Status]: {status}")

        try:
            vis_msg = self.bridge.cv2_to_imgmsg(rgb_vis, encoding="rgb8")
            vis_msg.header = rgb_msg.header
            self.vis_pub.publish(vis_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish ROI visualization: {e}")

if __name__ == "__main__":
    TrafficLightColorDetector()
    rospy.spin()
