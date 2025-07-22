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

        self.sync = ApproximateTimeSynchronizer([rgb_sub, sem_sub], queue_size=10, slop=0.2)
        self.sync.registerCallback(self.callback)

        self.pub = rospy.Publisher("/traffic_light_status", Int32, queue_size=1, latch=True)
        self.vis_pub = rospy.Publisher("/traffic_light_roi_vis", Image, queue_size=1)

    def callback(self, rgb_msg, sem_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        sem_img = self.bridge.imgmsg_to_cv2(sem_msg, "rgb8")
        rgb_vis = rgb_img.copy()

        sem_h, sem_w = sem_img.shape[:2]
        rgb_h, rgb_w = rgb_img.shape[:2]
        scale_x = rgb_w / sem_w
        scale_y = rgb_h / sem_h

        yellow_lower = np.array([180, 180, 0], dtype=np.uint8)
        yellow_upper = np.array([255, 255, 80], dtype=np.uint8)
        mask = cv2.inRange(sem_img, yellow_lower, yellow_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        total_red_pixels = 0
        total_green_pixels = 0
        total_pixels = 0

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 5 or h < 5:
                continue
            pad = 2
            x1 = max(x - pad, 0)
            y1 = max(y - pad, 0)
            x2 = min(x + w + pad, sem_w)
            y2 = min(y + h + pad, sem_h)
            rx1 = int(round(x1 * scale_x))
            ry1 = int(round(y1 * scale_y))
            rx2 = int(round(x2 * scale_x))
            ry2 = int(round(y2 * scale_y))
            roi = rgb_img[ry1:ry2, rx1:rx2]

            # 可视化绿色框
            cv2.rectangle(rgb_vis, (rx1, ry1), (rx2, ry2), (0,255,0), 2)

            if roi.size == 0:
                continue

            h_roi, w_roi = roi.shape[:2]
            # 只取竖直中心宽度（如4列），减少杂色干扰
            cx = w_roi // 2
            band_w = max(w_roi // 8, 2)
            center_band = roi[:, cx-band_w:cx+band_w]

            # 分上下半（上：红灯，下：绿灯）
            upper_half = center_band[:h_roi//2, :]
            lower_half = center_band[h_roi//2:, :]

            # 灰度提亮+颜色判别
            def count_color_pixels(img, color):
                gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                mask_bright = gray > 150
                pixels = img[mask_bright]
                if pixels.shape[0] == 0:
                    return 0
                R = pixels[:,0]
                G = pixels[:,1]
                B = pixels[:,2]
                if color == 'red':
                    return np.sum((R > G+15) & (R > B+15))
                else:
                    return np.sum((G > R+15) & (G > B+15))

            red_pixels = count_color_pixels(upper_half, 'red')
            green_pixels = count_color_pixels(lower_half, 'green')

            total_red_pixels += red_pixels
            total_green_pixels += green_pixels
            total_pixels += center_band.shape[0]*center_band.shape[1]

            rospy.loginfo(f"ROI at ({rx1},{ry1},{rx2-rx1},{ry2-ry1}), red_pixels={red_pixels}, green_pixels={green_pixels}")

        detected_status = -1
        min_area = 3   # 只要亮点稍多就判定，灵敏些
        if total_red_pixels > min_area or total_green_pixels > min_area:
            if total_green_pixels > total_red_pixels:
                detected_status = 1  # 绿灯
                rospy.loginfo(f"Green dominant: {total_green_pixels}/{total_pixels} pixels")
            else:
                detected_status = 0  # 红灯
                rospy.loginfo(f"Red dominant: {total_red_pixels}/{total_pixels} pixels")
        else:
            rospy.loginfo("No significant red or green detected.")

        if detected_status != -1:
            self.pub.publish(Int32(detected_status))
            rospy.loginfo(f"[Traffic Light Status]: {detected_status}")
        else:
            rospy.loginfo("[Traffic Light Status]: Not found")

        # 发布画框图片
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(rgb_vis, encoding="rgb8")
            vis_msg.header = rgb_msg.header
            self.vis_pub.publish(vis_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish ROI visualization: {e}")

if __name__ == "__main__":
    TrafficLightColorDetector()
    rospy.spin()
