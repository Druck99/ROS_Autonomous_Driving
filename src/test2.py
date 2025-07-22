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

        img_cx = rgb_w // 2
        img_cy = rgb_h // 2
        all_rois = []
        min_area = 800

        for idx, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 5 or h < 5:
                continue
            x1 = x
            y1 = y
            x2 = x + w
            y2 = y + h
            rx1 = int(round(x1 * scale_x))
            ry1 = int(round(y1 * scale_y))
            rx2 = int(round(x2 * scale_x))
            ry2 = int(round(y2 * scale_y))
            area = (rx2 - rx1) * (ry2 - ry1)
            cx = int((x + x + w) / 2 * scale_x)
            cy = int((y + y + h) / 2 * scale_y)
            roi = rgb_img[ry1:ry2, rx1:rx2]
            dist2center = np.sqrt((cx - img_cx) ** 2 + (cy - img_cy) ** 2)
            all_rois.append({
                'id': idx,
                'rect': (rx1, ry1, rx2, ry2),
                'center': (cx, cy),
                'area': area,
                'roi': roi,
                'dist2center': dist2center,
                'orig_bbox': (x, y, w, h),
            })

        # 可视化所有ROI
        for roi_info in all_rois:
            rx1, ry1, rx2, ry2 = roi_info['rect']
            cv2.rectangle(rgb_vis, (rx1, ry1), (rx2, ry2), (0,255,0), 2)
            rospy.loginfo(f"TL id={roi_info['id']} at ({rx1},{ry1},{rx2-rx1},{ry2-ry1}), center={roi_info['center']}, area={roi_info['area']}, dist2center={roi_info['dist2center']:.1f}")

        # 筛选主车道ROI
        main_roi = None
        for roi in all_rois:
            if roi['area'] >= min_area and abs(roi['center'][0] - img_cx) < rgb_w * 0.25:
                if (main_roi is None) or (roi['dist2center'] < main_roi['dist2center']):
                    main_roi = roi

        detected_status = -1
        distance = -1
        if main_roi is not None:
            roi = main_roi['roi']
            if roi.size > 0:
                hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
                gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
                mask_bright = gray > 120  # 只看亮像素
                h, s, v = hsv[:,:,0][mask_bright], hsv[:,:,1][mask_bright], hsv[:,:,2][mask_bright]
                green_mask = (h > 35) & (h < 95) & (s > 50) & (v > 100)
                n_green = np.sum(green_mask)
                n_bright = np.sum(mask_bright)
                rospy.loginfo(f"[Main TL id={main_roi['id']}] ROI {main_roi['rect']}, area={main_roi['area']}, n_bright={n_bright}, n_green={n_green}")

                # 极简判决：只要绿色像素>20就判绿灯，否则判红灯
                if n_bright > 5 and n_green > 20:
                    detected_status = 1
                    rospy.loginfo(f"Green detected: {n_green}/{n_bright}")
                else:
                    detected_status = 0
                    rospy.loginfo(f"Red detected: n_green={n_green}, n_bright={n_bright}")

                # 输出深度信息
                if depth_img is not None and depth_img.shape[:2] == rgb_img.shape[:2]:
                    rx1, ry1, rx2, ry2 = main_roi['rect']
                    depth_roi = depth_img[ry1:ry2, rx1:rx2]
                    valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 100)]
                    if valid_depth.size > 0:
                        distance = np.median(valid_depth)
                        rospy.loginfo(f"[Main TL id={main_roi['id']}] 距离车辆约: {distance:.2f} 米")
                    else:
                        rospy.loginfo("该区域无有效深度值")
            else:
                rospy.loginfo("ROI empty, skip.")
        else:
            rospy.loginfo("No valid main traffic light ROI found.")

        if detected_status != -1:
            self.pub.publish(Int32(detected_status))
            rospy.loginfo(f"[Traffic Light Status]: {detected_status}")
        else:
            rospy.loginfo("[Traffic Light Status]: Not found")

        try:
            vis_msg = self.bridge.cv2_to_imgmsg(rgb_vis, encoding="rgb8")
            vis_msg.header = rgb_msg.header
            self.vis_pub.publish(vis_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish ROI visualization: {e}")

if __name__ == "__main__":
    TrafficLightColorDetector()
    rospy.spin()
