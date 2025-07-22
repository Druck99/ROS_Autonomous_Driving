#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from message_filters import ApproximateTimeSynchronizer, Subscriber

class CarDetector:
    def __init__(self):
        rospy.init_node("car_detector")
        self.bridge = CvBridge()

        rgb_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/RGBCameraLeft/image_raw", Image)
        sem_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/SemanticCamera/image_raw", Image)
        depth_sub = Subscriber("/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw", Image)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, sem_sub, depth_sub], queue_size=10, slop=0.2)
        self.sync.registerCallback(self.callback)

        self.dist_pub = rospy.Publisher("/car_detection_distances", Float32MultiArray, queue_size=1)
        self.vis_pub = rospy.Publisher("/car_detection_roi_vis", Image, queue_size=1)

        self.first_run = True # 用于首次输出语义颜色

    def callback(self, rgb_msg, sem_msg, depth_msg):
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        sem_img = self.bridge.imgmsg_to_cv2(sem_msg, "bgr8")
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except:
            depth_img = None
        rgb_vis = rgb_img.copy()

        # 首次运行时打印语义图像所有颜色
        if self.first_run:
            unique_colors = np.unique(sem_img.reshape(-1, 3), axis=0)
            rospy.loginfo(f"[DEBUG] Unique semantic colors: {unique_colors}")
            self.first_run = False

        sem_h, sem_w = sem_img.shape[:2]
        rgb_h, rgb_w = rgb_img.shape[:2]
        scale_x = rgb_w / sem_w
        scale_y = rgb_h / sem_h

        # 建议阈值: 先用较宽范围，后续可根据打印出来的颜色调节
        car_lower = np.array([0, 0, 128], dtype=np.uint8)
        car_upper = np.array([60, 60, 255], dtype=np.uint8)
        mask = cv2.inRange(sem_img, car_lower, car_upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo(f"Contours found: {len(contours)}")
        rospy.loginfo(f"Red pixel count in mask: {np.sum(mask>0)}")

        car_distances = []

        for idx, cnt in enumerate(contours):
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 10 or h < 10:
                continue
            x1 = x
            y1 = y
            x2 = x + w
            y2 = y + h
            rx1 = int(round(x1 * scale_x))
            ry1 = int(round(y1 * scale_y))
            rx2 = int(round(x2 * scale_x))
            ry2 = int(round(y2 * scale_y))

            # 框出所有车辆（黄色框，BGR格式）
            cv2.rectangle(rgb_vis, (rx1, ry1), (rx2, ry2), (0, 255, 255), 2)
            rospy.loginfo(f"Car id={idx} at ({rx1},{ry1},{rx2-rx1},{ry2-ry1}), orig_bbox=({x},{y},{w},{h})")

            # ==============================
            # 输出深度信息（支持多类型/单位自动识别和转换为米）
            if (depth_img is not None and
                depth_img.shape[:2] == rgb_img.shape[:2]):
                depth_roi = depth_img[ry1:ry2, rx1:rx2]
                # --------- 类型和单位适配 ---------
                if depth_roi.dtype == np.uint16:
                    dmax = depth_roi.max()
                    if dmax > 10000:
                        valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 100000)]
                        scale = 1.0 / 1000   # 毫米
                    elif dmax > 200:
                        valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 10000)]
                        scale = 1.0 / 100    # 厘米
                    else:
                        valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 100)]
                        scale = 1.0          # 米
                elif np.issubdtype(depth_roi.dtype, np.floating):
                    valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 100)]
                    scale = 1.0
                else:
                    valid_depth = depth_roi[(depth_roi > 0) & (depth_roi < 255)]
                    scale = 1.0
                # --------- 计算距离（以米为单位）---------
                if valid_depth.size > 0:
                    distance = float(np.median(valid_depth)) * scale
                    car_distances.append(distance)
                    cv2.putText(rgb_vis, f"{distance:.2f}m", (rx1, ry1-7), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                    rospy.loginfo(f"Car id={idx} 距离: {distance:.2f} 米")
                else:
                    rospy.loginfo(f"Car id={idx} 该区域无有效深度值")
            else:
                rospy.logwarn("语义和深度分辨率不一致，无法配准")
            # ==============================

        # 发布所有车辆距离
        dist_msg = Float32MultiArray()
        dist_msg.data = car_distances
        self.dist_pub.publish(dist_msg)
        rospy.loginfo(f"已发布 {len(car_distances)} 个车辆距离: {car_distances}")

        # 发布可视化图像
        try:
            vis_msg = self.bridge.cv2_to_imgmsg(rgb_vis, encoding="bgr8")
            vis_msg.header = rgb_msg.header
            self.vis_pub.publish(vis_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish ROI visualization: {e}")

if __name__ == "__main__":
    CarDetector()
    rospy.spin()
