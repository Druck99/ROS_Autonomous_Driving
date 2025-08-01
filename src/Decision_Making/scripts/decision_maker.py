#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32, Float32MultiArray
from nav_msgs.msg import Odometry

class DecisionMaker:
    def __init__(self):
        rospy.init_node('decision_maker')

        # 参数
        self.stop_distance = rospy.get_param('~stop_distance', 20.0)
        self.judge_distance = rospy.get_param('~judge_distance', 10.0)
        self.goal_x = rospy.get_param('~goal_x', 16.0)
        self.emergency_distance_threshold = rospy.get_param('~emergency_distance_threshold', 5.0)

        # 状态
        self.traffic_light_is_green = True
        self.car_x = 0.0
        self.emergency = False
        self.nearest_car_distance = float('inf')

        # 订阅
        rospy.Subscriber('/traffic_light_status', Int32, self.on_traffic_light)
        rospy.Subscriber('/car_detection_distances', Float32MultiArray, self.on_car_distances)
        rospy.Subscriber('/odom', Odometry, self.on_odom)

        # 发布
        self.cmd_pub = rospy.Publisher('/ssc', Bool, queue_size=1)

        # 定时检查
        rospy.Timer(rospy.Duration(0.1), self.evaluate)

    def on_traffic_light(self, msg):
        if msg.data == 1:
            self.traffic_light_is_green = True
            # rospy.loginfo("🚦 Traffic light detected: GREEN")
        elif msg.data == 0:
            self.traffic_light_is_green = False
            # rospy.loginfo("🚦 Traffic light detected: RED")
        elif msg.data == -1:
            self.traffic_light_is_green = True  # 默认允许前行
            # rospy.logwarn("⚠️ No traffic light detected, assuming GREEN (continue driving)")

    def on_car_distances(self, msg):
        if msg.data:
            self.nearest_car_distance = min(msg.data)
            self.emergency = (self.nearest_car_distance < self.emergency_distance_threshold)
            # rospy.loginfo(f"🚗 Nearest car distance: {self.nearest_car_distance:.2f}m | Emergency: {self.emergency}")
        else:
            self.nearest_car_distance = float('inf')
            self.emergency = False
            # rospy.loginfo("🚗 No cars detected, Emergency: False")

    def on_odom(self, msg):
        self.car_x = msg.pose.pose.position.x
        # rospy.loginfo_throttle(1, f"📍 Current position: x = {self.car_x:.2f}")

    def evaluate(self, event):
        start = True

        if self.car_x >= self.goal_x:
            # rospy.loginfo("🏁 Goal reached, stopping")
            start = False

        elif self.emergency:
            # rospy.loginfo("🚨 Emergency brake triggered, stopping")
            start = False

        elif not self.traffic_light_is_green:
            # rospy.loginfo("🔴 Red light detected, stopping")
            start = False

        else:
            # rospy.loginfo("🟢 Green light or no traffic light detected — keep driving")
            start = True

        # rospy.loginfo(f"📤 Decision: {'GO' if start else 'STOP'}\n---")
        self.cmd_pub.publish(start)

if __name__ == '__main__':
    try:
        DecisionMaker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
