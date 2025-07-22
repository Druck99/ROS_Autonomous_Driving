import cv2
import numpy as np

img = cv2.imread("/home/wrxwrx/i2ros/my_homework/my_ros_homework/introtoros_2025/project/src/111.png")
# OpenCV默认BGR
lower_red = np.array([0, 0, 180], dtype=np.uint8)
upper_red = np.array([80, 80, 255], dtype=np.uint8)
mask = cv2.inRange(img, lower_red, upper_red)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
out = img.copy()
for cnt in contours:
    x, y, w, h = cv2.boundingRect(cnt)
    if w > 10 and h > 10:
        cv2.rectangle(out, (x, y), (x+w, y+h), (0,255,255), 2)
cv2.imwrite('out.png', out)
