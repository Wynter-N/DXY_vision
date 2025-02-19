import cv2
import numpy as np

# 读取图像
img = cv2.imread('综合考察3.jpg')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#定义红色的范围  色相 饱和度 明度（明度低颜色暗
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([20, 255, 255])
lower_red2 = np.array([150, 120, 70])
upper_red2 = np.array([180, 255, 255])
#使用inrange函数检查图像中的像素点是否在指定的范围内
mask_red1=cv2.inRange(hsv,lower_red1,upper_red1)
mask_red2=cv2.inRange(hsv,lower_red2,upper_red2)
mask=mask_red2+mask_red1
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
area_threshold=100
# 绘制矩形框
for contour in contours:
    area=cv2.contourArea(contour)
    if area>=area_threshold:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.imshow('detect',img)
cv2.waitKey(0)