import cv2
import numpy as np

img=cv2.imread('5.png')
#将bgr转换到hsv
hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

#定义红色的范围
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 70])
upper_red2 = np.array([180, 255, 255])
#使用inrange函数检查图像中的像素点是否在指定的范围内
mask_red1=cv2.inRange(hsv_img,lower_red1,upper_red1)
mask_red2=cv2.inRange(hsv_img,lower_red2,upper_red2)
#合并 原理：按位或运算
mask_red=cv2.bitwise_or(mask_red1,mask_red2)

#提取红色像素部分
red_select=cv2.bitwise_and(img,img,mask=mask_red)

lower_blue = np.array([100, 120, 70])
upper_blue = np.array([130, 255, 255])
mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
blue_select = cv2.bitwise_and(img, img, mask=mask_blue)
cv2.imshow('original', img)
cv2.imshow('select red', red_select)
cv2.imshow('select blue', blue_select)

cv2.waitKey(0)