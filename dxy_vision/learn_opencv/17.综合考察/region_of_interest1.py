import numpy as np
import cv2

from opencv.basic_drawing import circle

"""
检测圆形区域并标记
1.检测： 读取灰度图像 （减少噪声） 霍夫圆变换检测圆形
2.使用基础的绘图功能在原图像上绘制
3.保留圆形区域 其他区域设置为黑色
"""
img=cv2.imread("综合考察1.jpg")
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray=cv2.GaussianBlur(gray,(9,9),2)
#检测
circles=cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,minDist=20,param1=50,param2=30,minRadius=10,maxRadius=50)
mask=np.zeros_like(img)
circles=np.round(circles[0,:]).astype("int")
for (x,y,r) in circles:
    #cv2.circle(img,(x,y),r,(255,255,0),2)
    #cv2.circle(img,(x,y),r,(255,255,0),-1)
    cv2.circle(mask, (x, y), r, (255, 255, 255), -1)

#按位与

result=cv2.bitwise_and(img,mask)
cv2.imshow("roi_1",result)
cv2.waitKey(0)
