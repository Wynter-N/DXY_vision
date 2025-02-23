import cv2
import numpy as np
"""
读取图像 转为灰度图
检测轮廓和边缘
"""

img=cv2.imread('数据集/images/H_036.png')
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
blurred=cv2.GaussianBlur(gray,(5,5),0)
edges=cv2.Canny(blurred,50,150)
contours,_=cv2.findContours(edges.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
target_contours=[]
for con in contours:
    #计算轮廓的周长
    perimeter = cv2.arcLength(con, True)
    #进行多边形逼近
    approx=cv2.approxPolyDP(con,0.02*perimeter,True)
    area=cv2.contourArea(con)

    if len(approx)>5 and area>100:
        target_contours.append(con)

if target_contours:
    cv2.drawContours(img,target_contours,-1,(0,255,0),3)
    cv2.imshow("Detected",img)
    cv2.waitKey(0)