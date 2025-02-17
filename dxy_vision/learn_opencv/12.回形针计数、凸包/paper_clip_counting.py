import cv2
import  numpy as np
img=cv2.imread('12.png',cv2.IMREAD_GRAYSCALE)
#二值化
_, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)

# 检测轮廓
#轮廓检测
#cv2.findContours 函数用于在二值图像中检测物体的轮廓，
#轮廓可以理解为图像中连续的点集，通常代表了物体的边界。
contours,_=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
background=cv2.cvtColor(binary,cv2.COLOR_GRAY2BGR)
#回形针计数
paper_clips_count=0
#绘制轮廓/凸包
for contour in contours:
    paper_clips_count+=1
    hull=cv2.convexHull(contour)
    cv2.drawContours(background, [hull], -1, (0, 255, 0), 2)#凸包
    cv2.drawContours(background,[contour],-1,(0,0,255),2)#轮廓
cv2.imshow('show',background)
print(f"回形针个数为: {paper_clips_count-1}")
cv2.waitKey(0)