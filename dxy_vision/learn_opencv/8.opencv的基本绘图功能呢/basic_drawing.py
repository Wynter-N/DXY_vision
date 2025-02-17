#学习opencv中的基本绘图功能
#包括画点 线 图 矩形
import cv2
import numpy as np

#创建画布
img=np.zeros((512,512,3),dtype=np.uint8)

#1.dot
dot=(256,256)
cv2.circle(img,dot,5,(0,255,0),-1)

#2.line
st_line=(100,100)
end_line=(400,100)
cv2.line(img,st_line,end_line,(255,0,0),2)

#3.circle
circle=(256,256)
ras=50
cv2.circle(img,circle,ras,(0,0,255),2)

#4.rectangle
top_left=(150,200)
bottom_right=(350,400)
cv2.rectangle(img,top_left,bottom_right,(0,255,255),2)

cv2.imshow('drawing',img)
cv2.waitKey(0)