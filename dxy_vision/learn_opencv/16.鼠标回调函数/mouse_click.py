import cv2
import numpy as np
def mouse_callback(event,x,y,flag,param):
    if event==cv2.EVENT_LBUTTONDOWN:
        color=img[y,x]
        print(f"该像素点的坐标为：({x},{y})")
        print(f"该像素点的颜色为：B={color[0]},G={color[1]},R={color[2]}")

img=cv2.imread('综合考察3.jpg')
cv2.namedWindow('color')
cv2.setMouseCallback('color',mouse_callback)

while True:
    cv2.imshow('color',img)
    key = cv2.waitKey(1)

    if key == 27:
        break
