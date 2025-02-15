import cv2
import numpy as np

value=80
img=cv2.imread('4.png')

height,width,_=img.shape
for y in range(height):
    for x in range (width):
        b,g,r=img[y,x]
        ave=int((b+g+r)/3)
        img[y,x]=[ave,ave,ave]


for y in range(height):
    for x in range (width):
        b=img[y,x,0]
        if b>value :
            img[y,x]=[255,255,255]
        else:
            img[y,x]=[0,0,0]

cv2.imshow('threshold',img)
cv2.waitKey(0)