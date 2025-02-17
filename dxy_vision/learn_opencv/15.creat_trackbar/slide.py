import  cv2
import  numpy as np
img=cv2.imread('7.png',0)
cv2.namedWindow('photo')
def on_trackbar(val):
    _,binary=cv2.threshold(img,val,255,cv2.THRESH_BINARY)
    cv2.imshow('photo',binary)

cv2.createTrackbar('threshold','photo',127,255,on_trackbar)
_,binary=cv2.threshold(img,127,255,cv2.THRESH_BINARY)
cv2.imshow('photo',binary)

while True:
    key=cv2.waitKey(1)
    if key==27:
        break