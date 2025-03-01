import cv2
import numpy as np


img=cv2.imread('1.png')
(B,G,R)=cv2.split(img)
cv2.namedWindow('B', 0)
cv2.imshow('B', B)
cv2.namedWindow('G', 0)
cv2.imshow('G', G)
cv2.namedWindow('R', 0)
cv2.imshow('R', R)

cv2.waitKey(0)
