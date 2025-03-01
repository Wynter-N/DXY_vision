import cv2
import numpy as np

img = cv2.imread('j.png',0)
kernel = np.ones((3,3),np.uint8)
erosion = cv2.erode(img,kernel,iterations = 1)
dilation = cv2.dilate(img,kernel,iterations = 1)
cv2.imshow('original',img)
cv2.imshow('dilation',dilation)
cv2.imshow('erosion',erosion)
cv2.waitKey(0)