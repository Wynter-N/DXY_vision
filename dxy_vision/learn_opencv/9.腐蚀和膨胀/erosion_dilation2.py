import cv2
import numpy as np
img=cv2.imread('j.png')
gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel,iterations=3)
closed = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel,iterations=3)
cv2.imshow('open',opened)
cv2.imshow('close',closed)
cv2.waitKey(0)