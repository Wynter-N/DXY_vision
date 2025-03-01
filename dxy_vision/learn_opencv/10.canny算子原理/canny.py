import cv2
import numpy as np
from matplotlib import pyplot as plt


def nothing(x):
    pass

cv2.namedWindow('res')

cv2.createTrackbar('max', 'res', 0, 255, nothing)
cv2.createTrackbar('min', 'res', 0, 255, nothing)

img = cv2.imread('j.png', 0)

maxVal = 200
minVal = 100

while (1):

    if cv2.waitKey(20) & 0xFF == 27:
        break
    maxVal = cv2.getTrackbarPos('min', 'res')
    minVal = cv2.getTrackbarPos('max', 'res')
    if minVal < maxVal:
        edge = cv2.Canny(img, 100, 200)
        cv2.imshow('res', edge)
    else:
        edge = cv2.Canny(img, minVal, maxVal)
        cv2.imshow('res', edge)

