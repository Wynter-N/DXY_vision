import cv2
import numpy as np

image = cv2.imread('综合考察2.jpg')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray=cv2.GaussianBlur(gray,(17,17),2)
_, binary = cv2.threshold(gray, 180, 255,cv2.THRESH_BINARY)
kernel = np.ones((5, 5), np.uint8)
opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for contour in contours:
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
cv2.drawContours(image, contours, -1, (0, 255, 0), 2)
cv2.imshow('test',binary)
cv2.imshow('Original Image', image)
cv2.waitKey(0)




