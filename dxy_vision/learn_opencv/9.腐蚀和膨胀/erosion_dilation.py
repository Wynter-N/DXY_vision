import cv2
import numpy as np
import matplotlib.pyplot as plt

# 读取图像
image = cv2.imread('9.png', cv2.IMREAD_GRAYSCALE)

# 二值化图像
_, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

# 定义不同形状和大小的结构元素
kernel_rect = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
kernel_cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (5, 5))

# 腐蚀操作
eroded_rect = cv2.erode(binary_image, kernel_rect, iterations=1)
eroded_ellipse = cv2.erode(binary_image, kernel_ellipse, iterations=1)
eroded_cross = cv2.erode(binary_image, kernel_cross, iterations=1)

# 膨胀操作
dilated_rect = cv2.dilate(binary_image, kernel_rect, iterations=1)
dilated_ellipse = cv2.dilate(binary_image, kernel_ellipse, iterations=1)
dilated_cross = cv2.dilate(binary_image, kernel_cross, iterations=1)

# 开运算
opened_rect = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_rect)
opened_ellipse = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_ellipse)
opened_cross = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_cross)

# 闭运算
closed_rect = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel_rect)
closed_ellipse = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel_ellipse)
closed_cross = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel_cross)

# 显示结果
plt.figure(figsize=(12, 8))

plt.subplot(3, 4, 1)
plt.title('Original')
plt.imshow(binary_image, cmap='gray')

plt.subplot(3, 4, 2)
plt.title('Eroded Rect')
plt.imshow(eroded_rect, cmap='gray')

plt.subplot(3, 4, 3)
plt.title('Eroded Ellipse')
plt.imshow(eroded_ellipse, cmap='gray')

plt.subplot(3, 4, 4)
plt.title('Eroded Cross')
plt.imshow(eroded_cross, cmap='gray')

plt.subplot(3, 4, 5)
plt.title('Dilated Rect')
plt.imshow(dilated_rect, cmap='gray')

plt.subplot(3, 4, 6)
plt.title('Dilated Ellipse')
plt.imshow(dilated_ellipse, cmap='gray')

plt.subplot(3, 4, 7)
plt.title('Dilated Cross')
plt.imshow(dilated_cross, cmap='gray')

plt.subplot(3, 4, 8)
plt.title('Opened Rect')
plt.imshow(opened_rect, cmap='gray')

plt.subplot(3, 4, 9)
plt.title('Opened Ellipse')
plt.imshow(opened_ellipse, cmap='gray')

plt.subplot(3, 4, 10)
plt.title('Opened Cross')
plt.imshow(opened_cross, cmap='gray')

plt.subplot(3, 4, 11)
plt.title('Closed Rect')
plt.imshow(closed_rect, cmap='gray')

plt.subplot(3, 4, 12)
plt.title('Closed Ellipse')
plt.imshow(closed_ellipse, cmap='gray')

plt.tight_layout()
plt.show()