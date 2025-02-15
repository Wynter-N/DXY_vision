import cv2
import numpy as np
import copy

# 读取图像
image = cv2.imread('1.png')
cv2.imshow('Original Image1', image)

# 浅拷贝图像
shallow_copy = image.view()

# 深拷贝图像
deep_copy = copy.deepcopy(image)

# 设定阈值
threshold_value = 30

def process_image(img):
    height, width, _ = img.shape
    for y in range(height):
        for x in range(width):
            # 获取当前像素的 BGR 值
            b, g, r = img[y, x]
            # 计算三通道均值
            average = int((b + g + r) / 3)
            # 根据阈值进行二值化处理
            if average > threshold_value:
                    img[y, x] = [255, 255, 255]
            else:
                    img[y, x] = [0, 0, 0]
    return img

# 对浅拷贝图像进行处理
processed_shallow = process_image(shallow_copy)

# 对深拷贝图像进行处理
processed_deep = process_image(deep_copy)

# 显示原始图像
cv2.imshow('Original Image', image)

# 显示浅拷贝处理后的图像
cv2.imshow('Processed Shallow Copy', processed_shallow)

# 显示深拷贝处理后的图像
cv2.imshow('Processed Deep Copy', processed_deep)

cv2.waitKey(0)
cv2.destroyAllWindows()
