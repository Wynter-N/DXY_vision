import cv2
import numpy as np
import matplotlib.pyplot as plt


# 1. 高斯平滑
def gaussian_smoothing(image, kernel_size, sigma):
    # 创建高斯核
    kernel = np.fromfunction(
        lambda x, y: (1 / (2 * np.pi * sigma ** 2)) * np.exp(
            -((x - (kernel_size - 1) / 2) ** 2 + (y - (kernel_size - 1) / 2) ** 2) / (2 * sigma ** 2)),
        (kernel_size, kernel_size)
    )
    kernel = kernel / np.sum(kernel)
    # 进行卷积操作
    smoothed = cv2.filter2D(image, -1, kernel)
    return smoothed


# 2. 计算梯度
def gradient_calculation(image):
    # 使用Sobel算子计算x和y方向的梯度
    sobelx = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=3)
    # 计算梯度幅值
    gradient_magnitude = np.sqrt(sobelx ** 2 + sobely ** 2)
    # 计算梯度方向
    gradient_direction = np.arctan2(sobely, sobelx)
    return gradient_magnitude, gradient_direction


# 3. 非极大值抑制
def non_maximum_suppression(gradient_magnitude, gradient_direction):
    rows, cols = gradient_magnitude.shape
    suppressed = np.zeros((rows, cols), dtype=np.float32)
    angle = gradient_direction * 180. / np.pi
    angle[angle < 0] += 180

    for i in range(1, rows - 1):
        for j in range(1, cols - 1):
            try:
                q = 255
                r = 255

                # 0°
                if (0 <= angle[i, j] < 22.5) or (157.5 <= angle[i, j] <= 180):
                    q = gradient_magnitude[i, j + 1]
                    r = gradient_magnitude[i, j - 1]
                # 45°
                elif 22.5 <= angle[i, j] < 67.5:
                    q = gradient_magnitude[i + 1, j - 1]
                    r = gradient_magnitude[i - 1, j + 1]
                # 90°
                elif 67.5 <= angle[i, j] < 112.5:
                    q = gradient_magnitude[i + 1, j]
                    r = gradient_magnitude[i - 1, j]
                # 135°
                elif 112.5 <= angle[i, j] < 157.5:
                    q = gradient_magnitude[i - 1, j - 1]
                    r = gradient_magnitude[i + 1, j + 1]

                if (gradient_magnitude[i, j] >= q) and (gradient_magnitude[i, j] >= r):
                    suppressed[i, j] = gradient_magnitude[i, j]
                else:
                    suppressed[i, j] = 0

            except IndexError as e:
                pass

    return suppressed


# 4. 双阈值检测
def double_thresholding(image, low_threshold, high_threshold):
    strong_pixel = 255
    weak_pixel = 50
    high_threshold_image = (image > high_threshold) * strong_pixel
    low_threshold_image = (image >= low_threshold) * weak_pixel
    combined = high_threshold_image + low_threshold_image
    return combined


# 5. 边缘连接
def hysteresis(image):
    rows, cols = image.shape
    for i in range(1, rows - 1):
        for j in range(1, cols - 1):
            if image[i, j] == 50:
                if (image[i - 1, j - 1] == 255) or (image[i - 1, j] == 255) or (image[i - 1, j + 1] == 255) or \
                        (image[i, j - 1] == 255) or (image[i, j + 1] == 255) or \
                        (image[i + 1, j - 1] == 255) or (image[i + 1, j] == 255) or (image[i + 1, j + 1] == 255):
                    image[i, j] = 255
                else:
                    image[i, j] = 0
    return image


# 主函数
def canny_edge_detection(image, kernel_size=5, sigma=1.4, low_threshold=50, high_threshold=150):
    # 转换为灰度图像
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 1. 高斯平滑
    smoothed = gaussian_smoothing(gray, kernel_size, sigma)
    # 2. 计算梯度
    gradient_magnitude, gradient_direction = gradient_calculation(smoothed)
    # 3. 非极大值抑制
    suppressed = non_maximum_suppression(gradient_magnitude, gradient_direction)
    # 4. 双阈值检测
    thresholded = double_thresholding(suppressed, low_threshold, high_threshold)
    # 5. 边缘连接
    edges = hysteresis(thresholded)
    return edges


# 读取图像
image = cv2.imread('9.png')

# 进行Canny边缘检测
edges = canny_edge_detection(image)

# 显示结果
plt.subplot(121), plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122), plt.imshow(edges, cmap='gray')
plt.title('Canny Edge Detection'), plt.xticks([]), plt.yticks([])
plt.show()
