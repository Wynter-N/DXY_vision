import cv2
import numpy as np
img=cv2.imread('11.png',0)  #以灰度模式读取

ret, binary_image = cv2.threshold(img, 85, 255, cv2.THRESH_BINARY)
cv2.imshow('pre_treat',binary_image)
#连通域  该函数返回四个值 num_labels表示连通域的总数（包括背景），
# labels是一个与输入图像大小相同的矩阵，
# 其中每个像素的值表示该像素所属的连通域标签，
# stats是一个包含每个连通域统计信息的矩阵，
# centroids是一个包含每个连通域重心坐标的矩阵
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_image, connectivity=8)
#遍历每个连通域（从标签 1 开始，因为标签 0 表示背景），
# 获取其重心坐标，并使用cv2.circle函数在彩色图像上绘制红色圆点表示重心。
bk_img=cv2.cvtColor(binary_image,cv2.COLOR_GRAY2BGR)



#输出硬币个数 等于连通域数量减背景一个
coin_num=num_labels-1
print(f"硬币个数为: {coin_num}")
colors = np.random.randint(0, 256, size=(num_labels, 3), dtype=np.uint8)
colors[0] = [0, 0, 0]  # 背景颜色为黑色
bk_img= np.zeros((labels.shape[0], labels.shape[1], 3), dtype=np.uint8)
for label in range(num_labels):
    bk_img[labels == label] = colors[label]
for i in range(1,num_labels):
    center_gravity_x=int(centroids[i][0])
    center_gravity_y=int(centroids[i][1])
    #centroids 是一个包含每个连通域重心坐标的二维数组。这个数组的每一行对应一个连通域，
    # 每一行有两个元素，分别代表该连通域重心的横坐标和纵坐标。
    cv2.circle(bk_img,(center_gravity_x,center_gravity_y),5,(0,0,255),-1)

cv2.imshow('counting',bk_img)
cv2.waitKey(0)