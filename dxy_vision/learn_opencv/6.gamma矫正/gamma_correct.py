import cv2
import numpy as np

#定义gamma矫正函数
def gamma_cor(img, gamma=1.0):
    #利用numpy创建一个查找表 方便对像素值进行变换
    inv_gamma=1.0/gamma
    table=np.array([((i/255.0)**inv_gamma)*255 for i in np.arange(0,256)]).astype("uint8")
    """
    1.np.arange(0,256)
    创建一个等差数列 生成从0-》255的整数序列 代表图像中所有的像素值
    2.(i / 255.0) ** inv_gamma
    前半部分对正在处理的像素值进行归一化处理 取值范围为0到1  后半部分则是对像素值进行gamma变换
    3.((i / 255.0) ** inv_gamma) * 255
    还原像素值
    4.[((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]
    遍历 生成所有像素值进行gamma变换后的数值
    5.np.array(...)
    将、python列表转换为numpy数组
    6..astype("uint8")
    是numpy数组的一个方法 将数组的数据类型转换为uint8 因为图像的像素值通常使用无符号八位整数来表示
    """
    return cv2.LUT(img,table)

img=cv2.imread('6-1.jpg')
#img=cv2.imread('6-2.jpg')
gamma_value=1.8
correct_img=gamma_cor(img,gamma_value)
cv2.imshow('original img',img)
cv2.imshow('gamma corrected img',correct_img)

cv2.waitKey(0)

