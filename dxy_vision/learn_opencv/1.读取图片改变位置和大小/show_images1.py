#下载一张图片，使用imread读取，
# 然后用imshow显示图片，
#current 2560 x 1600
import cv2
img=cv2.imread('show_images.png')
#cv2.imshow('test',img)
#第一个参数为窗口名称 第二个为显示图像的数据
#设置新图像的尺寸
new_size=(700,350)
#调整图像大小
sized_img=cv2.resize(img,new_size,interpolation=cv2.INTER_LINEAR)

cv2.imshow('test',sized_img)#第一个参数为窗口名称 第二个为显示图像的数据
cv2.waitKey(0)#输入0表示一直等待


"""
# 了解如何改变输出图像的大小，并让图像分别完整显示在显示器的四个角
import cv2
#加载图像
img=cv2.imread('show_images.png')
#设置新图像的尺寸
new_size=(700,350)
#调整图像大小
sized_img=cv2.resize(img,new_size,interpolation=cv2.INTER_LINEAR)
#显示调整后的图像
cv2.imshow('调整后的图片大小',sized_img)
cv2.imshow('原始图像',img)
cv2.waitKey(0)
"""



