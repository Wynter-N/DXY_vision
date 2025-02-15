import cv2

img1=cv2.imread('2.png')
new_size=(500,400)
#调整图像大小
sized_img=cv2.resize(img1,new_size,interpolation=cv2.INTER_LINEAR)
# 创建并移动第一个窗口
cv2.namedWindow('Window 1')
cv2.moveWindow('Window 1', 0, 0)  # 移动到 (100, 100)
cv2.imshow('Window 1', sized_img)

cv2.namedWindow('Window 2')
cv2.moveWindow('Window 2', 0, 1600)  # 移动到 (100, 100)
cv2.imshow('Window 2', sized_img)


cv2.namedWindow('Window 3')
cv2.moveWindow('Window 3', 2560, 0)  # 移动到 (100, 100)
cv2.imshow('Window 3', sized_img)


cv2.namedWindow('Window 4')
cv2.moveWindow('Window 4', 2560, 1600)  # 移动到 (100, 100)
cv2.imshow('Window 4', sized_img)

cv2.waitKey(0)