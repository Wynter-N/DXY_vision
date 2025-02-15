import cv2
import numpy as np

img=cv2.imread('4.png')
ave=np.mean(img,axis=2,keepdims=True)  #axis=2表示沿通道轴即RGB通道计算均值  、keepdims为、true表示保持结果的维度 使其形状为、height width 1
ave = ave.astype(np.uint8)
cv2.imshow('original',img)
cv2.imshow('dealt',ave)
cv2.waitKey(0)