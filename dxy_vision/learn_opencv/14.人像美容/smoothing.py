import cv2
img1=cv2.imread('14-1.jpg')
#双边滤波进行磨皮
blurred1=cv2.bilateralFilter(img1,30,95,95)
alpha=1.2
beta=20 #亮度
whitened1=cv2.convertScaleAbs(blurred1,alpha=alpha, beta=beta)
cv2.imshow('original1',img1)
cv2.imshow('skin smoothing1',blurred1)
cv2.imshow('skin whitened1',whitened1)


img2=cv2.imread('14-2.jpg')
#双边滤波进行磨皮
blurred2=cv2.bilateralFilter(img2,7,75,75)
cv2.imshow('original2',img2)
cv2.imshow('skin smoothing2',blurred2)
cv2.waitKey(0)