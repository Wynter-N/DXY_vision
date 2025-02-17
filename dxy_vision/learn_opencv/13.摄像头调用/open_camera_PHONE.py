import time

import cv2  # 导入库

cv2.namedWindow("camera", 1)  # 定义启动窗口名称
cv2.resizeWindow('camera',300,300)
video = "http://admin:admin@192.168.10.101:8081/"
# 此处根据IP摄像头生成的局域网地址
capture = cv2.VideoCapture(video)
# 若vedio替换成本地视频文件的路径 则播放本地视频
fps=0
pre_time=0
while True:
    current_time=time.time()
    success, img = capture.read()  # 读取视频
    if not success:
        print("failed")
        break

    #测算帧率
    fps=1/(current_time-pre_time)
    pre_time=current_time
    fps_text=f"FPS: {fps:.2f}"
    cv2.putText(img,fps_text,(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2)
    cv2.imshow('Frame with FPS', img)
    #中值滤波
    me_filtered=cv2.medianBlur(img,5)
    cv2.imshow('median filter',me_filtered)
    # 均值滤波
    mean_filtered = cv2.blur(img, (5,5))
    cv2.imshow('mean filter', mean_filtered)
    # 高斯滤波
    gaussion_filtered = cv2.GaussianBlur(img, (5,5),0)
    cv2.imshow('median filter', gaussion_filtered)

    cv2.imshow("camera", img)
    key=cv2.waitKey(1)

    if key==27:
        break


capture.release()
# The method is automatically called by subsequent VideoCapture::open and by VideoCapture destructor.
cv2.destroyWindow()
# The function destroyWindow destroys the window with the given name.
