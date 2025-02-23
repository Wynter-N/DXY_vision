import cv2
import numpy as np


img=cv2.imread('数据集/images/H_009.jpg')
template=cv2.imread('数据集/images/H_008.jpg')  #模板
gray1=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
gray2=cv2.cvtColor(template,cv2.COLOR_BGR2GRAY)
result=cv2.matchTemplate(gray1,gray2,cv2.TM_CCOEFF_NORMED)
min_val,max_val,min_loc,max_loc=cv2.minMaxLoc(result)
#设定匹配阈值
threshold=0.4
locations=np.where(result>=threshold)
h,w=gray2.shape
target_count = 0
for loc in zip(*locations[::-1]):
    top_left=loc
    bottom_right=(loc[0]+w,loc[1]+h)
    cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
    print(f"Detected target {target_count + 1}:")
    print(f"  Top-left coordinates: ({top_left[0]}, {top_left[1]})")
    print(f"  Bottom-right coordinates: ({bottom_right[0]}, {bottom_right[1]})")
    contour = np.array([
        [top_left[0], top_left[1]],
        [bottom_right[0], top_left[1]],
        [bottom_right[0], bottom_right[1]],
        [top_left[0], bottom_right[1]]
    ])
    print(f"  Contour points: {contour}")
    target_count += 1


cv2.imshow("Detected Targets", img)
cv2.waitKey(0)
print(target_count)
