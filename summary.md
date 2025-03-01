# ros2
## ROS2中工作空间的构成，如何创建，安装依赖，编译，设置环境变量
### src代码空间 install安装空间 log日志空间 build编译空间
### 如何创建
```
mkdir -p ~/ws/src
ros2 pkg create --build-type ament_cmake pkg_name
```
### 安装依赖
```
#使用rosdep安装
#第一次使用需要初始化
sudo rosdep init
rosdep update
#在工作空间根目录下安装依赖
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
```
--from-paths src：指定从工作空间的src目录下查找软件包。 <br>
--ignore-src：忽略src目录下的源码包，只安装系统依赖。<br>
-r：在安装过程中遇到错误时继续尝试安装其他依赖。<br>
-y：自动确认所有安装提示。<br>

### 编译 
```
cd ~/ws
colcon build pkg_name
```

### 设置环境变量
```
source ~/ws/install/setup.bash
```
source命令用于在当前 shell 环境中执行指定的脚本文件。

## 了解不同语言功能包的组成，如何创建，编译功能包
功能包放在src文件夹中
### C++
```
ros2 pkg create --build-type ament_cmake cpp_ros2_pkg --denpendencies rclcpp std_msgs
#--dependencies rclcpp std_msgs 指定功能包依赖于rclcpp（ROS 2 的 C++ 客户端库）和std_msgs（标准消息库）
```
在功能包的src目录下创建代码文件，例如talker.cpp，写完代码后在功能包的根目录下编辑CMakeLists.txt文件，然后回到工作空间的根目录，编译该功能包。
```
cd ~/ws
colcon build --packages_select pkg_name
```
运行节点
```
source ~/ws/install/setup.bash
ros2 run pkg_name node_name
```
### Python
```
ros2 package_create --build-type ament_python pkg_name --dependencies rclpy
```
编写节点代码，在pkg_name的pkg_name目录下创建一个python文件，例如talker.py,写入代码，配置setup.py文件，在entry_points部分添加节点信息，回到工作空间的根目录，编译。
```
cd ~/ws
colcon build --packages-select pkg_name
#运行节点
source install/setup.bash
ros2 run pkg_name node_name
```

# opencv
import cv2
## 下载一张图片，使用imread读取，然后用imshow显示图片，了解如何改变输出图像的大小，并让图像分别完整显示在显示器的四个角
```
#显示图片
img=cv2.imread('图片路径')
cv2.imshow('name',img)
cv2.waitKey(0)
```

```
#调整图片大小
new_size=(700,350)
sized_img=cv2.resize(img,new_size,interpolation=cv2.INTER_AREA)
```
def resize(src, dsize, dst=None, fx=None, fy=None, interpolation=None) <br>
src：输入图像 <br>
dsize：变化后的尺寸 <br>
dst：输出图像 <br>
fx，fy：沿x轴，y轴方向的缩放比例 <br>
interpolation：中文意思是插值，表示使用什么算法来对图像进行改变 <br>
| 参数      | 说明 |
| ----------- | ----------- |
| cv2.INTER_NEAREST      | 最近邻插值       |
|cv2.INTER_LINEAR   | 双线性插值，默认值        |
|cv2.INTER_AREA|双三次插值|
|cv2.INTER_LANCZOS4 |	8x8 邻域的 Lanczos 插值|
|cv2.INTER_MAX	|插值代码的掩码|
|cv2.WARP_FILL_OUTLIERS	|标志，填充所有目标图像像素。如果其中一些对应于源图像中的异常值，则将它们设置为零|
|cv2.WARP_INVERSE_MAP	|标志，逆变换|

```
#移动窗口
cv2.namedWindow('window1')
cv2.moveWindow('window1',0,0)
cv2.imshow('window1',img)
…
```

## 了解OpenCV处理单个像素和通道的方法，将一张彩色图片每个像素的RGB值改为三通道均值average
BGR： blue green red  <br>
0为黑色 255为白色  <br>
```
ave=np.mean(img,axis=2,keepdims=True)
ave = ave.astype(np.uint8)
```
- np.mean函数用于计算均值
- axis=2表示沿着图像的第三个轴（通道轴，通常在RGB图像中表示红绿蓝）计算均值
- keepdims=True表示吉萨un后的结果人仍然保持三维形状，即height*width*1,而不是降为二维数组
- 将计算得到的均值数组 ave 的数据类型转换为 np.uint8（无符号 8 位整数类型）

## 了解threshold函数原理，并手动实现，即在任务二的基础上，在最前面声明一个变量，遍历每一个像素，当average值大于变量值时设为255，反之为0，观察不同变量值处理结果。

```
height,width,_=img.shape
for y in range(height):
    for x in range (width):
        b,g,r=img[y,x]  #访问第y行x列的像素，返回一个包含三个元素的数组
        ave=int((b+g+r)/3)
        img[y,x]=[ave,ave,ave]  #将当前像素的rgb通道值都设置为平均值


for y in range(height):
    for x in range (width):
        b=img[y,x,0] #访问该像素的蓝色通道值 
        if b>value :
            img[y,x]=[255,255,255]
        else:
            img[y,x]=[0,0,0]
#调整
```
## 了解深拷贝与浅拷贝的区别，分别对同一图像进行深浅拷贝，并对拷贝后的图像进行任务三的处理，观察两者有什么区别。
1. 深拷贝：把图像复制一份，对复制后的图像进行处理，被拷贝的图像不会变化。
2. 浅拷贝：复制图像地址，对复制后的图像进行处理，被拷贝的图像也会变化。
##  了解通道分离，将同一张彩色图片的三个通道分别分离
```
img=cv2.imread('1.png')
(B,G,R)=cv2.split(img)
```
## 了解Gamma矫正原理（为什么、怎么做），并手动实现，使用图片见图6-1、6-2
1. 为什么：![image](https://github.com/user-attachments/assets/ad239edd-a1fd-408f-8a21-ed9f93c0fc1d)
   RGB颜色值不能简单直接相加，而是必须用2.2次方换算成物理光功率。因为RGB值与功率并非简单的线性关系，而是幂函数关系，这个函数的指数称为Gamma值，一般为2.2，而这个换算过程，称为Gamma校正。<br>
   当gamma值小于1时(蓝色曲线)，图像的整体亮度值得到提升，同时低灰度处的对比度增加，高灰度处的对比度降低，更利于分辩低灰度值时的图像细节；
当gamma值大于1时(红色曲线)，图像的整体亮度值得到减小，同时低灰度处的对比度降低，高灰度处的对比度增加，更利于分辩高灰度值时的图像细节。
原文链接：<https://blog.csdn.net/wenhao_ir/article/details/51656802>
2. 怎么做：
```
def gamma_cor(img,gamma=1.0):
  inv_gamma=1.0/gamma
  table=np.array([((i/255.0)**inv_gamma)*255 for i in   np.arange(0,256)]).astype("uint8")
  return cv2.LUT(img,table)

img=cv2.imread('6-1.jpg')
#img=cv2.imread('6-2.jpg')
gamma_value=1.8
correct_img=gamma_cor(img,gamma_value)
cv2.imshow('original img',img)
cv2.imshow('gamma corrected img',correct_img)

cv2.waitKey(0)
```

    1. np.arange(0,256)
    创建一个等差数列 生成从0->255的整数序列 代表图像中所有的像素值
    2. (i / 255.0) ** inv_gamma
    前半部分对正在处理的像素值进行归一化处理 取值范围为0到1  后半部分则是对像素值进行gamma变换
    3. ((i / 255.0) ** inv_gamma) * 255
    还原像素值
    4. [((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]
    遍历 生成所有像素值进行gamma变换后的数值
    5. np.array(...)
    将、python列表转换为numpy数组
    6. .astype("uint8")
    是numpy数组的一个方法 将数组的数据类型转换为uint8 因为图像的像素值通常使用无符号八位整数来表示

## 了解HSV意义，将图片转化到HSV色域，并用inRange分别提取一张复杂彩色图片中红色和蓝色像素的部分，并思考在不同光线下参数应该怎么改。
![image](https://github.com/user-attachments/assets/077f6f6b-80c4-4720-b5b3-723534000fb8)
HSV：<https://zhuanlan.zhihu.com/p/431143027>
### 转化方法
```
hsv_img=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
```
### 不同光线
1. 色相(Hue)在不同光线下保持相对稳定
2. 饱和度(Saturation)受光线影响较大，在强光下饱和度降低，阴影中饱和度增加
3. 明度(Value)受光线影响最大，强光下明度值会增加，阴影中明度值会减少，变得更暗
### 分离
```
#定义红色的范围
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 70])
upper_red2 = np.array([180, 255, 255])
#使用inrange函数检查图像中的像素点是否在指定的范围内
mask_red1=cv2.inRange(hsv_img,lower_red1,upper_red1)
mask_red2=cv2.inRange(hsv_img,lower_red2,upper_red2)
#合并 原理：按位或运算
mask_red=cv2.bitwise_or(mask_red1,mask_red2)

#提取红色像素部分
red_select=cv2.bitwise_and(img,img,mask=mask_red)

lower_blue = np.array([100, 120, 70])
upper_blue = np.array([130, 255, 255])
mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
blue_select = cv2.bitwise_and(img, img, mask=mask_blue)
```

## 学习OpenCV中基本的绘图功能，实现：画点、画线、画圆、画矩形。




