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






