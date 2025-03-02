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

## 任务3
```
import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类

import cv2                              # OpenCV图像处理库
import numpy as np                      # Python数值计算库
```
### 使用ros节点，使用颜色识别识别出苹果，并标出轮廓
#### 界定红色范围
```
lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限
```
#### 定义检测函数
```
def object_detect(image):
    #转换图像类型
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_red = cv2.inRange(hsv_img, lower_red, upper_red)#返回一个二值图像 在限定范围内的像素值为255,不在为0
    #边框
    contours, hierarchy = cv2.findContours(mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    for cnt in contours:          # 去除一些轮廓面积太小的噪声
        if cnt.shape[0] < 150:
            continue
            
        (x, y, w, h) = cv2.boundingRect(cnt)    
        cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)                        # 将苹果的轮廓勾勒出来
        cv2.circle(image, (int(x+w/2), int(y+h/2)), 5, (0, 255, 0), -1)           # 将苹果的图像中心点画出来
	    
    cv2.imshow("object", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
```
![image](https://github.com/user-attachments/assets/45507018-fd35-4623-b408-ec4bc84da905)

- cv2.CHAIN_APPROX_NONE，表示不进行近似，将保存轮廓上的所有点，即把轮廓上的每一个像素点都保留下来
- cv2.CHAIN_APPROX_SIMPLE：只保留轮廓的端点，例如对于水平、垂直或对角线方向的直线段，只保留其两个端点，从而大大减少存储轮廓所需的点数
- 检测边框的方式

| 参数      | 说明 |
| ----------- | ----------- |
| cv2.RETR_LIST      | 检索所有的轮廓，但不建立轮廓之间的层次关系       |
|cv2.RETR_EXTERNAL  | 只检索最外层的轮廓|
|cv2.RETR_TREE  |检索所有轮廓并重建嵌套轮廓的完整层次结构|
#### 定义主函数
```
def main(args=None):                                                         
    rclpy.init(args=args)                                                    
    node = Node("node_object")  #实例化一个节点  节点名称为node_object                                             
    node.get_logger().info("ROS2节点示例：检测图片中的苹果") #打印一条日志信息

    image = cv2.imread('路径')  # 读取图像
    object_detect(image)   #调用函数                                          
    rclpy.spin(node)                                                               # 循环等待ROS2退出
    node.destroy_node()                                                            # 销毁节点对象
    rclpy.shutdown()                                                               # 关闭ROS2 Python接口
```
- 在 ROS 2 里，rclpy 是用于 Python 语言的客户端库，rclpy.init(args=args) 这行代码的主要功能是对 ROS 2 Python 客户端库进行初始化操作。初始化过程会为节点创建必要的上下文，加载 ROS 2 环境，并且处理命令行参数，只有完成初始化之后，才能创建和运行 ROS 2 节点
- rclpy.spin(node) 函数的主要功能是让指定的 ROS 2 节点进入一个阻塞式的循环，在这个循环中，节点会持续不断地处理各种传入的事件和回调，比如订阅话题接收到的新消息、服务请求、定时器到期等，从而保证节点能够及时响应并处理这些事件。

### 使用ros话题，打开摄像头并回传图像，并对单一颜色物体进行实时识别。
subscriber:
class ImageSubscriber(Node):
#### 节点初始化
```
    def __init__(self, name): 
        super().__init__(name)  #调用父类的构造方法
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        #（消息类型，订阅的话题名称 回调函数）
        #节点将监听这个话题上发布的图像消息
        #当订阅的话题有新消息发布时，会调用这个函数来处理接收到的消息
        self.cv_bridge = CvBridge()
```
- __init__ 是python类的构造函数，在创建该类的实例的时候会自动调用
- name 是传递给节点的名称 用于在ros2系统中标识该节点
- CvBridge 是 ROS 提供的一个工具类，用于在 ROS 图像消息（如 sensor_msgs.msg.Image）和 OpenCV 图像格式（如 numpy.ndarray）之间进行转换。

#### 检测函数
```
def object_detect(self, image):

```
#### 回调函数
```
def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')#提示信息
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')# 将ROS的图像消息转化成OpenCV图像
        self.object_detect(image)# 调用函数检测苹果

```
def main(args=None) :
```
    rclpy.init(args=args)
    node = ImageSubscriber("topic_webcam_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 
```

publisher:
class ImagePublisher(Node):
```
 def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)`
        self.timer = self.create_timer(0.1, self.timer_callback)#定时调用回调函数
        self.cap = cv2.VideoCapture(0) # 创建一个视频采集对象，驱动相机采集图像（相机设备号）（电脑摄像头）
        self.cv_bridge = CvBridge()   # 创建一个图像转换对象 用于稍后将OpenCV的图像转换成ROS的图像消息

    def timer_callback(self):
        ret, frame = self.cap.read() # 一帧一帧读取图像
        
        if ret == True:  # 如果图像读取成功
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))  # 发布图像消息

        self.get_logger().info('Publishing video frame')  # 输出日志信息，提示已经完成图像话题发布

```
#### 主函数
```
def main(args=None) :
    rclpy.init(args=args)
    node = ImageSubscriber("topic_webcam_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown() 
```

### 采用服务通讯的方式，在任务2的基础上，增加请求回传坐标节点
server：
```
class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)
        self.cv_bridge = CvBridge()

        self.srv = self.create_service(GetObjectPosition,  # 创建服务器对象（接口类型、服务名、服务器回调函数）
                                       'get_target_position',
                                       self.object_position_callback)    
        self.objectX = 0
        self.objectY = 0                              

    def object_detect(self, image):
    #检测苹果略
            
            self.objectX = int(x+w/2)
            self.objectY = int(y+h/2)

        cv2.imshow("object", image)
        cv2.waitKey(50)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(image) #调用检测函数

    def object_position_callback(self, request, response):  # 创建回调函数，执行收到请求后对数据的处理
        if request.get == True:
            response.x = self.objectX  # 目标物体的XY坐标
            response.y = self.objectY
            self.get_logger().info('Object position\nx: %d y: %d' %
                                   (response.x, response.y))   # 输出日志信息，提示已经反馈
        else:
            response.x = 0
            response.y = 0
            self.get_logger().info('Invalid command')  # 输出日志信息，提示已经反馈
        return response


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImageSubscriber("service_object_server")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```
client：
```
class objectClient(Node):
    def __init__(self, name):
        super().__init__(name)                                                                  # ROS2节点父类初始化
        self.client = self.create_client(GetObjectPosition, 'get_target_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = GetObjectPosition.Request()
                    
    def send_request(self):
        self.request.get = True
        self.future = self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)                             # ROS2 Python接口初始化
    node = objectClient("service_object_client")       # 创建ROS2节点对象并进行初始化
    node.send_request()
    
    while rclpy.ok():
        rclpy.spin_once(node)

        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                node.get_logger().info(
                    'Result of object position:\n x: %d y: %d' %
                    (response.x, response.y))
            break
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
```


# opencv
<https://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/imgproc/table_of_content_imgproc/table_of_content_imgproc.html>
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
 OpenCV 的坐标系统中，第一个值是 y 坐标（对应图像的行），第二个值是 x 坐标（对应图像的列）
```
#创建一个三通道的512*512的画布
img=np.zeros((512,512,3),dtype=np.uint8)
```
### 点
```
dot=(256,256)
cv2.circle(img,dot,5,(0,255,0),-1)#5表示半径 线宽为-1表示绘制一个实心的圆形
```
### 线
```
st_line=(100,100)
end_line=(400,400)
cv2.line(img,st_line,end_line,(255,0,0),2)
```
### 圆
```
circle=(256,256)
ras=50
cv2.circle(img,circle,ras,(255,0,0),2)
```
### 矩形
```
top_left=(150,200)
bottom_right=(350,400)
cv2.rectangle(img,top_left,bottom_right,(0,255,255),2)
```
## 了解腐蚀、膨胀、开运算和闭运算，使用不同形状大小的算子，使用图片见图9
腐蚀和膨胀是最基本的形态学运算。 <br>
腐蚀和膨胀是针对白色部分（高亮部分）而言的。<br>
膨胀就是对图像高亮部分进行“领域扩张”，效果图拥有比原图更大的高亮区域；腐蚀是原图中的高亮区域被蚕食，效果图拥有比原图更小的高亮区域。 <br>
<https://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html>
![image](https://github.com/user-attachments/assets/060f42ec-314c-47ec-8218-484ba70be604)
![image](https://github.com/user-attachments/assets/21a17ef7-3d54-4452-bea2-c69a4983be46)
![image](https://github.com/user-attachments/assets/5216f6bf-4bf0-4be5-8008-716ac189ffec)
```
kernel = np.ones((3,3),np.uint8)
erosion = cv2.erode(img,kernel,iterations = 1)
dilation = cv2.dilate(img,kernel,iterations = 1)
```
- 开运算：先腐蚀后膨胀，用于移除由图像噪音形成的斑点
- 闭运算：先膨胀后腐蚀，用来连接被误分为许多小块的对象
```
kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
opened = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel,iterations=3)
closed = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel,iterations=3)
```
## 了解canny算子原理（以及其进一步改进方法），并手动实现
Canny边缘检测算子是John F. Canny于1986年开发出来的一个多级边缘检测算法。更为重要的是Canny创立了“边缘检测计算理论”（computational theory of edge detection）解释这项技术如何工作。
### opencv提供的canny函数
1. 使用高斯模糊，去除噪音点（cv2.GaussianBlur）
2. 灰度转换（cv2.cvtColor）
3. 使用sobel算子，计算出每个点的梯度大小和梯度方向
4. 使用非极大值抑制(只有最大的保留)，消除边缘检测带来的杂散效应
5. 应用双阈值，来确定真实和潜在的边缘
6. 通过抑制弱边缘来完成最终的边缘检测



