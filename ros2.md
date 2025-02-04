# ros2

## 工作空间
存放项目开发相关文件的文件夹，包括四个部分。
1. src 代码空间
2. build 编译空间
3. install 安装空间
4. log 日志空间

### 创建工作空间
` mkdir -p ~/ws/src `

- -p参数用于创建目录以及及其所有必要的父目录，如果父目录存在也不会报错
- ~/ws表示在用户的主目录下创建名为ws（workspace）的子目录
- /src一般用于存放源文件

### 初始化工作空间
```
cd ~/ws/src
ros2 pkg create --build-type ament_cmake my_package
```

- --build-type ament_cmake 指定了构建类型为ament_cmake
- my_package是要创建的ros2软件包的名称，可自行修改

### 构建工作空间
```
cd ~/ws
colcon build
```

- colcon是ros2中用于构建工作空间的工具，自动检测工作空间中的软件包并且按照正确的顺序进行构建

### 配置环境
```
source install /setup.bash
```
构建完成后需要将工作空间的环境变量添加到当前的终端会话中，以便能够正确使用工作空间中的软件包  
这条命令会将install文件夹中的setup.bash文件中的环境变量设置加载到当前终端中 <br>
*如果希望每次打开终端都自动加载，可以添加到~/.bashrc文件中*

```
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ git clone https://gitee.com/guyuehome/ros2_21_tutorials.git  #下载教程的代码


$ sudo apt install -y python3-pip  
$ sudo pip3 install rosdepc
$ sudo rosdepc init
$ rosdepc update
$ cd .. #切换到上一级目录
$ rosdepc install -i --from-path src --rosdistro humble -y #可以手动下载也可以用rosdep（rosdepc）工具自动安装
```
##### 关于rosdepc的参数
- rosdepc install：用于安装 ROS 软件包的依赖项
- -i：表示忽略缺失的依赖项，若某些依赖项无法找到，不会导致安装过程失败。
- --from-path src：指定从src目录下的 ROS 软件包中读取依赖信息，然后安装这些软件包所需的系统依赖项。
- --rosdistro humble：指定 ROS 的发行版本为humble，不同的 ROS 发行版可能有不同的依赖要求，明确指定发行版可确保安装正确的依赖项。
- -y：自动确认所有安装询问，使安装过程自动化。

```
$ sudo apt install python3-colcon-ros #安装 便于使用colcon命令构建工作空间
$ cd ~/dev_ws/
$ colcon build
$ source install/local_setup.sh # 仅在当前终端生效
$ echo " source ~/dev_ws/install/local_setup.sh" >> ~/.bashrc # 所有终端均生效
```
## 不同语言功能包
将不同功能的代码划分到不同的功能包中，减轻它们之间的耦合关系。

```
cd ~/ws/src
```

### python
#### 创建功能包
使用 ros2 pkg create 命令创建一个python的功能包，以py_ros2_pkg为例
```
ros2 pkg create --build-type ament_python py_ros2_pkg --dependencies rclpy
#创建功能包
```
- --build-type ament_python 指定使用python构建类型
- py_ros2_pkg 功能包的名称
- --dependencies rclpy 指定功能包依赖于rclpy（ROS 2 的 Python 客户端库）



#### 编写节点代码
#在py_ros2_pkg功能包的py_ros2_pkg目录下（注意有两个同名目录），创建一个 Python 文件，例如talker.py
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
#### 配置setup.py文件
在py_ros2_pkg功能包的根目录下，编辑setup.py文件，在entry_points部分添加节点信息
```
entry_points={
    'console_scripts': [
        'talker = py_ros2_pkg.talker:main',
    ],
},
```
#### 编译功能包
回到工作空间的根目录
```
cd ~/ws
colcon build --packages-select py_ros2_pkg
```
- --packages-select py_ros2_pkg 表示只编译py_ros2_pkg
#### 运行节点
```
source install/setup.bash #设置环境变量
ros2 run py_ros2_pkg talker #运行节点
```

### C++
#### 创建功能包
使用 ros2 pkg create 命令创建一个C++的功能包，以cpp_ros2_pkg为例
```
ros2 pkg create --build-type ament_cmake cpp_ros2_pkg --dependencies rclcpp std_msgs
#创建功能包
```
- --build-type ament_python 指定使用CMake构建类型
- cpp_ros2_pkg 功能包的名称
- --dependencies rclcpp std_msgs 指定功能包依赖于rclcpp（ROS 2 的 C++ 客户端库）和std_msgs（标准消息库）

#### 编写节点代码
在cpp_ros2_pkg功能包的src目录下，创建一个 C++ 文件，例如talker.cpp
```
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    Talker()
    : Node("talker")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&Talker::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Talker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
#### 配置CMakeLists.txt文件
在cpp_ros2_pkg功能包的根目录下，编辑CMakeLists.txt文件，添加以下内容：
```
add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})
```
#### 编译功能包
回到工作空间的根目录
```
cd ~/ws
colcon build --packages-select cpp_ros2_pkg
```
- --packages-select py_ros2_pkg 表示只编译cpp_ros2_pkg
#### 运行节点
```
source install/setup.bash #设置环境变量
ros2 run cpp_ros2_pkg talker #运行节点
```
### 功能包的结构
Python功能包： 
- pcckage.xml 包含功能包的版权描述，和各种依赖的声明
- setup.py :
1. 包的元数据定义：在setup.py文件里，能够定义包的基本信息，像包名、版本号、作者、描述等。这些信息会在包的分发和安装过程中被使用。
2. 依赖项声明：可以明确该包所依赖的其他 Python 包，保证在安装此包时，所有必要的依赖项也会一同被安装。
3. 可执行脚本指定：指定哪些 Python 脚本可以作为可执行文件运行，这样在安装包之后，就能够直接在命令行中调用这些脚本。
4. 包的安装配置：确定包的安装方式，例如是将包安装到 
    
C++功能包：
- package.xml
- CMakerLists.txt：编译规则

## 节点
每一个节点都是进程，执行具体任务，是一个独立运行的可执行文件，可以用不同的编程语言。 <br>
节点可分布式运行在不同的主机，通过节点名称进行管理。<br>
节点存储在功能包中，工作空间包括功能包。 <br>
实际开发中，节点之间可以通过话题、服务、动作等方式进行通信。（topic service action）<br>
### 实现一个节点的基本流程：
已经创建完工作空间和功能包>>
- 编程接口初始化：指对ros2的客户端库（如rclpy和rclcpp）进行初始化操作，以便节点能够与ros2系统进行交互，初始化过程会为节点分配必要的系统资源，如内存、线程等。
例如`rclpy.init(args=args)`完成了客户端库的初始化，args参数用于传递命令行参数
- 创建节点并初始化
- 实现节点功能
- 销毁节点并关闭接口
### 案例：循环打印Hello World节点
####  面向过程
node_helloworld.py
```
import rclpy    #ros2接口库
from rclpy.node import Node    #ros2 节点类
import time

def main(args=None):    #主入口
    rclpy.init(args=args)    #初始化接口
    node = Node("node_helloworld")    #创建ros2节点对象并进行初始化
       
    while rclpy.ok():    #检查系统正常运行
        node.get_logger().info("Hello World")    #日志输出
        time.sleep(0.5)
    node.destroy_node()    #销毁节点对象
    rclpy.shutdown()    #关闭接口
           
```
编写代码完成后需要设置功能包的编译选项，让系统知道Python程序的入口
setup.py
```
#打开文件，增加如下入口点的配置
entry_points={
    'console_scripts' : [
        'node_helloworld        = learning_node.node_helloworld:main',
    ],
    …
}
```
#### 面向对象
```
import rclpy                                     
from rclpy.node import Node                      
import time
class HelloWorldNode(Node):
    def __init__(self, name):
        super().__init__(name)                     
        while rclpy.ok():                          
            self.get_logger().info("Hello World") #使用节点的日志记录输出一条信息日志，内容为helloworld
            time.sleep(0.5)                        

def main(args=None):                               
    rclpy.init(args=args)                          
    node = HelloWorldNode("node_helloworld_class") #创建一个HelloWorld的实例，节点名称为node_helloworld_class
    rclpy.spin(node)进入节点的主循环，该函数会阻塞当前进程，直到节点被关闭或者ros2系统退出，在这个循环中，节点会处理接收到的消息和事件                               
    node.destroy_node()                            
    rclpy.shutdown()
```
日志记录器可以方便地记录节点的运行信息，便于调试和监控

增加入口点的配置
```
    entry_points={
        'console_scripts': [
         'node_helloworld       = learning_node.node_helloworld:main',
         'node_helloworld_class = learning_node.node_helloworld_class:main',
        ],
```

### 案例：物体识别节点（视觉任务二ros2巩固）
待补充
***
## 通信接口
ros2提供了多种通信接口，用于在不同节点之间实现数据的交换和交互。<br>
给传递的数据定义一个标准的结构，就是接口。<br>

### 话题：异步通信
topic是一种实现节点间通信的核心机制，用于在不同节点之间进行异步、单向的数据传输。
1. 节点可以扮演发布者或者订阅者的角色（Publisher | Subscriber）发布者负责生产并向特定话题发送消息，订阅者关注这些消息并处理。一个话题可以有多个发布者和订阅者，这种松耦合的方式让系统具有良好的可扩展性和灵活性。
2. 每一个话题的名称是唯一的，用于标识，发布者和订阅者通过话题名称来进行匹配。
3. 话题上传递的消息遵循特定的消息类型，消息类型定义了消息的结构，包括消息中包含的字段及其数据类型。使用同一个话题的发布者和订阅者，必须使用相同的消息类型。
4. 使用场景：传感器数据传输（激光雷达/摄像头-->定位/建图节点），状态信息共享（机器人的各个部件如电机/关节-->电机的转速/电流），可视化数据展示（数据-->可视化操作）。
5. 异步操作，不适合逻辑性强的指令，比如修改某一个参数。
### 服务：同步通信
基于请求--响应模式的通信机制
service client向提供服务的节点（service server）发送请求，后者根据收到的请求进行相应的处理之后，返回一个响应给客户端。<br>
这种通信方式是同步的，即客户端发送请求后会等待服务器的响应，直到收到响应才会继续执行后续操作。
1. 每个服务的名称是唯一的。
2. 服务使用特定的服务类型来定义请求和响应的结构，服务类型文件（通常以.srv为扩展名）包括请求部分和响应部分，分别定义了客户端发送的请求数据结构和服务器返回的响应数据结构。
3. 使用场景：一次性任务请求（初始化、重置请求）、配置参数设置（客户端发送修改运动速度、传感器采样频率等参数，服务器接收请求后更新相应的配置参数，并返回设置结果）、复杂计算请求（将计算任务封装在服务器中）
### 动作
动作通常用于执行那些需要一段时间才能完成，并且可能需要在执行过程中进行状态反馈和允许被取消的任务，例如机器人的导航、机械臂的运动等。与服务不同，服务是一次性的请求 - 响应模式，而动作允许在任务执行期间持续地接收反馈信息；与话题不同，话题是单向的消息发布 - 订阅，动作提供了请求 - 响应和反馈的双向交互。
### 案例
#### 通信接口的定义与使用
##### 服务接口
![image](https://github.com/user-attachments/assets/977ac3db-76f2-46d6-8431-7fbf029d2224)

1. 接口定义：
```
#~/ws/src/learning_interface/srv/GetObjectPosition.srv
bool get;  #get为true则表示需要一次位置，客户端发送请求，服务端反馈位置
int32 x;
int32 y;
```
此外，还需要在功能包的CMakeList.txt中配置编译选项，让编译器在编译过程中根据接口定义，自动生成不同语言的代码。
```
...

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectPosition.srv"
)

...
```
package.xml文件中也需要添加代码生成的功能依赖
```
 ...

 <build_depend>rosidl_default_generators</build_depend>
 <exec_depend>rosidl_default_runtime</exec_depend>
 <member_of_group>rosidl_interface_packages</member_of_group>

 ...
```
2. 程序调用--客户端接口调用

```
#~/ws/src/learning_interface/service_object_client.py

import rclpy
from rclpy.node import Node  #Node-> ros2节点类 用于管理和创建节点
from learning_interface.srv import GetObjectPosition   #自定义的服务接口

#定义客户端类节点
class objectClient(Node):
    def __init__(self, name):  #类的构造函数 用于初始化节点 self是实例的引用，name是节点的名称
        super().__init__(name)   #调用父类Node的构造函数，初始化节点名称                       
        self.client = self.create_client(GetObjectPosition, 'get_target_position')    #调用Node类的creat_client方法创建一个服务客户端，该方法的语法为creat_client(service_type,service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')  #等待一秒如果服务不可用返回
        self.request = GetObjectPosition.Request()  #创建一个服务请求对象 
        
    def send_request(self):    #定义一个send_request的方法 用于发送服务请求
        self.request.get = True    #设置服务请求对象的get字段为true，表示请求获取目标对象的位置
        self.future = self.client.call_async(self.request)
        

#主函数
def main(args=None):    #args是传给ros2的初始化参数 默认为None
    rclpy.init(args=args)
    node = objectClient("service_object_client") 创建一个实例并命名
    node.send_request()    #调用方法发送请求
    


 while rclpy.ok():    #检查ros2函数是否正常运行的函数 
        rclpy.spin_once(node)

        if node.future.done():    #检查调用是否完成
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


 node.destroy_node()       # 销毁节点对象
    rclpy.shutdown()       # 关闭ROS2 Python接口
```
3. 程序调用--服务端接口调用 

```
#~/ws/src/learning_service/service_object_server.py

import rclpy                                           # ROS2 Python接口库
from rclpy.node import Node                            # ROS2 节点类
from sensor_msgs.msg import Image                      # 图像消息类型
import numpy as np                                     # Python数值计算库
from cv_bridge import CvBridge                         # ROS与OpenCV图像转换类
import cv2                                             # Opencv图像处理库
from learning_interface.srv import GetObjectPosition   # 自定义的服务接口

lower_red = np.array([0, 90, 128])     # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])  # 红色的HSV阈值上限

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                              # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10) # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()                         # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换

        self.srv = self.create_service(GetObjectPosition,   # 创建服务器对象（接口类型、服务名、服务器回调函数）
                                       'get_target_position',
                                       self.object_position_callback)    
        self.objectX = 0
        self.objectY = 0                              

    def object_detect(self, image):
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)      # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red) # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)   # 图像中轮廓检测

        for cnt in contours:                                  # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)              # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)# 将苹果的轮廓勾勒出来
            cv2.circle(image, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                       # 将苹果的图像中心点画出来

            self.objectX = int(x+w/2)
            self.objectY = int(y+h/2)

        cv2.imshow("object", image)                            # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(50)

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')        # 输出日志信息，提示已进入回调函数
        image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')     # 将ROS的图像消息转化成OpenCV图像
        self.object_detect(image)                              # 苹果检测

    def object_position_callback(self, request, response):     # 创建回调函数，执行收到请求后对数据的处理
        if request.get == True:
            response.x = self.objectX                          # 目标物体的XY坐标
            response.y = self.objectY
            self.get_logger().info('Object position\nx: %d y: %d' %
                                   (response.x, response.y))   # 输出日志信息，提示已经反馈
        else:
            response.x = 0
            response.y = 0
            self.get_logger().info('Invalid command')          # 输出日志信息，提示已经反馈
        return response


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImageSubscriber("service_object_server")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
    
```

### 服务：求和
### 话题：通信 实现消息的传递（消息类型包含一个或者多个字段）
### 小海龟仿真器（原理）
## ROS2的常用命令行
1. 工作空间相关
- 创建工作空间     ` mkdir -p ~/ros2_ws/src    cd ~/ros2_ws/src`
- 编译工作空间    ` cd ~/ros2_ws    colcon build`
- 配置环境变量    ` source install/setup.bash`  #对bash终端
2. 功能包相关
- 创建功能包    ` ros2 pkg creat --build-type ament_cmake my_package --dependencies rclpy`
- 列出所有功能包    ` ros2 pkg list`
- 查找功能包的位置    ` ros2 pkg prefix my_package`
3. 节点相关
- 运行节点    ` ros2 run pkg_name node_name`
- 列出所有运行节点    ` ros2 node list`
- 查看节点信息    ` ros2 node info node_name`
4. 话题相关
- 列出所有话题    ` ros2 topic list`
- 查看话题消息类型    ` ros2 topic type /topic_name`
- 发布话题消息    ` ros2 topic pub -r 1 /my_topic std_msgs/msg/String "{data: 'Hello, ROS2!'}"`  #以1Hz的频率向话题发布（）类型的消息 消息内容为{……}
- 订阅话题消息    ` ros2 topic echo /topic_name`
5. 服务相关
- 列出所有服务    ` ros2 service list`
- 查看服务类型    ` ros2 service type /service_name`
- 调用服务    ` ros2 service call /my_service my_package/srv/MyService "{request_param: 'value'}"`
6. 参数相关
- 列出节点的所有参数    `ros2 param list node_name `
- 获取节点参数的值    ` ros2 param get node_name parameter_name`  #获取node_name中parameter_name节点的所有参数
- 设置节点参数的值    ` ros2 param set node_name parameter_name "value"`
7. 动作相关
- 列出所有动作    `ros2 action list `
- 查看动作类型    ` ros2 action type /action_name`
- 发送动作目标    ` ros2 action send_goal /my_action my_package/action/MyAction "{goal_param: 'value'}"`

## 补充

