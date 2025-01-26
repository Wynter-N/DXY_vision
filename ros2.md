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


