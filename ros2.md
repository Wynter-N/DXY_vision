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
这条命令会将install文件夹中的setup.bash文件中的环境变量设置加载到当前终端中
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
