import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#定义发布者类  继承自Node
class HelloWorldPublisher(Node):
    def __init__(self):
		#调用父类的构造函数 设置节点名称
        super().__init__('hello_world_publisher')
		#创建一个发布者 发布到 话题 消息类型  队列大小
        self.publisher=self.create_publisher(String,'hello_world_topic',10)
		#设置定时器周期为1s
        timer_period=1.0
		#创建一个定时器，周期调用timercallback函数
        self.timer=self.create_timer(timer_period,self.timer_callback)
		#创建一个String类型的消息对象
        self.msg=String()
		#设置消息内容
        self.msg.data='HEllo World'


	#定义定时器回调函数   1.发布消息  2.打印消息日至
    def timer_callback(self):
            self.publisher_.publish(self.msg)
            self.get_logger().info('Publishing: "%s"' % self.msg.data)

            
#定义主函数  初始化 创建发布者对象 进入事件循环等待回调    销毁节点
def main(args=None):
     rclpy.init(args=args)
     hello_world_publisher=HelloWorldPublisher()
     rclpy.spin(hello_world_publisher)
     hello_world_publisher.destroy_node()
     rclpy.shutdown()

#程序入口
if __name__ == '__main__':
     main()
	