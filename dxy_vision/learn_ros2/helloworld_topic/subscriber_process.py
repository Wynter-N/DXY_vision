import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#定义主函数
def main(args=None):
	#初始化ros2客户端
    rclpy.init(args=args)
	#创建节点
    node=Node('hello_world_subscriber')
	#定义回调函数，当收到消息的时候调用
    def callback(msg):
		#打印收到的消息内容
        node.get_logger().info('heard: "%s"'%msg.data)
		
		
	#创建一个订阅者 
    subscription=node.create_subscription(String,'hello_world_topic',callback,10)
	#进入事件循环等待消息
    rclpy.spin(node)
	#销毁节点
    node.destroy_node()
	#关闭ros2客户端
    rclpy.shutdown()
	
#程序入口
if __name__ == '__main__':
    main()
