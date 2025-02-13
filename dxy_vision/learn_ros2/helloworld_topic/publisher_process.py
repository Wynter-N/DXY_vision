import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#定义主函数
def main(args=None):
    #初始化ros2客户端库
    rclpy.init(args=args)
    #创建一个ros2节点，节点名称为 'hello_world_publisher'
    node=Node('hello_world_publisher')
    #创建一个发布者，发布到'hello_world_topic' 话题，消息类型为String 队列大小为10
    publisher=node.create_publisher(String,'hello_world_topic',10)
    #创建一个String类型的消息对象
    msg=String()
    #设置消息内容
    msg.data='Hello World'


    #循环发布消息    ：1.发布消息  2.打印日志信息  3.等待一次回调，非阻塞  
    while rclpy.ok():
        publisher.publish(msg)
        node.get_logger().info('Publishing : "%s"' % msg.data)
        rclpy.spin_once(node)
    #销毁节点
    node.destroy_node()
    #关闭ros2客户端库
    rclpy.shutdown()

#程序入口
if __name__ =='__main__':
    main()
