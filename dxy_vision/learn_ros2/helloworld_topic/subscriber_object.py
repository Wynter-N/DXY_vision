import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldSubscriber(Node):
    def __init__ (self):
        super().__init__('hello_world_subscriber')
        self.subscription=self.create_subscription(String,'hello_world_topic',self.listener_callback,10)
        self.subscription

    def listener_callback(self,msg):
        self.get_logger().info('heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    hello_world_subscription=HelloWorldSubscriber()
    rclpy.spin(hello_world_subscription)
    hello_world_subscription.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()