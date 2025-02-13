import rclpy
from custom_message_pkg.msg import CustomMessage

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('listener')
    subscription = node.create_subscription(CustomMessage, 'custom_topic', callback, 10)

    def callback(msg):
        node.get_logger().info('Received: Name="%s", Age=%d, Score=%.1f' % (msg.name, msg.age, msg.score))

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
