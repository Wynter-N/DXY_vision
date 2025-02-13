import rclpy
from custom_message_pkg.msg import CustomMessage

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(CustomMessage, 'custom_topic', 10)

    msg = CustomMessage()
    msg.name = 'wynter'
    msg.age = 17
    msg.score = 34


    while True:
        publisher.publish(msg)
        node.get_logger().info('Publishing: "%s", %d, %.1f' % (msg.name, msg.age, msg.score))
        rclpy.spin_once(node, timeout_sec=0.1)

if __name__ == '__main__':
    main()