import rclpy
from rclpy.node import Node

class PrintStringNode(Node):
    def __init__(self):
        super().__init__('print_string_node')
        # 声明参数，默认值为 "Hello, ROS 2!"
        self.declare_parameter('print_string', 'Hello, ROS 2!')
        # 获取参数值
        self.string_to_print = self.get_parameter('print_string').get_parameter_value().string_value
        # 创建一个定时器，每 1 秒执行一次回调函数
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 获取最新的参数值
        self.string_to_print = self.get_parameter('print_string').get_parameter_value().string_value
        # 打印参数值
        self.get_logger().info(f"Printing: {self.string_to_print}")

def main(args=None):
    rclpy.init(args=args)
    node = PrintStringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

