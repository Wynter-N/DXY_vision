import rclpy
from rclpy.node import Node
from sum_service_py.srv import Sum

class SumServer(Node):
    def __init__(self):
        super().__init__('sum_server')  #调用父类构造方法命名
        self.srv=self.create_service(Sum,'sum_service',self.handle_request) #Sum表示服务消息类型  ‘’服务名称  回调函数


    def handle_request(self ,request,response):
        response.sum=request.a+request.b
        self.get_logger().info(f'sum:  {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node=SumServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=='__main__':
    main()
