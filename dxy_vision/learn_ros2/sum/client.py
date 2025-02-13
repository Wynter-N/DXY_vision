import sys
import rclpy
from rclpy.node import Node
from sum_service_py.srv import Sum

class SumClient(Node):
    def __init__(self):
        super().__init__('sum_client')
        self.client=self.create_client(Sum,'sum_service')

    def send_request(self,a,b):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting……')

        request=Sum.request()
        request.a=a
        request.b=b

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,future)

        if future.result() is not None:
            self.get_logger().info(f'Sum: {future.result().sum}')
        else:
            self.get_logger().error('no response')


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv)!=3:
        print('client: a b')
        return
    client=SumClient()
    client.send_request(int(sys.argv[1]),int(sys.argv[2]))
    client.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
