import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
'''
http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
'''

class MyServiceClient(Node):

    def __init__(self):
        super().__init__('my_service_client')
        self.my_client = self.create_client(Trigger, 'draw_circle')
        while not self.my_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        self.req = Trigger.Request()


    def send_request(self):
        self.future = self.my_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



def main(args=None):
    rclpy.init(args=args)

    my_service_client = MyServiceClient()
    response = my_service_client.send_request()
    my_service_client.get_logger().info(f'Response successful: {response.success}')
    my_service_client.get_logger().info(f'Response message: {response.message}')

    # destroy the node when it is not used anymore
    my_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
