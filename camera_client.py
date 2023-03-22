import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
'''
http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

'''


class CameraClient(Node):

    def __init__(self):
        super().__init__('camera_client')
        self.my_client = self.create_client(Trigger, 'camera_server')
        while not self.my_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service <{self.my_client.srv_name}> to become available...')
        self.req = Trigger.Request()

    
    def send_request(self):
        self.future = self.my_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    camera_client = CameraClient()
    while True:
        response = camera_client.send_request()
        camera_client.get_logger().info(f'''
        Result for triggering "log_client":
            Successful: {response.success}
            Message:    {response.message}''')
        if response.success:
            break
    
    camera_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
