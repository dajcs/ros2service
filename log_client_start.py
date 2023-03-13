import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
'''
http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages


http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html

bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages

'''


class OdometerClient(Node):

    def __init__(self):
        super().__init__('odometer_client')
        self.my_client = self.create_client(SetBool, 'log_client')
        while not self.my_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "log_client" to become available...')
        self.req = SetBool.Request()
        self.req.data = True              # start logging

    
    def send_request(self):
        self.future = self.my_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    log_client = OdometerClient()
    response = log_client.send_request()
    log_client.get_logger().info(f'''
    Result for triggering "log_client":
        Successful: {response.success}
        Message:    {response.message}''')
    
    log_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
