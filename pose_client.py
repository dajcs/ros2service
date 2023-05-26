import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
'''
# create new frame and set tf to parent:
ros2 run tf2_ros static_transform_publisher   0.4 0 0   0 -0.2 0   leo09/camera_frame leo09/resource

# get tf between two frames:
ros2 run tf2_ros tf2_echo leo09/base_link leo09/resource

# launch ajordan5's VRRP -> topic /Attila/pose
ros2 launch vrpn_client_ros sample.launch.py

# launch Dave's /Attila/pose -> /tf broadcast
ros2 launch optitrack_tf optitrack_tf.launch.py

# tf from world \ Attila --> leo09/base_footprint \ leo09/base_link
ros2 run tf2_ros static_transform_publisher  0 0 -0.05  0 0 0  Attila leo09/base_footprint



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


class PoseClient(Node):

    def __init__(self):
        super().__init__('pose_client')
        self.my_client = self.create_client(Trigger, 'get_pose')
        while not self.my_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service <{self.my_client.srv_name}> to become available...')
        self.req = Trigger.Request()

    
    def send_request(self):
        self.future = self.my_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    


def main(args=None):
    rclpy.init(args=args)

    log_client = PoseClient()
    response = log_client.send_request()
    log_client.get_logger().info(f'''
    Result for triggering "get_pose":
        Successful: {response.success}
        Message:\n{response.message}''')
    
    log_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()