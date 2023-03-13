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

from geometry_msgs.msg import Twist


class MyServiceServer(Node):
    
    def __init__(self):
        super().__init__('odom_server')
        self.my_service = self.create_service(SetBool, 'log_client', self.log_client_callback)
        self.my_service = self.create_service(Trigger, 'reset_client', self.reset_client_callback)
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    
    def draw_circle_callback(self, request, response):
        request # placeholder, we're not processing it since it is empty
        self.get_logger().info('Received request to draw a circle!')
        response.success = True
        response.message = "Starting to draw a circle!"
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity_callback)

        return response
    

    def publish_velocity_callback(self):
        my_velocity = Twist()
        my_velocity.linear.x = 0.5
        my_velocity.angular.z = 0.5
        self.publisher.publish(my_velocity)
        self.get_logger().info(f'''
        Publishing velocity:
            linear.x: {my_velocity.linear.x}
            angular.z: {my_velocity.angular.z}''')
        
    

def main(args=None):
    rclpy.init(args=args)

    my_service_server = MyServiceServer()
    rclpy.spin(my_service_server)

    # destroy the node when not used anymore
    my_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()




