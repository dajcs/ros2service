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

'''
ros2 interface show leo_msgs/msg/WheelOdom
# This message represents the pose and velocity of a differential wheeled robot, 
# estimated from the wheel encoders.
#
# The velocity_* fields represent the linear and angular velocity of the robot.
# The pose_* fields represent the x, y and yaw pose of the robot w.r.t. the starting pose.
#
# The coordinate frame that represents the robot is located at the center of rotation.

builtin_interfaces/Time stamp
float32 velocity_lin
float32 velocity_ang
float32 pose_x
float32 pose_y
float32 pose_yaw

stamp:
  sec: 1665152368
  nanosec: 683160648
velocity_lin: 0.0
velocity_ang: 0.0
pose_x: 0.1369149535894394
pose_y: -0.00023565816809423268
pose_yaw: 0.00815676711499691

rate: 20 Hz
'''

from math import pi
from leo_msgs.msg import WheelOdom
# from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


MY_LEO = 'leo09'



class OdometerServer(Node):
    
    def __init__(self):
        super().__init__('odometer_server')
        self.my_service1 = self.create_service(SetBool, 'log_client', self.log_client_callback)
        self.my_service2 = self.create_service(Trigger, 'reset_client', self.reset_client_callback)

        self.odometer_subscriber = self.create_subscription(
            WheelOdom,
            f'/{MY_LEO}/firmware/wheel_odom',   
            self.odometer_msg_callback,
            qos_profile = qos_profile
        )

        self.odom_state_publisher = self.create_publisher(WheelOdom, f'/{MY_LEO}/odom_state', 1)

        self.i = 0                # counter = 0
        self.log_active = False   # initially the log printing is deactivated
        self.x0 = 0.0             # initially the reference values are 0
        self.y0 = 0.0
        self.yaw0 = 0.0
        self.latest_pose_x = 0    #   to be on safe side, normally this is set 
        self.latest_pose_y = 0    #      after receiving the first wheel_odom message
        self.latest_pose_yaw = 0


    
    def log_client_callback(self, request, response):                           # log client start / stop
        self.log_active = request.data  # turn on/off odometer prints

        if request.data:
            self.get_logger().info('Received request: odometer logs turn On')
        else:
            self.get_logger().info('Received request: odometer logs turn Off')

        response.success = True
        response.message = f'Odometer logging active: {self.log_active}'

        return response



    def reset_client_callback(self, request, response):                           # reset client received
        request # placeholder

        self.get_logger().info('Received request: odometer logs Reset')

        self.x0 = self.latest_pose_x
        self.y0 = self.latest_pose_y
        self.yaw0 = self.latest_pose_yaw

        response.success = True
        response.message = f'''
        Reset at positions:
            pose_x: {self.x0}
            pose_y: {self.y0}
            pose_yaw: {self.yaw0}\n'''

        return response





    def odometer_msg_callback(self, msg):                                       # odometer message from LEO rover
        self.latest_pose_x = msg.pose_x
        self.latest_pose_y = msg.pose_y
        self.latest_pose_yaw = msg.pose_yaw

        if self.log_active:
            msg.pose_x -= self.x0
            msg.pose_y -= self.y0
            msg.pose_yaw = (msg.pose_yaw - self.yaw0) % (2*pi)

            self.odom_state_publisher.publish(msg)                              # publish relative pose to leoXX/odom_state

            if self.i % 20 == 0:                                                # print every 20th message (about 1 Hz)
                self.get_logger().info(f'''publishing on /{MY_LEO}/odom_state
                stamp:
                    sec: {msg.stamp.sec}   
                    nanosec: {msg.stamp.nanosec}
                velocity_lin: {msg.velocity_lin}
                velocity_ang: {msg.velocity_ang}
                pose_x: {msg.pose_x}
                pose_y: {msg.pose_y}
                pose_yaw: {msg.pose_yaw}
                ''')
        
        self.i += 1
    

def main(args=None):
    rclpy.init(args=args)

    odometer_server = OdometerServer()
    rclpy.spin(odometer_server)

    # destroy the node when not used anymore
    odometer_server.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()

