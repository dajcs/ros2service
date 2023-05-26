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
ros2 topic info /Attila/pose
Type: geometry_msgs/msg/PoseStamped

ros2 interface show geometry_msgs/msg/PoseStamped
# A Pose with reference coordinate frame and timestamp

std_msgs/Header header
Pose pose

ros2 interface show geometry_msgs/msg/Pose
# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation
ism_student_ros2@ll106:~$ ros2 interface show geometry_msgs/msg/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
ism_student_ros2@ll106:~$ ros2 interface show geometry_msgs/msg/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x 0
float64 y 0
float64 z 0
float64 w 1


--
header:
  stamp:
    sec: 1684920756
    nanosec: 952136136
  frame_id: world
pose:
  position:
    x: 3.7631547451019287
    y: -3.253174304962158
    z: 0.3303624391555786
  orientation:
    x: -0.02281196229159832
    y: -0.03939484804868698
    z: -0.5883009433746338
    w: -0.8073598146438599
---

rate: 100 Hz
'''

from math import pi
from geometry_msgs.msg import PoseStamped
# from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


MY_LEO = 'leo09'
MY_OPTI = 'Attila'



class PoseServer(Node):
    
    def __init__(self):
        super().__init__('pose_server')
        self.my_service = self.create_service(Trigger, 'get_pose', self.provide_pose_callback)

        self.odometer_subscriber = self.create_subscription(
            PoseStamped,
            f'/{MY_OPTI}/pose',   
            self.pose_msg_callback,
            qos_profile = qos_profile
        )

        self.position = None



    
    def provide_pose_callback(self, request, response):                           # provide_pose_callback received
        request # placeholder

        self.get_logger().info('Received request: get_pose')

        response.success = True
        response.message = f'''
        Latest Pose:
        {self.position}\n'''

        return response



    def pose_msg_callback(self, msg):                                       # odometer message from LEO rover
        self.position = msg.pose.position



def main(args=None):
    rclpy.init(args=args)

    pose_server = PoseServer()
    rclpy.spin(pose_server)

    # destroy the node when not used anymore
    pose_server.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()

