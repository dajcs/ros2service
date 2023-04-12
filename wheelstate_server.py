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

# /leo04/firmware/wheel_odom

'''
ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 topic info /leo04/firmware/wheel_odom
Type: leo_msgs/msg/WheelOdom
Publisher count: 1
Subscription count: 0

ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 interface show leo_msgs/msg/WheelOdom
# This message represents the pose and velocity of a differential wheeled robot, estimated from the wheel encoders.
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

ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 topic echo /leo04/firmware/wheel_odom

stamp:
  sec: 1671454094
  nanosec: 832663350
velocity_lin: 0.2358250916004181
velocity_ang: 0.02482498437166214
pose_x: 6.079624176025391
pose_y: 2.10227370262146
pose_yaw: 1.6158596277236938
---

rate: 20 Hz

'''


# /leo04/firmware/wheel_states

'''
ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 topic info /leo04/firmware/wheel_states
Type: leo_msgs/msg/WheelStates
Publisher count: 1
Subscription count: 0

ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 topic hz /leo04/firmware/wheel_states
average rate: 20.022
	min: 0.047s max: 0.054s std dev: 0.00199s window: 22

ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 interface show leo_msgs/msg/WheelStates
# This message describes the states of the wheels in Leo Rover
#
# The state of each wheel is defined by:
#  * the position of the wheel (rad),
#  * the velocity of the wheel (rad/s)
#  * the torque that is applied in the wheel (Nm)
#  * the PWM Duty cycle (%)
#
# The stamp specifies the time at which the wheel states were recorded.
#
# This message consists of a multiple arrays, one for each part of the wheel state.
# The order of the wheels in each array is: FL, RL, FR, RR

builtin_interfaces/Time stamp

float32[4] position
float32[4] velocity
float32[4] torque
float32[4] pwm_duty_cycle

ism_student_ros2@ll106:~/course_ws/src/my_turtlesim/my_turtlesim$ ros2 topic echo /leo04/firmware/wheel_states
stamp:
  sec: 1671454107
  nanosec: 982663350
position:
- 121.85830688476562
- 102.15898895263672
- 209.57540893554688
- 190.6271514892578
velocity:
- -2.7181358337402344
- -2.932725191116333
- 2.932725191116333
- 2.932725191116333
torque:
- -100.0
- -90.71089172363281
- 86.46089172363281
- 97.96089172363281
pwm_duty_cycle:
- -100.0
- -90.71089172363281
- 86.46089172363281
- 97.96089172363281
---

'''


from math import pi
#from leo_msgs.msg import WheelOdom
from leo_msgs.msg import WheelStates
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


MY_LEO = 'leo04'


class WheelStateServer(Node):

    def __init__(self):
        super().__init__('wheelstate_server')
        self.my_service = self.create_service(Trigger, 'wheelstate_client', self.wheelstate_client_callback)

        self.odometer_subscriber = self.create_subscription(
            WheelStates,
            f'/{MY_LEO}/firmware/wheel_states',   
            self.wheelstate_msg_callback,
            qos_profile = qos_profile
        )

        self.turtle_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 1)   # monitoring cmd_vel on turtle_sim


        self.i = 0                # counter = 0
        self.log_active = False   # initially the log printing is deactivated
        self.x0 = 0.0             # initially the reference values are 0
        self.y0 = 0.0
        self.yaw0 = 0.0
        self.latest_pose_x = 0    #   to be on safe side, normally this is set 
        self.latest_pose_y = 0    #      after receiving the first wheel_odom message
        self.latest_pose_yaw = 0

