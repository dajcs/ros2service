
'''
# create new frame and set tf to parent:
ros2 run tf2_ros static_transform_publisher   0.4 0 0   0 -0.2 0   leo02/camera_frame leo02/resource
ros2 run tf2_ros static_transform_publisher   0.4 0 0   0 -0.2 0   leo03/camera_frame leo03/resource

# get tf between two frames:
ros2 run tf2_ros tf2_echo leo02/map leo02/resource
ros2 run tf2_ros tf2_echo leo03/map leo03/resource
ros2 run tf2_ros tf2_echo world leo02/resource

- Translation: [4.545, -2.294, -0.064]
ros2 run tf2_ros static_transform_publisher   4.545 -2.294 -0.064   0 0 0   leo03/map leo03/resource2


# launch ajordan5's VRRP -> topic /Attila/pose
ros2 launch vrpn_client_ros sample.launch.py

# launch Dave's /Attila/pose -> /tf broadcast
ros2 launch optitrack_tf optitrack_tf.launch.py

# tf from world \ Attila --> leo09/base_footprint \ leo09/base_link
ros2 run tf2_ros static_transform_publisher  0 0 -0.05  0 0 0  Attila leo09/base_footprint
ros2 run tf2_ros static_transform_publisher  -0.12 0 -0.40  0 0 0  Attila leo02/base_footprint
ros2 run tf2_ros static_transform_publisher  -0.12 0 -0.40  0 0 0  Attila leo03/base_footprint

# add image to rviz
-> image -> /leo02/camera/image_raw

# teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/leo02/cmd_vel

# SLAM
.bashrc 
export ROS_DOMAIN_ID=51

# ethernet -> internet
# usb -> mesh Network

# ros2 topic list: /leo03/robot_description, joint_states

# 172.19.1.122 # Leo02 RPI
# 172.19.1.123 #       Jetson

# 172.19.1.132 # Leo03 RPI
# 172.19.1.133 #       Jetson

ssh xavier@172.19.1.123 # for Leo02
ssh xavier@172.19.1.133 # for Leo03
psw: xavier

foxy
ros2 launch leorover_bringup leorover_mapping_bringup.launch.py

# check from lab:
ros2 run rqt_tf_tree rqt_tf_tree
ros2 run rqt_graph rqt_graph


# for SLAM disconnect Attila tf
            /leo03/map
            /leo03/local_grid_ground
            /leo03/local_grid_obstacle
\ by topic /local_grid_obstacle/PointCloud2
           /map/Map



http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages


http://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html

bool data # e.g. for hardware enabling / disabling
---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages



exp:

ism_student_ros2@ll106:~$ ros2 run my_turtlesim pose_client 
[INFO] [1686126508.073553641] [pose_client]: 
    Result for triggering "get_pose":
        Successful: True
        Message:

        Latest Pose:
        Position:
        geometry_msgs.msg.Point(x=1.8829190731048584, y=-4.717029094696045, z=0.6529276371002197)
        Orientation:
        geometry_msgs.msg.Quaternion(x=0.1386466920375824, y=0.016347544267773628, z=-0.577654242515564, w=0.8042546510696411)

ism_student_ros2@ll106:~$ ros2 run my_turtlesim pose_client 
[INFO] [1686126509.981352389] [pose_client]: 
    Result for triggering "get_pose":
        Successful: True
        Message:

        Latest Pose:
        Position:
        geometry_msgs.msg.Point(x=1.8834675550460815, y=-4.717531681060791, z=0.652991533279419)
        Orientation:
        geometry_msgs.msg.Quaternion(x=0.13628806173801422, y=0.014444611966609955, z=-0.5753024220466614, w=0.8063771724700928)

        
ism_student_ros2@ll106:~$ ros2 run tf2_ros tf2_echo world Attila
At time 1686642331.216446501
- Translation: [3.724, -1.577, 0.575]
- Rotation: in Quaternion [0.023, -0.058, -0.768, 0.638]


ism_student_ros2@ll106:~$ ros2 run tf2_ros tf2_echo Attila leo03/resource
At time 0.0
- Translation: [0.368, 0.000, -0.328]
- Rotation: in Quaternion [0.000, 0.005, 0.000, 1.000]


ism_student_ros2@ll106:~$ ros2 run tf2_ros tf2_echo world leo03/resource
At time 1686642475.847782934
- Translation: [3.683, -1.954, 0.262]
- Rotation: in Quaternion [0.021, -0.047, -0.769, 0.637]

'''

import rclpy
from rclpy.node import Node
# from std_srvs.srv import Trigger, SetBool
import time
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Duration

class PoseTf(Node):

    def __init__(self):
        super().__init__('pose_tf')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # transform_a_b = self.tf_buffer.lookup_transform('A', 'B', self.get_clock().now())
        # self.get_logger().info(f"getting tf A -> B: {self.transform_ab}")

        # Allow some time for the buffer to fill up.
        time.sleep(5.0)

        # Check if the transformation is available
        # Wait up to 5 seconds for it
        if self.tf_buffer.can_transform('world', 'turtle1', self.get_clock().now(), timeout=Duration(seconds=5.0)):
            transform = self.tf_buffer.lookup_transform('world', 'turtle1', self.get_clock().now())
            self.get_logger().info("Transformation found!")
            self.get_logger().info(f"getting tf A -> B: {self.transform}")
        else:
            self.get_logger().warn("Transformation from 'A' to 'B' not available.")



    


def main(args=None):
    rclpy.init(args=args)

    pose_tf = PoseTf()
    rclpy.spin(pose_tf)

    # destroy the node when not used anymore
    pose_tf.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
