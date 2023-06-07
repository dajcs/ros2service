import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
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
ros2 run tf2_ros static_transform_publisher  0 0 -0.05  0 0 0  Attila leo09/base_link
ros2 run tf2_ros static_transform_publisher  -0.12 0 -0.40  0 0 0  Attila leo02/base_footprint

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
[INFO] [1686126520.711680481] [tf2_echo]: Waiting for transform world ->  Attila: Invalid frame ID "world" passed to canTransform argument target_frame - frame does not exist
At time 1686126521.670799271
- Translation: [1.891, -4.702, 0.650]
- Rotation: in Quaternion [0.214, -0.106, -0.553, 0.798]
At time 1686126521.800705911
- Translation: [1.883, -4.717, 0.653]
- Rotation: in Quaternion [0.135, 0.013, -0.575, 0.806]
At time 1686126522.920637227
- Translation: [1.890, -4.702, 0.651]
- Rotation: in Quaternion [0.203, -0.093, -0.549, 0.805]
At time 1686126524.680494213
- Translation: [1.883, -4.717, 0.653]
- Rotation: in Quaternion [0.137, 0.013, -0.578, 0.804]
At time 1686126525.680545003
- Translation: [1.914, -4.675, 0.637]
- Rotation: in Quaternion [0.481, 0.120, -0.324, 0.806]
At time 1686126526.650872360
- Translation: [1.870, -4.729, 0.656]
- Rotation: in Quaternion [0.001, -0.020, -0.679, 0.733]
^C[INFO] [1686126526.771579932] [rclcpp]: signal_handler(signal_value=2)
ism_student_ros2@ll106:~$ ros2 run tf2_ros tf2_echo Attila leo02/resource
[INFO] [1686126544.578260951] [tf2_echo]: Waiting for transform Attila ->  leo02/resource: Invalid frame ID "Attila" passed to canTransform argument target_frame - frame does not exist
At time 0.0
- Translation: [0.368, 0.000, -0.328]
- Rotation: in Quaternion [0.000, 0.005, 0.000, 1.000]
At time 0.0
- Translation: [0.368, 0.000, -0.328]
- Rotation: in Quaternion [0.000, 0.005, 0.000, 1.000]
^C[INFO] [1686126548.500518919] [rclcpp]: signal_handler(signal_value=2)


ism_student_ros2@ll106:~$ ros2 run tf2_ros tf2_echo world leo02/resource
[INFO] [1686126603.329063190] [tf2_echo]: Waiting for transform world ->  leo02/resource: Invalid frame ID "world" passed to canTransform argument target_frame - frame does not exist
At time 1686126605.281063572
- Translation: [2.161, -5.033, 0.395]
- Rotation: in Quaternion [0.266, -0.222, -0.618, 0.706]
At time 1686126606.201115461
- Translation: [2.038, -4.984, 0.268]
- Rotation: in Quaternion [0.140, 0.020, -0.589, 0.795]
At time 1686126607.300996200
- Translation: [2.096, -5.063, 0.372]
- Rotation: in Quaternion [0.210, -0.198, -0.664, 0.689]
At time 1686126608.290956314
- Translation: [2.050, -4.982, 0.272]
- Rotation: in Quaternion [0.138, 0.016, -0.578, 0.804]
At time 1686126609.281270339
- Translation: [1.750, -5.121, 0.360]
- Rotation: in Quaternion [-0.102, 0.076, -0.728, 0.674]
At time 1686126610.281229215
- Translation: [2.090, -5.064, 0.368]
- Rotation: in Quaternion [0.203, -0.191, -0.667, 0.691]
At time 1686126611.291344046
- Translation: [2.050, -4.982, 0.272]
- Rotation: in Quaternion [0.138, 0.017, -0.577, 0.805]
At time 1686126612.301247348
- Translation: [1.905, -5.105, 0.338]
- Rotation: in Quaternion [0.002, -0.015, -0.680, 0.733]
At time 1686126613.291106344
- Translation: [1.906, -5.105, 0.338]
- Rotation: in Quaternion [0.003, -0.016, -0.680, 0.733]
At time 1686126614.301197952
- Translation: [2.051, -4.981, 0.272]
- Rotation: in Quaternion [0.139, 0.017, -0.576, 0.805]
At time 1686126615.281139383
- Translation: [2.051, -4.982, 0.272]
- Rotation: in Quaternion [0.138, 0.017, -0.576, 0.806]



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
