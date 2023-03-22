import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
'''
http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html

---
bool success   # indicate successful run of triggered service
string message # informational, e.g. for error messages
'''

'''
ros2 topic info /leo09/camera/image_color
Type: sensor_msgs/msg/Image
#########################

ros2 interface show sensor_msgs/msg/Image

# This message contains an uncompressed image
# (0, 0) is at top-left corner of image

std_msgs/Header header # Header timestamp should be acquisition time of image
                             # Header frame_id should be optical frame of camera
                             # origin of frame should be optical center of cameara
                             # +x should point to the right in the image
                             # +y should point down in the image
                             # +z should point into to plane of the image
                             # If the frame_id here and the frame_id of the CameraInfo
                             # message associated with the image conflict
                             # the behavior is undefined

uint32 height                # image height, that is, number of rows
uint32 width                 # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.ros.org and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.hpp

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

###################################################################################

ros2 topic echo /leo10/camera/image_color

header:
  stamp:
    sec: 1674742402
    nanosec: 192149572
  frame_id: camera_optical_frame
height: 480
width: 640
encoding: rgb8
is_bigendian: 0
step: 1920
data:
- 79
- 61
- 7
- 147
- '...'
---

'''

# from leo_msgs.msg import WheelOdom
# from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
qos_profile = QoSProfile(
    reliability = QoSReliabilityPolicy.BEST_EFFORT,
    history = QoSHistoryPolicy.KEEP_LAST,
    depth = 1
)


MY_LEO = 'leo09'


import pickle
import sys

def save_image(image_message, img_nr):

    # with open('ros2_img_msg.pkl', 'wb') as f:
    #     pickle.dump(image_message, f) 

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    # cv2.imshow('', cv_image)

    # with open('cv2_bridge_img.pkl', 'wb') as f:
    #     pickle.dump(cv_image, f) 

    fname = f'image_{img_nr}.png'
    result = cv2.imwrite(fname, cv_image)
    print(f'cv2.imwrite result: {result}')
    return fname



class CameraServer(Node):
    
    def __init__(self):
        super().__init__('camera_server')
        self.my_service = self.create_service(Trigger, 'camera_server', self.camera_server_callback)

        self.camera_subscriber = self.create_subscription(
            Image,
            f'/{MY_LEO}/camera/image_color',   
            self.camera_msg_callback,
            qos_profile = qos_profile
        )

        self.i = 0                # counter = 0
        self.image_message = None # we haven't yet received image from rover


    def camera_msg_callback(self, msg):                                       # camera/image_color message from LEO rover
        self.image_message = msg         # store image
        self.i += 1


    def camera_server_callback(self, request, response):                           # reset client received
        request # placeholder, request empty for Trigger

        self.get_logger().info('Received request: Please take a picture')

        if self.image_message:
            location = save_image(self.image_message, self.i)
            response.success = True
            response.message = f'\nThe picture has been saved at location: {location}\n'
        else:
            response.success = False
            response.message = f'There is no picture received yet from topic /{MY_LEO}/camera/image_color\n'

        return response




def main(args=None):
    rclpy.init(args=args)

    camera_server = CameraServer()
    rclpy.spin(camera_server)

    # destroy the node when not used anymore
    camera_server.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()

