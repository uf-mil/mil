#!/usr/bin/env python
from __future__ import print_function
import sys
import tf
import mil_ros_tools
import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError
from sub8_vision_tools import MultiObservation
from image_geometry import PinholeCameraModel
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Pose, Point
from sub8_msgs.srv import VisionRequest, VisionRequestResponse

from mil_ros_tools import Image_Subscriber, Image_Publisher
from cv_bridge import CvBridge, CvBridgeError

'''
CLAHE utiltiy script that takes in images and returns a CLAHE balanced image. Emphasizes color differences and contrast in an image. 
'''


class CLAHE_generator:

    def __init__(self):

        self.camera = rospy.get_param('~camera_topic',
                                      '/camera/front/left/image_rect_color')

        # Instantiate remaining variables and objects
        self.last_image_time = None
        self.last_image = None
        self.tf_listener = tf.TransformListener()
        self.status = ''
        self.est = None
        self.visual_id = 0
        self.enabled = False
        self.bridge = CvBridge()

        # Image Subscriber and Camera Information

        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        '''
        These variables store the camera information required to perform
        the transformations on the coordinates to move from the subs
        perspective to our global map perspective. This information is
        also necessary to perform the least squares intersection which
        will find the 3D coordinates of the torpedo board based on 2D
        observations from the Camera. More info on this can be found in
        sub8_vision_tools.
        '''

        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()

        self.image_pub = Image_Publisher("CLAHE/debug")

        # Debug
        self.debug = rospy.get_param('~debug', True)

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find the torpedo board.
        '''

        self.last_image = image

        if self.last_image_time is not None and \
                self.image_sub.last_image_time < self.last_image_time:
            # Clear tf buffer if time went backwards (nice for playing bags in
            # loop)
            self.tf_listener.clear()

        self.last_image_time = self.image_sub.last_image_time
        self.CLAHE(image)
        print('published')

    def CLAHE(self, cv_image):
        '''
        CLAHE (Contrast Limited Adaptive Histogram Equalization)
        This increases the contrast between color channels and allows us to
        better differentiate colors under certain lighting conditions.
        '''
        clahe = cv2.createCLAHE(clipLimit=9.5, tileGridSize=(4, 4))

        # convert from BGR to LAB color space
        lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)  # split on 3 different channels

        l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        lab = cv2.merge((l2, a, b))  # merge channels
        cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        self.image_pub.publish(cv_image)


def main(args):
    rospy.init_node('CLAHE', anonymous=False)
    CLAHE_generator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
