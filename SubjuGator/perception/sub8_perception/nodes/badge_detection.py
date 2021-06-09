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
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridgeError
from image_geometry import PinholeCameraModel
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sub8_msgs.srv import VisionRequest, VisionRequestResponse
from mil_ros_tools import Image_Subscriber, Image_Publisher
from cv_bridge import CvBridge
from mil_misc_tools import FprintFactory
from visualization_msgs.msg import Marker
from sub8_msgs.msg import ThrusterCmd



'''
Perception component of the Torpedo Board Challenge for RoboSub2021.
Authors: Alex Perez and Andrew Knee
'''

class Badge_Detection:

    def __init__(self):

        self.print_info = FprintFactory(title="MISSION").fprint

        # Image Subscriber and Camera Information
        self.camera = rospy.get_param('~camera_topic', '/camera/front/left/image_rect_color')
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        
        #or maybe use self.image_sub = yield self.nh.subscribe(
        #                               "/camera/front/left/image_raw", self.image_cb)
        
        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

        # Publisher for BadgeTakedown Mission
        self.move_info_pub = rospy.Publisher("/move_to_badge", ThrusterCmd, queue_size=10)

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find the torpedo board.
        '''

        msg = ThrusterCmd()
        #we will insert the command into msg.name
        #we will insert the movement into msg.thrust
        #we will use move_info_pub.publish(msg)
        msg.name = "FORWARD"
        msg.thrust = 1.0
        self.move_info_pub.publish(msg)

        return None

def main(args):
    rospy.init_node('badge_detection', anonymous=False)
    Badge_Detection()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
