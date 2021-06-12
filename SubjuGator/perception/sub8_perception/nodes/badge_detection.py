#!/usr/bin/env python
from __future__ import print_function
import sys
import tf
import mil_ros_tools
import cv2
import numpy as np
import rospy
import imutils
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
from sub8_msgs.msg import PathPoint



'''
Perception component of the Torpedo Board Challenge for RoboSub2021.
Authors: Alex Perez and Andrew Knee
'''

class Badge_Detection:

    def __init__(self):

        self.print_info = FprintFactory(title="MISSION").fprint

        #Image Subscriber and Camera Information
        self.lower = rospy.get_param('~lower_color_threshold_badge', [15, 50, 100])
        self.upper = rospy.get_param('~upper_color_threshold_badge', [30, 255, 180])
        self.min_radius = rospy.get_param('~min_radius', 10)
        self.camera = rospy.get_param('~camera_topic', '/camera/front/left/image_rect_color')
        #self.camera = rospy.get_param('~camera_topic', '/usb_cam/image_raw')
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        
        #or maybe use self.image_sub = yield self.nh.subscribe(
        #                               "/camera/front/left/image_raw", self.image_cb)
        
        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

        # Publisher for BadgeTakedown Mission
        self.move_info_pub = rospy.Publisher("/move_to_badge", PathPoint, queue_size=10)

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find the torpedo board.
        '''

        image = self.process(image)
        center, big, direction = self.findCenter(image, self.camera_info.width, self.camera_info.height)
        vector = None

        msg = PathPoint()

        #0.0 is we don't see the badge
        #1.0 we have a vector
        #2.0 we've arrived

        if direction is 0:
            msg.yaw = 0.0
            if big is True:
                msg.yaw = 2.0
        else:
            msg.yaw = direction

        self.move_info_pub.publish(msg)

        return None


    def process(self, image):
        clahe = cv2.createCLAHE(5., (4, 4))
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l2 = clahe.apply(l)

        lab = cv2.merge((l2, a, b))
        image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return image


    def findCenter(self, image, width, height):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
        lower = np.array(self.lower, dtype="uint8")
        upper = np.array(self.upper, dtype="uint8")

        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        found = False
        center = (0,0)
        big = False
        rotateDir = 0

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            Center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            #print("Radius: " + str(radius))
            if radius > 40:
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0))
                cv2.circle(image, center, 5, (255, 0, 0), -1)
                found = True
                center = Center
                center_varL = (width / 2.0) * .9
                center_varR = (width / 2.0) * 1.1
                if center[0] < center_varL:
                    rotateDir = -1
                elif center[0] > center_varR:
                    rotateDir = 1
                else:
                    rotateDir = 0
                if (radius > 400):
                    big = True

        #print(center[0])

        if found and center is not (0,0):
            return center, big, rotateDir
        else:
            return None, None, -2 
        

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
