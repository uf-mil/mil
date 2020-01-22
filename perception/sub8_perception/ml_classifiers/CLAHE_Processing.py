#!/usr/bin/env python
from __future__ import print_function
import sys
import tf
import cv2
import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import Image_Subscriber, Image_Publisher
from cv_bridge import CvBridge

'''
CLAHE utiltiy script that takes in images and returns a CLAHE balanced image.
Emphasizes color differences and contrast in an image.
'''


class CLAHEGenerator:

    def __init__(self):


        # Instantiate remaining variables and objects
        self.last_image_time = None
        self.last_image = None
        self.status = ''
        self.est = None
        self.visual_id = 0
        self.enabled = False
        self.bridge = CvBridge()



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

        return cv_image


def main(args):
    rospy.init_node('CLAHE', anonymous=False)
    CLAHEGenerator()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
