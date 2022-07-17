#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

class HSVCalibration:
    def __init__(self):
        self.camera = "/camera/front/left/image_raw"
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.image_pub = Image_Publisher("/image/hsv")
        self.reconfigure_server = DynamicReconfigureServer(HSVCalibrationConfig, self.reconfigure)
        self.lower = np.array([0,0,0], dtype=np.uint8)
        self.upper = np.array([0,0,0], dtype=np.uint8)

    def image_cb(self, img):
        img = np.array(img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, self.lower, self.upper)
        img = cv2.bitwise_and(img, img, mask=mask)
        self.image_pub.publish(img)

    def reconfigure(self, config, level):
        try:
            self.lower = np.array(self.parse_string(config['dyn_lower']))
            self.upper = np.array(self.parse_string(config['dyn_upper']))
        
        except ValueError as e:
            rospy.logwarn('Invalid dynamic reconfigure: {}'.format(e))
            return self.last_config
        
        self.last_config = config
        return config

def main():
    rospy.init_node('hsv_calibration', anonymous=False)
    HSVCalibration()

    rospy.spin()

if __name__ == '__main__':
    main()
