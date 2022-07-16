#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher

class HSVCalibration:
    def __init__(self):
        self.camera = "/camera/front/left/image_raw"
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.image_pub = Image_Publisher("/image/hsv")
        self.file_name = "hsv.txt"
        self.default = np.array([[0, 0, 0], [179, 255, 255]])

    def image_cb(self, img):
        data = None
        try:
            data = np.genfromtxt(self.file_name, delimiter=",", usemask=True)
        except Exception:
            np.savetxt(self.file_name, self.default)
            data = self.default
        img = np.array(img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, data[0], data[1])
        img = cv2.bitwise_and(img, img, mask=mask)
        self.image_pub.publish(img)

def main():
    rospy.init_node('hsv_calibration', anonymous=False)
    HSVCalibration()

    rospy.spin()

if __name__ == '__main__':
    main()
