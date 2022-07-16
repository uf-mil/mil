#!/usr/bin/env python
import cv2
import rospy
from datetime import datetime
import time
from Queue import Queue
from yaml import dump
import os
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraCalibration:
    def __init__(self):
        self.camera = "/camera/front/left/image_raw"
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.image_pub = Image_Publisher("/image/checkerboard")
        self.bridge = CvBridge()
        self.curr_time = self.getMs()
        self.checkerboard = (6,9) # measured from inner corners
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((1, self.checkerboard[0] * self.checkerboard[1], 3), np.float32)
        self.objp[0, :, :2] = np.mgrid[0:self.checkerboard[0], 0:self.checkerboard[1]].T.reshape(-1, 2) * 25.4
        self.objpoints = Queue()
        self.imgpoints = Queue()
        self.width = None
        self.height = None
        self.shape = None
        

    def getMs(self):
        dt_obj = datetime.strptime('20.12.2016 09:38:42,76','%d.%m.%Y %H:%M:%S,%f')
        return int(time.mktime(dt_obj.utctimetuple()) * 1000 + dt_obj.microsecond / 1000) 

    def calculatePoints(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret == True:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            img = cv2.drawChessboardCorners(image, self.checkerboard, corners2, ret)
            self.image_pub.publish(img)
            self.height, self.width = img.shape[:2]
            self.shape = gray.shape[::-1]
            if self.getMs() - self.curr_time > 500:
                self.imgpoints.put(corners2)
                self.objpoints.put(self.objp)
                self.curr_time = self.getMs()

    def image_cb(self, image):
        self.calculatePoints(image)

    def drain(queue):
        while True:
            try:
                yield queue.get_nowait()
            except queue.Empty():
                break

    def calculate_matrix(self):
        obj = list(self.objpoints)
        img = list(self.imgpoints)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, self.shape, None, None)
        if ret == True:
            stream = file("calibration.yaml", 'w')
            yaml.dump(mtx, stream)
            yaml.dump(ret, stream)

    def __del__(self):
        self.calculate_matrix()

def main():
    rospy.init_node('camera_calibration', anonymous=False)
    CameraCalibration()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Calculating camera matrix")

if __name__ == '__main__':
    main()
