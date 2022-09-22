#!/usr/bin/env python3
from collections import deque

import cv2
import numpy as np
import rospy
from mil_ros_tools import Image_Publisher, Image_Subscriber
from sensor_msgs.msg import Image


class CameraCalibration:
    def __init__(self):
        self.camera = "/usb_cam/image_raw"
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.image_pub = Image_Publisher("/image/checkerboard")
        self.count = 0
        self.checkerboard = (6, 9)  # measured from inner corners
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros(
            (1, self.checkerboard[0] * self.checkerboard[1], 3), np.float32
        )
        self.objp[0, :, :2] = (
            np.mgrid[0 : self.checkerboard[0], 0 : self.checkerboard[1]].T.reshape(
                -1, 2
            )
            * 25.4
        )
        self.objpoints = deque()
        self.imgpoints = deque()
        self.width = None
        self.height = None
        self.shape = None

    def calculatePoints(self, image):
        self.count = self.count + 1
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.checkerboard,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if ret == True:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), self.criteria
            )
            img = cv2.drawChessboardCorners(image, self.checkerboard, corners2, ret)
            self.image_pub.publish(img)
            self.height, self.width = img.shape[:2]
            self.shape = gray.shape[::-1]
            if self.count >= 2:
                self.imgpoints.append(corners2)
                self.objpoints.append(self.objp)
                self.count = 0
                print(len(self.imgpoints))
                if len(self.imgpoints) > 100:
                    self.calculate_matrix()

    def undistort(self, image):
        h, w = image.shape[:2]
        mtx = np.asarray(
            [
                [722.25663718, 0, 652.46372445],
                [0, 722.68908779, 374.64826018],
                [
                    0,
                    0,
                    1,
                ],
            ]
        )
        dst = np.asarray(
            [[0.06689877, -0.13360559, 0.0073066, -0.00161229, 0.07727962]]
        )
        newmtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dst, (w, h), 1, (w, h))
        dist = cv2.undistort(image, mtx, dst, None, newmtx)
        x, y, w, h = roi
        dist = dist[y : y + h, x : x + w]
        self.image_pub.publish(dist)

    def image_cb(self, image):
        # self.undistort(image)
        self.calculatePoints(image)

    def calculate_matrix(self):
        print("Calculating camera matrix")
        obj = list(self.objpoints)
        img = list(self.imgpoints)
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj, img, self.shape, None, None
        )
        print("done")
        print(mtx)
        print(dist)
        print(rvecs)
        print(tvecs)


def main():
    rospy.init_node("camera_calibration", anonymous=False)
    CameraCalibration()

    rospy.spin()


if __name__ == "__main__":
    main()
