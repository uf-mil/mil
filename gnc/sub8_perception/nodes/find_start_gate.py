#!/usr/bin/env python
import sys
import cv2
import rospy
import itertools
import numpy as np

import image_geometry
import message_filters

from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Gate:
    def __init__(self):
        # Throwback to the good ol' days (We'll have to fine tune for the situation)
        self.yellow_min = np.array([0, 50,50], np.uint8)
        self.yellow_max = np.array([29,250,250], np.uint8)

        # Detected gate info & RViz visualization
        self.marker = Marker()
        self.marker.type = Marker.CUBE

        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.5
        self.marker.color.a = 1.0

        # Orientation information
        self.gate_pose = None
        self.gate_distance = None


class GateFinder:
    def __init__(self, gate):
        self.last_image = None
        self.last_image_timestamp = None
        self.last_draw_image = None
        self.gate = gate

        self.lcam_info, self.rcam_info = None, None
        self.lP, self.rP = None, None

        self.lcam_info_sub = rospy.Subscriber('/stereo/left/camera_info', CameraInfo, self.cinfo_cb, callback_args = 'l')
        self.rcam_info_sub = rospy.Subscriber('/stereo/right/camera_info', CameraInfo, self.cinfo_cb, callback_args = 'r')

        # Hacky shit to get the camera info and unsubscribe
        # TODO: Implement service that gathers camera_info
        # TODO: Reimplement this w/ message filters
        while self.lcam_info is None or self.rcam_info is None:
            pass

        print 'Left Camera Info: ', self.lcam_info
        print 'Right Camera Info: ', self.rcam_info, '\n'

        # Unsubscribe from camera message topics
        self.lcam_info_sub.unregister()
        self.rcam_info_sub.unregister()

        # Begin stereo camera dirty work
        self.bridge = CvBridge()
        self.lcam_image_sub = message_filters.Subscriber('/stereo/left/image_rect_color', Image)
        self.rcam_image_sub = message_filters.Subscriber('/stereo/right/image_rect_color', Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.lcam_image_sub, self.rcam_image_sub], 5, 1)
        self.ts.registerCallback(self.image_cb)

    # Callback functions
    def cinfo_cb(self, cam_msg, cam):
        if cam == 'l':
            self.lcam_info = cam_msg
        elif cam == 'r':
            self.rcam_info = cam_msg
        else:
            pass

    def image_cb(self, limage, rimage):
        try:
            x_sol = []
            limg = self.bridge.imgmsg_to_cv2(limage, "bgr8")
            rimg = self.bridge.imgmsg_to_cv2(rimage, "bgr8")

            l_pts = self.find_contours(limg, 'l')
            r_pts = self.find_contours(rimg, 'r')

            if l_pts is not None and r_pts is not None:
                assert len(l_pts) == len(r_pts), 'Lists have different dimensions'

                self.fig = plt.figure()
                ax = self.fig.add_subplot(111, projection='3d')

                for i, pt in enumerate(l_pts):
                    print 'Attempting to triangulate points:'
                    x_sol.append(self.triangulate_Linear_LS(l_pts[i], r_pts[i], self.lP, self.rP))

                # Plot results
                for pts in x_sol:
                    print 'Ploting: ', pts[0], pts[1], pts[2]
                    ax.scatter(pts[0], pts[1], pts[2], c = 'r', marker='o')
                plt.show()  # Blocks until window is closed
            else:
                pass
        except CvBridgeError as e:
            print e

    # Image processing
    def ncc(self, image, scale=15):
        # Using template matching to emulate NCC
        # Using circle kernel
        kernel = np.ones((scale, scale)) * -1
        midpoint = (scale // 2, scale // 2)
        cv2.circle(kernel, midpoint, midpoint[0], 1, -1)

        res = cv2.matchTemplate(image, kernel, cv2.TM_CCOEFF_NORMED)
        cv2.imshow('kernel test', kernel)
        return res

    def triangulate_Linear_LS(self, lpoint, rpoint, lP, rP):
        # From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997

        # lpoint -> Left camera point (homogenous)
        # rpoint -> Right camera point (homogenous)
        # lP -> Left camera projection matrix
        # rP -> Right camera projection matrix

        # Build matrix A for homogenous equation system Ax = 0
        # Assume X = (x,y,z,1), for Linear-LS method
        # Which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1

        # !!! No need to provide homogenous coordinates, (u, v) suffice !!!

        print 'Left Point: ', lpoint
        print 'Right Point: ', rpoint, '\n'

        A = np.zeros((4,3))
        B = np.zeros((4,1))
        X = np.zeros((3,1))

        A[0][0] = lpoint[0]*lP[2][0] - lP[0][0]
        A[0][1] = lpoint[0]*lP[2][1] - lP[0][1]
        A[0][2] = lpoint[0]*lP[2][2] - lP[0][2]

        A[1][0] = lpoint[1]*lP[2][0] - lP[1][0]
        A[1][1] = lpoint[1]*lP[2][1] - lP[1][1]
        A[1][2] = lpoint[1]*lP[2][2] - lP[1][2]

        A[2][0] = rpoint[0]*rP[2][0] - rP[0][0]
        A[2][1] = rpoint[0]*rP[2][1] - rP[0][1]
        A[2][2] = rpoint[0]*rP[2][2] - rP[0][2]

        A[3][0] = rpoint[1]*rP[2][0] - rP[1][0]
        A[3][1] = rpoint[1]*rP[2][1] - rP[1][1]
        A[3][2] = rpoint[1]*rP[2][2] - rP[1][2]

        B[0][0] = -(lpoint[0]*lP[2][3] - lP[0][3])
        B[1][0] = -(lpoint[1]*lP[2][3] - lP[1][3])
        B[2][0] = -(rpoint[0]*rP[2][3] - rP[0][3])
        B[3][0] = -(rpoint[1]*rP[2][3] - rP[1][3])

        print 'A-Matrix: \n', A, '\n'
        print 'B-Matrix: \n', B, '\n'
        cv2.solve(A, B, X, cv2.DECOMP_SVD)
        print 'Solution: \n', X, '\n'
        return X

    # All hail the mighty bepis!
    def find_contours(self, image, cam):
        # TODO: Resize image to process faster
        # TODO: Limit this to the largest three contours
        # TODO: Add some critera to throw away results (throw away contours)
        # TODO: Add Normal-Cross-Correlation for filtering
        # TODO: Kalman filter centroid locations (Or time-filter the contours themselves)
        # TODO: PCA for orientation (Differentiating b/w long and tall tubes)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #cv2.erode(hsv, np.ones((15, 15)), iterations=2)
        #cv2.dilate(hsv, np.ones((15, 15)), iterations=2)
        ghsv = cv2.GaussianBlur(hsv,(7,7),0)    # Seems to work as well as expand/shrink operations

        mask = cv2.inRange(ghsv, self.gate.yellow_min, self.gate.yellow_max)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        gate_moments, largest_contours, edge_pts = [], [], []

        # Testing image slicing
        h, s, v = cv2.split(ghsv)
        v = cv2.bitwise_and(v, v, mask = mask)

        if contours is None:
            print 'No contours found'
        else:
            for cnt in contours:
                if cv2.contourArea(cnt) > 600 and cv2.contourArea(cnt) < 1400:
                    largest_contours.append(cnt)
                    moment = cv2.moments(cnt)
                    gate_moments.append((int(moment['m10']/moment['m00']), int(moment['m01']/moment['m00'])))   # Pixel location

        # Using bounding box to approximate the shape of the start gate tubes
        plane_pts = []
        for cnt in largest_contours:
            # Point of no return, really hacky stuff bringing things in and out of the correct format
            rect = cv2.minAreaRect(cnt)
            box_points = cv2.cv.BoxPoints(rect)

            # Drawing functions
            box = [np.int0(cv2.cv.BoxPoints(rect))]
            cv2.drawContours(image, box, 0, (0,0,255), 2)

            # Let's get some box midpoints
            pts = [list(point) for point in box_points]

            # Here we would need to check the orientation of the contour
            plane_pts.extend([tuple((np.add(pts[0][:], pts[3][:]) // 2).astype(int)),
                              tuple((np.add(pts[2][:], pts[1][:]) // 2).astype(int))])

        for point in plane_pts:
            cv2.circle(image, point, 4, (255,0,0), -1)

        if len(plane_pts) == 4:
            # If we have enough points for reconstruction
            dplane_pts = np.asarray([list(pt) for pt in plane_pts])

            # Camera Shenanigans
            if cam == 'l':
                self.lP = np.asarray(self.lcam_info.P).reshape((3, 4))
                P = self.lP
            elif cam == 'r':
                self.rP = np.asarray(self.rcam_info.P).reshape((3, 4))
                P = self.rP
            else:
                print 'NAN Argument'
            return plane_pts
        else:
            return None


if __name__ =='__main__':
    rospy.init_node('gate_finder', anonymous=False)
    sg = Gate()
    gf = GateFinder(sg)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting Down"

    cv2.destroyAllWindows()