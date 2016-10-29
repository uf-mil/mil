#!/usr/bin/env python
import roslib
import sys
from collections import deque
from tf import TransformListener
import rospy
import cv2
import tf as tf2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import navigator_tools as nt
import os
import numpy as np
from scan_the_code_lib.model_tracker import ModelTracker
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse, ObjectDBSingleQuery, ObjectDBSingleQueryRequest
from image_geometry import PinholeCameraModel


class ScanTheCode:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_cache = deque()
        self.model_tracker = ModelTracker()
        self.pinhole_cam = PinholeCameraModel()
        self.last_cam_info = None
        self.tf = TransformListener()
        self.mission_complete = False
        self.colors = []
        self.scan_the_code = None
        self.on = False
        self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.image_cb, queue_size=10)
        self.cam_info_sub = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.info_cb, queue_size=10)
        # self.cascade = cv2.CascadeClassifier('cascade1.xml')
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.img1 = cv2.imread(dir_path + '/train.png', 0)
        self.orb = cv2.ORB(nlevels=3, edgeThreshold=5)
        self.kp1, self.des1 = self.orb.detectAndCompute(self.img1, None)

    def drawMatches(self, img1, kp1, img2, kp2, matches):
        """
        My own implementation of cv2.drawMatches as OpenCV 2.4.9.

        does not have this function available but it's supported in
        OpenCV 3.0.0

        This function takes in two images with their associated
        keypoints, as well as a list of DMatch data structure (matches)
        that contains which keypoints matched in which images.

        An image will be produced where a montage is shown with
        the first image followed by the second image beside it.

        Keypoints are delineated with circles, while lines are connected
        between matching keypoints.

        img1,img2 - Grayscale images
        kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint
                  detection algorithms
        matches - A list of matches of corresponding keypoints through any
                  OpenCV keypoint matching algorithm
        """
        # Create a new output image that concatenates the two images together
        # (a.k.a) a montage
        rows1 = img1.shape[0]
        cols1 = img1.shape[1]
        rows2 = img2.shape[0]
        cols2 = img2.shape[1]

        out = np.zeros((max([rows1, rows2]), cols1 + cols2, 3), dtype='uint8')

        # Place the first image to the left
        out[:rows1, :cols1, :] = np.dstack([img1, img1, img1])

        # Place the next image to the right of it
        out[:rows2, cols1:cols1 + cols2, :] = np.dstack([img2, img2, img2])

        # For each pair of points we have between both images
        # draw circles, then connect a line between them
        for mat in matches:

            # Get the matching keypoints for each of the images
            img1_idx = mat.queryIdx
            img2_idx = mat.trainIdx

            # x - columns
            # y - rows
            (x1, y1) = kp1[img1_idx].pt
            (x2, y2) = kp2[img2_idx].pt

            # Draw a small circle at both co-ordinates
            # radius 4
            # colour blue
            # thickness = 1
            cv2.circle(out, (int(x1), int(y1)), 4, (255, 0, 0), 1)
            cv2.circle(out, (int(x2) + cols1, int(y2)), 4, (255, 0, 0), 1)

            # Draw a line in between the two points
            # thickness = 1
            # colour blue
            cv2.line(out, (int(x1), int(y1)), (int(x2) + cols1, int(y2)), (255, 0, 0), 1)

        # Show the image
        cv2.imshow('Matched Features', out)
        cv2.waitKey(33)
        # cv2.destroyAllWindows()

    def image_cb(self, image):
        if len(self.image_cache) > 50:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def info_cb(self, info):
        self.last_cam_info = info

    def convert_point(self, point):
        K = self.last_cam_info.K
        K = np.array(K).reshape(3, 3)
        if point.point is None:
            pl = point
        else:
            pl = [point.point.x, point.point.y, point.point.z]
        pl = K.dot(pl)
        pl[0] /= pl[2]
        pl[1] /= pl[2]
        pl[1] -= 20
        if pl[0] < 0:
            pl[0] = 0
        if pl[1] < 0:
            pl[1] = 0
        return(int(pl[0]), int(pl[1]))

    def get_2d_points(self, points_3d_enu, header, img):
        points_3d = []
        points_2d = []
        for point in points_3d_enu:
            p = PointStamped()
            p.point = point
            p.header = header
            points_3d.append(self.tf.transformPoint('stereo_left_cam', p))

        # xmin = 1000
        # zmin = 0
        # ymin = 1000
        # for i, poin in enumerate(points_3d):
        #     point = poin.point
        #     if(point.y < ymin):
        #         ymin = point.y
        #         zmin = point.z

        # buff = .3

        # for i, poin in enumerate(points_3d):
        #     point = poin.point
        #     ppoint = [point.x, point.y, point.z]
        #     if(point.y < ymin + buff and point.y > ymin - buff and point.x < xmin):
        #         xmin = point.x
        #         zmin = point.z
        #         cv2.circle(img, self.convert_point(ppoint), 3, (255, 255, 255), -1)
        # cv2.imshow("salient_points_cascade1", img)
        # cv2.waitKey(33)
        # ymin -= .75
        # xmin -= .1

        # points_3d = [[xmin, ymin, zmin], [xmin + .5, ymin + .75, zmin], [xmin, ymin + .75, zmin], [xmin + .5, ymin, zmin]]

        for i in range(0, len(points_3d)):
            point = points_3d[i]
            points_2d.append(self.convert_point(point))
        return points_2d

    def get_scan_the_code(self):
        serv = rospy.ServiceProxy("/database/single", ObjectDBSingleQuery)
        req = ObjectDBSingleQueryRequest()
        req.type = 'scan_the_code'
        scan_the_code = serv(req)
        return scan_the_code.object

    def get_rectangle(self, img, bounding_rect):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 13, 13)
        xmin, ymin, xmax, ymax = bounding_rect
        roi = gray[ymin:ymax, xmin:xmax]

        # img2 = roi

        # kp2, des2 = self.orb.detectAndCompute(img2, None)

        # # create BFMatcher object
        # bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # # Match descriptors.
        # matches = bf.match(self.des1, des2)

        # # Sort them in the order of their distance.
        # matches = sorted(matches, key=lambda x: x.distance)

        # # Draw first 10 matches.
        # # Show only the top 10 matches
        # self.drawMatches(self.img1, self.kp1, img2, kp2, matches[:4])

        # h, w, r = img.shape
        # smear = 5
        # xmin -= smear
        # ymin -= smear
        # xmax += smear
        # ymax += smear
        # if(xmin < 0):
        #     xmin = 0
        # if(ymin < 0):
        #     ymin = 0
        # if(xmax > w):
        #     xmax = w
        # if(ymax > h):
        #     ymax = h

        # roi_cascade = gray[y:y + h, x:x + w]
        edges = cv2.Canny(roi, 50, 150, apertureSize=3)
        pooop = roi.copy()

        lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=30, minLineLength=10)
        sobelx = cv2.Sobel(roi, cv2.CV_64F, 1, 0, ksize=5)
        sobely = cv2.Sobel(roi, cv2.CV_64F, 0, 1, ksize=5)

        if lines is None:
            return

        i = 0

        vert_lines = []

        one_line = False

        for x1, y1, x2, y2 in lines[0]:
            # a = np.cos(theta)
            # b = np.sin(theta)
            # x0 = a * rho
            # y0 = b * rho
            # x1 = int(x0 + 100 * (-b))
            # y1 = int(y0 + 100 * (a))
            # x2 = int(x0 - 100 * (-b))
            # y2 = int(y0 - 100 * (a))
            slope = (y2 - y1) / (x2 - x1 + 1E-100)
            interc = y2 - slope * x2
            # cv2.line(pooop, (x1, y1), (x2, y2), (0, 0, 255), 2)

            if abs(slope) > .95:
                print "new line: ", slope, interc, x1
                dud = False
                for line in vert_lines:
                    print line
                    if x1 + 10 > line[2] and x1 - 10 < line[2]:
                        print "dud"
                        dud = True
                        break
                if not dud:
                    print "======="
                    print slope, interc, x1, y1, x2, y2
                    cv2.line(pooop, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    vert_lines.append((slope, interc, x1))
                    one_line = True
                print "++++"
            i += 1
        if not one_line:
            return
        print "---------"

        cv2.imshow("salient_points_cascade2", pooop)
        cv2.waitKey(33)

        fp = cv2.goodFeaturesToTrack(roi, 4, .05, 10.0)
        if len(fp) is 0:
            return

        drawimgpp = roi.copy()
        for i, val in enumerate(fp):
            cv2.circle(drawimgpp, tuple(val[0]), 3, (255, 255, 255), -1)
        cv2.imshow("salient_points_cascade", drawimgpp)
        cv2.waitKey(33)

        keep = self.geometry_test(fp)

        if(keep):
            # for v in fp:
            #     v = v[0]
            #     cv2.waitKey(33)

            #     v[0] += xmin_a + xmin

            #     v[1] += ymin_a + ymin
            return fp

        return None

    def start_mission(self):
        while not self.mission_complete:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_cache[len(self.image_cache) - 1], "bgr8").copy()
            img_clone = cv_image.copy()
            img_clone1 = cv_image.copy()

            scan_the_code = self.get_scan_the_code()

            points_2d = self.get_2d_points(scan_the_code.points, scan_the_code.header, img_clone1)
            xmin, ymin, xmax, ymax = self.get_bounding_rect(points_2d, cv_image)
            xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)

            # DEBUG
            for i, val in enumerate(points_2d):
                cv2.circle(img_clone, (int(val[0]), int(val[1])), 3, (255, 255, 255), -1)
            cv2.rectangle(img_clone, (xmin, ymin), (xmax, ymax), (255, 255, 255))
            cv2.imshow("bounding_rect", img_clone)
            cv2.waitKey(33)

            h, w, r = cv_image.shape

            self.mission_complete = self.model_tracker.track_models(cv_image)

            if(self.mission_complete):
                print "MISSION COMPLETE"

            rect = self.get_rectangle(cv_image, (xmin, ymin, xmax, ymax))

            # if(rect is not None):
            #     self.model_tracker.register_model(rect, cv_image)

            # if(rect is not None):
            #     for i, val in enumerate(rect):
            #         cv2.circle(img_clone1, tuple(val[0]), 3, (255, 255, 255), -1)
            #     cv2.imshow("model", img_clone1)
            #     cv2.waitKey(33)

        # TODO Get the closest image in the deque

    def get_bounding_rect(self, points_2d, img):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, point in enumerate(points_2d):
            if(point[0] < xmin):
                xmin = point[0]
            if(point[0] > xmax):
                xmax = point[0]
            if(point[1] > ymax):
                ymax = point[1]
            if(point[1] < ymin):
                ymin = point[1]
        return xmin, ymin, xmax, ymax

    def get_non_furthest_points(self, i, points):
        dist_points = []
        for ind, val in enumerate(points):
            if(ind != i):
                diff = np.subtract(points[i], val)
                dist_points.append([np.linalg.norm(diff), points[ind]])

        dist_points = sorted(dist_points, key=lambda x: x[0])

        return dist_points[0][1], dist_points[1][1]

    def geometry_test(self, fp):
        if(len(fp) < 4):
            return False
        for i, p in enumerate(fp):
            p1, p2 = self.get_non_furthest_points(i, fp)
            diff1 = np.subtract(p1, p)[0]
            diff2 = np.subtract(p2, p)[0]
            diff1 /= np.linalg.norm(diff1)
            diff2 /= np.linalg.norm(diff2)

            val = abs(np.dot(diff1, diff2))
            if(val > .05):
                return False

        return True

    def get_salient_points(self, img, rect):
        if(rect[1] == rect[3] or rect[0] == rect[2]):
            return
        roi = img[rect[1]:rect[3], rect[0]:rect[2]]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        fp = cv2.goodFeaturesToTrack(gray, 35, .05, 10.0)  # [, corners[, mask[, blockSize[, useHarrisDetector[, k]]]]]
        if(fp is None):
            return
        # DEBUG
        img_clone = gray.copy()
        for i, val in enumerate(fp):
            cv2.circle(img_clone, tuple(val[0]), 3, (255, 255, 255), -1)

        cv2.imshow("salient_points", img_clone)
        cv2.waitKey(33)

    def mission_status(self, req):
        obs = False
        colors = None
        if(len(self.model_tracker.models) > 0):
            obs = True
        if(self.mission_complete):
            colors = self.model_tracker.colors
        found = self.mission_complete
        return ScanTheCodeMissionResponse(obs, found, colors)

    def activate(self, req):
        if(self.on):
            del self.model_tracker
            self.model_tracker = ModelTracker()
            self.mission_complete = False
            self.colors = []
            self.on = False
        if(not self.on):
            self.on = True
            self.scan_the_code = req.object
            self.start_mission()
        return ScanTheCodeMissionResponse(None, None, None)


def main(args):
    rospy.init_node('model_detector', anonymous=True)
    ic = ScanTheCode()
    status = rospy.Service('/vision/scan_the_code_status', ScanTheCodeMission, ic.mission_status)
    activate = rospy.Service('/vision/scan_the_code_activate', ScanTheCodeMission, ic.activate)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
